# TM221 PLC Python Toolkit

Real-time data acquisition, logging, and visualization for the **Schneider Electric Modicon TM221CE16R** programmable logic controller over Modbus TCP.

![Dashboard Screenshot](docs/dashboard_screenshot.png)

## Overview

This toolkit provides three Python applications for communicating with a Modicon M221 series PLC:

| Script | Purpose |
|--------|---------|
| `process_monitor.py` | Real-time data acquisition, CSV logging, and live trend dashboard |
| `plc_terminal.py` | Interactive command-line shell for reading/writing PLC registers |
| `find_plc.py` | Network scanner to locate M221 PLCs on the `10.10.x.x` subnet |

The applications use **Modbus TCP** (port 502) over the PLC's built-in Ethernet port. No proprietary software or drivers are required on the monitoring side — only Python and the open-source `pymodbus` library.

## Hardware Requirements

- **PLC**: Schneider Electric Modicon TM221CE16R (or any M221 series controller with Ethernet)
- **Power**: 100–240 VAC to the PLC's L/N terminals
- **Network**: Standard Ethernet cable (Cat5e or Cat6) — direct PC-to-PLC or through a switch
- **24VDC supply** *(optional)*: Required only if testing physical I/O (digital inputs, analog sensors, output indicators)

## Software Requirements

- Python 3.8+
- `pymodbus` (required)
- `matplotlib` (optional — enables the graphical dashboard; falls back to console mode without it)

```bash
pip install pymodbus matplotlib
```

> **Note on pymodbus versions:** This project uses pymodbus 4.x API conventions (`device_id=` parameter). If you are on pymodbus 3.x, replace `device_id=` with `slave=` in all Modbus calls. If you are on pymodbus 2.x, replace it with `unit=`.

## Quick Start

### 1. Find Your PLC's IP Address

The M221 defaults to `10.10.x.y`, where `x` and `y` are derived from the last two bytes of the MAC address printed on the front label.

**Manual calculation:** If the MAC is `00:00:54:6A:27:DC`, the last two bytes are `27` and `DC`. Converting hex to decimal: `0x27 = 39`, `0xDC = 220`. The default IP is **`10.10.39.220`**.

**Automatic discovery:**
```bash
python find_plc.py
```

### 2. Configure Your PC's Network

Set your Ethernet adapter to a static IP on the same subnet:

| Setting | Value |
|---------|-------|
| IP Address | `10.10.39.50` (or any unused address in `10.x.x.x`) |
| Subnet Mask | `255.0.0.0` |
| Gateway | *(leave blank)* |

Verify connectivity:
```bash
ping 10.10.39.220
```

### 3. Download a PLC Program

The M221's Modbus registers are not allocated until a program has been downloaded. Use **EcoStruxure Machine Expert – Basic** (free, Windows only) to create and download at least a minimal program. Without a loaded program, all Modbus reads will return "Illegal Data Address" (exception code 2).

### 4. Run the Monitor

```bash
# Full dashboard (requires matplotlib + display)
python process_monitor.py --ip 10.10.39.220

# Console-only mode (no GUI required)
python process_monitor.py --ip 10.10.39.220 --headless

# Custom polling interval and run duration
python process_monitor.py --ip 10.10.39.220 --interval 0.5 --duration 3600
```

## Process Monitor

`process_monitor.py` connects to the PLC, polls configured tags at a fixed interval, logs every sample to daily CSV files, and renders a live six-panel trend dashboard.

### Simulated Process: Mixing Tank

The default configuration monitors a mixing tank with the following I/O:

**Analog Inputs (sensors)**

| Tag | Register | Raw Range | Unit | Eng. Range |
|-----|----------|-----------|------|------------|
| Temperature | `%IW0` | 0–1000 | °C | 0.0–100.0 |
| Pressure | `%IW1` | 0–1000 | bar | 0.0–10.0 |
| Tank Level | `%IW2` | 0–1000 | % | 0.0–100.0 |
| Flow Rate | `%IW3` | 0–1000 | L/min | 0.0–50.0 |

**Digital Outputs (actuators)**

| Tag | Address |
|-----|---------|
| Heater | `%Q0.0` |
| Mixer Motor | `%Q0.1` |
| Inlet Valve | `%Q0.2` |
| Outlet Valve | `%Q0.3` |

**Holding Registers**

| Tag | Address | Purpose |
|-----|---------|---------|
| Temperature Setpoint | `%MW0` | Operator setpoint (written by HMI/SCADA) |
| Pressure Setpoint | `%MW1` | Operator setpoint |
| Alarm Word | `%MW10` | 16-bit packed alarm flags |
| Cycle Counter | `%MW20` | PLC scan counter |

**Alarm Bits (`%MW10`)**

| Bit | Alarm |
|-----|-------|
| 0 | Over_Temp |
| 1 | Over_Pressure |
| 2 | Low_Level |
| 3 | High_Level |
| 4 | Flow_Fault |
| 5 | Heater_Fault |
| 6 | Motor_Fault |
| 7 | E_Stop |

### Command-Line Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--ip` | `10.10.39.220` | PLC IP address |
| `--port` | `502` | Modbus TCP port |
| `--slave` | `1` | Modbus slave/unit ID |
| `--interval` | `1.0` | Polling interval in seconds |
| `--duration` | `0` (forever) | Run time in seconds; 0 = infinite |
| `--headless` | `false` | Disable GUI; console and CSV only |
| `--logdir` | `logs` | Directory for CSV log files |
| `--history` | `300` | Max data points retained in chart |

### CSV Output

Log files are created daily in the `--logdir` directory with automatic rotation at midnight:

```
logs/
├── process_log_2026-04-28.csv
├── process_log_2026-04-29.csv
└── ...
```

Each row contains a timestamp (millisecond resolution), all analog values in engineering units, digital output states (0/1), the alarm word in hex, and the PLC cycle counter.

### Customizing for Your Process

All process-specific configuration lives in four tag tables at the top of `process_monitor.py`. To adapt the monitor for a different process, edit these tables — no other code changes are needed:

```python
ANALOG_TAGS = [
    # (name, register_type, address, scale_min, scale_max, unit, eng_min, eng_max)
    ("My_Sensor", "input", 0, 0, 4095, "psi", 0.0, 150.0),
]
```

## PLC Terminal

`plc_terminal.py` provides an interactive command-line shell for manual register access, real-time monitoring, and diagnostics.

```bash
pip install pymodbus rich   # rich is optional, adds colored tables
python plc_terminal.py
```

### Commands

**Connection**
```
connect tcp 10.10.39.220     Connect via Modbus TCP
connect rtu /dev/ttyUSB0     Connect via Modbus RTU (serial)
disconnect                   Close connection
slave <id>                   Set Modbus slave ID
```

**Reading**
```
ri  [start] [count]          Read discrete inputs (%I)
rq  [start] [count]          Read coils / outputs (%Q)
rmw [start] [count]          Read holding registers (%MW)
riw [start] [count]          Read input registers (%IW)
rfloat <register>            Read 32-bit float from two registers
```

**Writing**
```
wq  <addr> <0|1>             Write a single coil
wmw <start> <val> ...        Write holding register(s)
wfloat <reg> <value>         Write 32-bit float to two registers
toggle <addr>                Toggle a coil on/off
```

**Tools**
```
scan                         Full I/O snapshot
monitor <mw|i|q> [start] [count]   Live-watch values (Enter to stop)
dump [start] [count]         Hex dump of registers
```

## PLC Finder

`find_plc.py` scans the network for devices with Modbus TCP (port 502) open, then attempts to identify each one by reading registers and querying the Modbus Device Identification object.

```bash
python find_plc.py                                         # Full 10.10.x.x scan
python find_plc.py --start 10.10.39.1 --end 10.10.39.254  # Narrow scan
python find_plc.py --timeout 0.3 --workers 1000            # Faster scan
```

## Network & Wiring

### Ethernet Connection

Connect a standard Ethernet cable between your PC and the PLC's RJ45 port. Modern hardware auto-negotiates, so a regular straight-through patch cable works — no crossover cable needed.

### PLC Default IP Address

The M221 defaults to `10.10.x.y` with a `255.0.0.0` subnet mask, where `x.y` is derived from the last two bytes of the MAC address on the front label. If a DHCP server is configured but unreachable, the PLC falls back to this address.

To confirm the IP via packet capture:

```bash
# Listen for ARP announcements from the PLC (replace en0 with your interface)
sudo tcpdump -i en0 -n ether host <PLC_MAC_ADDRESS>
```

### Power

The TM221CE16R accepts **100–240 VAC** on its L/N terminals. For benchtop testing, wire a standard appliance cord to the terminals and plug it into a switched power strip. A separate **24VDC supply** is needed only for powering I/O circuits (sensors, input simulation switches, output indicator LEDs).

## M221 Modbus Address Map

All addresses are 0-based (pymodbus convention).

| PLC Variable | Modbus FC | Address | Direction | Description |
|-------------|-----------|---------|-----------|-------------|
| `%M0`–`%M255` | FC 1/5/15 | 0–255 | Read/Write | Memory bits (coils) |
| `%M0`–`%M255` | FC 2 | 0–255 | Read | Memory bits (discrete inputs) |
| `%MW0`–`%MW255` | FC 3/6/16 | 0–255 | Read/Write | Memory words (holding registers) |
| `%MW0`–`%MW255` | FC 4 | 0–255 | Read | Memory words (input registers) |
| `%Q0.0`–`%Q0.15` | FC 1 | 128–143 | Read | Physical outputs |
| `%I0.0`–`%I0.15` | FC 2 | 128–143 | Read | Physical inputs |

> **Important:** Physical I/O (`%I`, `%Q`, `%IW`) is accessible at offset addresses, not at address 0. To make physical I/O readable at convenient addresses, copy them into `%MW` registers in the PLC program.

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `Connection timed out` | Wrong IP or subnet mismatch | Verify IP with `find_plc.py` or `tcpdump`; check your PC's static IP is on the same subnet |
| `Connection refused` | Modbus TCP disabled | Enable in EcoStruxure Machine Expert – Basic under Ethernet settings |
| `Exception code 2` (Illegal Data Address) | No program loaded, or wrong register address | Download a program to the PLC; verify address mapping |
| `unexpected keyword argument 'slave'` | pymodbus version mismatch | Use `device_id=` (v4.x), `slave=` (v3.x), or `unit=` (v2.x) |
| Dashboard doesn't appear | matplotlib not installed | `pip install matplotlib`; runs in headless mode without it |
| Ping works but Modbus fails | Port 502 blocked or PLC in fault state | Test with `nc -zv <ip> 502`; check PLC ERR LED |

## Project Structure

```
├── process_monitor.py        # Data acquisition, logging, and dashboard
├── plc_terminal.py           # Interactive register read/write shell
├── find_plc.py               # Network scanner for M221 PLCs
├── docs/
│   ├── dashboard_screenshot.png
│   ├── tm221_guide.docx      # Development & configuration guide
│   └── five_patterns_article.docx
├── logs/                     # CSV data files (created at runtime)
└── README.md
```

## License

MIT
