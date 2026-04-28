#!/usr/bin/env python3
"""
TM221 Process Monitor
=====================
Real-time data acquisition, CSV logging, and live trend dashboard
for a Modicon TM221CE16R PLC over Modbus TCP.

Simulated process:  Mixing Tank
  - Temperature sensor     → %IW0  (analog input, 0–1000 = 0.0–100.0 °C)
  - Pressure sensor        → %IW1  (analog input, 0–1000 = 0.0–10.0 bar)
  - Tank level sensor      → %IW2  (analog input, 0–1000 = 0.0–100.0 %)
  - Flow rate sensor       → %IW3  (analog input, 0–1000 = 0.0–50.0 L/min)
  - Heater output          → %Q0.0 (digital output)
  - Mixer motor            → %Q0.1 (digital output)
  - Inlet valve            → %Q0.2 (digital output)
  - Outlet valve           → %Q0.3 (digital output)
  - Temperature setpoint   → %MW0  (holding register, written by HMI/SCADA)
  - Pressure setpoint      → %MW1
  - Alarm word             → %MW10 (bit-packed alarm flags from PLC logic)
  - PLC cycle counter      → %MW20

Install
-------
    pip install pymodbus matplotlib

Usage
-----
    python process_monitor.py                        # default 192.168.1.100
    python process_monitor.py --ip 10.0.0.50         # custom IP
    python process_monitor.py --headless              # log only, no GUI
    python process_monitor.py --interval 0.5          # sample every 500 ms
    python process_monitor.py --duration 3600         # run for 1 hour then stop
"""

import argparse
import csv
import os
import signal
import sys
import threading
import time
from collections import deque
from datetime import datetime, timedelta
from typing import Optional

try:
    from pymodbus.client import ModbusTcpClient
except ImportError:
    sys.exit("pymodbus not found.  Install with:  pip install pymodbus")

# ── Process tag definitions ──────────────────────────────────────────────────
# Each tag describes one piece of data to read from the PLC.

ANALOG_TAGS = [
    # (name,       register_type, address, scale_min, scale_max, unit,   eng_min, eng_max)
    ("Temperature", "input",  0, 0, 1000, "°C",    0.0, 100.0),
    ("Pressure",    "input",  1, 0, 1000, "bar",   0.0,  10.0),
    ("Tank_Level",  "input",  2, 0, 1000, "%",     0.0, 100.0),
    ("Flow_Rate",   "input",  3, 0, 1000, "L/min", 0.0,  50.0),
]

DIGITAL_TAGS = [
    # (name,         address)
    ("Heater",       0),
    ("Mixer_Motor",  1),
    ("Inlet_Valve",  2),
    ("Outlet_Valve", 3),
]

SETPOINT_TAGS = [
    # (name,              address, unit)
    ("Temp_Setpoint",     0, "°C"),
    ("Pressure_Setpoint", 1, "bar"),
]

ALARM_BITS = [
    "Over_Temp",
    "Over_Pressure",
    "Low_Level",
    "High_Level",
    "Flow_Fault",
    "Heater_Fault",
    "Motor_Fault",
    "E_Stop",
]


# ── Scaling helper ───────────────────────────────────────────────────────────

def scale(raw, raw_lo, raw_hi, eng_lo, eng_hi):
    """Linear scale: raw ADC counts → engineering units."""
    if raw_hi == raw_lo:
        return eng_lo
    return eng_lo + (raw - raw_lo) * (eng_hi - eng_lo) / (raw_hi - raw_lo)


# ── Data logger ──────────────────────────────────────────────────────────────

class CSVLogger:
    """Thread-safe CSV logger with automatic daily file rotation."""

    def __init__(self, directory="logs"):
        self.directory = directory
        os.makedirs(directory, exist_ok=True)
        self._file = None
        self._writer = None
        self._current_date = None
        self._lock = threading.Lock()

    def _rotate_if_needed(self):
        today = datetime.now().date()
        if today != self._current_date:
            if self._file:
                self._file.close()
            filename = os.path.join(
                self.directory, f"process_log_{today.isoformat()}.csv"
            )
            is_new = not os.path.exists(filename)
            self._file = open(filename, "a", newline="")
            self._writer = csv.writer(self._file)
            if is_new:
                self._writer.writerow(self._header())
            self._current_date = today

    @staticmethod
    def _header():
        cols = ["Timestamp"]
        for name, *_ in ANALOG_TAGS:
            cols.append(name)
        for name, _ in DIGITAL_TAGS:
            cols.append(name)
        for name, _, _ in SETPOINT_TAGS:
            cols.append(name)
        cols.append("Alarms")
        cols.append("Cycle_Count")
        return cols

    def log(self, record: dict):
        with self._lock:
            self._rotate_if_needed()
            row = [record.get("timestamp", "")]
            for name, *_ in ANALOG_TAGS:
                row.append(f"{record.get(name, 0.0):.2f}")
            for name, _ in DIGITAL_TAGS:
                row.append(int(record.get(name, False)))
            for name, _, _ in SETPOINT_TAGS:
                row.append(record.get(name, 0))
            row.append(record.get("alarms_hex", "0x0000"))
            row.append(record.get("cycle_count", 0))
            self._writer.writerow(row)
            self._file.flush()

    def close(self):
        with self._lock:
            if self._file:
                self._file.close()


# ── PLC reader ───────────────────────────────────────────────────────────────

class PLCReader:
    """Reads all configured tags from the PLC in one scan cycle."""

    def __init__(self, ip, port=502, slave=1, timeout=3):
        self.client = ModbusTcpClient(ip, port=port, timeout=timeout)
        self.slave = slave
        self.connected = False

    def connect(self):
        self.connected = self.client.connect()
        return self.connected

    def disconnect(self):
        self.client.close()
        self.connected = False

    def read_all(self) -> Optional[dict]:
        """Return a dict with all tag values, or None on comm failure."""
        record = {"timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]}

        # ── Analog inputs (%IW) ──────────────────────────────────────────
        try:
            result = self.client.read_input_registers(0, count=4, device_id=self.slave)
            if result.isError():
                return None
            for i, (name, _rtype, _addr, rlo, rhi, _unit, elo, ehi) in enumerate(ANALOG_TAGS):
                record[name] = round(scale(result.registers[i], rlo, rhi, elo, ehi), 2)
        except Exception:
            return None

        # ── Digital outputs (%Q) ─────────────────────────────────────────
        try:
            result = self.client.read_coils(0, count=4, device_id=self.slave)
            if result.isError():
                return None
            for i, (name, _addr) in enumerate(DIGITAL_TAGS):
                record[name] = bool(result.bits[i])
        except Exception:
            return None

        # ── Setpoints (%MW0, %MW1) ───────────────────────────────────────
        try:
            result = self.client.read_holding_registers(0, count=2, device_id=self.slave)
            if not result.isError():
                for i, (name, _addr, _unit) in enumerate(SETPOINT_TAGS):
                    record[name] = result.registers[i]
        except Exception:
            pass  # non-critical

        # ── Alarm word (%MW10) & cycle counter (%MW20) ───────────────────
        try:
            result = self.client.read_holding_registers(10, count=1, device_id=self.slave)
            if not result.isError():
                alarm_word = result.registers[0]
                record["alarm_word"] = alarm_word
                record["alarms_hex"] = f"0x{alarm_word:04X}"
                record["active_alarms"] = [
                    ALARM_BITS[b] for b in range(min(len(ALARM_BITS), 16))
                    if alarm_word & (1 << b)
                ]
        except Exception:
            record["alarm_word"] = 0
            record["active_alarms"] = []
            record["alarms_hex"] = "0x0000"

        try:
            result = self.client.read_holding_registers(20, count=1, device_id=self.slave)
            if not result.isError():
                record["cycle_count"] = result.registers[0]
        except Exception:
            record["cycle_count"] = 0

        return record


# ── Live dashboard (matplotlib) ──────────────────────────────────────────────

def run_dashboard(history, stop_event, interval):
    """Matplotlib-based real-time trend dashboard."""
    try:
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        import matplotlib.dates as mdates
    except ImportError:
        print("[WARN] matplotlib not available — running in headless mode.")
        stop_event.wait()
        return

    fig, axes = plt.subplots(3, 2, figsize=(14, 9))
    fig.suptitle("TM221 Process Monitor — Mixing Tank", fontsize=14, fontweight="bold")
    fig.patch.set_facecolor("#1e1e2e")

    COLORS = {
        "Temperature": "#ef4444",
        "Pressure":    "#3b82f6",
        "Tank_Level":  "#22c55e",
        "Flow_Rate":   "#f59e0b",
    }

    style = {
        "bg":   "#1e1e2e",
        "fg":   "#cdd6f4",
        "grid": "#45475a",
        "line": 1.8,
    }

    for ax_row in axes:
        for ax in ax_row:
            ax.set_facecolor(style["bg"])
            ax.tick_params(colors=style["fg"], labelsize=8)
            for spine in ax.spines.values():
                spine.set_color(style["grid"])
            ax.grid(True, color=style["grid"], alpha=0.3, linewidth=0.5)

    # Axes assignments
    ax_temp   = axes[0][0]
    ax_press  = axes[0][1]
    ax_level  = axes[1][0]
    ax_flow   = axes[1][1]
    ax_dio    = axes[2][0]
    ax_alarms = axes[2][1]

    def update(_frame):
        if stop_event.is_set():
            plt.close(fig)
            return

        data = list(history)
        if not data:
            return

        times = [datetime.strptime(d["timestamp"], "%Y-%m-%d %H:%M:%S.%f") for d in data]

        # ── Analog trends ────────────────────────────────────────────────
        for ax, tag_name, unit, ylim in [
            (ax_temp,  "Temperature", "°C",    (0, 110)),
            (ax_press, "Pressure",    "bar",   (0, 12)),
            (ax_level, "Tank_Level",  "%",     (0, 105)),
            (ax_flow,  "Flow_Rate",   "L/min", (0, 55)),
        ]:
            ax.clear()
            ax.set_facecolor(style["bg"])
            ax.grid(True, color=style["grid"], alpha=0.3, linewidth=0.5)
            values = [d.get(tag_name, 0) for d in data]
            ax.plot(times, values, color=COLORS[tag_name], linewidth=style["line"])
            ax.fill_between(times, values, alpha=0.08, color=COLORS[tag_name])
            ax.set_ylim(ylim)
            ax.set_title(f"{tag_name.replace('_', ' ')} ({unit})",
                         color=style["fg"], fontsize=10, pad=6)
            ax.tick_params(colors=style["fg"], labelsize=7)

            # Setpoint line for temperature
            if tag_name == "Temperature" and data:
                sp = data[-1].get("Temp_Setpoint", None)
                if sp is not None and sp > 0:
                    ax.axhline(y=sp, color="#f87171", linestyle="--",
                               linewidth=1, alpha=0.6, label=f"SP {sp}")
                    ax.legend(loc="upper left", fontsize=7,
                              facecolor=style["bg"], edgecolor=style["grid"],
                              labelcolor=style["fg"])

            ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
            plt.setp(ax.xaxis.get_majorticklabels(), rotation=30, ha="right")

        # ── Digital I/O status ───────────────────────────────────────────
        ax_dio.clear()
        ax_dio.set_facecolor(style["bg"])
        ax_dio.set_title("Digital Outputs", color=style["fg"], fontsize=10, pad=6)
        ax_dio.set_xlim(0, 1)
        ax_dio.set_ylim(-0.5, len(DIGITAL_TAGS) - 0.5)
        ax_dio.set_yticks(range(len(DIGITAL_TAGS)))
        ax_dio.set_yticklabels([n.replace("_", " ") for n, _ in DIGITAL_TAGS],
                               fontsize=9, color=style["fg"])
        ax_dio.set_xticks([])
        latest = data[-1] if data else {}
        for i, (name, _addr) in enumerate(DIGITAL_TAGS):
            state = latest.get(name, False)
            color = "#22c55e" if state else "#6c7086"
            ax_dio.barh(i, 0.6 if state else 0.2, left=0.2,
                        height=0.5, color=color, edgecolor="none", alpha=0.9)
            ax_dio.text(0.85, i, "ON" if state else "OFF",
                        va="center", ha="center", fontsize=9, fontweight="bold",
                        color="#1e1e2e" if state else style["fg"])

        # ── Alarm panel ──────────────────────────────────────────────────
        ax_alarms.clear()
        ax_alarms.set_facecolor(style["bg"])
        ax_alarms.set_title("Alarms", color=style["fg"], fontsize=10, pad=6)
        ax_alarms.set_xlim(0, 1)
        ax_alarms.set_ylim(-0.5, len(ALARM_BITS) - 0.5)
        ax_alarms.set_yticks(range(len(ALARM_BITS)))
        ax_alarms.set_yticklabels([a.replace("_", " ") for a in ALARM_BITS],
                                  fontsize=8, color=style["fg"])
        ax_alarms.set_xticks([])
        active = latest.get("active_alarms", [])
        for i, alarm_name in enumerate(ALARM_BITS):
            is_active = alarm_name in active
            color = "#ef4444" if is_active else "#313244"
            ax_alarms.barh(i, 0.5, left=0.25, height=0.45,
                           color=color, edgecolor="none", alpha=0.9)
            if is_active:
                ax_alarms.text(0.5, i, "ACTIVE", va="center", ha="center",
                               fontsize=8, fontweight="bold", color="white")

        fig.tight_layout(rect=[0, 0, 1, 0.95])

    anim = FuncAnimation(fig, update, interval=int(interval * 1000), cache_frame_data=False)
    plt.show()


# ── Console display (headless fallback) ──────────────────────────────────────

def print_console_record(record, sample_num):
    ts = record["timestamp"]
    temp  = record.get("Temperature", 0)
    press = record.get("Pressure", 0)
    level = record.get("Tank_Level", 0)
    flow  = record.get("Flow_Rate", 0)

    dio = " | ".join(
        f"{name}={'ON' if record.get(name, False) else 'OFF'}"
        for name, _ in DIGITAL_TAGS
    )

    alarms = record.get("active_alarms", [])
    alarm_str = ", ".join(alarms) if alarms else "None"

    print(
        f"[{ts}] #{sample_num:>5}  "
        f"T={temp:6.1f}°C  P={press:5.2f}bar  "
        f"Level={level:5.1f}%  Flow={flow:5.1f}L/min  "
        f"| {dio} | Alarms: {alarm_str}"
    )


# ── Main loop ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="TM221 Process Monitor — real-time data logging & visualization"
    )
    parser.add_argument("--ip",       default="10.10.39.220", help="PLC IP address")
    parser.add_argument("--port",     type=int, default=502,   help="Modbus TCP port")
    parser.add_argument("--slave",    type=int, default=1,     help="Modbus slave ID")
    parser.add_argument("--interval", type=float, default=1.0, help="Polling interval (seconds)")
    parser.add_argument("--duration", type=int, default=0,     help="Run for N seconds (0=forever)")
    parser.add_argument("--headless", action="store_true",     help="Console only, no GUI")
    parser.add_argument("--logdir",   default="logs",          help="CSV log directory")
    parser.add_argument("--history",  type=int, default=300,   help="Max data points in chart")
    args = parser.parse_args()

    # Ring buffer for chart data
    history = deque(maxlen=args.history)

    # Stop event for clean shutdown
    stop_event = threading.Event()

    def on_signal(_sig, _frame):
        print("\nShutting down...")
        stop_event.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # Connect to PLC
    reader = PLCReader(args.ip, port=args.port, slave=args.slave)
    print(f"Connecting to PLC at {args.ip}:{args.port} (slave {args.slave})...")

    if not reader.connect():
        sys.exit(f"ERROR: Cannot connect to {args.ip}:{args.port}")

    print(f"Connected.  Polling every {args.interval}s.  Logging to ./{args.logdir}/")
    if args.duration:
        print(f"Will run for {args.duration} seconds.")
    print("Press Ctrl+C to stop.\n")

    logger = CSVLogger(directory=args.logdir)

    # Data collection thread
    sample_count = 0
    start_time = time.time()
    comm_errors = 0

    def collect_loop():
        nonlocal sample_count, comm_errors
        while not stop_event.is_set():
            # Duration check
            if args.duration and (time.time() - start_time) >= args.duration:
                stop_event.set()
                break

            record = reader.read_all()

            if record is None:
                comm_errors += 1
                if comm_errors <= 3:
                    print(f"[WARN] Communication error #{comm_errors} — retrying...")
                if comm_errors >= 10:
                    print("[ERR] Too many consecutive errors. Check PLC connection.")
                    stop_event.set()
                    break
                # Try to reconnect
                reader.disconnect()
                time.sleep(1)
                reader.connect()
                continue

            comm_errors = 0
            sample_count += 1

            # Log to CSV
            logger.log(record)

            # Store for chart
            history.append(record)

            # Console output
            if args.headless:
                print_console_record(record, sample_count)

            stop_event.wait(args.interval)

    collector = threading.Thread(target=collect_loop, daemon=True)
    collector.start()

    # Run dashboard or wait in headless mode
    if args.headless:
        collector.join()
    else:
        run_dashboard(history, stop_event, args.interval)
        stop_event.set()
        collector.join(timeout=5)

    # Cleanup
    reader.disconnect()
    logger.close()

    elapsed = time.time() - start_time
    print(f"\nSession complete: {sample_count} samples in {timedelta(seconds=int(elapsed))}")
    print(f"Logs saved in ./{args.logdir}/")


if __name__ == "__main__":
    main()
