#!/usr/bin/env python3
"""
TM221 Process Monitor
=====================
Real-time data acquisition, CSV logging, and live trend dashboard
for a Modicon TM221CE16R PLC over Modbus TCP.

The TM221CE16R has 2 built-in analog inputs (0–10V, 10-bit resolution),
9 digital inputs, and 7 relay outputs. This script reads all available
I/O and presents it in a live dashboard.

Supports a --simulate flag to run the full dashboard and logging pipeline
with realistic simulated process data, no PLC hardware required.

Install
-------
    pip install pymodbus matplotlib

Usage
-----
    python process_monitor.py --simulate              # simulated data, no PLC needed
    python process_monitor.py --ip 10.10.39.220       # real PLC
    python process_monitor.py --headless --simulate    # simulated, console only
"""

import argparse
import csv
import math
import os
import random
import signal
import sys
import threading
import time
from collections import deque
from datetime import datetime, timedelta
from typing import Optional

# ── Process tag definitions ──────────────────────────────────────────────────

ANALOG_TAGS = [
    # (name,       register_type, address, scale_min, scale_max, unit,   eng_min, eng_max)
    ("Temperature", "input",  0, 0, 1023, "°C",  0.0, 100.0),
    ("Pressure",    "input",  1, 0, 1023, "bar", 0.0,  10.0),
]

DIGITAL_OUTPUT_TAGS = [
    ("Heater",       0),
    ("Mixer_Motor",  1),
    ("Inlet_Valve",  2),
    ("Outlet_Valve", 3),
]

DIGITAL_INPUT_TAGS = [
    ("Start_Button",     0),
    ("Stop_Button",      1),
    ("HiTemp_Switch",    2),
    ("HiPress_Switch",   3),
    ("Level_High",       4),
    ("Level_Low",        5),
]

SETPOINT_TAGS = [
    ("Temp_Setpoint",     0, "°C"),
    ("Pressure_Setpoint", 1, "bar"),
]

ALARM_BITS = [
    "Over_Temp",
    "Over_Pressure",
    "Low_Level",
    "High_Level",
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
        for name, _ in DIGITAL_OUTPUT_TAGS:
            cols.append(f"Out_{name}")
        for name, _ in DIGITAL_INPUT_TAGS:
            cols.append(f"In_{name}")
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
            for name, _ in DIGITAL_OUTPUT_TAGS:
                row.append(int(record.get(f"out_{name}", False)))
            for name, _ in DIGITAL_INPUT_TAGS:
                row.append(int(record.get(f"in_{name}", False)))
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


# ── Simulated PLC reader ────────────────────────────────────────────────────

class SimulatedPLCReader:
    """Generates realistic simulated process data without a real PLC.

    Simulates a mixing-tank process:
      - Temperature ramps toward setpoint with PID-like behavior
      - Pressure follows a slow sine wave with noise
      - Digital outputs cycle based on process state
      - Alarms trigger when values exceed thresholds
    """

    def __init__(self):
        self.connected = False
        self._tick = 0
        self._cycle_count = 0

        # Process state
        self._temp = 25.0       # starting temperature (cold tank)
        self._pressure = 3.5    # starting pressure
        self._temp_sp = 65      # temperature setpoint
        self._press_sp = 5      # pressure setpoint
        self._running = False   # process running state
        self._start_tick = 0

    def connect(self):
        self.connected = True
        return True

    def disconnect(self):
        self.connected = False

    def read_all(self) -> Optional[dict]:
        self._tick += 1
        self._cycle_count += 1
        t = self._tick
        record = {"timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]}

        # ── Process simulation ───────────────────────────────────────────
        # Start the process after 5 seconds
        if t == 5:
            self._running = True
            self._start_tick = t

        elapsed = t - self._start_tick if self._running else 0

        # Temperature: ramp toward setpoint with overshoot, then settle
        if self._running:
            # Simulate a first-order response with slight overshoot
            tau = 60.0  # time constant in ticks
            target = self._temp_sp
            error = target - self._temp
            self._temp += error * (1.0 / tau) + random.gauss(0, 0.3)

            # Add occasional disturbances
            if t % 120 == 0:
                self._temp += random.gauss(0, 2.0)
        else:
            self._temp = 25.0 + random.gauss(0, 0.2)

        # Pressure: slow oscillation around setpoint
        self._pressure = (self._press_sp
                          + 0.8 * math.sin(t * 0.04)
                          + 0.3 * math.sin(t * 0.11 + 1.5)
                          + random.gauss(0, 0.15))

        # Clamp to realistic ranges
        self._temp = max(0, min(100, self._temp))
        self._pressure = max(0, min(10, self._pressure))

        record["Temperature"] = round(self._temp, 2)
        record["Pressure"] = round(self._pressure, 2)

        # ── Digital outputs (process-dependent) ──────────────────────────
        if self._running:
            # Heater: ON when below setpoint - 2°C, OFF when above setpoint
            heater_on = self._temp < (self._temp_sp - 1.0)
            # Mixer: always ON when running
            mixer_on = True
            # Inlet valve: ON during first 60 ticks of run (filling)
            inlet_on = elapsed < 60
            # Outlet valve: ON after 180 ticks (draining cycle)
            outlet_on = elapsed > 180 and (elapsed % 120) > 60
        else:
            heater_on = False
            mixer_on = False
            inlet_on = False
            outlet_on = False

        record["out_Heater"] = heater_on
        record["out_Mixer_Motor"] = mixer_on
        record["out_Inlet_Valve"] = inlet_on
        record["out_Outlet_Valve"] = outlet_on

        # ── Digital inputs (simulated field signals) ─────────────────────
        record["in_Start_Button"] = (t == 5)  # momentary press at tick 5
        record["in_Stop_Button"] = False
        record["in_HiTemp_Switch"] = self._temp > 80.0
        record["in_HiPress_Switch"] = self._pressure > 8.0
        record["in_Level_High"] = elapsed > 50 and self._running
        record["in_Level_Low"] = elapsed < 20 or not self._running

        # ── Setpoints ────────────────────────────────────────────────────
        record["Temp_Setpoint"] = self._temp_sp
        record["Pressure_Setpoint"] = self._press_sp

        # ── Alarms (threshold-based) ─────────────────────────────────────
        alarm_word = 0
        if self._temp > 80.0:
            alarm_word |= (1 << 0)   # Over_Temp
        if self._pressure > 8.0:
            alarm_word |= (1 << 1)   # Over_Pressure
        if record["in_Level_Low"]:
            alarm_word |= (1 << 2)   # Low_Level
        if False:                    # High_Level placeholder
            alarm_word |= (1 << 3)

        record["alarm_word"] = alarm_word
        record["alarms_hex"] = f"0x{alarm_word:04X}"
        record["active_alarms"] = [
            ALARM_BITS[b] for b in range(min(len(ALARM_BITS), 16))
            if alarm_word & (1 << b)
        ]

        record["cycle_count"] = self._cycle_count

        return record


# ── Real PLC reader ──────────────────────────────────────────────────────────

class PLCReader:
    """Reads all configured tags from the PLC in one scan cycle."""

    def __init__(self, ip, port=502, slave=1, timeout=3):
        try:
            from pymodbus.client import ModbusTcpClient
        except ImportError:
            sys.exit("pymodbus not found.  Install with:  pip install pymodbus")
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

        # ── Analog inputs (%IW0, %IW1 — 2 built-in channels) ────────────
        try:
            num_analog = len(ANALOG_TAGS)
            result = self.client.read_input_registers(0, count=num_analog, device_id=self.slave)
            if result.isError():
                return None
            for i, (name, _rtype, _addr, rlo, rhi, _unit, elo, ehi) in enumerate(ANALOG_TAGS):
                record[name] = round(scale(result.registers[i], rlo, rhi, elo, ehi), 2)
        except Exception:
            return None

        # ── Digital outputs (%Q — 7 relay outputs) ───────────────────────
        try:
            num_outputs = len(DIGITAL_OUTPUT_TAGS)
            result = self.client.read_coils(0, count=num_outputs, device_id=self.slave)
            if result.isError():
                return None
            for i, (name, _addr) in enumerate(DIGITAL_OUTPUT_TAGS):
                record[f"out_{name}"] = bool(result.bits[i])
        except Exception:
            return None

        # ── Digital inputs (%I — 9 digital inputs) ───────────────────────
        try:
            num_inputs = len(DIGITAL_INPUT_TAGS)
            result = self.client.read_discrete_inputs(0, count=num_inputs, device_id=self.slave)
            if not result.isError():
                for i, (name, _addr) in enumerate(DIGITAL_INPUT_TAGS):
                    record[f"in_{name}"] = bool(result.bits[i])
        except Exception:
            pass

        # ── Setpoints (%MW0, %MW1) ───────────────────────────────────────
        try:
            result = self.client.read_holding_registers(0, count=2, device_id=self.slave)
            if not result.isError():
                for i, (name, _addr, _unit) in enumerate(SETPOINT_TAGS):
                    record[name] = result.registers[i]
        except Exception:
            pass

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

    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle("TM221CE16R Process Monitor — Mixing Tank", fontsize=14, fontweight="bold")
    fig.text(0.99, 0.97, "GREEN SHOE GARAGE", fontsize=9, color="#6c7086",
             fontfamily="monospace", fontweight="bold", ha="right", va="top", alpha=0.7)
    fig.patch.set_facecolor("#1e1e2e")

    COLORS = {
        "Temperature": "#ef4444",
        "Pressure":    "#3b82f6",
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

    ax_temp   = axes[0][0]
    ax_press  = axes[0][1]
    ax_dio    = axes[1][0]
    ax_alarms = axes[1][1]

    def update(_frame):
        if stop_event.is_set():
            plt.close(fig)
            return

        data = list(history)
        if not data:
            return

        times = [datetime.strptime(d["timestamp"], "%Y-%m-%d %H:%M:%S.%f") for d in data]

        for ax, tag_name, unit, ylim, sp_tag in [
            (ax_temp,  "Temperature", "°C",  (0, 110),  "Temp_Setpoint"),
            (ax_press, "Pressure",    "bar", (0, 12),   "Pressure_Setpoint"),
        ]:
            ax.clear()
            ax.set_facecolor(style["bg"])
            ax.grid(True, color=style["grid"], alpha=0.3, linewidth=0.5)
            values = [d.get(tag_name, 0) for d in data]
            color = COLORS[tag_name]
            ax.plot(times, values, color=color, linewidth=style["line"])
            ax.fill_between(times, values, alpha=0.08, color=color)
            ax.set_ylim(ylim)
            ax.set_title(f"{tag_name.replace('_', ' ')} ({unit})",
                         color=style["fg"], fontsize=11, pad=6)
            ax.tick_params(colors=style["fg"], labelsize=7)

            if data and sp_tag:
                sp = data[-1].get(sp_tag, None)
                if sp is not None and sp > 0:
                    ax.axhline(y=sp, color="#f87171", linestyle="--",
                               linewidth=1, alpha=0.6, label=f"SP {sp}")
                    ax.legend(loc="upper left", fontsize=7,
                              facecolor=style["bg"], edgecolor=style["grid"],
                              labelcolor=style["fg"])

            ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
            plt.setp(ax.xaxis.get_majorticklabels(), rotation=30, ha="right")

        # ── Digital I/O panel ────────────────────────────────────────────
        ax_dio.clear()
        ax_dio.set_facecolor(style["bg"])
        ax_dio.set_title("Digital I/O", color=style["fg"], fontsize=11, pad=6)

        all_dio = []
        for name, _addr in DIGITAL_OUTPUT_TAGS:
            all_dio.append((f"Q  {name.replace('_', ' ')}", f"out_{name}"))
        for name, _addr in DIGITAL_INPUT_TAGS:
            all_dio.append((f"I   {name.replace('_', ' ')}", f"in_{name}"))

        ax_dio.set_xlim(0, 1)
        ax_dio.set_ylim(-0.5, len(all_dio) - 0.5)
        ax_dio.set_yticks(range(len(all_dio)))
        ax_dio.set_yticklabels([label for label, _ in all_dio],
                               fontsize=8, color=style["fg"], fontfamily="monospace")
        ax_dio.set_xticks([])

        latest = data[-1] if data else {}
        for i, (label, key) in enumerate(all_dio):
            state = latest.get(key, False)
            color = "#22c55e" if state else "#6c7086"
            width = 0.5 if state else 0.15
            ax_dio.barh(i, width, left=0.2, height=0.45,
                        color=color, edgecolor="none", alpha=0.9)
            ax_dio.text(0.78, i, "ON" if state else "OFF",
                        va="center", ha="center", fontsize=8, fontweight="bold",
                        color="#1e1e2e" if state else style["fg"])

        # ── Alarm panel ──────────────────────────────────────────────────
        ax_alarms.clear()
        ax_alarms.set_facecolor(style["bg"])
        ax_alarms.set_title("Alarms", color=style["fg"], fontsize=11, pad=6)
        ax_alarms.set_xlim(0, 1)
        ax_alarms.set_ylim(-0.5, len(ALARM_BITS) - 0.5)
        ax_alarms.set_yticks(range(len(ALARM_BITS)))
        ax_alarms.set_yticklabels([a.replace("_", " ") for a in ALARM_BITS],
                                  fontsize=9, color=style["fg"])
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


# ── Console display ──────────────────────────────────────────────────────────

def print_console_record(record, sample_num):
    ts = record["timestamp"]
    temp  = record.get("Temperature", 0)
    press = record.get("Pressure", 0)

    outputs = " ".join(
        f"{name[:3]}={'ON' if record.get(f'out_{name}', False) else 'off'}"
        for name, _ in DIGITAL_OUTPUT_TAGS
    )

    inputs = " ".join(
        f"{name[:3]}={'ON' if record.get(f'in_{name}', False) else 'off'}"
        for name, _ in DIGITAL_INPUT_TAGS
    )

    alarms = record.get("active_alarms", [])
    alarm_str = ", ".join(alarms) if alarms else "None"

    print(
        f"[{ts}] #{sample_num:>5}  "
        f"T={temp:6.1f}°C  P={press:5.2f}bar  "
        f"| Q: {outputs} | I: {inputs} "
        f"| Alarms: {alarm_str}"
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
    parser.add_argument("--simulate", action="store_true",     help="Use simulated data (no PLC needed)")
    parser.add_argument("--logdir",   default="logs",          help="CSV log directory")
    parser.add_argument("--history",  type=int, default=300,   help="Max data points in chart")
    args = parser.parse_args()

    history = deque(maxlen=args.history)
    stop_event = threading.Event()

    def on_signal(_sig, _frame):
        print("\nShutting down...")
        stop_event.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # ── Select reader ────────────────────────────────────────────────────
    print("┌─────────────────────────────────────────────┐")
    print("│  GREEN SHOE GARAGE — TM221 Process Monitor  │")
    print("└─────────────────────────────────────────────┘")
    print()
    if args.simulate:
        reader = SimulatedPLCReader()
        reader.connect()
        print("Running in SIMULATION mode — no PLC connection required.")
        print(f"Polling every {args.interval}s.  Logging to ./{args.logdir}/")
        print("Simulating: mixing-tank fill → heat → hold cycle")
    else:
        reader = PLCReader(args.ip, port=args.port, slave=args.slave)
        print(f"Connecting to PLC at {args.ip}:{args.port} (slave {args.slave})...")
        if not reader.connect():
            sys.exit(f"ERROR: Cannot connect to {args.ip}:{args.port}")
        print(f"Connected.  Polling every {args.interval}s.  Logging to ./{args.logdir}/")
        print(f"Hardware: TM221CE16R — 2 analog inputs (10-bit), 9 DI, 7 DO (relay)")

    if args.duration:
        print(f"Will run for {args.duration} seconds.")
    print("Press Ctrl+C to stop.\n")

    logger = CSVLogger(directory=args.logdir)

    sample_count = 0
    start_time = time.time()
    comm_errors = 0

    def collect_loop():
        nonlocal sample_count, comm_errors
        while not stop_event.is_set():
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
                reader.disconnect()
                time.sleep(1)
                reader.connect()
                continue

            comm_errors = 0
            sample_count += 1
            logger.log(record)
            history.append(record)

            if args.headless:
                print_console_record(record, sample_count)

            stop_event.wait(args.interval)

    collector = threading.Thread(target=collect_loop, daemon=True)
    collector.start()

    if args.headless:
        collector.join()
    else:
        run_dashboard(history, stop_event, args.interval)
        stop_event.set()
        collector.join(timeout=5)

    reader.disconnect()
    logger.close()

    elapsed = time.time() - start_time
    print(f"\nSession complete: {sample_count} samples in {timedelta(seconds=int(elapsed))}")
    print(f"Logs saved in ./{args.logdir}/")


if __name__ == "__main__":
    main()
