#!/usr/bin/env python3
"""
TM221 PLC Finder
================
Scans the 10.10.0.1 – 10.10.255.255 range to locate a Modicon M221
by probing Modbus TCP (port 502).

Two-phase approach:
  Phase 1 — Fast TCP port scan (async, ~500 connections at a time)
            Finds any device with port 502 open.
  Phase 2 — Modbus identification read on each hit.
            Confirms it's actually an M221 by reading holding registers.

Install:  pip install pymodbus
Usage:    python find_plc.py
          python find_plc.py --start 10.10.0.1 --end 10.10.255.255
          python find_plc.py --timeout 0.3 --workers 1000
"""

import argparse
import asyncio
import ipaddress
import socket
import struct
import sys
import time
from concurrent.futures import ThreadPoolExecutor

# Optional: pymodbus for phase 2 verification
try:
    from pymodbus.client import ModbusTcpClient
    HAS_PYMODBUS = True
except ImportError:
    HAS_PYMODBUS = False


# ── Phase 1: Async TCP port scan ─────────────────────────────────────────────

async def check_port(ip: str, port: int, timeout: float, semaphore: asyncio.Semaphore):
    """Try to open a TCP connection. Returns ip if port is open, else None."""
    async with semaphore:
        try:
            _, writer = await asyncio.wait_for(
                asyncio.open_connection(ip, port),
                timeout=timeout,
            )
            writer.close()
            await writer.wait_closed()
            return ip
        except (asyncio.TimeoutError, ConnectionRefusedError, OSError):
            return None


async def scan_range(start_ip: str, end_ip: str, port: int, timeout: float, workers: int):
    """Scan an IP range for open ports. Returns list of IPs with port open."""
    start = int(ipaddress.IPv4Address(start_ip))
    end = int(ipaddress.IPv4Address(end_ip))
    total = end - start + 1

    print(f"Phase 1: Scanning {total:,} addresses ({start_ip} → {end_ip}) on port {port}")
    print(f"         Timeout: {timeout}s | Concurrent workers: {workers}")
    print()

    semaphore = asyncio.Semaphore(workers)
    tasks = []
    for ip_int in range(start, end + 1):
        ip_str = str(ipaddress.IPv4Address(ip_int))
        tasks.append(check_port(ip_str, port, timeout, semaphore))

    hits = []
    done = 0
    scan_start = time.time()

    # Process in batches for progress reporting
    batch_size = 5000
    for i in range(0, len(tasks), batch_size):
        batch = tasks[i : i + batch_size]
        results = await asyncio.gather(*batch)
        for result in results:
            if result is not None:
                hits.append(result)
                print(f"  ✓ FOUND open port 502 at {result}")
        done += len(batch)
        elapsed = time.time() - scan_start
        rate = done / elapsed if elapsed > 0 else 0
        pct = done / total * 100
        print(f"  [{pct:5.1f}%] {done:,}/{total:,} scanned  |  {rate:.0f} addr/s  |  {len(hits)} hit(s)", end="\r")

    elapsed = time.time() - scan_start
    print(f"\n\nPhase 1 complete: {total:,} addresses in {elapsed:.1f}s ({total/elapsed:.0f} addr/s)")
    print(f"  Hosts with port 502 open: {len(hits)}")
    return hits


# ── Phase 2: Modbus identification ───────────────────────────────────────────

def identify_plc(ip: str, timeout: float = 2.0) -> dict:
    """Try to read Modbus registers and identify the device."""
    info = {"ip": ip, "modbus": False, "identity": "Unknown"}

    # Method 1: Raw Modbus TCP — read holding registers (FC 3)
    # This works even without pymodbus
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((ip, 502))

        # Modbus TCP: Read Holding Registers, address 0, quantity 5
        # Transaction ID | Protocol ID | Length | Unit ID | FC | Start Addr | Quantity
        request = struct.pack(">HHHBBHH", 0x0001, 0x0000, 0x0006, 0x01, 0x03, 0x0000, 0x0005)
        sock.sendall(request)
        response = sock.recv(256)
        sock.close()

        if len(response) >= 9:
            info["modbus"] = True
            # Parse the response
            unit_id = response[6]
            fc = response[7]
            if fc == 0x03:  # Successful read
                byte_count = response[8]
                num_regs = byte_count // 2
                registers = []
                for i in range(num_regs):
                    reg_val = struct.unpack(">H", response[9 + i*2 : 11 + i*2])[0]
                    registers.append(reg_val)
                info["registers_MW0_4"] = registers
                info["identity"] = "Modbus TCP device (holding registers readable)"
            elif fc == 0x83:  # Exception
                info["identity"] = f"Modbus device (exception code: {response[8]})"
    except Exception as e:
        info["error"] = str(e)
        if info.get("modbus"):
            pass  # We already got some info
        try:
            sock.close()
        except Exception:
            pass

    # Method 2: Modbus Device Identification (FC 43/14) — MEI Read Device ID
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((ip, 502))

        # FC 43 (0x2B), MEI type 14 (0x0E), Read Device ID code 1, Object ID 0
        request = struct.pack(">HHHBBBBB", 0x0002, 0x0000, 0x0005, 0x01, 0x2B, 0x0E, 0x01, 0x00)
        sock.sendall(request)
        response = sock.recv(512)
        sock.close()

        if len(response) > 9 and response[7] == 0x2B:
            info["modbus"] = True
            # Parse device identification objects
            # Objects start after the header
            try:
                idx = 15  # Approximate start of object data
                objects = {}
                obj_names = {0: "VendorName", 1: "ProductCode", 2: "MajorMinorRevision"}
                num_objects = response[14] if len(response) > 14 else 0
                for _ in range(num_objects):
                    if idx + 2 > len(response):
                        break
                    obj_id = response[idx]
                    obj_len = response[idx + 1]
                    obj_val = response[idx + 2 : idx + 2 + obj_len].decode("ascii", errors="replace")
                    label = obj_names.get(obj_id, f"Object_{obj_id}")
                    objects[label] = obj_val
                    idx += 2 + obj_len
                if objects:
                    info["device_id"] = objects
                    info["identity"] = " | ".join(f"{k}: {v}" for k, v in objects.items())
            except Exception:
                pass
    except Exception:
        try:
            sock.close()
        except Exception:
            pass

    # Method 3: If pymodbus is available, try a clean read for extra details
    if HAS_PYMODBUS and info.get("modbus"):
        try:
            client = ModbusTcpClient(ip, port=502, timeout=timeout)
            if client.connect():
                # Read a few coils to see if digital I/O is accessible
                coil_result = client.read_coils(0, count=8, slave=1)
                if not coil_result.isError():
                    info["coils_0_7"] = coil_result.bits[:8]

                # Read discrete inputs
                di_result = client.read_discrete_inputs(0, count=8, slave=1)
                if not di_result.isError():
                    info["inputs_0_7"] = di_result.bits[:8]

                client.close()
        except Exception:
            pass

    return info


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Find Modicon M221 PLC on the network")
    parser.add_argument("--start",   default="10.10.0.1",   help="Start IP (default: 10.10.0.1)")
    parser.add_argument("--end",     default="10.10.255.255", help="End IP (default: 10.10.255.255)")
    parser.add_argument("--port",    type=int, default=502,  help="Port to scan (default: 502)")
    parser.add_argument("--timeout", type=float, default=0.5, help="TCP timeout per host (default: 0.5s)")
    parser.add_argument("--workers", type=int, default=500,  help="Concurrent connections (default: 500)")
    args = parser.parse_args()

    print()
    print("╔══════════════════════════════════════════════════╗")
    print("║          TM221 PLC Finder                       ║")
    print("║          Scanning for Modbus TCP devices         ║")
    print("╚══════════════════════════════════════════════════╝")
    print()

    # Preflight: make sure we're on the right subnet
    print("Preflight: Checking local network interfaces...")
    try:
        import netifaces
        for iface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(iface).get(netifaces.AF_INET, [])
            for addr in addrs:
                if addr["addr"].startswith("10.10."):
                    print(f"  ✓ Interface {iface} has IP {addr['addr']} — good!")
    except ImportError:
        # Fallback: just try to get our IP
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("10.10.0.1", 502))
            local_ip = s.getsockname()[0]
            s.close()
            if local_ip.startswith("10."):
                print(f"  ✓ Local IP appears to be {local_ip}")
            else:
                print(f"  ⚠ Local IP is {local_ip} — you may not be on the 10.10.x.x subnet!")
                print(f"    Set your Ethernet adapter to a static IP like 10.10.0.50 / 255.0.0.0")
        except Exception:
            print("  ⚠ Could not determine local IP. Make sure you're on the 10.10.x.x subnet.")
    print()

    # Phase 1: Port scan
    hits = asyncio.run(scan_range(args.start, args.end, args.port, args.timeout, args.workers))

    if not hits:
        print("\n  No devices found with port 502 open.")
        print("\n  Troubleshooting:")
        print("    1. Is the PLC powered on? Check the PWR LED.")
        print("    2. Is the Ethernet cable connected? Check for a link light on the PLC.")
        print("    3. Is your PC on the 10.10.x.x subnet with mask 255.0.0.0?")
        print("    4. Try a shorter timeout: python find_plc.py --timeout 1.0 --workers 200")
        print("    5. Check the MAC label on the PLC and calculate the IP manually:")
        print("       MAC 00:80:F4:xx:AA:BB → IP is 10.10.[AA decimal].[BB decimal]")
        return

    # Phase 2: Identify each hit
    print(f"\nPhase 2: Identifying {len(hits)} device(s)...\n")
    print("=" * 72)

    for ip in sorted(hits, key=lambda x: int(ipaddress.IPv4Address(x))):
        info = identify_plc(ip, timeout=2.0)
        print(f"\n  IP Address:  {ip}")
        print(f"  Modbus TCP:  {'Yes' if info.get('modbus') else 'No'}")
        print(f"  Identity:    {info.get('identity', 'Unknown')}")

        if "device_id" in info:
            print(f"  Device ID:")
            for k, v in info["device_id"].items():
                print(f"    {k}: {v}")

        if "registers_MW0_4" in info:
            regs = info["registers_MW0_4"]
            print(f"  MW0–MW4:     {regs}")

        if "coils_0_7" in info:
            coil_str = " ".join("ON" if b else "OFF" for b in info["coils_0_7"])
            print(f"  Outputs:     {coil_str}")

        if "inputs_0_7" in info:
            inp_str = " ".join("ON" if b else "OFF" for b in info["inputs_0_7"])
            print(f"  Inputs:      {inp_str}")

        if "error" in info:
            print(f"  Note:        {info['error']}")

    print("\n" + "=" * 72)
    print(f"\n  Scan complete. Found {len(hits)} device(s) with Modbus TCP on port 502.")
    print(f"  Update your process_monitor.py with:  --ip {hits[0]}")
    print()


if __name__ == "__main__":
    main()
