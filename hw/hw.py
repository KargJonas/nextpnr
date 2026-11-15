#!/usr/bin/env python3
"""Flash MCU-format bitstreams to the Shrike and measure the PMOD pins with the LA.

    hw.py flash <bitstream_mcu.bin>
    hw.py capture [--ms 2000] [--rate 1000000] [-o out.vcd]
    hw.py ab <bitstream_mcu.bin>...     flash + capture + report each in turn

The Shrike's RP2040 runs the HITL firmware (PING/FLASH protocol). The logic
analyzer is an fx2lafw clone whose D0..D7 are FPGA GPIO8..15.
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

import serial

MCU_SIZE = 46408


def flash_once(path, port="/dev/ttyACM0"):
    data = Path(path).read_bytes()
    if len(data) != MCU_SIZE:
        sys.exit(f"{path}: {len(data)} bytes, expected the {MCU_SIZE}-byte MCU format")
    with serial.Serial(port, 115200, timeout=5) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        for _ in range(3):
            s.write(b"PING\n")
            if s.readline().strip() == b"PONG":
                break
            time.sleep(1)
        else:
            sys.exit("firmware does not answer PING")
        s.write(f"FLASH {len(data)}\n".encode())
        while True:
            line = s.readline().decode(errors="replace").strip()
            if line == "READY":
                break
            if not line:
                sys.exit("no READY from firmware")
        # In pieces: one write of the whole image can stall the firmware's USB serial.
        for i in range(0, len(data), 1024):
            s.write(data[i:i + 1024]); s.flush(); time.sleep(0.002)
        config = None
        deadline = time.time() + 60
        while time.time() < deadline:
            line = s.readline().decode(errors="replace").strip()
            if line.startswith("CONFIG="):
                config = line
            if line == "OK":
                return config
            if line.startswith("ERR"):
                sys.exit(f"firmware: {line}")
        sys.exit("no OK from firmware")


def flash(path):
    """flash_once, again after a USB reset when the firmware's serial link stalls (it does, now and then)."""
    for attempt in range(4):
        try:
            return flash_once(path)
        except SystemExit as e:
            if attempt == 3 or "firmware" not in str(e.code) or str(e.code).startswith("firmware: ERR"):
                raise
            subprocess.run(["usbreset", "2e8a:000a"], capture_output=True)
            time.sleep(3)


def capture(out, ms=2000, rate=1_000_000):
    subprocess.run(["sigrok-cli", "-d", "fx2lafw", "--config", f"samplerate={rate}",
                    "--time", str(ms), "-O", "vcd", "-o", str(out)],
                   check=True, capture_output=True)
    return transitions(out)


def transitions(vcd):
    """Per-channel transition counts and final values from a sigrok VCD."""
    ids, last, count = {}, {}, {}
    for line in Path(vcd).read_text().splitlines():
        if line.startswith("$var"):
            f = line.split()
            ids[f[3]] = f[4]
        elif line.startswith("#"):
            for tok in line.split()[1:]:
                name = ids.get(tok[1:])
                if name is None or tok[0] not in "01":
                    continue
                if name in last and last[name] != tok[0]:
                    count[name] = count.get(name, 0) + 1
                last[name] = tok[0]
    return {n: (count.get(n, 0), last.get(n)) for n in sorted(ids.values())}


def report(label, res, ms):
    cells = []
    for n, (c, v) in res.items():
        cells.append(f"{n}={c/2/(ms/1000)/1000:.1f}kHz" if c else f"{n}={v}")
    alive = any(c for c, _ in res.values())
    print(f"{'ALIVE' if alive else 'static':6} {label}: " + " ".join(cells), flush=True)


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("flash"); f.add_argument("bin")
    c = sub.add_parser("capture"); c.add_argument("-o", default="capture.vcd")
    c.add_argument("--ms", type=int, default=2000); c.add_argument("--rate", type=int, default=1_000_000)
    a = sub.add_parser("ab"); a.add_argument("bins", nargs="+")
    a.add_argument("--ms", type=int, default=2000); a.add_argument("--rate", type=int, default=1_000_000)
    args = p.parse_args()
    if args.cmd == "flash":
        print(flash(args.bin))
    elif args.cmd == "capture":
        report(args.o, capture(args.o, args.ms, args.rate), args.ms)
    else:
        for b in args.bins:
            cfg = flash(b)
            time.sleep(0.5)
            vcd = Path(b).with_suffix(".vcd")
            report(f"{b} ({cfg})", capture(vcd, args.ms, args.rate), args.ms)


if __name__ == "__main__":
    main()
