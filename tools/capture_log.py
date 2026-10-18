#!/usr/bin/env python3
"""Capture a CSV log from the Pico chirp receiver over its USB serial port.

Sends any setup keys, turns logging on with 'l', records for a while, turns
logging off again and writes only the CSV lines (including '#' comments) to
the output file. Uses only the Python standard library (Linux/macOS).

Examples:
  # Chirp through the air with pilot tracking (4 = chirp, t t = off->decision->pilot)
  python3 tools/capture_log.py --keys 4tt --seconds 30 pilot.csv

  # Log whatever the board is currently doing
  python3 tools/capture_log.py --seconds 10 now.csv

  # Sweep all 64 symbols, 1 s each, tracking off (characterise each symbol)
  python3 tools/capture_log.py --keys 4 --sweep 1 sweep.csv
"""

import argparse
import os
import re
import select
import sys
import termios
import time
import tty

ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def is_log_line(line):
    return line.startswith("#") or line.startswith("frame,") or line[:1].isdigit()


def send(fd, keys, gap=0.1):
    for key in keys:
        os.write(fd, key.encode())
        time.sleep(gap)


def capture(fd, seconds, sweep_period=None, symbols=0):
    start = time.monotonic()
    deadline = start + seconds
    next_step = start + sweep_period if sweep_period else None
    steps = 0
    pending = b""
    lines = []
    while time.monotonic() < deadline:
        if next_step is not None and time.monotonic() >= next_step and steps < symbols - 1:
            send(fd, "p", gap=0)  # Next transmitted symbol
            steps += 1
            next_step += sweep_period
        ready, _, _ = select.select([fd], [], [], 0.1)
        if not ready:
            continue
        pending += os.read(fd, 65536)
        *complete, pending = pending.split(b"\n")
        for raw in complete:
            line = ANSI_ESCAPE.sub("", raw.decode(errors="replace")).rstrip("\r")
            if is_log_line(line):
                lines.append(line)
    return lines


def summarise(lines):
    header = next((l for l in lines if l.startswith("frame,")), None)
    data = [l for l in lines if l[:1].isdigit()]
    if header is None or not data:
        return "no log data captured (is logging working, and was it off before starting?)"

    columns = header.split(",")
    frames = [int(l.split(",", 1)[0]) for l in data]
    gaps = sum(1 for a, b in zip(frames, frames[1:]) if b != a + 1)
    last = dict(zip(columns, data[-1].split(",")))
    return (f"{len(data)} frames ({frames[0]}..{frames[-1]}), {gaps} gaps in frame numbers, "
            f"rx_missed={last.get('rx_missed')} tx_missed={last.get('tx_missed')} "
            f"dropped={last.get('dropped')}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("output", help="CSV file to write")
    parser.add_argument("--port", default="/dev/ttyACM0", help="serial port (default /dev/ttyACM0)")
    parser.add_argument("--keys", default="", help="keys to send before logging, e.g. '4tt'")
    parser.add_argument("--settle", type=float, default=2.0, help="seconds to wait after the keys (default 2)")
    parser.add_argument("--seconds", type=float, default=20.0, help="seconds to log (default 20)")
    parser.add_argument("--sweep", type=float, metavar="SECONDS",
                        help="step the transmitted symbol with 'p' every SECONDS through all symbols; "
                             "overrides --seconds")
    parser.add_argument("--symbols", type=int, default=64, help="number of symbols for --sweep (default 64)")
    args = parser.parse_args()
    if args.sweep:
        args.seconds = args.sweep * args.symbols

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY)
    saved = termios.tcgetattr(fd)
    logging_on = False
    try:
        tty.setraw(fd)
        send(fd, args.keys)
        time.sleep(args.settle)
        termios.tcflush(fd, termios.TCIFLUSH)  # Discard the terminal display output
        send(fd, "l")
        logging_on = True
        lines = capture(fd, args.seconds, args.sweep, args.symbols)
    finally:
        if logging_on:
            send(fd, "l")
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        os.close(fd)

    with open(args.output, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"{args.output}: {summarise(lines)}", file=sys.stderr)


if __name__ == "__main__":
    main()
