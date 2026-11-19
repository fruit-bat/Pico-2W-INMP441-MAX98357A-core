#!/usr/bin/env python3
"""Link soak test: send many packets and record how each one ended.

    # Two boards: A sends to B, then (--both) B to A, for every condition
    python3 tools/chirp_soak.py run --port /dev/ttyACM0 --port /dev/ttyACM1 --both soak.csv
    # One board hearing itself (rx.during_tx=on), or the simulator
    python3 tools/chirp_soak.py run --port /dev/ttyACM0 self.csv
    python3 tools/chirp_soak.py run --sim tests/build/protocol_sim --speed 20 sim.csv
    # The summary table again, from one or more files
    python3 tools/chirp_soak.py summary soak.csv older.csv

The conditions are every combination of --spacing, --volume and --length
(default 2,4 x -28,-34,-40 x 16), --count packets each, in each direction.
Before each condition's packets the receiver is watched for --idle seconds
with nothing sent, to count false locks in quiet; locks between packets
are counted too, separately.

Packets go one at a time: the next is sent once the receiver has
concluded (or a time limit ran out), so each outcome belongs to one packet.
Payloads are pseudo-random from the run's seed and the packet number, so a
packet with a bad CRC can still be compared bit by bit with what was sent.

Outcomes (the receiver's conclusion, see protocol.md):
    ok, crc_bad          rx.packet crc=ok / crc=bad
    header_failed        locked and trained, header check failed
    train_failed, tone   locked, then gave up
    lost:<reason>        given up part-way
    no_lock              no rx.locked at all
    unconcluded          locked, but no conclusion within the time limit
    tx_abandoned         the sender cut the packet short

The CSV has one row per packet, plus one "idle" row per idle period. The
settings each board had are put back afterwards (also after Ctrl-C).
"""

import argparse
import csv
import math
import os
import random
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from chirp_host import Board  # noqa: E402

FRAME_S = 1024 / 44100
HEADER_BITS = 16
CRC_BITS = 16
PREAMBLE_FRAMES = 16
CONCLUSIONS = ("rx.packet", "rx.header_failed", "rx.train_failed", "rx.tone", "rx.lost")

COLUMNS = ["time", "seed", "dir", "spacing", "volume", "length", "n", "outcome",
           "locks", "false_before", "crc", "rx_len", "min_margin", "weakest", "weak",
           "align", "fraction", "moved", "b_start", "coherence", "strongest", "earliest", "rx_delay",
           "byte_errors", "bit_errors", "symbol_errors", "error_symbols",
           "tx_frames", "idle_s", "lost_events", "bad_lines"]


def bits_per_symbol(spacing):
    return int(math.log2(64 // spacing))


def data_symbols(length, spacing):
    b = bits_per_symbol(spacing)
    return -(-(HEADER_BITS + 8 * length + CRC_BITS) // b)


def payload_for(seed, n, length):
    return random.Random("%d:%d" % (seed, n)).randbytes(length)


def compare(sent, got, spacing):
    """(byte errors, bit errors, data symbols with an error) over the payload.

    Data symbol numbers count from the first header symbol. The CRC bytes
    aren't reported by the board, so errors there can't be seen.
    """
    b = bits_per_symbol(spacing)
    byte_errors = bit_errors = 0
    symbols = set()
    for i, (s, g) in enumerate(zip(sent, got)):
        diff = s ^ g
        if not diff:
            continue
        byte_errors += 1
        for bit in range(8):
            if diff & (0x80 >> bit):
                bit_errors += 1
                symbols.add((HEADER_BITS + 8 * i + bit) // b)
    return byte_errors, bit_errors, sorted(symbols)


class Watcher:
    """Collects tx.* events from the sender and rx.* events from the receiver."""

    def __init__(self, tx, rx):
        self.tx, self.rx = tx, rx
        self._lock = threading.Condition()
        self._tx_events = []
        self._rx_events = []
        boards = {id(tx): tx, id(rx): rx}
        for board in boards.values():
            board.queue_streams = False
            board.on_event = self._make_handler(board)

    def _make_handler(self, board):
        def handler(e):
            with self._lock:
                if board is self.tx and e.name.startswith("tx."):
                    self._tx_events.append(e)
                if board is self.rx and e.name.startswith("rx."):
                    self._rx_events.append(e)
                self._lock.notify_all()
        return handler

    def take_rx(self):
        with self._lock:
            out, self._rx_events = self._rx_events, []
        return out

    def take_tx(self):
        with self._lock:
            out, self._tx_events = self._tx_events, []
        return out

    def wait(self, until, timeout):
        """Waits until until() is true (called with the lock held) or timeout seconds."""
        deadline = time.monotonic() + timeout
        with self._lock:
            while not until():
                left = deadline - time.monotonic()
                if left <= 0:
                    return False
                self._lock.wait(left)
            return True

    def tx_done(self, pid):
        return any(e.name == "tx.done" and e["id"] == pid for e in self._tx_events)

    def rx_open(self):
        """True while the receiver has locked without concluding."""
        open_ = False
        for e in self._rx_events:
            if e.name == "rx.locked":
                open_ = True
            elif e.name in CONCLUSIONS:
                open_ = False
        return open_


def count_locks(events):
    return sum(1 for e in events if e.name == "rx.locked")


def judge(events):
    """The packet's outcome from the receiver's events in its window, plus
    the rx.locked and rx.trained events that led to it."""
    locked = trained = None
    outcome, packet = None, None
    for e in events:
        if e.name == "rx.locked":
            locked, trained = e, None
        elif e.name == "rx.trained":
            trained = e
        elif e.name == "rx.packet" and packet is None:
            packet = e
            outcome = "ok" if e["crc"] == "ok" else "crc_bad"
            break
        elif e.name in CONCLUSIONS:
            outcome = e.name[3:] if e.name != "rx.lost" else "lost:%s" % e.raw("reason")
    if outcome is None:
        if locked is None:
            outcome = "no_lock"
        else:
            outcome = "unconcluded"
    return outcome, packet, locked, trained


class Soak:
    def __init__(self, boards, labels, args, out):
        self.boards = boards
        self.labels = labels
        self.args = args
        self.speed = args.speed if args.sim else 1.0
        self.seed = args.seed if args.seed is not None else int(time.time())
        self.writer = csv.DictWriter(out, COLUMNS)
        self.out = out
        self.rows = []
        self.n = 0
        self.stopped = False

    def _seconds(self, s):
        return max(s / self.speed, 0.05)

    def _write(self, row):
        row.setdefault("time", time.strftime("%Y-%m-%dT%H:%M:%S"))
        row.setdefault("seed", self.seed)
        self.writer.writerow(row)
        self.out.flush()
        self.rows.append({k: str(v) for k, v in row.items()})

    def _health(self):
        return (sum(b.lost_events for b in set(self.boards)), sum(b.bad_lines for b in set(self.boards)))

    def directions(self):
        a = self.boards[0]
        if len(self.boards) == 1:
            return [(a, a, "%s>%s" % (self.labels[0], self.labels[0]))]
        b = self.boards[1]
        out = [(a, b, "%s>%s" % (self.labels[0], self.labels[1]))]
        if self.args.both:
            out.append((b, a, "%s>%s" % (self.labels[1], self.labels[0])))
        return out

    def run(self):
        conditions = [(s, v, l) for s in self.args.spacing for v in self.args.volume for l in self.args.length]
        for spacing, volume, length in conditions:
            for tx, rx, label in self.directions():
                self.condition(tx, rx, label, spacing, volume, length)

    def condition(self, tx, rx, label, spacing, volume, length):
        for board in set((tx, rx)):
            if board.settings.get("link.spacing") != spacing:
                board.set({"link.spacing": spacing})
        tx.set({"tx.volume": volume})
        if tx is rx:
            rx.set({"rx.during_tx": "on"})
        watcher = Watcher(tx, rx)
        time.sleep(self._seconds(0.5))  # Let any spacing change settle
        watcher.take_rx()
        watcher.take_tx()
        common = {"dir": label, "spacing": spacing, "volume": volume, "length": length}

        # Quiet: false locks with nothing sent
        if self.args.idle > 0:
            health = self._health()
            start = time.monotonic()
            time.sleep(self._seconds(self.args.idle))
            events = watcher.take_rx()
            lost, bad = (x - y for x, y in zip(self._health(), health))
            self._write(dict(common, n="", outcome="idle", locks=count_locks(events),
                             idle_s="%.1f" % ((time.monotonic() - start) * self.speed),
                             lost_events=lost, bad_lines=bad))
            print("%s spacing %d %+d dB: idle %.0f s, %d false locks"
                  % (label, spacing, volume, self.args.idle, count_locks(events)), flush=True)

        frames = PREAMBLE_FRAMES + data_symbols(length, spacing)
        tx_timeout = self._seconds(frames * FRAME_S * 1.5 + 3.0)
        for _ in range(self.args.count):
            self.packet(tx, rx, watcher, common, length, spacing, frames, tx_timeout)
            time.sleep(self._seconds(self.args.gap))

    def packet(self, tx, rx, watcher, common, length, spacing, frames, tx_timeout):
        self.n += 1
        n = self.n
        payload = payload_for(self.seed, n, length)
        health = self._health()
        before = watcher.take_rx()  # Since the last packet's window: false locks
        watcher.take_tx()
        pid = tx.send(payload)
        sent = watcher.wait(lambda: watcher.tx_done(pid), tx_timeout)
        # Then the receiver: until it concludes, or a short while if it never locked
        watcher.wait(lambda: not watcher.rx_open(), self._seconds(3.0))
        time.sleep(self._seconds(0.3))  # Late events (conclusions come about with tx.done)
        watcher.wait(lambda: not watcher.rx_open(), self._seconds(3.0))
        events = watcher.take_rx()
        tx_events = watcher.take_tx()

        outcome, p, locked, trained = judge(events)
        done = next((e for e in tx_events if e.name == "tx.done" and e["id"] == pid), None)
        start = next((e for e in tx_events if e.name == "tx.start" and e["id"] == pid), None)
        if not sent or done is None:
            outcome = "tx_timeout"
        elif done.raw("result") != "sent":
            outcome = "tx_abandoned"

        row = dict(common, n=n, outcome=outcome, locks=count_locks(events), false_before=count_locks(before))
        if start is not None and done is not None:
            row["tx_frames"] = done["frame"] - start["frame"]
        if locked is not None:
            row.update(align=locked.raw("align"), fraction=locked.raw("fraction"), moved=locked.raw("moved"))
        if trained is not None:
            row.update(b_start=trained.raw("b_start"), coherence=trained.raw("coherence"),
                       strongest=trained.raw("strongest"), earliest=trained.raw("earliest"))
        if p is not None:
            row.update(crc=p.raw("crc"), rx_len=p.raw("len"), min_margin=p.raw("min_margin"),
                       weakest=p.raw("weakest"), weak=p.raw("weak"))
            got = p.payload()
            if len(got) == length:
                byte_errors, bit_errors, symbols = compare(payload, got, spacing)
                row.update(byte_errors=byte_errors, bit_errors=bit_errors, symbol_errors=len(symbols),
                           error_symbols=";".join(str(s) for s in symbols))
        row["rx_delay"] = rx.get("rx.delay")["rx.delay"]
        lost, bad = (x - y for x, y in zip(self._health(), health))
        row.update(lost_events=lost, bad_lines=bad)
        self._write(row)

        detail = ""
        if p is not None:
            detail = " margin %s" % p.raw("min_margin")
            if row.get("symbol_errors"):
                detail += ", %s symbol errors" % row["symbol_errors"]
        if lost or bad:
            detail += "  [events lost %d, bad lines %d: don't trust this row]" % (lost, bad)
        print("%s spacing %d %+d dB %d bytes #%d: %s%s"
              % (common["dir"], spacing, common["volume"], length, n, outcome, detail), flush=True)


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
def _percentile(values, p):
    if not values:
        return None
    values = sorted(values)
    k = (len(values) - 1) * p / 100
    lo, hi = math.floor(k), math.ceil(k)
    return values[lo] + (values[hi] - values[lo]) * (k - lo)


def summary(rows, file=sys.stdout):
    groups = {}
    for r in rows:
        key = (r["dir"], int(r["spacing"]), int(r["volume"]), int(r["length"]))
        groups.setdefault(key, []).append(r)
    head = ("dir", "spc", "vol", "len", "n", "ok", "crc_bad", "hdr", "train", "lost", "no_lock", "other",
            "false_idle/min", "false_btw", "margin p10/p50", "sym_err/pkt", "untrusted")
    widths = (5, 3, 4, 4, 4, 4, 7, 4, 5, 4, 7, 5, 14, 9, 14, 11, 9)
    print("  ".join(h.rjust(w) for h, w in zip(head, widths)), file=file)
    for key in sorted(groups, key=lambda k: (k[1], -k[2], k[3], k[0])):
        rs = groups[key]
        packets = [r for r in rs if r["outcome"] != "idle"]
        idle = [r for r in rs if r["outcome"] == "idle"]
        out = {}
        for r in packets:
            o = r["outcome"]
            o = "lost" if o.startswith("lost:") else o
            out[o] = out.get(o, 0) + 1
        named = ("ok", "crc_bad", "header_failed", "train_failed", "lost", "no_lock")
        other = sum(v for k, v in out.items() if k not in named)
        idle_s = sum(float(r["idle_s"] or 0) for r in idle)
        idle_locks = sum(int(r["locks"] or 0) for r in idle)
        margins = [float(r["min_margin"]) for r in packets if r["min_margin"]]
        p10, p50 = _percentile(margins, 10), _percentile(margins, 50)
        sym = [int(r["symbol_errors"]) for r in packets if r["symbol_errors"] != ""]
        untrusted = sum(1 for r in rs if int(r["lost_events"] or 0) or int(r["bad_lines"] or 0))
        cells = (key[0], key[1], key[2], key[3], len(packets), out.get("ok", 0), out.get("crc_bad", 0),
                 out.get("header_failed", 0), out.get("train_failed", 0), out.get("lost", 0),
                 out.get("no_lock", 0), other,
                 "%.2f" % (idle_locks * 60 / idle_s) if idle_s else "-",
                 sum(int(r["false_before"] or 0) for r in packets),
                 "%.2f / %.2f" % (p10, p50) if margins else "-",
                 "%.2f" % (sum(sym) / len(sym)) if sym else "-",
                 untrusted)
        print("  ".join(str(c).rjust(w) for c, w in zip(cells, widths)), file=file)


def read_rows(paths):
    rows = []
    for path in paths:
        with open(path) as f:
            rows.extend(csv.DictReader(line for line in f if line.strip() and not line.startswith("#")))
    return rows


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------
def _list(kind):
    return lambda text: [kind(x) for x in text.split(",")]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="run the soak test")
    where = r.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", action="append", help="serial port; once for one board hearing itself, "
                                                       "twice for the first sending to the second")
    where.add_argument("--sim", help="path to tests/build/protocol_sim (hears itself)")
    r.add_argument("--speed", type=float, default=1.0, help="simulator clock speed")
    r.add_argument("--both", action="store_true", help="two boards: each direction in turn")
    r.add_argument("--spacing", type=_list(int), default=[2, 4], help="bins per symbol (default 2,4)")
    r.add_argument("--volume", type=_list(int), default=[-28, -34, -40], help="tx.volume (default -28,-34,-40)")
    r.add_argument("--length", type=_list(int), default=[16], help="payload bytes (default 16)")
    r.add_argument("--count", type=int, default=50, help="packets per condition and direction (default 50)")
    r.add_argument("--idle", type=float, default=30.0, help="quiet seconds before each condition (default 30)")
    r.add_argument("--gap", type=float, default=0.5, help="seconds between packets (default 0.5)")
    r.add_argument("--seed", type=int, help="payload seed (default: the time)")
    r.add_argument("--append", action="store_true", help="add to the CSV instead of replacing it")
    r.add_argument("output", help="CSV file, one row per packet")
    s = sub.add_parser("summary", help="print the summary table from soak CSV files")
    s.add_argument("files", nargs="+")
    args = ap.parse_args()

    if args.cmd == "summary":
        summary(read_rows(args.files))
        return

    if args.port and len(args.port) > 2:
        ap.error("at most two --port")
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    if args.sim:
        boards = [Board.sim(args.sim, speed=args.speed)]
    else:
        boards = [Board.serial(p) for p in args.port]
    labels = ["A", "B"][:len(boards)]
    restore = []
    try:
        hellos = [b.hello() for b in boards]
        for b in boards:
            restore.append((b, b.get("link.spacing", "tx.volume", "rx.during_tx")))
        new = not (args.append and os.path.exists(args.output) and os.path.getsize(args.output) > 0)
        with open(args.output, "w" if new else "a", newline="") as out:
            soak = Soak(boards, labels, args, out)
            if new:
                out.write("# chirp soak v1\n")
            for label, h, b in zip(labels, hellos, boards):
                out.write("# board %s id=%s version=%s port=%s rx.align=%s rx.tracking=%s\n"
                          % (label, h.get("id"), h.get("version"), args.sim or args.port[labels.index(label)],
                             *[b.get(k)[k] for k in ("rx.align", "rx.tracking")]))
            out.write("# run start=%s seed=%d\n" % (time.strftime("%Y-%m-%dT%H:%M:%S"), soak.seed))
            if new:
                soak.writer.writeheader()
            try:
                soak.run()
            except KeyboardInterrupt:
                print("\nstopped", flush=True)
            out.write("# run end=%s\n" % time.strftime("%Y-%m-%dT%H:%M:%S"))
    finally:
        for b, settings in restore:
            try:
                b.set(settings)
            except Exception as e:  # Still restore the others, and close
                print("could not restore settings: %s" % e, file=sys.stderr)
        for b in boards:
            b.close()
    print()
    summary(read_rows([args.output]))
    lost = sum(b.lost_events for b in boards)
    bad = sum(b.bad_lines for b in boards)
    print("\nevents lost %d, bad lines %d" % (lost, bad))


if __name__ == "__main__":
    main()
