#!/usr/bin/env python3
"""Host side of the chirp board protocol (see protocol.md).

A small library and a console for typing at a board. Standard library
only (Linux/macOS).

Library:

    from chirp_host import Board
    with Board.serial("/dev/ttyACM0") as b:       # or Board.sim("tests/build/protocol_sim")
        print(b.hello())
        b.set({"tx.volume": -31})
        print(b.get("tx.volume", "count.*"))
        packet_id = b.send(b"hello")
        b.wait_event("tx.done", lambda e: e["id"] == packet_id, timeout=20)
        b.sub("frame", every=4, fields="delay,rx_symbol")
        for e in b.events(timeout=1):
            print(e)

Every line both ways carries a CRC-16 check ("XXXX " in front); the
library adds it, checks it and strips it, so callers see plain lines.
Replies are matched to commands by tag; events arrive at any time and wait
in a queue until read. Event sequence numbers are watched: `lost_events`
counts gaps, and `restarts` counts "* hello seq=0".

Console:

    python3 tools/chirp_host.py console --port /dev/ttyACM0
    python3 tools/chirp_host.py console --sim tests/build/protocol_sim

Type commands without a tag (e.g. "get tx.volume"); the console adds a tag
and the check. Replies and events are printed as they come; stream lines
are hidden unless --streams is given. A line starting with '!' is sent as
typed (with its check, but no tag added), for trying odd lines.
"""

import argparse
import collections
import itertools
import os
import select
import subprocess
import sys
import termios
import threading
import time
import tty

STREAMS = ("frame", "folded", "band", "shape", "spectrum")


# ---------------------------------------------------------------------------
# Lines
# ---------------------------------------------------------------------------
def _crc_table():
    table = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if c & 0x8000 else (c << 1)
        table.append(c & 0xFFFF)
    return table


_CRC_TABLE = _crc_table()


def crc16(text):
    """CRC-16/CCITT (poly 0x1021, init 0xFFFF) of a str or bytes."""
    data = text.encode("ascii", "replace") if isinstance(text, str) else text
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


def encode(line):
    """A plain line -> the bytes to send, check in front, LF at the end."""
    return ("%04X %s\n" % (crc16(line), line)).encode("ascii")


def decode(raw):
    """Bytes of one line (no LF) -> the plain line, or None if its check fails."""
    raw = raw.rstrip(b"\r")
    if len(raw) < 5 or raw[4:5] != b" ":
        return None
    try:
        value = int(raw[:4], 16)
    except ValueError:
        return None
    if value != crc16(raw[5:]):
        return None
    return raw[5:].decode("ascii", "replace")


def convert(value):
    """'12' -> 12, '0.5' -> 0.5, anything else stays a string."""
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def parse_fields(words):
    """['a=1', 'b=x', 'help=free text ...'] -> {'a': 1, 'b': 'x', 'help': 'free text ...'}.

    help= is free text and runs to the end of the line.
    """
    fields = {}
    for i, w in enumerate(words):
        key, eq, value = w.partition("=")
        if not eq:
            continue
        if key == "help":
            fields[key] = " ".join([value] + words[i + 1:])
            break
        fields[key] = value if key in ("values", "hex", "fields") else convert(value)
    return fields


class Event:
    """An event line: '* <name> seq=<n> ...'. Fields are available by key."""

    def __init__(self, line):
        self.line = line
        words = line.split(" ")
        self.name = words[1] if len(words) > 1 else ""
        if self.name == "err":
            # '* err seq=<n> <code> <text>'
            self.fields = parse_fields(words[2:3])
            self.fields["code"] = convert(words[3]) if len(words) > 3 else None
            self.fields["text"] = " ".join(words[4:])
        else:
            self.fields = parse_fields(words[2:])
        self.seq = self.fields.get("seq")
        self.time = time.monotonic()

    def __getitem__(self, key):
        return self.fields[key]

    def get(self, key, default=None):
        return self.fields.get(key, default)

    def values(self):
        """A spectrum stream's values as floats."""
        return [float(v) for v in self.fields["values"].split(",")]

    def payload(self):
        """An rx.packet's payload as bytes."""
        return bytes.fromhex(self.fields["hex"])

    def __repr__(self):
        return "Event(%s)" % self.line


class Reply:
    """All the lines of one command's reply (without the tag)."""

    def __init__(self, tag):
        self.tag = tag
        self.lines = []   # Data lines
        self.ok = None    # True / False once the last line is in
        self.code = None  # Error code
        self.text = ""    # Error text
        self.fields = {}  # From the ok line (e.g. id=, side effects of set)
        self.done = threading.Event()

    def __repr__(self):
        state = "ok" if self.ok else "err %s %s" % (self.code, self.text)
        return "Reply(%s, %s, %r)" % (self.tag, state, self.lines)


class ProtocolError(Exception):
    def __init__(self, code, text, command=""):
        super().__init__("err %s %s (%s)" % (code, text, command))
        self.code = code
        self.text = text


# ---------------------------------------------------------------------------
# Transports: something with fileno() to read from and a write(bytes)
# ---------------------------------------------------------------------------
class SerialTransport:
    def __init__(self, port, baud=115200):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        termios.tcflush(self.fd, termios.TCIOFLUSH)  # Nothing stale either way
        attrs = termios.tcgetattr(self.fd)
        speed = getattr(termios, "B%d" % baud, termios.B115200)
        attrs[4] = attrs[5] = speed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def fileno(self):
        return self.fd

    def read(self):
        return os.read(self.fd, 4096)

    def write(self, data):
        os.write(self.fd, data)

    def close(self):
        os.close(self.fd)


class ProcessTransport:
    """A program speaking the protocol on stdin/stdout (tests/build/protocol_sim)."""

    def __init__(self, argv):
        self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)

    def fileno(self):
        return self.proc.stdout.fileno()

    def read(self):
        return os.read(self.proc.stdout.fileno(), 4096)

    def write(self, data):
        self.proc.stdin.write(data)

    def close(self):
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
        self.proc.stdout.close()


# ---------------------------------------------------------------------------
# The board
# ---------------------------------------------------------------------------
class Board:
    def __init__(self, transport, timeout=2.0, max_events=100000):
        self.transport = transport
        self.timeout = timeout
        self._tags = itertools.count(1)
        self._pending = {}  # tag -> Reply
        self._lock = threading.Lock()
        self._events = collections.deque(maxlen=max_events)
        self._event_ready = threading.Condition(self._lock)
        self.on_event = None  # Optional callback(Event), called on the reader thread
        self.on_stray = None  # Optional callback(line) for replies to tags we didn't send
        self.last_seq = None
        self.lost_events = 0  # Gaps in seq
        self.restarts = 0     # "* hello seq=0" seen
        self.bad_lines = 0    # Lines from the board that failed their check
        self.raw_log = None   # Optional file: every decoded line, as received
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    @classmethod
    def serial(cls, port, baud=115200, **kw):
        board = cls(SerialTransport(port, baud), **kw)
        # An empty line ends any half-line left in the board's input (from a
        # previous program, or rubbish); the board ignores empty lines
        board.send_bytes(b"\n")
        return board

    @classmethod
    def sim(cls, path, speed=1.0, save=False, **kw):
        argv = [path, "--speed", str(speed)] + (["--save"] if save else [])
        return cls(ProcessTransport(argv), **kw)

    def close(self):
        self._running = False
        self._reader.join(timeout=1)
        self.transport.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # --- reading --------------------------------------------------------
    def _read_loop(self):
        buf = b""
        while self._running:
            try:
                r, _, _ = select.select([self.transport], [], [], 0.1)
                if not r:
                    continue
                data = self.transport.read()
            except (OSError, ValueError):
                break
            if not data:
                break
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                if not raw.strip():
                    continue
                line = decode(raw)
                if line is None:
                    self.bad_lines += 1
                    continue
                self._handle(line)

    def _handle(self, line):
        if self.raw_log:
            self.raw_log.write(line + "\n")
        if line.startswith("* "):
            e = Event(line)
            seq = e.seq
            if isinstance(seq, int):
                if seq == 0:
                    if self.last_seq is not None:
                        self.restarts += 1
                elif self.last_seq is not None and seq != self.last_seq + 1:
                    self.lost_events += (seq - self.last_seq - 1) % (1 << 32)
                self.last_seq = seq
            if self.on_event:
                self.on_event(e)
            with self._lock:
                self._events.append(e)
                self._event_ready.notify_all()
            return
        tag, _, rest = line.partition(" ")
        with self._lock:
            reply = self._pending.get(tag)
        if reply is None:
            if self.on_stray:  # Not ours (sent with send_raw, or a timed-out command)
                self.on_stray(line)
            return
        words = rest.split(" ")
        if words[0] == "ok":
            reply.fields = parse_fields(words[1:])
            reply.ok = True
        elif words[0] == "err":
            reply.ok = False
            reply.code = convert(words[1]) if len(words) > 1 else None
            reply.text = " ".join(words[2:])
        else:
            reply.lines.append(rest)
            return
        with self._lock:
            self._pending.pop(tag, None)
        reply.done.set()

    # --- commands -------------------------------------------------------
    def send_raw(self, line):
        """Sends one line as given (the check is added, no tag)."""
        self.transport.write(encode(line))

    def send_bytes(self, data):
        """Sends bytes exactly as given (no check): for testing bad lines."""
        self.transport.write(data)

    def command(self, text, timeout=None, check=True):
        """Sends '<tag> <text>' and waits for the whole reply."""
        tag = "h%d" % next(self._tags)
        reply = Reply(tag)
        with self._lock:
            self._pending[tag] = reply
        self.transport.write(encode("%s %s" % (tag, text)))
        if not reply.done.wait(self.timeout if timeout is None else timeout):
            with self._lock:
                self._pending.pop(tag, None)
            raise TimeoutError("no reply to: %s" % text)
        if check and not reply.ok:
            raise ProtocolError(reply.code, reply.text, text)
        return reply

    def hello(self):
        return parse_fields(self.command("hello").lines[0].split(" "))

    def get(self, *names):
        """{name: value} for settings and status (wildcards allowed)."""
        reply = self.command("get " + " ".join(names))
        out = {}
        for line in reply.lines:
            name, _, value = line.partition("=")
            out[name] = convert(value)
        return out

    def set(self, settings):
        """Changes settings ({name: value}); returns any side effects."""
        args = " ".join("%s=%s" % (k, v) for k, v in settings.items())
        return self.command("set " + args).fields

    def list(self, *names):
        return [parse_fields(line.split(" ")) for line in self.command("list " + " ".join(names)).lines]

    def send(self, payload):
        """Queues a packet; returns its id (watch for tx.done with that id)."""
        return self.command("send hex=" + bytes(payload).hex()).fields["id"]

    def sub(self, stream, every=None, fields=None, units=None, start=None, end=None):
        args = ["stream=" + stream]
        if every is not None:
            args.append("every=%d" % every)
        if fields is not None:
            args.append("fields=" + (fields if isinstance(fields, str) else ",".join(fields)))
        if units is not None:
            args.append("units=" + units)
        if start is not None:
            args.append("from=%d" % start)
        if end is not None:
            args.append("to=%d" % end)
        self.command("sub " + " ".join(args))

    def unsub(self, stream="all"):
        self.command("unsub all" if stream == "all" else "unsub stream=" + stream)

    def nudge(self, samples):
        return self.command("nudge samples=%d" % samples).fields["delay"]

    # --- events ---------------------------------------------------------
    def next_event(self, timeout=None):
        """The oldest unread event, or None after timeout seconds."""
        with self._lock:
            if not self._events:
                self._event_ready.wait(timeout)
            return self._events.popleft() if self._events else None

    def events(self, timeout=1.0):
        """Unread events, as they come, until none for timeout seconds."""
        while True:
            e = self.next_event(timeout)
            if e is None:
                return
            yield e

    def wait_event(self, name, match=None, timeout=10.0, keep_others=True):
        """The first unread event called name (and matching, if given).

        Other events read on the way are put back (keep_others) or dropped.
        """
        deadline = time.monotonic() + timeout
        skipped = []
        try:
            while True:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise TimeoutError("no %s event" % name)
                e = self.next_event(left)
                if e is None:
                    continue
                if e.name == name and (match is None or match(e)):
                    return e
                skipped.append(e)
        finally:
            if keep_others and skipped:
                with self._lock:
                    self._events.extendleft(reversed(skipped))

    def clear_events(self):
        with self._lock:
            self._events.clear()


# ---------------------------------------------------------------------------
# Console
# ---------------------------------------------------------------------------
def console(board, show_streams):
    def on_event(e):
        if e.name in STREAMS and not show_streams:
            return
        line = e.line if len(e.line) < 200 else e.line[:197] + "..."
        sys.stdout.write("\r%s\n> " % line)
        sys.stdout.flush()

    def on_stray(line):
        sys.stdout.write("\r  %s\n> " % line)
        sys.stdout.flush()

    board.on_event = on_event
    board.on_stray = on_stray
    print("Type commands without a tag (help lists them); '!' sends a line as typed; ctrl-D ends.")
    while True:
        try:
            text = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not text:
            continue
        if text.startswith("!"):
            board.send_raw(text[1:])
            continue
        try:
            reply = board.command(text, check=False)
        except TimeoutError as err:
            print("  (%s)" % err)
            continue
        for line in reply.lines:
            print("  " + line)
        if reply.ok:
            extra = " ".join("%s=%s" % kv for kv in reply.fields.items())
            print("  ok" + (" " + extra if extra else ""))
        else:
            print("  err %s %s" % (reply.code, reply.text))
        if board.lost_events or board.bad_lines:
            print("  (events lost so far: %d, bad lines: %d)" % (board.lost_events, board.bad_lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("console", help="type commands at a board")
    where = c.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="serial port, e.g. /dev/ttyACM0")
    where.add_argument("--sim", help="path to tests/build/protocol_sim")
    c.add_argument("--speed", type=float, default=1.0, help="simulator clock speed")
    c.add_argument("--streams", action="store_true", help="print stream lines too")
    args = ap.parse_args()

    board = Board.serial(args.port) if args.port else Board.sim(args.sim, speed=args.speed)
    with board:
        console(board, args.streams)


if __name__ == "__main__":
    main()
