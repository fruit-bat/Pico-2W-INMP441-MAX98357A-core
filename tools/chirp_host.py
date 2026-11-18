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

CSV log (the format the firmware printed before the protocol):

    python3 tools/chirp_host.py log --port /dev/ttyACM0 --seconds 30 run.csv
    python3 tools/chirp_host.py log --port /dev/ttyACM0 --set rx.tracking=pilot --sweep 1 sweep.csv

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
        fields[key] = value if key in ("values", "hex", "fields", "id", "version") else convert(value)
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
        self._words = words
        self.time = time.monotonic()

    def __getitem__(self, key):
        return self.fields[key]

    def get(self, key, default=None):
        return self.fields.get(key, default)

    def raw(self, key, default=""):
        """A field exactly as the board wrote it (e.g. '+0.300'), as a string."""
        prefix = key + "="
        for w in self._words:
            if w.startswith(prefix):
                return w[len(prefix):]
        return default

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
        self.queue_streams = True  # False: stream lines go only to on_event, not the queue
        self.settings = {}    # The settings as this client last read or set them
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
            if e.name in STREAMS and not self.queue_streams:
                return
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
        self.settings.update((k, v) for k, v in out.items() if k in self._setting_names())
        return out

    def _setting_names(self):
        if not hasattr(self, "_settings_known"):
            self._settings_known = {d["name"] for d in self.list() if d.get("kind") == "setting"}
        return self._settings_known

    def set(self, settings):
        """Changes settings ({name: value}); returns any side effects."""
        args = " ".join("%s=%s" % (k, v) for k, v in settings.items())
        side = self.command("set " + args).fields
        self.settings.update((k, convert(str(v))) for k, v in settings.items())
        self.settings.update(side)
        return side

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
# CSV log: the format the firmware printed before the protocol (log v8),
# rebuilt from the streams and events, so tools/analyse_log.py and older
# scripts read it unchanged
# ---------------------------------------------------------------------------
LOG_MODE_NAMES = {"loopback": "loopback", "mic": "mic-level", "tone": "tone-440",
                  "chirp": "chirp", "sequence": "sequence", "packet": "packet"}
# packet_rx.h's PacketRxEvent_t numbers, as the log's packet_event column
LOG_EVENT_CODES = {"rx.locked": 1, "rx.trained": 2, "rx.train_failed": 3, "rx.tone": 4,
                   "rx.header_failed": 5, "rx.packet": 6, "rx.lost": 7}
LOG_COLUMNS = (["frame", "mode", "tx_symbol", "delay", "tracking", "rx_symbol", "angle", "error_angle",
                "magnitude", "strength", "peak", "timing_error", "timing_rate", "rx_missed", "tx_missed", "dropped"]
               + ["b%d" % k for k in range(-64, 64)] + ["f%d" % k for k in range(64)]
               + ["packet_state", "packet_event", "packet_corr", "packet_coherence", "packet_b_start",
                  "packet_strongest", "packet_earliest"]
               + ["s%d" % k for k in range(64)]
               + ["packet_align", "packet_align_on", "packet_data_symbol", "packet_data_margin",
                  "packet_align_fraction", "bins_per_symbol", "tx_volume_db", "packet_data_quality"])


def _printable(data):
    return "".join(chr(c) if 0x20 <= c < 0x7F and c != 0x5C else "\\x%02X" % c for c in data)


class CsvLog:
    """Writes one CSV row per received frame, as the old firmware log did.

        log = CsvLog(board, open("run.csv", "w"))
        log.start()      # reads the settings, writes the header, subscribes
        ...              # (settings changed with board.set() are picked up)
        log.stop()       # unsubscribes, writes the last rows and "# log end"

    Needs the frame, band, folded and shape streams (about 100 KB/s: USB,
    not a slow UART). A frame whose lines didn't all arrive is left out and
    counted in `incomplete`.
    """

    def __init__(self, board, out):
        self.board = board
        self.out = out
        self.rows = 0
        self.incomplete = 0
        self.gaps = 0  # Jumps in the frame numbers of the rows written
        self._pending = {}  # frame -> {"frame": Event, "band": Event, "folded": Event, "shape": Event, "events": [...]}
        self._shape = ["0.000e+00"] * 64
        self._lock = {"align": "+0", "fraction": "+0.000"}
        self._train = {"coherence": "0.000", "b_start": "0", "strongest": "+0", "earliest": "+0"}
        self._last_frame = None
        self._chained = None
        self._ready = False  # Set once every stream is subscribed
        self._skip = None    # The frame in progress then: left out

    def start(self):
        b = self.board
        b.get()  # All settings and status, into b.settings
        info = b.get("info.*", "link.*")
        hello = b.hello()
        spacing = b.settings["link.spacing"]
        symbols = 64 // spacing
        w = self.out.write
        w("# chirp receiver log v8\n")
        w("# written by tools/chirp_host.py from the protocol's streams (firmware %s, board %s)\n"
          % (hello["version"], hello["id"]))
        w("# sample_rate=%.1f fft_size=%d bins=%d bins_per_symbol=%d symbols=%d f0=%.1f bandwidth=%.2f "
          "samples_per_symbol_step=%d\n" % (info["info.sample_rate"], info["info.fft_size"], info["info.bins"],
                                             spacing, symbols, info["info.f0"], info["info.bandwidth"],
                                             info["info.fft_size"] // symbols))
        w("# track_alpha=%g track_beta=%g track_max_rate=%g track_rate_window=%g track_min_magnitude=%g\n"
          % (info["info.track_alpha"], info["info.track_beta"], info["info.track_max_rate"],
             info["info.track_rate_window"], info["info.track_min_magnitude"]))
        w("# rx_highpass_hz=%g (2nd order, applied to the mic stream; peak is measured after it)\n"
          % info["info.highpass_hz"])
        for line in (
                "delay: window delay used for this frame (samples); timing_error: samples, positive = increase delay",
                "angle/error_angle: radians; b<k>: de-chirped FFT power |X|^2 of bin k (k mod fft_size)",
                "f<k>: coherently folded power of bin k, which the decoder decides from",
                "packet_*: packet receiver (packet_rx.h) after this frame; packet_event 2 = preamble found (b_start, paths);",
                "s<k>: channel shape it last learned, by bin offset k from the window position",
                "packet_data_symbol/margin/quality: last data symbol decided, its margin (best/second score)",
                "  and quality (correlation with the learned shape; noise ~0.3, symbols ~0.85)",
                "'# packet lost' lines: header or payload abandoned when the signal went",
                "'# packet' lines: a received packet (after its frame's row); '# packet header failed': bad CRC-8",
                "packet_align (+ packet_align_fraction): at the last lock, bins the window was late "
                "(moved by that if packet_align_on)",
                "mode, tracking, packet_align_on, bins_per_symbol, tx_volume_db: the settings as this client knows them"):
            w("# %s\n" % line)
        w(",".join(LOG_COLUMNS) + "\n")
        self._chained = b.on_event
        b.on_event = self._on_event
        b.queue_streams = False
        b.sub("frame")
        b.sub("band", units="power")
        b.sub("folded", units="power")
        b.sub("shape", units="power")
        # The board sends its lines in order, so everything after the last
        # sub's reply is for frames that had all four streams; anything
        # earlier (a frame caught between two subs) is left out
        self._ready = True

    def stop(self):
        b = self.board
        b.unsub()
        time.sleep(0.2)  # Lines already on their way
        b.on_event = self._chained
        b.queue_streams = True
        for frame in sorted(self._pending):
            self._flush(frame)
        self._pending.clear()
        self.out.write("# log end\n")
        self.out.flush()

    def _on_event(self, e):
        if self._chained:
            self._chained(e)
        if not self._ready:
            return
        frame = e.get("frame")
        if not isinstance(frame, int) or (e.name not in STREAMS and e.name not in LOG_EVENT_CODES):
            return
        if self._skip is None:
            self._skip = frame  # _ready may have been set part-way through this frame's lines
        if frame <= self._skip:
            return
        if self._last_frame is not None and frame <= self._last_frame:
            return  # Before the log started, or already written
        # A frame's events come first, then its stream lines; a newer frame
        # means the older ones are complete
        for older in sorted(f for f in self._pending if f < frame):
            self._flush(older)
        p = self._pending.setdefault(frame, {"events": []})
        if e.name in STREAMS:
            p[e.name] = e
        else:
            p["events"].append(e)

    def _flush(self, frame):
        p = self._pending.pop(frame)
        code = 0
        notes = []
        for e in p["events"]:
            code = LOG_EVENT_CODES[e.name]
            if e.name == "rx.locked":
                self._lock = {"align": e.raw("align"), "fraction": e.raw("fraction")}
            elif e.name == "rx.trained":
                self._train = {k: e.raw(k) for k in ("coherence", "b_start", "strongest", "earliest")}
            elif e.name == "rx.packet":
                bits = 64 // self.board.settings.get("link.spacing", 4)
                bits = bits.bit_length() - 1
                length = e["len"]
                notes.append("# packet frame=%d crc_ok=%d length=%d data_symbols=%d min_margin=%s weakest=%s "
                             "weak=%s payload=%s" % (frame, e["crc"] == "ok", length,
                                                     (16 + 8 * length + 16 + bits - 1) // bits, e.raw("min_margin"),
                                                     e.raw("weakest"), e.raw("weak"), _printable(e.payload())))
            elif e.name == "rx.header_failed":
                notes.append("# packet header failed: length byte %s" % e.raw("length"))
            elif e.name == "rx.lost":
                notes.append("# packet lost: signal gone after %s data symbols" % e.raw("symbols"))
        if "shape" in p:
            self._shape = p["shape"].raw("values").split(",")
        f, band, folded = p.get("frame"), p.get("band"), p.get("folded")
        if not (f and band and folded):
            self.incomplete += 1
        else:
            st = self.board.settings
            row = ([str(frame), LOG_MODE_NAMES.get(st.get("test.mode"), str(st.get("test.mode"))),
                    f.raw("tx_symbol"), f.raw("delay"), str(st.get("rx.tracking")), f.raw("rx_symbol"),
                    f.raw("angle"), f.raw("error_angle"), f.raw("magnitude"), f.raw("strength"), f.raw("peak"),
                    f.raw("timing_error"), f.raw("timing_rate"), f.raw("rx_missed"), f.raw("tx_missed"),
                    f.raw("records_dropped")]
                   + band.raw("values").split(",") + folded.raw("values").split(",")
                   + [f.raw("state"), str(code), f.raw("corr"), self._train["coherence"], self._train["b_start"],
                      self._train["strongest"], self._train["earliest"]]
                   + self._shape
                   + [self._lock["align"], "1" if st.get("rx.align") == "on" else "0", f.raw("data_symbol"),
                      f.raw("data_margin"), self._lock["fraction"], str(st.get("link.spacing")),
                      "%+d" % st.get("tx.volume", 0), f.raw("data_quality")])
            self.out.write(",".join(row) + "\n")
            if self._last_frame is not None and frame != self._last_frame + 1:
                self.gaps += 1
            self.rows += 1
        self._last_frame = frame
        for n in notes:
            self.out.write(n + "\n")


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
    lg = sub.add_parser("log", help="write the CSV log (the old firmware format) for a while")
    where = lg.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="serial port, e.g. /dev/ttyACM0")
    where.add_argument("--sim", help="path to tests/build/protocol_sim")
    lg.add_argument("--speed", type=float, default=1.0, help="simulator clock speed")
    lg.add_argument("--seconds", type=float, default=10.0, help="how long to log (default 10)")
    lg.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="settings for the run (restored afterwards); may be repeated")
    lg.add_argument("--sweep", type=float, metavar="SECONDS",
                    help="chirp mode, stepping test.symbol through every symbol, this long each")
    lg.add_argument("output", help="CSV file to write")
    args = ap.parse_args()

    board = Board.serial(args.port) if args.port else Board.sim(args.sim, speed=args.speed)
    with board:
        if args.cmd == "console":
            console(board, args.streams)
        else:
            run_log(board, args)


def run_log(board, args):
    changes = dict(item.split("=", 1) for item in args.set)
    if args.sweep:
        changes.setdefault("test.mode", "chirp")
    restore = list(changes) + (["test.symbol"] if args.sweep else [])
    before = board.get(*restore) if restore else {}
    if changes:
        board.set(changes)
    with open(args.output, "w") as out:
        log = CsvLog(board, out)
        log.start()
        try:
            if args.sweep:
                symbols = 64 // board.settings["link.spacing"]
                for s in range(symbols):
                    board.set({"test.symbol": s})
                    time.sleep(args.sweep)
            else:
                time.sleep(args.seconds)
        finally:
            log.stop()
            if before:
                board.set(before)
    print("%s: %d rows, %d incomplete frames left out, %d gaps; events lost %d, bad lines %d"
          % (args.output, log.rows, log.incomplete, log.gaps, board.lost_events, board.bad_lines))


if __name__ == "__main__":
    main()
