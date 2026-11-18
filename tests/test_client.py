#!/usr/bin/env python3
"""The Python host client (tools/chirp_host.py) against the simulated board
(tests/build/protocol_sim), end to end through a pipe.

    python3 tests/test_client.py tests/build/protocol_sim
"""

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import chirp_host  # noqa: E402
from chirp_host import Board, ProtocolError  # noqa: E402

SIM = None
SPEED = 20.0  # Board clock x20: a frame every 1.2 ms


class Lines(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(chirp_host.crc16("123456789"), 0x29B1)

    def test_encode_decode(self):
        raw = chirp_host.encode("a1 hello")
        self.assertEqual(raw[4:5], b" ")
        self.assertEqual(chirp_host.decode(raw[:-1]), "a1 hello")
        self.assertIsNone(chirp_host.decode(raw[:-2] + b"x"))
        self.assertIsNone(chirp_host.decode(b"a1 hello"))

    def test_parse(self):
        e = chirp_host.Event("* rx.packet seq=4 frame=29 crc=ok len=2 min_margin=2.000 hex=4869")
        self.assertEqual((e.name, e.seq, e["frame"], e["crc"], e["min_margin"]), ("rx.packet", 4, 29, "ok", 2.0))
        self.assertEqual(e.payload(), b"Hi")
        e = chirp_host.Event("* err seq=3 500 line failed its check")
        self.assertEqual((e.seq, e["code"], e["text"]), (3, 500, "line failed its check"))
        f = chirp_host.parse_fields("name=tx.volume kind=setting help=Transmit level now".split(" "))
        self.assertEqual(f["help"], "Transmit level now")


class WithSim(unittest.TestCase):
    def setUp(self):
        self.b = Board.sim(SIM, speed=SPEED)
        self.b.wait_event("hello", timeout=5)

    def tearDown(self):
        self.assertEqual(self.b.lost_events, 0)
        self.assertEqual(self.b.bad_lines, 0)
        self.b.close()

    def test_hello_get_set(self):
        h = self.b.hello()
        self.assertEqual((h["proto"], h["version"]), (1, "test"))
        self.assertEqual(self.b.get("tx.volume"), {"tx.volume": -28})
        everything = self.b.get()
        self.assertEqual(everything["link.bits"], 4)
        self.assertEqual(everything["rx.state"], "search")
        self.assertEqual(self.b.set({"tx.volume": -31}), {})
        self.assertEqual(self.b.set({"link.spacing": 1, "test.symbol": 40}), {})
        self.assertEqual(self.b.set({"link.spacing": 4}), {"test.symbol": 8})
        with self.assertRaises(ProtocolError) as cm:
            self.b.set({"tx.volume": -60})
        self.assertEqual(cm.exception.code, 503)
        self.assertEqual(self.b.get("tx.volume")["tx.volume"], -31)
        described = {d["name"]: d for d in self.b.list("tx.*")}
        self.assertEqual(described["tx.volume"]["range"], "-55..-1")

    def test_send_and_hear(self):
        payload = b"chirp packet 001"
        pid = self.b.send(payload)
        start = self.b.wait_event("tx.start", lambda e: e["id"] == pid, timeout=5)
        done = self.b.wait_event("tx.done", lambda e: e["id"] == pid, timeout=5)
        heard = self.b.wait_event("rx.packet", timeout=5)
        self.assertEqual(heard.payload(), payload)
        self.assertEqual(heard["crc"], "ok")
        # 16 bytes at 4 bits/symbol: 16 preamble + (16 + 128 + 16) / 4 data frames
        self.assertEqual(done["frame"] - start["frame"], 16 + 40)

    def test_commands_while_events_flow(self):
        self.b.sub("frame", fields="delay")
        self.b.sub("spectrum", units="power")
        pid = self.b.send(bytes(range(255)))
        for i in range(50):  # Replies are matched by tag, whatever comes in between
            self.assertEqual(self.b.get("tx.volume")["tx.volume"], -28)
        heard = self.b.wait_event("rx.packet", timeout=30)
        self.assertEqual(heard.payload(), bytes(range(255)))
        done = self.b.wait_event("tx.done", lambda e: e["id"] == pid, timeout=1)
        self.assertLess(done.seq, heard.seq)

    def test_streams(self):
        self.b.sub("frame", every=2, fields="delay,rx_symbol")
        self.b.clear_events()
        frames = [self.b.wait_event("frame", timeout=2, keep_others=False) for _ in range(3)]
        self.assertEqual([f["frame"] - frames[0]["frame"] for f in frames], [0, 2, 4])
        self.assertEqual(set(frames[0].fields), {"seq", "frame", "delay", "rx_symbol"})
        self.b.sub("spectrum", start=0, end=511)
        spec = self.b.wait_event("spectrum", timeout=2, keep_others=False)
        values = spec.values()
        self.assertEqual(len(values), 512)
        self.assertTrue(all(-45 < v < -35 for v in values))  # The fake board's noise, about -40 dB
        listed = self.b.command("sub").lines
        self.assertEqual(listed, ["stream=frame every=2 fields=delay,rx_symbol",
                                  "stream=spectrum every=1 units=db from=0 to=511"])
        self.b.unsub()
        self.assertEqual(self.b.command("sub").lines, [])

    def test_bad_lines(self):
        self.b.send_bytes(b"a1 hello\n")  # No check
        e = self.b.wait_event("err", timeout=2)
        self.assertEqual((e["code"], e["text"]), (500, "line failed its check"))
        self.b.send_raw("nonsense!")  # Checked, but no valid tag
        e = self.b.wait_event("err", timeout=2)
        self.assertEqual(e["text"], "line has no valid tag")
        self.assertEqual(self.b.get("count.link_errors")["count.link_errors"], 1)

    def test_csv_log(self):
        import io
        import csv
        out = io.StringIO()
        log = chirp_host.CsvLog(self.b, out)
        log.start()
        pid = self.b.send(b"chirp packet 042")
        self.b.wait_event("rx.packet", timeout=10)
        time.sleep(0.2)
        log.stop()
        self.assertEqual(self.b.wait_event("tx.done", lambda e: e["id"] == pid, timeout=1)["result"], "sent")
        text = out.getvalue()
        lines = text.splitlines()
        self.assertEqual(lines[0], "# chirp receiver log v8")
        self.assertIn("# sample_rate=44100.0 fft_size=1024 bins=64 bins_per_symbol=4 symbols=16", text)
        self.assertEqual(lines[-1], "# log end")
        rows = list(csv.DictReader(l for l in lines if l and not l.startswith("#")))
        self.assertEqual(list(rows[0].keys()), chirp_host.LOG_COLUMNS)
        self.assertEqual((log.incomplete, log.gaps), (0, 0))
        frames = [int(r["frame"]) for r in rows]
        self.assertEqual(frames, list(range(frames[0], frames[0] + len(frames))))
        events = {int(r["packet_event"]): r for r in rows if r["packet_event"] != "0"}
        self.assertEqual(sorted(events), [1, 2, 6])  # Locked, trained, packet
        trained = events[2]
        self.assertEqual((trained["packet_coherence"], trained["packet_strongest"]), ("0.830", "-1"))
        self.assertEqual(trained["s0"], "1.000e+00")  # The shape learned on that frame
        self.assertEqual(events[1]["packet_align_fraction"], "+0.120")
        self.assertEqual((rows[0]["mode"], rows[0]["tracking"], rows[0]["bins_per_symbol"], rows[0]["tx_volume_db"],
                          rows[0]["packet_align_on"]), ("packet", "off", "4", "-28", "1"))
        # The packet's comment line follows its row
        i = lines.index(next(l for l in lines if l.startswith(events[6]["frame"] + ",")))
        self.assertEqual(lines[i + 1], "# packet frame=%s crc_ok=1 length=16 data_symbols=40 min_margin=2.000 "
                                       "weakest=0 weak=0 payload=chirp packet 042" % events[6]["frame"])

    def test_log_command(self):
        """The CLI: a sweep through every symbol in chirp mode, settings restored after, and analyse_log.py reads it."""
        import subprocess
        import tempfile
        tools = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools")
        self.b.close()
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "sweep.csv")
            r = subprocess.run([sys.executable, os.path.join(tools, "chirp_host.py"), "log", "--sim", SIM,
                                "--speed", str(SPEED), "--set", "rx.tracking=pilot", "--sweep", "0.05", path],
                               capture_output=True, text=True, timeout=60)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn("0 incomplete frames left out, 0 gaps; events lost 0, bad lines 0", r.stdout)
            with open(path) as f:
                text = f.read()
            self.assertIn(",chirp,", text)
            self.assertIn(",pilot,", text)
            a = subprocess.run([sys.executable, os.path.join(tools, "analyse_log.py"), path],
                               capture_output=True, text=True, timeout=60)
            self.assertEqual(a.returncode, 0, a.stderr)
            table = [l.split() for l in a.stdout.splitlines() if l.strip()[:1].isdigit()]
            self.assertEqual([int(t[0]) for t in table], list(range(16)))  # Every symbol, each decoded right
            self.assertTrue(all(t[2] == "100.0" for t in table))
        self.b = Board.sim(SIM, speed=SPEED)  # For tearDown

    def test_nudge(self):
        self.assertEqual(self.b.nudge(-8), 952)
        self.assertEqual(self.b.get("rx.delay")["rx.delay"], 952)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    SIM = os.path.abspath(sys.argv.pop(1))
    unittest.main(verbosity=2)
