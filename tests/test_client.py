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
import chirp_soak  # noqa: E402
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


class Soak(unittest.TestCase):
    """tools/chirp_soak.py: the parts the simulator can't reach (it makes no errors)."""

    def test_compare(self):
        sent = bytes([0x00, 0xFF, 0x55])
        self.assertEqual(chirp_soak.compare(sent, sent, 4), (0, 0, []))
        # 4 bits/symbol: header is symbols 0-7, payload byte 0 is symbols 8-9, byte 2 is 12-13
        self.assertEqual(chirp_soak.compare(sent, bytes([0x01, 0xFF, 0x54]), 4), (2, 2, [9, 13]))
        # 5 bits/symbol: byte 1 bit 0 is payload bit 8 = symbol 8 + 1
        self.assertEqual(chirp_soak.compare(sent, bytes([0x00, 0x7F, 0x55]), 2), (1, 1, [9]))
        self.assertEqual(chirp_soak.data_symbols(16, 4, 0), 44)
        self.assertEqual(chirp_soak.data_symbols(16, 2, 0), 37)
        self.assertEqual(chirp_soak.data_symbols(63, 4, 0), 138)
        # rs2 adds 4 check bytes (packet_format.md airtime table)
        self.assertEqual(chirp_soak.data_symbols(16, 4, 1), 52)
        self.assertEqual(chirp_soak.data_symbols(16, 2, 1), 44)
        self.assertEqual(chirp_soak.data_symbols(63, 4, 1), 146)

    def test_judge(self):
        E = chirp_host.Event
        locked = E("* rx.locked seq=1 frame=10 align=+0 fraction=+0.120 moved=+2")
        trained = E("* rx.trained seq=2 frame=20 b_start=18 coherence=0.830 strongest=-1 earliest=+0 moved=+0")
        bad = E("* rx.packet seq=3 frame=60 crc=bad len=2 min_margin=0.100 weakest=4 weak=1 hex=4869")
        lost = E("* rx.lost seq=3 frame=40 symbols=5 reason=signal")
        self.assertEqual(chirp_soak.judge([])[0], "no_lock")
        self.assertEqual(chirp_soak.judge([locked])[0], "unconcluded")
        outcome, p, l, t, d = chirp_soak.judge([locked, trained, bad])
        self.assertEqual((outcome, p, l, t, d), ("crc_bad", bad, locked, trained, None))
        # rx.symbols straight after the packet, or after a failed header
        sym = E("* rx.symbols seq=4 frame=60 count=1 symbols=03 second=02 margins=1002")
        self.assertIs(chirp_soak.judge([locked, trained, bad, sym])[4], sym)
        failed = E("* rx.header_failed seq=3 frame=24 length=200")
        outcome, p, l, t, d = chirp_soak.judge([locked, trained, failed, sym])
        self.assertEqual((outcome, p, d), ("header_failed", None, sym))
        self.assertEqual(chirp_soak.judge([locked, trained, lost])[0], "lost:signal")
        # A false lock that gave up, then the real packet: the packet's own lock is reported
        early = E("* rx.locked seq=0 frame=1 align=+3 fraction=+0.000 moved=+48")
        tone = E("* rx.tone seq=0 frame=5")
        outcome, p, l, t, d = chirp_soak.judge([early, tone, locked, trained, bad])
        self.assertEqual((outcome, l), ("crc_bad", locked))
        self.assertEqual(chirp_soak.judge([early, tone])[0], "tone")

    def test_decisions(self):
        # The symbols packet_tx sends for "Hi" (printed by the C, 2026-11-20; header at spacing 8);
        # with rs2 the same as test_packet.c's test_hi_symbols
        self.assertEqual(chirp_soak.sent_symbols(b"Hi", 4, 0), [1, 0, 0, 3, 2, 1, 3, 4, 8, 10, 7, 12, 5, 11, 3, 3])
        self.assertEqual(chirp_soak.sent_symbols(b"Hi", 2, 0), [0, 7, 0, 3, 5, 3, 1, 3, 17, 22, 6, 11, 13, 21, 31])
        self.assertEqual(chirp_soak.sent_symbols(b"Hi", 4, 1),
                         [1, 1, 0, 3, 6, 5, 1, 5, 8, 10, 7, 12, 4, 2, 15, 10, 1, 8, 0, 2, 2, 12, 7, 8])
        self.assertEqual(chirp_soak.sent_symbols(b"Hi", 2, 1),
                         [0, 6, 0, 3, 1, 7, 3, 2, 17, 22, 6, 10, 31, 24, 22, 10, 29, 0, 22, 28, 7])
        self.assertEqual(chirp_soak.crc16(b"123456789"), 0x29B1)
        # Reed-Solomon: the QR code's published example (version 1-M, 10 check bytes; same field
        # and roots), then the packet's 4 check bytes as test_packet.c pins them
        qr = bytes([32, 91, 11, 120, 209, 114, 220, 77, 67, 64, 236, 17, 236, 17, 236, 17])
        self.assertEqual(list(chirp_soak.rs_parity(qr, 10)), [196, 35, 39, 119, 235, 215, 231, 226, 93, 23])
        self.assertEqual(chirp_soak.rs_parity(b"Hello, chirp!").hex(), "fe46ec5b")
        self.assertEqual(chirp_soak.rs_parity(bytes(range(20))).hex(), "fa1d5bbc")
        self.assertEqual(chirp_soak.golay_encode(0x210) >> 12, 0x210)
        # Symbol 1 (header) and 9 wrong: 1's runner-up was right; margins rank 9 lowest, then 1
        sent = chirp_soak.sent_symbols(b"Hi", 4, 0)
        got = list(sent)
        got[1], got[9] = 2, 12
        second = [0] * len(sent)
        second[1] = 0
        margins = [1500] * len(sent)
        margins[1], margins[9], margins[4] = 1010, 1001, 1020
        ev = chirp_host.Event("* rx.symbols seq=1 frame=2 count=%d symbols=%s second=%s margins=%s"
                              % (len(got), bytes(got).hex(), bytes(second).hex(), ",".join(map(str, margins))))
        c = chirp_soak.check_decisions(sent, ev, 4)
        self.assertEqual((c["decided"], c["wrong"], c["header_wrong"], c["wrong_second"], c["wrong_ranks"]),
                         (16, "1;9", 1, 1, "2;1"))
        # A failed header: only the header's 8 symbols come (here symbol 3 wrong, with the lowest margin)
        ev = chirp_host.Event("* rx.symbols seq=1 frame=2 count=8 symbols=0100000502010304 second=0000000000000000 "
                              "margins=1100,1200,1300,1050,1400,1400,1400,1400")
        c = chirp_soak.check_decisions(sent, ev, 4)
        self.assertEqual((c["decided"], c["wrong"], c["header_wrong"], c["wrong_ranks"]), (8, "3", 1, "1"))

    def test_run(self):
        import csv
        import subprocess
        import tempfile
        tool = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools", "chirp_soak.py")
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "soak.csv")
            r = subprocess.run([sys.executable, tool, "run", "--sim", SIM, "--speed", str(SPEED), "--count", "2",
                                "--idle", "2", "--ecc", "none,rs2", "--volume", "-30", "--length", "4,40", "--seed", "7",
                                path],
                               capture_output=True, text=True, timeout=120)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn("events lost 0, bad lines 0", r.stdout)
            with open(path) as f:
                rows = list(csv.DictReader(l for l in f if not l.startswith("#")))
            packets = [row for row in rows if row["outcome"] != "idle"]
            self.assertEqual(len(packets), 16)  # 2 spacings x 2 codes x 2 lengths x 2
            self.assertEqual(len(rows) - len(packets), 8)  # An idle period per condition
            self.assertEqual(sorted(set(row["ecc"] for row in packets)), ["none", "rs2"])
            self.assertTrue(all(row["corrected"] == ("0" if row["ecc"] == "rs2" else "") for row in packets))
            self.assertTrue(all(row["outcome"] == "ok" and row["bit_errors"] == "0" for row in packets))
            # The simulator's rx.symbols are 3 made-up decisions, but they get through
            self.assertTrue(all(row["decided"] == "3" and row["margins"] == "1500,1002,65535" for row in packets))
            for row in packets:  # Airtime as the board reported it
                self.assertEqual(int(row["tx_frames"]), 16 + chirp_soak.data_symbols(
                    int(row["length"]), int(row["spacing"]), chirp_soak.ECC_NAMES[row["ecc"]]))
            # Run again with --append: the header isn't repeated, and summary reads both runs
            r = subprocess.run([sys.executable, tool, "run", "--sim", SIM, "--speed", str(SPEED), "--count", "1",
                                "--idle", "0", "--spacing", "4", "--volume", "-30", "--length", "4", "--append",
                                path], capture_output=True, text=True, timeout=120)
            self.assertEqual(r.returncode, 0, r.stderr)
            s = subprocess.run([sys.executable, tool, "summary", path], capture_output=True, text=True, timeout=30)
            table = [l.split() for l in s.stdout.splitlines()[1:]]
            self.assertIn("Symbol decisions", s.stdout)
            self.assertIn(["A>A", "4", "rs2", "-30", "4", "3", "3"], [t[:7] for t in table])
            with open(path) as f:
                self.assertEqual(sum(1 for l in f if l.startswith("time,")), 1)


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
        # 16 bytes at 4 bits/symbol with rs2: 16 preamble + 8 header + (128 + 16 + 32) / 4 frames
        self.assertEqual(done["frame"] - start["frame"], 16 + 8 + 44)
        self.assertEqual((heard["spacing"], heard["fixed"], heard.raw("ecc"), heard["corrected"]), (4, 0, "rs2", 0))

    def test_commands_while_events_flow(self):
        self.b.sub("frame", fields="delay")
        self.b.sub("spectrum", units="power")
        pid = self.b.send(bytes(range(63)))
        for i in range(50):  # Replies are matched by tag, whatever comes in between
            self.assertEqual(self.b.get("tx.volume")["tx.volume"], -28)
        heard = self.b.wait_event("rx.packet", timeout=30)
        self.assertEqual(heard.payload(), bytes(range(63)))
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
        self.assertEqual(lines[i + 1], "# packet frame=%s crc_ok=1 length=16 data_symbols=52 min_margin=2.000 "
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
