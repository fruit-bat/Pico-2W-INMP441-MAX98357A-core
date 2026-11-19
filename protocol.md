# Host protocol — DRAFT for review

Status: the engine (`src/protocol.c`) runs on the PC against a pretend
board (`tests/fake_board.c`). `tests/test_protocol.c` has the scenarios.
The Python client (`tools/chirp_host.py`) talks to the same engine
through `tests/build/protocol_sim`, and to the boards: since firmware
0.2.0 the protocol is the board's only interface over USB serial.
Numbers in *italics* are guesses.

The board talks to a host (a PC or another microcontroller) over a byte
stream: USB serial today, perhaps a UART later. This replaces the key
presses, the full-screen display and the CSV log; everything they give us
today must still be available (see the inventory at the end).

## Goals

- Easy to read and type: text lines, like SMTP or IMAP, not binary or JSON.
- Events (packets received, signal lost, spectra) can arrive at any time,
  including between a command and its reply. Every line says what it is,
  so the host never has to guess.
- One table of settings: the protocol, the help text and the tests all
  come from it.
- Lean enough for a slow UART: the host chooses what it is sent, and how
  often.
- Board side: no malloc; all of it runs on core 0; core 1 is unchanged
  apart from passing a few more events across.

## Lines

- **Every line, both ways, starts with a check:** four hex digits and a
  space, then the line itself:
  ```
  38E8 k1 tx.volume=-28
  ```
  The four digits are the CRC-16/CCITT (poly 0x1021, init 0xFFFF, the same
  CRC as the packets) of everything after the space, without the LF (or
  CR). The board sends upper case and accepts either. The check is always
  there, on every link; see Link errors for why. **The examples in this
  document leave it out, for readability.** People typing use the PC
  client, which adds and removes it.
- ASCII text. Each line ends with LF (`\n`). The board accepts and ignores
  a CR before the LF, and sends LF only.
- Words are separated by single spaces (the board forgives runs of
  spaces). Arguments are `name=value`, and a value contains no spaces.
- **Free text** (spaces allowed) is only ever the last thing on a line:
  the text after an `err` code, and `help=` in `list` and `help` replies.
  A parser can take the rest of the line once it reaches it.
- Printable ASCII only: a line with tabs or other control characters, or
  with bytes over 0x7E, is rejected (`err 500`).
- Empty lines (or only spaces) are ignored, and need no check.
- Maximum line length 1024 characters with the check from the host. That
  is enough for a 255-byte packet in hex (510 characters) plus the rest of
  the line. Lines from the board can be up to 6 KB: a 512-bin spectrum as
  raw power is about 5.2 KB. A host with little memory just doesn't
  subscribe to the long ones. A longer line can't be checked: it is thrown away up to its LF and
  answered with the event `* err seq=<n> 500 line too long`.
- Values:

  | Kind | Written as | Example |
  |---|---|---|
  | integer | decimal | `-28` |
  | real number | decimal with a `.` | `0.851` |
  | choice | a lower-case word | `packet`, `on`, `off` |
  | bytes | hex, lower case (either case accepted) | `48656c6c6f` |
  | list | numbers separated by commas | `-61.2,-58.0,...` |

## Three kinds of line

| Sent by | Line starts with | Meaning |
|---|---|---|
| host | a tag | A command: `<tag> <command> [name=value ...]` |
| board | the same tag | Part of that command's reply. The last line is `<tag> ok [...]` or `<tag> err <code> <text>` |
| board | `*` | An event: `* <event> seq=<n> [name=value ...]`. It is never a reply to a command. |

A tag is 1–8 letters and digits, chosen by the host (e.g. `a1`, `7`,
`x42`). The board doesn't check or remember tags, so the host may reuse
them. If a line has no valid tag, the board can't address a reply to
it, so it sends `* err seq=<n> 500 line has no valid tag`. That is the
one event caused directly by the host.

### Ordering rules (the board promises these)

1. **Lines are whole.** Only one place in the code writes to the port,
   one complete line at a time. A line half-sent when a command arrives is
   finished first.
2. **A reply and the events around it come in no fixed order.** Events
   waiting to be sent may go before or after a reply; the host tells them
   apart by the first word.
3. **A command's reply lines stay together.** No event comes between the
   first and last line of one reply.
4. **A reply comes before anything it causes.** `a7 ok id=12` always comes
   before `* tx.done id=12`.
5. **Commands are answered in the order sent,** each within *100 ms*.
   Nothing waits for the radio: `send` replies when the packet has been
   queued, not when it has been sent.

So a host is a loop that reads a line and acts on its first word: `*` is
an event, and anything else is a reply to that tag.

## Commands

| Command | Arguments | Reply data | Notes |
|---|---|---|---|
| `hello` | — | `proto=1 version=... id=...` | id: the Pico's unique id (useful with several boards) |
| `get` | names (none = all) | one `name=value` line per setting/status | wildcards: `get rx.*` |
| `set` | `name=value ...` | other settings it changed, on the `ok` line | all or nothing: one bad value rejects the whole command. E.g. `a8 set link.spacing=4` → `a8 ok test.symbol=8` (the symbol wrapped) |
| `nudge` | `samples=<n>` (−1023 .. 1023) | `ok delay=<new>` | moves the receive window; positive = later (more delay). Today keys `z` `x`. Relative, so not safe to repeat blindly (see Link errors) |
| `list` | names (none = all) | one line per item: name, kind, type, range, units, help | for clients and humans; generated from the table |
| `send` | `hex=<bytes>` | `ok id=<n>` | queues a packet; later `* tx.start id=` and `* tx.done id=` |
| `sub` | `stream=<name> [every=<n>] [fields=a,b,...] [units=db\|power] [from=<bin> to=<bin>]`; or nothing | with no arguments: one line per subscription in force | start or replace a stream (see Streams) |
| `unsub` | `stream=<name>` or `all` | — | safe to repeat: unsubscribing what isn't subscribed is fine |
| `reset` | `counters` / `rx` / `settings` | — | clear counters, abandon the packet being received, restore the built-in default settings (the saved ones are not touched) |
| `save` | — | — | save the current settings; used at start-up. *Until flash storage exists: `err 452`* |
| `load` | — | — | go back to the saved settings (the defaults if none are saved) |
| `help` | — | one line per command | |

Example:

```
host:   a1 hello
board:  a1 proto=1 version=0.9.0 id=e6614c311b2a4f2c
board:  a1 ok
host:   a2 set tx.volume=-31 link.spacing=2
board:  a2 ok
host:   a3 set tx.volume=-60
board:  a3 err 503 tx.volume out of range -55..-1
host:   a4 send hex=6869
board:  * rx.packet seq=40 frame=1803 crc=ok len=16 min_margin=2.310 weakest=3 weak=0 hex=...
board:  a4 ok id=7
board:  * tx.start seq=41 frame=1811 id=7
board:  * tx.done seq=42 frame=1837 id=7 result=sent
```

## Settings and status

Names are dotted and lower case. *Settings* can be read and set; *status*
items can only be read. Out-of-range values are rejected, never clamped.

### Settings

| Name | Values | Default | Today |
|---|---|---|---|
| `link.spacing` | 1, 2, 4 bins per symbol | 4 | key `b` |
| `tx.volume` | −55 .. −1 dBFS (integer) | −28 | keys `[` `]` (3 dB steps) |
| `rx.align` | on, off | on | key `a` |
| `rx.tracking` | off, decision, pilot | off | key `t` |
| `rx.during_tx` | on, off | on | compile-time `PACKET_RX_WHILE_TRANSMITTING` |
| `test.mode` | packet, sequence, chirp, tone, mic, loopback | packet | keys `1`–`6` |
| `test.symbol` | 0 .. symbols−1 | 0 | keys `o` `p` (chirp mode) |

Only the host changes settings. When one `set` changes another setting as
a side effect, the `ok` line lists it. (See Decisions for the parked
`* setting` event.)

### Status (read only)

| Name | Meaning |
|---|---|
| `info.sample_rate`, `info.fft_size`, `info.bins`, `info.f0`, `info.bandwidth` | fixed constants (today: the log header) |
| `info.track_alpha` ... `info.highpass_hz` | other constants from the log header |
| `link.symbols`, `link.bits` | follow from `link.spacing` |
| `count.frames` | frames processed |
| `count.rx_missed`, `count.tx_missed` | audio blocks core 1 was late for |
| `count.records_dropped` | per-frame records lost between the cores |
| `count.good`, `count.bad`, `count.header_failed`, `count.lost`, `count.rx_dropped` | packets received (today: the display's Packets line) |
| `count.events_dropped` | events thrown away because the host wasn't reading (see Flow) |
| `count.link_errors` | lines from the host that failed their check (see Link errors) |
| `count.stream_dropped` | stream lines dropped to keep room for replies and events (see Flow) |
| `tx.queued` | packets waiting to be sent |
| `rx.state` | search, lock, train, header, payload |
| `rx.delay` | receive window delay, 0 .. 1023 samples (start 960). Moved by alignment, tracking and `nudge` |
| `rx.last_lock.*` | `align`, `fraction`, `moved` from the last lock |
| `rx.last_train.*` | `frame`, `coherence`, `strongest`, `earliest`, `moved` from the last training |

## Events

Each event is sent when something happens. None are sent on a schedule;
schedules are what Streams are for.

Every event's first field is `seq=<n>`: 0 for `* hello` at start-up, then
one more for each event the board makes, including any it has to drop. A
gap in `seq` therefore means lines were lost, either dropped because the
host wasn't reading (and then `* overflow` says how many) or damaged on
the way. A `seq=0` means the board has restarted. The fields shown below
come after `seq`.

| Event | Fields | Today |
|---|---|---|
| `* hello` | as the `hello` reply | start-up banner; also tells the host the board has restarted |
| `* rx.locked` | `frame align fraction moved=<samples>` | display line "last lock" |
| `* rx.trained` | `frame b_start coherence strongest earliest moved=<samples>` | display; log `packet_event` 2. `moved`: the window moved back because the lock's move was wrong (packet_format.md), else 0 |
| `* rx.train_failed` | `frame` | display "failed" count |
| `* rx.tone` | `frame` | (not shown today) |
| `* rx.header_failed` | `frame length` | `# packet header failed` |
| `* rx.lost` | `frame symbols reason=signal\|spacing\|reset\|transmit` | `# packet lost`. A packet given up part-way: the signal went, `link.spacing` changed, `reset rx`, or this board started sending with `rx.during_tx=off` |
| `* rx.packet` | `frame crc=ok\|bad len min_margin weakest weak hex` | `# packet ...` |
| `* tx.start` | `frame id` | (new) |
| `* tx.done` | `frame id result=sent\|abandoned` | (new). `abandoned`: cut short, e.g. by a `link.spacing` change, so a host waiting on that id always hears how it ended |
| `* overflow` | `dropped=<n>` | (new) see Flow |
| `* err` | `500 <text>` | (new) a line from the host that failed its check, was too long, or had no valid tag (see Link errors) |

Every `rx.locked` ends in exactly one of `rx.tone`, `rx.train_failed`,
`rx.header_failed`, `rx.lost` or `rx.packet`, so a host following a packet
always hears how it ended.

`rx.packet` is sent for bad packets too. The payload is often still
useful for diagnosis, and the host can decide what to do with it.

## Streams (subscriptions)

A stream is a regular event, one per frame (every 23.2 ms), that the host
must ask for with `sub`. A second `sub` for the same stream replaces the
first, and a rejected one leaves it as it was. `sub` on its own lists what
is in force:

```
host:   a1 sub stream=frame fields=delay,rx_symbol every=4
board:  a1 ok
host:   a2 sub stream=spectrum from=10 to=19 units=power
board:  a2 ok
host:   a3 sub
board:  a3 stream=frame every=4 fields=delay,rx_symbol
board:  a3 stream=spectrum every=1 units=power from=10 to=19
board:  a3 ok
board:  * frame seq=12 frame=4005 delay=960 rx_symbol=3
board:  * spectrum seq=13 frame=4005 units=power from=10 values=1.375e-04,1.500e-04,...
```

| Stream | Content | Arguments | Size per line (about) | Today |
|---|---|---|---|---|
| `frame` | `tx_symbol delay rx_symbol angle error_angle magnitude strength peak timing_error timing_rate state corr data_symbol data_margin data_quality rx_missed tx_missed records_dropped` | `every`, `fields` (default `all`) | 330 B | log columns other than b, f, s |
| `folded` | 64 values: what the decoder decides from | `every`, `units` | 350 B (dB) | log `f0..f63`; folded display view (`f`) |
| `band` | 128 values: bins −64..63 of the de-chirped FFT | `every`, `units` | 700 B (dB) | log `b-64..b63` |
| `shape` | 64 values: the learned channel shape | `units`; sent once on the next frame after subscribing (the current shape), then only when a new one is learned, so no `every` | 350 B (dB) | log `s0..s63` |
| `spectrum` | the de-chirped FFT, bins `from`..`to` (default 0..511, at most 512 of 0..1023); core 1 copies a frame's spectrum only when core 0 has finished with the last, so it may skip frames (on the boards it kept up with every frame) | `every`, `units`, `from`, `to` | 3 KB (512 in dB), 5.2 KB (512 as power) | the paged display (`c` `v`) |

Every stream line is `* <stream> seq=<n> frame=<f> ...`. The frame number
always comes first, so the host can line streams up with each other and
with events, much like the current CSV, and see which frames are missing.
`every=n` (1..1000) sends the next frame and then one in n.

Spectrum streams (`folded`, `band`, `shape`, `spectrum`) take
`units=db` (the default) or `units=power`, and say which on every line
(`units=db values=...`):
- `db` gives 10 log10 of the power to one decimal place, e.g. `-61.2`, or
  `-200.0` for nothing at all. It is short, and it is what we plot. The
  board formats it without printf, because a spectrum is hundreds of
  values a frame.
- `power` gives the raw value as the log writes it now (`%.3e`), for the
  offline analysis.

The format belongs to the subscription, so it can never change under a
stream already flowing, and different streams can use different formats.

**Bandwidth.** Each event carries its frame number, so the host can line
up streams again, much like the current CSV.

| Link | Bytes/s | Per frame (43/s) | Enough for |
|---|---|---|---|
| UART 115200 | 11.5 KB | 270 B | `frame` plus events |
| UART 921600 | 92 KB | 2.1 KB | `frame`, `folded`, `band`, `shape` |
| USB serial (today's log: about 3 KB/frame) | ~130 KB used | ~3 KB | everything |

## Flow: when the host doesn't read

Output goes through one fixed-size queue on core 0 (*24 KB*); one reply is
at most *6 KB* with its checks (`list` of everything is about 4 KB). The
protocol's state comes to 37 KB of RAM on core 0 in all. If the queue fills:

1. Stream lines are dropped first. A stream line is only queued if it
   leaves room for a whole reply and *2 KB* of events, so streams never
   delay a reply or crowd out an event. Dropped stream lines are counted
   in `count.stream_dropped` and show as missing frame numbers. They
   don't use up sequence numbers, so a gap in `seq` still means lost
   events and nothing else. While an overflow is waiting to be reported,
   stream lines are dropped too.
2. Then other events, which are counted. When there is room again (for
   the notice and the longest event after it), the board sends one
   `* overflow dropped=<n>`, and `count.events_dropped` keeps the total.
   Once events are being dropped, every later one is dropped until that
   notice has gone, so nothing slips through out of order.
3. Replies are never dropped. A command is only run when there is room for
   the longest possible reply (6 KB). Until then it waits, untouched, and
   the board takes in no more input, so a host that sends faster than it
   reads is slowed down rather than losing replies.

(`seq` makes this checkable: the dropped events used up sequence numbers,
so the gap just before `* overflow` matches its `dropped=`.)

Core 1 is never held up by any of this. It already drops records when its
queue to core 0 is full, and counts them (`count.records_dropped`).

## Link errors

Over USB the link is reliable: USB puts a CRC on every packet and retries
failures in hardware. A UART is not reliable. The usual faults are lost
bytes (overruns, where one side didn't read in time) and rubbish at
power-up or when a board resets in the middle of a line.

The usual mess comes from adding error detection to a protocol that was
first used on a reliable link. Hosts written without it break when moved
to a worse link, and an optional checksum makes lines ambiguous. So this
protocol has the check on every line, on every link, from the start.

1. **Every line is checked** (see Lines). It costs 5 bytes a line and
   about 1% of core 0 at the busiest (table-driven CRC). A microcontroller
   host needs about 15 lines of C for it. A line from the host that fails
   (no check, a wrong check, no space after it, or too long to check) is
   thrown away and counted in `count.link_errors`. Nothing on it can be
   trusted, not even its tag, so the board answers with an event, not a
   reply: `* err seq=<n> 500 line failed its check`. Someone typing
   without the client therefore sees why nothing happened, and power-up
   rubbish shows as one or two such events.
2. **Lines re-synchronise.** After any amount of rubbish, the next LF
   starts clean, so damage stays within one line.
3. **Recovery is the host's job, using the protocol's own tools:** tags
   and time limits show a reply that never came; `seq` shows a missing
   event (and `seq=0` a restart); almost everything can be read again with
   `get`. Only the host knows whether a command is safe to repeat, so
   nothing retransmits at the link level (no XMODEM-style acknowledgement
   of each line). That would duplicate tags and time limits, the board
   would have to keep state for every line, and nobody acknowledges
   events. Commands are kept safe to repeat:
   - `get`, `set`, `list`, `reset`, `save`, `load` and `hello` already are.
   - `nudge` is not, because it is relative. Repeating it moves the
     window twice. It is kept as it is: it is a diagnostic, mostly used
     by hand. A host retrying it should first read `rx.delay` to see
     whether the first one happened. It may be removed one day.
   - `send` is not: a repeat would transmit the packet twice. *Not built
     yet:* an optional host-chosen `ref=<word>`. The board remembers the
     last few refs and, on a repeat, replies with the original
     `ok id=<n>` without queuing again.

## Error codes

These follow the SMTP pattern: 4xx means it may work if you try again
later; 5xx means the command is wrong.

| Code | Meaning |
|---|---|
| 450 | transmit queue full |
| 451 | busy (e.g. spacing change still being applied) |
| 452 | not available yet (e.g. `save` before flash storage exists) |
| 500 | line not understood: bad characters (as a reply); failed its check, too long or no valid tag (as a `* err` event, since there is no trustworthy tag to reply to) |
| 501 | unknown command |
| 502 | unknown setting or status name |
| 503 | value out of range or of the wrong type |
| 504 | read only |
| 505 | missing or extra argument |
| 506 | bad hex (odd length, not hex, too long for a packet) |
| 507 | unknown stream or field |

The text after the code is for people. Clients act on the code.

## Testing on the PC first

The protocol code (line parser, command and settings table, output queue)
is plain C with no Pico calls. Below it sits a narrow board interface:
read a setting, write one, queue a packet, poll for events. That interface
has two implementations:

- **A fake for the PC.** It gives plausible, made-up data on a simulated
  clock (a test says "advance 23.2 ms" and a frame happens), so runs are
  repeatable and fast. It can inject events at awkward moments.
- **The real one on the Pico,** wired to main.c.

Scenario tests run in ctest like the others, for example:
- an event arrives in the middle of a reply;
- a burst of stream events sits in front of a reply;
- the output queue is full (overflow, and replies are still delivered);
- the signal is lost mid-packet;
- input problems: a line that is too long, an unknown command, a value out
  of range, a reused tag, CR+LF line ends, an empty line, bytes that aren't
  ASCII.

Then the Python client, `tools/chirp_host.py`, runs against the PC build
through a pipe. `tests/build/protocol_sim` is the engine and the fake
board as a program, on a real-time clock (`--speed x` runs it faster).
`tests/test_client.py` drives it end to end, in ctest. The same client
will talk to a real board over USB serial:

```
python3 tools/chirp_host.py console --sim tests/build/protocol_sim
python3 tools/chirp_host.py console --port /dev/ttyACM0
```

The console adds the tag and the check to what you type, and prints
replies and events as they come (stream lines only with `--streams`). A
line starting with `!` is sent as typed, with its check but no tag added.

`chirp_host.py log` (and the `CsvLog` class) rebuilds the old CSV log
from the `frame`, `band`, `folded` and `shape` streams and the events, with
the same columns and formats, so `analyse_log.py` and older scripts read it
unchanged.

## Inventory: today's diagnostics and where they go

| Today | Becomes |
|---|---|
| Log header comments (`sample_rate`, `fft_size`, track and high-pass constants) | `info.*` status |
| Log columns `frame, mode, tx_symbol, delay, tracking, rx_symbol, angle, error_angle, magnitude, strength, peak, timing_error, timing_rate` | `frame` stream (`mode`, `tracking` are settings) |
| `rx_missed, tx_missed, dropped` | `count.*` status |
| `b-64..b63` | `band` stream |
| `f0..f63` | `folded` stream |
| `packet_state, packet_corr` | `frame` stream (`state`, `corr`) |
| `packet_event, packet_coherence, packet_b_start, packet_strongest, packet_earliest` | events `rx.locked`, `rx.trained`, `rx.train_failed`, `rx.tone` |
| `s0..s63` | `shape` stream (on change) |
| `packet_align, packet_align_fraction` | `rx.locked` event; `rx.last_lock.*` |
| `packet_align_on` | `rx.align` setting |
| `packet_data_symbol, packet_data_margin, packet_data_quality` | `frame` stream |
| `bins_per_symbol, tx_volume_db` | settings |
| `# packet ...`, `# packet header failed`, `# packet lost` lines | `rx.packet`, `rx.header_failed`, `rx.lost` events |
| Display: spectrum pages, folded view | `spectrum` and `folded` streams, drawn by the client |
| Display: mode/spacing/volume/symbol/delay/peak line | settings and the `frame` stream |
| Display: RX symbol, centroid, error, strength, symbol bar | `frame` stream |
| Display: tracking line | `rx.tracking`, `frame` stream (`timing_error`, `timing_rate`, `magnitude`) |
| Display: Packet RX line (found/failed counts, last B) | events; `rx.last_train.*` |
| Display: packet counters and last payload | `count.*`; `rx.packet` |
| Display: frames, missed, dropped | `count.*` |
| Keys `1`–`6`, `b`, `[` `]`, `o` `p`, `t`, `a` | `set` (see the Settings table) |
| Keys `z` `x` (window delay) | `nudge`; `rx.delay` status |
| Keys `s` `S` (test packets) | `send`; the client builds the test packets |
| Keys `c` `v` `f` (display pages, folded view) | client side |
| Key `l` (log on/off) | `sub` / `unsub` |
| `[MODE]`, `[SPACING]`, `[VOLUME]`, `[PACKET] Queued/Queue full` messages | `set` and `send` replies (`ok id=`, `err 450`) |

New things the protocol adds:
- `tx.start`/`tx.done` events;
- `tx.queued`;
- `count.events_dropped`;
- `rx.during_tx` as a run-time setting;
- the board's unique id.

## Decisions

Decided 2026-11-18:
- **Keys and display:** the protocol owns the USB port. The keys and the
  full-screen display go, and a PC client draws the display.
- **`send`:** hex only. The client converts text.
- **Saved settings:** yes. There are `save`, `load` and `reset settings`
  commands; the flash storage comes later.
- **Spectrum format:** chosen per subscription (`units=db` or `power`),
  not by a setting. The format then always belongs to the stream it was
  asked for, and two streams can differ.
- **`rx.delay` is read-only status.** The board moves the window itself
  (alignment on each lock, and tracking on every frame), so it is state,
  not a setting. The host moves it with `nudge`.
- **No `* setting` event for now.** With the keys gone, only the board's
  own operation changes anything. Side effects of a `set` are listed in
  its reply.

Parked, in case testing on the real boards shows we need it:
- **`* setting name=value`**, sent whenever a setting changes without the
  host asking, with `rx.delay` a setting that announces its own moves.
  We dropped it because those moves could come 43 times a second, and
  nothing else would trigger it. It comes back if settings ever start
  changing by other routes (another board over the air, a second host, a
  button), or if following the window in the `frame` stream turns out to
  be awkward.
