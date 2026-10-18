#!/usr/bin/env python3
"""Per-transmitted-symbol statistics from a chirp receiver CSV log.

Frames are grouped by tx_symbol. The first few frames after each change of
tx_symbol are skipped (the acoustic path and receive window still contain the
previous symbol). Uses only the Python standard library.

Columns of the table (per transmitted symbol):
  n           frames used
  ok%         frames decoded as exactly the transmitted symbol
  label       most common (rx_symbol - tx_symbol) mod symbols, and its share
  offset      position of the decoded peak relative to the transmitted symbol,
              in samples (mean, standard deviation, min, max); 16 samples = 1 symbol
  error       decoder error_angle converted to samples (relative to the
              nearest symbol, what decision tracking sees): mean and std
  mag         centroid magnitude: mean and min

Example:
  python3 tools/analyse_log.py sweep.csv
"""

import argparse
import csv
import math
import statistics
import sys
from collections import Counter, defaultdict


def read_log(path):
    config = {}
    rows = []
    with open(path) as f:
        data_lines = []
        for line in f:
            if line.startswith("#"):
                for item in line[1:].split():
                    if "=" in item:
                        key, value = item.split("=", 1)
                        config[key] = value
            elif line.strip():
                data_lines.append(line)
        rows = list(csv.DictReader(data_lines))
    return config, rows


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def stats(values):
    if not values:
        return (float("nan"),) * 4
    sd = statistics.pstdev(values) if len(values) > 1 else 0.0
    return statistics.fmean(values), sd, min(values), max(values)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="CSV log from capture_log.py")
    parser.add_argument("--skip", type=int, default=4,
                        help="frames to skip after each tx_symbol change (default 4)")
    args = parser.parse_args()

    config, rows = read_log(args.log)
    if not rows:
        sys.exit(f"{args.log}: no data rows")

    fft_size = int(config.get("fft_size", 1024))
    symbols = int(config.get("symbols", 64))
    samples_per_radian = fft_size / (2.0 * math.pi)
    radians_per_symbol = 2.0 * math.pi / symbols

    groups = defaultdict(list)
    previous_tx = None
    run = 0
    for row in rows:
        tx = int(row["tx_symbol"])
        run = run + 1 if tx == previous_tx else 0
        previous_tx = tx
        if run >= args.skip:
            groups[tx].append(row)

    modes = Counter(r["mode"] for r in rows)
    tracking = Counter(r["tracking"] for r in rows)
    delays = [int(r["delay"]) for r in rows]
    print(f"{args.log}: {len(rows)} frames; mode {dict(modes)}; tracking {dict(tracking)}; "
          f"delay {min(delays)}..{max(delays)}")
    print(f"skipping {args.skip} frames after each symbol change; {fft_size // symbols} samples = 1 symbol\n")

    print(f"{'tx':>3} {'n':>4} {'ok%':>5}  {'label':>9}  "
          f"{'offset mean':>11} {'sd':>6} {'min':>7} {'max':>7}  "
          f"{'error mean':>10} {'sd':>6}  {'mag mean':>8} {'min':>6}")

    all_offsets = []
    for tx in sorted(groups):
        g = groups[tx]
        rx = [int(r["rx_symbol"]) for r in g]
        ok = 100.0 * sum(1 for s in rx if s == tx) / len(g)
        label, count = Counter((s - tx) % symbols for s in rx).most_common(1)[0]
        offsets = [wrap_pi(float(r["angle"]) - tx * radians_per_symbol) * samples_per_radian for r in g]
        errors = [float(r["error_angle"]) * samples_per_radian for r in g]
        mags = [float(r["magnitude"]) for r in g]
        all_offsets.extend(offsets)

        o_mean, o_sd, o_min, o_max = stats(offsets)
        e_mean, e_sd, _, _ = stats(errors)
        m_mean, _, m_min, _ = stats(mags)
        print(f"{tx:>3} {len(g):>4} {ok:>5.1f}  {label:>+3d} {100.0 * count / len(g):>4.0f}%  "
              f"{o_mean:>+11.2f} {o_sd:>6.2f} {o_min:>+7.2f} {o_max:>+7.2f}  "
              f"{e_mean:>+10.2f} {e_sd:>6.2f}  {m_mean:>8.3f} {m_min:>6.3f}")

    if all_offsets:
        mean, sd, lo, hi = stats(all_offsets)
        means = [statistics.fmean(
            [wrap_pi(float(r["angle"]) - tx * radians_per_symbol) * samples_per_radian for r in groups[tx]])
            for tx in groups]
        print(f"\nall symbols: offset mean {mean:+.2f} sd {sd:.2f} (range {lo:+.2f}..{hi:+.2f}) samples; "
              f"spread of per-symbol means {min(means):+.2f}..{max(means):+.2f} samples")


if __name__ == "__main__":
    main()
