#!/usr/bin/env python3

import csv
import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SUMMARY_CSV = os.path.join(SCRIPT_DIR, "results", "summary.csv")
METRICS = [
    "odom_stability",
    "loop_closure",
    "map_quality",
    "speed",
    "failure_recovery",
    "overall",
]


def load_rows(path: str):
    if not os.path.exists(path):
        print(f"No summary file found at {path}")
        return []

    with open(path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    cleaned = []
    for row in rows:
        try:
            for metric in METRICS:
                row[metric] = float(row[metric])
            cleaned.append(row)
        except (KeyError, ValueError):
            continue
    return cleaned


def print_best(rows, metric: str):
    best = max(rows, key=lambda row: row[metric])
    print(
        f"Best {metric}: {best['run_id']}  "
        f"score={best[metric]:.2f}  "
        f"preset={best['preset']}"
    )


def print_table(rows):
    print("\nOverall ranking:")
    print("rank  overall  odom  loop  map  speed  recover  preset  run_id")
    for index, row in enumerate(
        sorted(rows, key=lambda current: current["overall"], reverse=True),
        start=1,
    ):
        print(
            f"{index:>4}  "
            f"{row['overall']:>7.2f}  "
            f"{row['odom_stability']:>4.1f}  "
            f"{row['loop_closure']:>4.1f}  "
            f"{row['map_quality']:>3.1f}  "
            f"{row['speed']:>5.1f}  "
            f"{row['failure_recovery']:>7.1f}  "
            f"{row['preset']:<22}  "
            f"{row['run_id']}"
        )


def main():
    path = SUMMARY_CSV
    if len(sys.argv) > 1:
        path = sys.argv[1]

    rows = load_rows(path)
    if not rows:
        sys.exit(0)

    print(f"Loaded {len(rows)} scored runs from {path}\n")

    for metric in METRICS:
        print_best(rows, metric)

    print_table(rows)


if __name__ == "__main__":
    main()
