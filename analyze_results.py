#!/usr/bin/env python3
"""Aggregate LoRa mesh simulator CSV output by protocol."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path
from statistics import mean, stdev
from typing import Dict, List


METRICS = [
    # Baseline Metrics
    "unicast_pdr",
    "broadcast_coverage",
    "avg_delay_s",
    "tx_count",
    "data_tx",
    "control_tx",
    "total_airtime_s",
    "airtime_per_delivery_s",
    "collision_fail",
    "duplicate_rx",
    "suppressed_forwards",
    "route_cache_hits",
    "route_cache_misses",
    # Extended Research Metrics
    "avg_distance_m",
    "avg_rssi_dbm",
    "min_rssi_dbm",
    "avg_snr_db",
    "min_snr_db",
    "avg_sinr_db",
    "min_sinr_db",
    "obstacle_loss_events",
    "interference_fail_count",
    "node_failures_count",
    "node_recoveries_count",
    "energy_depletions_count",
    "total_energy_consumed_j",
    "avg_remaining_energy_j",
    "min_remaining_energy_j",
    "energy_per_delivery_j",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Aggregate LoRa mesh simulator CSV")
    parser.add_argument("csv_path", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    grouped: Dict[str, List[dict]] = defaultdict(list)
    
    # Use utf-8-sig to automatically handle Excel BOM headers
    with args.csv_path.open(newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            grouped[row["protocol"]].append(row)

    for protocol, rows in grouped.items():
        print(f"\n{protocol}  n={len(rows)}")
        print("-" * (len(protocol) + 6 + len(str(len(rows)))))
        for metric in METRICS:
            # Safely extract metric values if they exist in the CSV row
            values = [
                float(row[metric])
                for row in rows
                if metric in row and row[metric] != ""
            ]
            if not values:
                continue
            
            mu = mean(values)
            if len(values) >= 2:
                sd = stdev(values)
                print(f"{metric:26s} mean={mu:.6f}  sd={sd:.6f}")
            else:
                print(f"{metric:26s} mean={mu:.6f}")


if __name__ == "__main__":
    main()