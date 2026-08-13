#!/usr/bin/env python3
"""Compare board RKNN JSONL with the frozen PC JSONL and summarize latency."""

import argparse
import json
import math
import statistics
from pathlib import Path


FIELDS = (
    "detector_score",
    "bbox_xyxy_normalized",
    "detector_keypoints_7x2",
    "presence_score",
    "handedness_score",
    "landmarks_21x3_image",
)


def flatten(value):
    if isinstance(value, list):
        return [item for child in value for item in flatten(child)]
    return [float(value)]


def percentile(values, percent):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * percent / 100
    low, high = math.floor(index), math.ceil(index)
    return ordered[low] if low == high else ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def read(path):
    with path.open(encoding="utf-8") as stream:
        return {(row["source"], int(row["frame_index"])): row for row in map(json.loads, stream)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pc", type=Path, required=True)
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    pc, board = read(args.pc), read(args.board)
    common = sorted(pc.keys() & board.keys())
    errors = {field: [] for field in FIELDS}
    validity_mismatches = []
    for key in common:
        if bool(pc[key]["valid"]) != bool(board[key]["valid"]):
            validity_mismatches.append(key)
        for field in FIELDS:
            if field not in pc[key] or field not in board[key]:
                continue
            left, right = flatten(pc[key][field]), flatten(board[key][field])
            if field == "landmarks_21x3_image":
                left = [value for index, value in enumerate(left) if index % 3 != 2]
                right = [value for index, value in enumerate(right) if index % 3 != 2]
            if len(left) != len(right):
                raise ValueError("shape mismatch for {} {}".format(key, field))
            errors[field].extend(abs(a - b) for a, b in zip(left, right))
    timing = {}
    for stage in ("preprocess", "detector", "postprocess", "landmark"):
        values = [row["timing_ms"][stage] for row in board.values() if stage in row.get("timing_ms", {})]
        timing[stage] = {"mean_ms": statistics.fmean(values), "p95_ms": percentile(values, 95), "p99_ms": percentile(values, 99)} if values else None
    summary = {
        "pc_records": len(pc),
        "board_records": len(board),
        "common_records": len(common),
        "validity_mismatches": len(validity_mismatches),
        "validity_agreement": 1 - len(validity_mismatches) / len(common) if common else 0,
        "errors": {
            field: {"count": len(values), "mean_abs": statistics.fmean(values), "p95_abs": percentile(values, 95), "max_abs": max(values)}
            if values else None for field, values in errors.items()
        },
        "timing": timing,
        "mismatch_samples": [list(key) for key in validity_mismatches[:100]],
    }
    args.output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
