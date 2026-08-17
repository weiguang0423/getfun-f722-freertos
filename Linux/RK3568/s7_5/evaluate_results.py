#!/usr/bin/env python3
"""
文件作用：评估 S7.5 固定视频或真人分段录制的 JSONL 手势结果。
核心功能：按 CSV 序号区间统计 ACTIVE 混淆矩阵、首次进入延迟和标注区间外误触发。
关键约束：只使用 Python 标准库；标签列固定为 start_sequence,end_sequence,expected_gesture。
"""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path

GESTURES = ("UNKNOWN", "OPEN_PALM", "FIST", "POINT", "V_SIGN")


def load_labels(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    labels = []
    for row in rows:
        expected = row["expected_gesture"].strip().upper()
        if expected not in GESTURES:
            raise ValueError(f"unsupported gesture: {expected}")
        start, end = int(row["start_sequence"]), int(row["end_sequence"])
        if start > end:
            raise ValueError(f"invalid sequence range: {start}..{end}")
        labels.append((start, end, expected))
    return labels


def expected_at(labels, sequence):
    matches = [expected for start, end, expected in labels if start <= sequence <= end]
    if len(matches) > 1:
        raise ValueError(f"overlapping labels at sequence {sequence}")
    return matches[0] if matches else "UNKNOWN"


def evaluate(labels, records):
    confusion = Counter()
    false_triggers = []
    for record in records:
        sequence = int(record.get("sequence", record.get("frame_index", 0)))
        expected = expected_at(labels, sequence)
        predicted = record.get("active_gesture_name", "UNKNOWN")
        if predicted not in GESTURES:
            predicted = "UNKNOWN"
        confusion[expected, predicted] += 1
        if expected == "UNKNOWN" and predicted != "UNKNOWN":
            false_triggers.append(sequence)
    delays_ms = []
    for start_sequence, end_sequence, gesture in labels:
        if gesture == "UNKNOWN":
            continue
        segment = [record for record in records
                   if start_sequence <= int(record.get("sequence", record.get("frame_index", 0))) <= end_sequence
                   and record.get("capture_timestamp_us") is not None]
        first = int(segment[0]["capture_timestamp_us"]) if segment else None
        active = next((int(record["capture_timestamp_us"]) for record in segment
                       if record.get("active_gesture_name") == gesture), None)
        delays_ms.append({
            "start_sequence": start_sequence,
            "end_sequence": end_sequence,
            "expected_gesture": gesture,
            "delay_ms": None if first is None or active is None else (active - first) / 1000.0,
        })
    return confusion, delays_ms, false_triggers


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("labels", type=Path)
    parser.add_argument("results", type=Path)
    args = parser.parse_args()
    labels = load_labels(args.labels)
    with args.results.open(encoding="utf-8") as stream:
        records = [json.loads(line) for line in stream if line.strip()]
    confusion, delays, false_triggers = evaluate(labels, records)
    summary = {
        "gestures": list(GESTURES),
        "confusion": {expected: {predicted: confusion[expected, predicted]
                                  for predicted in GESTURES}
                      for expected in GESTURES},
        "entry_delay_ms": delays,
        "false_trigger_count": len(false_triggers),
        "false_trigger_sequences": false_triggers,
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
