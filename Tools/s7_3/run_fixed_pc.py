#!/usr/bin/env python3
"""Run the frozen S7.2 reference on the byte-identical board JPEG corpus."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2

from s7_2_pc_reference import Reference


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--testset", type=Path, required=True)
    parser.add_argument("--detector", type=Path, required=True)
    parser.add_argument("--landmark", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root, output = args.testset.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    reference = Reference(args.detector.resolve(), args.landmark.resolve())
    records = []
    with (root / "manifest.csv").open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            path = root / row["file"]
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            records.append(reference.run(image, row["file_sha256"], 0, row["file"]))
    payload = "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in records)
    (output / "results.jsonl").write_text(payload, encoding="utf-8", newline="\n")
    summary = {
        "records": len(records),
        "valid": sum(bool(row["valid"]) for row in records),
        "rejected": sum(not row["valid"] for row in records),
        "results_sha256": hashlib.sha256(payload.encode()).hexdigest(),
        "fixed_manifest_sha256": sha256(root / "manifest.csv"),
        "detector_sha256": sha256(args.detector),
        "landmark_sha256": sha256(args.landmark),
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
