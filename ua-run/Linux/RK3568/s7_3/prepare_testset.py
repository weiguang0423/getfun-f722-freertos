#!/usr/bin/env python3
"""Freeze every S7.2 image/video frame as a board-decodable JPEG corpus."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def media(root):
    return sorted(
        (path for path in root.rglob("*") if path.suffix.lower() in (".jpg", ".mp4")),
        key=lambda path: path.relative_to(root).as_posix(),
    )


def frames(path):
    if path.suffix.lower() == ".jpg":
        yield 0, cv2.imread(str(path), cv2.IMREAD_COLOR)
        return
    capture = cv2.VideoCapture(str(path))
    index = 0
    while capture.isOpened():
        ok, frame = capture.read()
        if not ok:
            break
        yield index, frame
        index += 1
    capture.release()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--testset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root, output = args.testset.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for path in media(root):
        original_hash = sha256(path)
        for frame_index, frame in frames(path):
            if frame is None:
                raise RuntimeError("cannot decode {} frame {}".format(path, frame_index))
            name = "{:06d}.jpg".format(len(rows))
            target = output / name
            if not cv2.imwrite(str(target), frame, [cv2.IMWRITE_JPEG_QUALITY, 100]):
                raise RuntimeError("cannot write {}".format(target))
            rows.append(
                {
                    "file": name,
                    "source": path.relative_to(root).as_posix(),
                    "frame_index": frame_index,
                    "source_sha256": original_hash,
                    "file_sha256": sha256(target),
                }
            )
    with (output / "manifest.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "records": len(rows),
        "manifest_sha256": sha256(output / "manifest.csv"),
        "jpeg_quality": 100,
        "source_root": str(root),
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
