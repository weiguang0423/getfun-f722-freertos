#!/usr/bin/env python3
"""Create deterministic detector lists and landmark ROI crops for RKNN INT8 calibration."""

import argparse
import csv
import sys
from pathlib import Path

import cv2

# Reuse the frozen S7.2 reference without requiring a caller-specific PYTHONPATH.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "s7_2"))
from s7_2_pc_reference import Reference, decode_detections, extract_roi, resize_pad


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--testset", type=Path, required=True)
    parser.add_argument("--detector", type=Path, required=True)
    parser.add_argument("--landmark", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--count", type=int, default=200)
    args = parser.parse_args()
    root, output = args.testset.resolve(), args.output.resolve()
    crops = output / "landmark_rois"
    crops.mkdir(parents=True, exist_ok=True)
    reference = Reference(args.detector.resolve(), args.landmark.resolve())
    with (root / "manifest.csv").open(encoding="utf-8", newline="") as stream:
        files = [root / row["file"] for row in csv.DictReader(stream)]
    # Even spacing makes the frozen subset deterministic without adding a sampler.
    selected = [files[i * len(files) // args.count] for i in range(args.count)]
    detector_lines, landmark_lines = [], []
    for path in selected:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        detector_lines.append(path.as_posix())
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        detector_input, scale, pad = resize_pad(rgb, 192)
        outputs = reference.invoke(reference.detector, detector_input)
        detections = decode_detections(outputs[0][0], outputs[1][0])
        if not detections:
            continue
        detection = detections[0]
        box = detection["bbox"].copy()
        box[[0, 2]] = (box[[0, 2]] * 192 - pad[0]) / scale / image.shape[1]
        box[[1, 3]] = (box[[1, 3]] * 192 - pad[1]) / scale / image.shape[0]
        detection["bbox"] = box
        points = detection["keypoints"].copy()
        points[:, 0] = (points[:, 0] * 192 - pad[0]) / scale / image.shape[1]
        points[:, 1] = (points[:, 1] * 192 - pad[1]) / scale / image.shape[0]
        detection["keypoints"] = points
        roi, _, _ = extract_roi(rgb, detection)
        target = crops / path.name
        cv2.imwrite(str(target), cv2.cvtColor(roi, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 100])
        landmark_lines.append(target.as_posix())
    (output / "detector_dataset.txt").write_text("\n".join(detector_lines) + "\n", encoding="utf-8")
    (output / "landmark_dataset.txt").write_text("\n".join(landmark_lines) + "\n", encoding="utf-8")
    print("detector={} landmark={}".format(len(detector_lines), len(landmark_lines)))


if __name__ == "__main__":
    main()
