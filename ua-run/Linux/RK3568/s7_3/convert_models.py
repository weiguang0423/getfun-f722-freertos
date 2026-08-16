#!/usr/bin/env python3
"""Convert the frozen S7.2 TFLite hand models for RK3568."""

import argparse
import hashlib
import json
from pathlib import Path

from rknn.api import RKNN


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def convert(source, output, quantize, dataset, log_file):
    rknn = RKNN(verbose=True, verbose_file=str(log_file))
    try:
        ret = rknn.config(
            mean_values=[[0.0, 0.0, 0.0]],
            std_values=[[255.0, 255.0, 255.0]],
            target_platform="rk3568",
        )
        if ret != 0:
            raise RuntimeError("rknn.config failed: {}".format(ret))
        ret = rknn.load_tflite(model=str(source))
        if ret != 0:
            raise RuntimeError("rknn.load_tflite failed: {}".format(ret))
        ret = rknn.build(
            do_quantization=quantize,
            dataset=str(dataset) if quantize else None,
        )
        if ret != 0:
            raise RuntimeError("rknn.build failed: {}".format(ret))
        ret = rknn.export_rknn(str(output))
        if ret != 0:
            raise RuntimeError("rknn.export_rknn failed: {}".format(ret))
    finally:
        rknn.release()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--detector", type=Path, required=True)
    parser.add_argument("--landmark", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--quantize", action="store_true")
    parser.add_argument("--detector-dataset", type=Path)
    parser.add_argument("--landmark-dataset", type=Path)
    args = parser.parse_args()

    if args.quantize and (not args.detector_dataset or not args.landmark_dataset):
        parser.error("quantized conversion requires both dataset files")
    args.output.mkdir(parents=True, exist_ok=True)
    suffix = "int8" if args.quantize else "fp16"
    models = [
        ("hand_detector", args.detector, args.detector_dataset),
        ("hand_landmarks_detector", args.landmark, args.landmark_dataset),
    ]
    manifest = {
        "toolkit": "1.3.0",
        "target": "rk3568",
        "quantized": args.quantize,
        "preprocess": {"color": "RGB", "mean": [0, 0, 0], "std": [255, 255, 255]},
        "models": [],
    }
    for name, source, dataset in models:
        output = args.output / "{}_{}.rknn".format(name, suffix)
        log_file = args.output / "{}_{}_convert.log".format(name, suffix)
        convert(source.resolve(), output, args.quantize, dataset, log_file)
        manifest["models"].append(
            {
                "name": name,
                "source_sha256": sha256(source),
                "output": output.name,
                "output_bytes": output.stat().st_size,
                "output_sha256": sha256(output),
                "dataset": str(dataset) if dataset else None,
                "log": log_file.name,
            }
        )
    (args.output / "conversion_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
