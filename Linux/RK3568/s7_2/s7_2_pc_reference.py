#!/usr/bin/env python3
"""S7.2 deterministic TFLite CPU reference for the fixed hand models."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
import importlib.metadata
from pathlib import Path

import cv2
import numpy as np

try:
    from ai_edge_litert.interpreter import Interpreter
except ImportError as exc:
    raise SystemExit("missing ai-edge-litert; install it in the PC validation environment") from exc


CONFIG = {
    "detector_threshold": 0.75,
    "presence_threshold": 0.5,
    "nms_iou": 0.3,
    "mirror": False,
    "handedness_view": "model_observer_view",
    "roi_kp1": 0,
    "roi_kp2": 2,
    "roi_theta0": math.pi / 2,
    "roi_scale": 2.6,
    "roi_y_offset": -0.5,
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def tensor_contract(interpreter: Interpreter) -> dict:
    def clean(detail: dict) -> dict:
        quant = detail["quantization_parameters"]
        return {
            "index": int(detail["index"]),
            "name": detail["name"],
            "shape": detail["shape"].tolist(),
            "dtype": np.dtype(detail["dtype"]).name,
            "quantization_scales": quant["scales"].tolist(),
            "quantization_zero_points": quant["zero_points"].tolist(),
        }

    return {
        "inputs": [clean(x) for x in interpreter.get_input_details()],
        "outputs": [clean(x) for x in interpreter.get_output_details()],
    }


def resize_pad(image: np.ndarray, size: int) -> tuple[np.ndarray, float, tuple[int, int]]:
    height, width = image.shape[:2]
    scale = size / max(height, width)
    resized_width = max(1, int(width * scale))
    resized_height = max(1, int(height * scale))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    pad_left = (size - resized_width) // 2
    pad_top = (size - resized_height) // 2
    padded = np.zeros((size, size, 3), dtype=np.uint8)
    padded[pad_top : pad_top + resized_height, pad_left : pad_left + resized_width] = resized
    return padded, scale, (pad_left, pad_top)


def anchors() -> np.ndarray:
    result: list[tuple[float, float]] = []
    strides = (8, 16, 16, 16)
    layer = 0
    while layer < len(strides):
        same_stride = layer
        anchors_per_cell = 0
        while same_stride < len(strides) and strides[same_stride] == strides[layer]:
            anchors_per_cell += 2
            same_stride += 1
        grid = math.ceil(192 / strides[layer])
        for y in range(grid):
            for x in range(grid):
                result.extend(((x + 0.5) / grid, (y + 0.5) / grid) for _ in range(anchors_per_cell))
        layer = same_stride
    value = np.asarray(result, dtype=np.float32)
    assert value.shape == (2016, 2)
    return value


ANCHORS = anchors()


def decode_detections(raw_boxes: np.ndarray, raw_scores: np.ndarray) -> list[dict]:
    logits = np.clip(raw_scores.reshape(-1), -100.0, 100.0).astype(np.float64)
    scores = 1.0 / (1.0 + np.exp(-logits))
    selected = np.flatnonzero(scores >= CONFIG["detector_threshold"])
    detections = []
    for index in selected:
        raw = raw_boxes[index]
        center = raw[:2] / 192.0 + ANCHORS[index]
        size = raw[2:4] / 192.0
        keypoints = raw[4:].reshape(7, 2) / 192.0 + ANCHORS[index]
        detections.append(
            {
                "score": float(scores[index]),
                "bbox": np.array(
                    [center[0] - size[0] / 2, center[1] - size[1] / 2,
                     center[0] + size[0] / 2, center[1] + size[1] / 2],
                    dtype=np.float32,
                ),
                "keypoints": keypoints,
            }
        )
    return weighted_nms(detections)


def iou(a: np.ndarray, b: np.ndarray) -> float:
    intersection = max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0.0, min(a[3], b[3]) - max(a[1], b[1])
    )
    union = max(0.0, (a[2] - a[0]) * (a[3] - a[1])) + max(
        0.0, (b[2] - b[0]) * (b[3] - b[1])
    ) - intersection
    return intersection / union if union > 0 else 0.0


def weighted_nms(detections: list[dict]) -> list[dict]:
    pending = sorted(detections, key=lambda x: x["score"], reverse=True)
    result = []
    while pending:
        first = pending.pop(0)
        overlap = [first]
        remaining = []
        for candidate in pending:
            (overlap if iou(first["bbox"], candidate["bbox"]) > CONFIG["nms_iou"] else remaining).append(candidate)
        pending = remaining
        weights = np.asarray([x["score"] for x in overlap], dtype=np.float32)
        result.append(
            {
                "score": float(weights.mean()),
                "bbox": np.average(np.stack([x["bbox"] for x in overlap]), axis=0, weights=weights),
                "keypoints": np.average(np.stack([x["keypoints"] for x in overlap]), axis=0, weights=weights),
            }
        )
    return result


def detection_to_roi(detection: dict, image_shape: tuple[int, int, int]) -> tuple[np.ndarray, dict]:
    height, width = image_shape[:2]
    box = detection["bbox"] * np.array([width, height, width, height], dtype=np.float32)
    keypoints = detection["keypoints"] * np.array([width, height], dtype=np.float32)
    center = np.array([(box[0] + box[2]) / 2, (box[1] + box[3]) / 2], dtype=np.float32)
    box_size = float(box[2] - box[0])
    center[1] += CONFIG["roi_y_offset"] * box_size
    roi_size = box_size * CONFIG["roi_scale"]
    p0, p2 = keypoints[CONFIG["roi_kp1"]], keypoints[CONFIG["roi_kp2"]]
    theta = math.atan2(float(p0[1] - p2[1]), float(p0[0] - p2[0])) - CONFIG["roi_theta0"]
    cos_theta, sin_theta = math.cos(theta), math.sin(theta)
    unit = np.array([[-1, -1], [-1, 1], [1, -1], [1, 1]], dtype=np.float32) * (roi_size / 2)
    rotation = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]], dtype=np.float32)
    corners = unit @ rotation.T + center
    target = np.array([[0, 0], [0, 223], [223, 0]], dtype=np.float32)
    affine = cv2.getAffineTransform(corners[:3], target)
    inverse = cv2.invertAffineTransform(affine)
    return inverse, {
        "center_scale_rotation": [float(center[0]), float(center[1]), roi_size, theta],
        "corners": corners,
        "affine": affine,
    }


def extract_roi(image: np.ndarray, detection: dict) -> tuple[np.ndarray, np.ndarray, dict]:
    inverse, info = detection_to_roi(detection, image.shape)
    affine = info["affine"]
    return cv2.warpAffine(image, affine, (224, 224)), inverse, info


class Reference:
    def __init__(self, detector_model: Path, landmark_model: Path):
        self.detector_model = detector_model
        self.landmark_model = landmark_model
        self.detector = Interpreter(model_path=str(detector_model), num_threads=1)
        self.landmark = Interpreter(model_path=str(landmark_model), num_threads=1)
        self.detector.allocate_tensors()
        self.landmark.allocate_tensors()
        self.contract = {
            "detector": tensor_contract(self.detector),
            "landmark": tensor_contract(self.landmark),
        }

    @staticmethod
    def invoke(interpreter: Interpreter, image: np.ndarray) -> list[np.ndarray]:
        detail = interpreter.get_input_details()[0]
        interpreter.set_tensor(detail["index"], np.expand_dims(image.astype(np.float32) / 255.0, 0))
        interpreter.invoke()
        return [interpreter.get_tensor(x["index"]) for x in interpreter.get_output_details()]

    def run(self, image: np.ndarray, sample_hash: str, frame_index: int, source: str) -> dict:
        if image is None or image.ndim != 3 or image.shape[2] != 3 or min(image.shape[:2]) == 0:
            return rejected(sample_hash, frame_index, source, "invalid_image")
        if CONFIG["mirror"]:
            image = cv2.flip(image, 1)
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        detector_input, scale, pad = resize_pad(rgb, 192)
        detector_outputs = self.invoke(self.detector, detector_input)
        detections = decode_detections(detector_outputs[0][0], detector_outputs[1][0])
        if not detections:
            return rejected(sample_hash, frame_index, source, "no_hand")

        detection = detections[0]
        normalized = detection["bbox"].copy()
        normalized[[0, 2]] = (normalized[[0, 2]] * 192 - pad[0]) / scale / image.shape[1]
        normalized[[1, 3]] = (normalized[[1, 3]] * 192 - pad[1]) / scale / image.shape[0]
        detection["bbox"] = normalized
        keypoints = detection["keypoints"].copy()
        keypoints[:, 0] = (keypoints[:, 0] * 192 - pad[0]) / scale / image.shape[1]
        keypoints[:, 1] = (keypoints[:, 1] * 192 - pad[1]) / scale / image.shape[0]
        detection["keypoints"] = keypoints

        roi, inverse, roi_info = extract_roi(rgb, detection)
        landmark_outputs = self.invoke(self.landmark, roi)
        landmarks_roi = landmark_outputs[0][0].reshape(21, 3)
        presence = float(landmark_outputs[1].reshape(-1)[0])
        handedness = float(landmark_outputs[2].reshape(-1)[0])
        valid = presence >= CONFIG["presence_threshold"]
        xy1 = np.column_stack([landmarks_roi[:, :2], np.ones(21, dtype=np.float32)])
        landmarks_image = landmarks_roi.copy()
        landmarks_image[:, :2] = xy1 @ inverse.T
        landmarks_image[:, 0] /= image.shape[1]
        landmarks_image[:, 1] /= image.shape[0]

        return rounded(
            {
                "sample_sha256": sample_hash,
                "frame_index": frame_index,
                "source": source,
                "detector_input_shape": [1, 192, 192, 3],
                "detector_score": detection["score"],
                "bbox_xyxy_normalized": detection["bbox"].tolist(),
                "detector_keypoints_7x2": detection["keypoints"].tolist(),
                "roi_center_scale_rotation": roi_info["center_scale_rotation"],
                "landmark_input_shape": [1, 224, 224, 3],
                "presence_score": presence,
                "handedness_score": handedness,
                "landmarks_21x3_roi": landmarks_roi.tolist(),
                "landmarks_21x3_image": landmarks_image.tolist(),
                "valid": valid,
                "reject_reason": "" if valid else "low_presence",
            }
        )


def rounded(value):
    if isinstance(value, float):
        return round(value, 6)
    if isinstance(value, list):
        return [rounded(x) for x in value]
    if isinstance(value, dict):
        return {key: rounded(item) for key, item in value.items()}
    return value


def rejected(sample_hash: str, frame_index: int, source: str, reason: str) -> dict:
    return {
        "sample_sha256": sample_hash,
        "frame_index": frame_index,
        "source": source,
        "valid": False,
        "reject_reason": reason,
    }


def read_manifest(root: Path) -> list[Path]:
    result = []
    for name in ("manifest.csv", "manifest_errors.csv"):
        with (root / name).open(encoding="utf-8-sig", newline="") as stream:
            for row in csv.DictReader(stream):
                if name == "manifest.csv":
                    path = Path(row["file"])
                    candidate = root / path if path.parts[0] == row["category"] else root / row["category"] / path
                else:
                    candidate = root / row["file"]
                if not candidate.is_file():
                    matches = list((root / row["category"]).rglob(Path(row["file"]).name))
                    if len(matches) != 1:
                        raise FileNotFoundError(f"manifest path cannot be resolved uniquely: {row['file']}")
                    candidate = matches[0]
                result.append(candidate)
    listed = {path.resolve() for path in result}
    media = {path.resolve() for pattern in ("*.jpg", "*.mp4") for path in root.rglob(pattern)}
    return sorted(listed | media, key=lambda path: path.relative_to(root).as_posix())


def frames(path: Path):
    if path.suffix.lower() != ".mp4":
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            yield 0, None
        else:
            yield 0, image
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
    if index == 0:
        yield 0, None


def run(args: argparse.Namespace) -> int:
    root = args.testset.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    reference = Reference(args.detector.resolve(), args.landmark.resolve())
    files = read_manifest(root)
    records = []
    for path in files:
        file_hash = sha256(path)
        for frame_index, image in frames(path):
            records.append(reference.run(image, file_hash, frame_index, path.relative_to(root).as_posix()))

    environment = {
        "python": sys.version,
        "platform": platform.platform(),
        "numpy": np.__version__,
        "opencv": cv2.__version__,
        "litert": importlib.metadata.version("ai-edge-litert"),
        "detector_sha256": sha256(args.detector),
        "landmark_sha256": sha256(args.landmark),
        "manifest_sha256": sha256(root / "manifest.csv"),
        "manifest_errors_sha256": sha256(root / "manifest_errors.csv"),
        "config": CONFIG,
        "tensor_contract": reference.contract,
        "command": "python Linux/RK3568/s7_2/s7_2_pc_reference.py --testset <gesture_testset> --detector <hand_detector.tflite> --landmark <hand_landmarks_detector.tflite> --output <run>",
    }
    (output / "environment.json").write_text(json.dumps(environment, ensure_ascii=False, indent=2), encoding="utf-8")
    jsonl = "".join(json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n" for record in records)
    (output / "results.jsonl").write_bytes(jsonl.encode("utf-8"))
    summary = {
        "files": len(files),
        "records": len(records),
        "valid": sum(bool(x["valid"]) for x in records),
        "rejected": sum(not x["valid"] for x in records),
        "results_sha256": hashlib.sha256(jsonl.encode()).hexdigest(),
    }
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--testset", type=Path, required=True)
    parser.add_argument("--detector", type=Path, required=True)
    parser.add_argument("--landmark", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
