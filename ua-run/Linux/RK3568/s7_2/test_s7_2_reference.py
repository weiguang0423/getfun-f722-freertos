#!/usr/bin/env python3
"""Small runnable check for S7.2 preprocessing and postprocessing math."""

import numpy as np

from s7_2_pc_reference import decode_detections, detection_to_roi, iou, resize_pad


def main() -> None:
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    padded, scale, pad = resize_pad(image, 192)
    assert padded.shape == (192, 192, 3) and scale == 0.96 and pad == (0, 48)

    assert iou(np.array([0, 0, 1, 1]), np.array([0, 0, 1, 1])) == 1.0
    boxes = np.zeros((2016, 18), dtype=np.float32)
    scores = np.full((2016, 1), -100.0, dtype=np.float32)
    scores[0] = 100.0
    detection = decode_detections(boxes, scores)[0]
    assert len(detection["keypoints"]) == 7

    detection["bbox"] = np.array([0.25, 0.25, 0.75, 0.75], dtype=np.float32)
    detection["keypoints"] = np.array([[0.4, 0.5]] * 7, dtype=np.float32)
    detection["keypoints"][2] = [0.6, 0.5]
    inverse, info = detection_to_roi(detection, image.shape)
    affine = info["affine"]
    point = np.array([[[100.0, 50.0]]], dtype=np.float32)
    roundtrip = np.concatenate([point[0], np.ones((1, 1), dtype=np.float32)], axis=1) @ affine.T
    roundtrip = np.concatenate([roundtrip, np.ones((1, 1), dtype=np.float32)], axis=1) @ inverse.T
    assert np.allclose(roundtrip, point[0], atol=1e-4)
    print("S7_2_REFERENCE_SELF_CHECK_PASS")


if __name__ == "__main__":
    main()
