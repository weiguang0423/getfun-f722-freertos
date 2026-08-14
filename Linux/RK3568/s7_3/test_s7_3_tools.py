#!/usr/bin/env python3
"""Minimal self-check for S7.3 comparison helpers."""

import importlib.util
import json
import tempfile
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("compare_results.py")
SPEC = importlib.util.spec_from_file_location("compare_results", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def main():
    assert MODULE.flatten([[1, 2], [3]]) == [1.0, 2.0, 3.0]
    assert MODULE.percentile([0, 10], 95) == 9.5
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "one.jsonl"
        path.write_text(json.dumps({"source": "a.jpg", "frame_index": 0, "valid": False}) + "\n")
        assert MODULE.read(path)[("a.jpg", 0)]["valid"] is False
    print("self-test ok")


if __name__ == "__main__":
    main()
