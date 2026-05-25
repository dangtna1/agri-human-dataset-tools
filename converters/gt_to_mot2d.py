"""
Convert a ground-truth annotations JSON file into MOTChallenge 2D format.

Each annotation record must have the structure::

    {
        "File": "frame_0001.png",
        "Labels": [
            {"Class": "human1", "BoundingBoxes": [x, y, w, h]},
            ...
        ]
    }

Class names with a numeric suffix (e.g. ``human3``) map to that integer as the
track ID. Names without a suffix get a stable ID derived from their SHA-1 hash.

Usage:
    python gt_to_mot2d.py \\
        --ann-json /path/to/annotations.json \\
        --out /path/to/ground_truth.txt
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Iterable, Tuple


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def _iter_labels(record: dict) -> Iterable[Tuple[str, Tuple[float, float, float, float]]]:
    """Yield ``(class_name, (x, y, w, h))`` pairs from one annotation record."""
    for label in record.get("Labels", []):
        if not isinstance(label, dict):
            continue
        cls = label.get("Class") or label.get("class") or label.get("label")
        box = label.get("BoundingBoxes") or label.get("bbox") or label.get("box")
        if cls is None or box is None or len(box) != 4:
            continue
        yield str(cls), tuple(float(v) for v in box)  # type: ignore[return-value]


def _class_to_track_id(class_name: str) -> int:
    """Return a stable integer track ID for *class_name*.

    Numeric suffixes (e.g. ``"human7"``) are used directly.  All other names
    are hashed with SHA-1 so the mapping is deterministic across runs.
    """
    digits = "".join(c for c in class_name if c.isdigit())
    if digits:
        return int(digits)
    return int(hashlib.sha1(class_name.encode()).hexdigest()[:8], 16) % 100_000


# ---------------------------------------------------------------------------
# Conversion
# ---------------------------------------------------------------------------

def convert(ann_json: Path, out: Path) -> None:
    """Write *ann_json* to MOTChallenge text format at *out*."""
    records = json.loads(ann_json.read_text(encoding="utf-8"))
    lines: list[str] = []

    for frame_idx, record in enumerate(records, start=1):
        for cls, (x, y, w, h) in _iter_labels(record):
            tid = _class_to_track_id(cls)
            lines.append(f"{frame_idx},{tid},{x:.6f},{y:.6f},{w:.6f},{h:.6f},1,-1,-1,-1\n")

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("".join(lines), encoding="utf-8")
    print(f"Wrote {len(lines)} annotations across {len(records)} frames → {out}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert an annotations JSON file to MOTChallenge 2D format.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--ann-json",
        required=True,
        type=Path,
        metavar="PATH",
        help="Annotation JSON file (list of frame records with File + Labels).",
    )
    parser.add_argument(
        "--out",
        required=True,
        type=Path,
        metavar="PATH",
        help="Output MOTChallenge .txt file.",
    )
    return parser


def main() -> None:
    args = _build_parser().parse_args()
    convert(args.ann_json, args.out)


if __name__ == "__main__":
    main()
