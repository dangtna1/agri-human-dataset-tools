"""
Convert a LiDAR annotation JSON file into a per-sequence MOT3D CSV file.

Expects an annotation JSON at ``<sequence-dir>/annotations/lidar_ann.json``
by default; override with ``--ann-json`` when your layout differs.

Each annotation record must follow the structure::

    {
        "File": "frame_0001.pcd",
        "Labels": [
            {
                "Class": "human1",
                "BoundingBoxes": [[x, y, z, dx, dy, dz, rx, ry, rz], ...]
            },
            ...
        ]
    }

All frames in the JSON are written — no split file filtering.  If you only want
a subset of frames, pre-filter the JSON before passing it to this script.

Output CSV columns:
    frame_id, track_id, x, y, z, l, w, h, yaw, score

Usage:
    python gt_to_mot3d.py \\
        --sequence-dir /data/scene_01 \\
        --out /data/gt_mot3d/scene_01.csv

    # Custom annotation path
    python gt_to_mot3d.py \\
        --ann-json /data/scene_01/lidar_labels.json \\
        --out /data/gt_mot3d/scene_01.csv
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import List, Optional


# ---------------------------------------------------------------------------
# Parsing helpers
# ---------------------------------------------------------------------------

def _class_to_track_id(class_name: str) -> int:
    """Return a stable integer track ID for *class_name*.

    Numeric suffixes (e.g. ``"human3"``) are used directly.  All other names
    are hashed with SHA-1 so the mapping is deterministic across runs.
    """
    digits = "".join(c for c in class_name if c.isdigit())
    if digits:
        return int(digits)
    return int(hashlib.sha1(class_name.encode()).hexdigest()[:8], 16) % 100_000


def _parse_box(raw: object) -> Optional[List[float]]:
    """Return ``[x, y, z, l, w, h, yaw]`` from a *BoundingBoxes* value.

    Accepts either a bare 9-element list or a list-of-lists where the first
    element is the 9-element box (the outer wrapper is ignored).
    """
    box = raw
    if (
        isinstance(box, list)
        and len(box) == 2
        and all(isinstance(item, list) and len(item) == 9 for item in box)
    ):
        box = box[0]
    if not isinstance(box, list) or len(box) != 9:
        return None
    x, y, z, dx, dy, dz, _rx, _ry, rz = (float(v) for v in box)
    return [x, y, z, dx, dy, dz, float(rz)]


# ---------------------------------------------------------------------------
# Conversion
# ---------------------------------------------------------------------------

def convert(ann_json: Path, out: Path, class_prefix: str = "human") -> None:
    """Write *ann_json* to MOT3D CSV format at *out*.

    Only labels whose ``Class`` (lower-cased) starts with *class_prefix* are
    included.  Pass ``class_prefix=""`` to include every label.
    """
    records: list = json.loads(ann_json.read_text(encoding="utf-8"))

    rows: list[str] = ["frame_id,track_id,x,y,z,l,w,h,yaw,score\n"]
    frame_id = 0
    ann_count = 0

    for record in records:
        filename = str(record.get("File", ""))
        if not filename.endswith(".pcd"):
            continue

        frame_id += 1
        for label in record.get("Labels", []):
            cls = str(label.get("Class", "")).lower()
            if class_prefix and not cls.startswith(class_prefix):
                continue
            box = _parse_box(label.get("BoundingBoxes"))
            if box is None:
                continue
            tid = _class_to_track_id(cls)
            x, y, z, l, w, h, yaw = box
            rows.append(
                f"{frame_id},{tid},{x:.4f},{y:.4f},{z:.4f},"
                f"{l:.4f},{w:.4f},{h:.4f},{yaw:.4f},1.0000\n"
            )
            ann_count += 1

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("".join(rows), encoding="utf-8")
    print(f"Wrote {ann_count} annotations across {frame_id} frames → {out}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert a LiDAR annotation JSON to MOT3D CSV format.",
        formatter_class=argparse.RawTextHelpFormatter,
    )

    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--sequence-dir",
        type=Path,
        metavar="DIR",
        help=(
            "Sequence (scene) root directory.  The annotation JSON is expected "
            "at <sequence-dir>/annotations/lidar_ann.json."
        ),
    )
    source.add_argument(
        "--ann-json",
        type=Path,
        metavar="PATH",
        help="Path to the annotation JSON file directly (bypasses --sequence-dir convention).",
    )

    parser.add_argument(
        "--out",
        required=True,
        type=Path,
        metavar="PATH",
        help="Output MOT3D CSV file.",
    )
    parser.add_argument(
        "--class-prefix",
        default="human",
        metavar="PREFIX",
        help=(
            "Only include labels whose Class (lowercased) starts with this "
            "string.  Use an empty string to include all labels. (default: human)"
        ),
    )
    return parser


def main() -> None:
    args = _build_parser().parse_args()

    if args.ann_json:
        ann_json = args.ann_json
    else:
        ann_json = args.sequence_dir / "annotations" / "lidar_ann.json"

    if not ann_json.exists():
        raise FileNotFoundError(f"Annotation file not found: {ann_json}")

    convert(ann_json, args.out, class_prefix=args.class_prefix)


if __name__ == "__main__":
    main()
