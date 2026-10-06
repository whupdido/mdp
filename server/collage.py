"""Collect Task 1 detection frames and render one contact sheet per run."""

from __future__ import annotations

import math
import re
import threading
from pathlib import Path

import cv2
import numpy as np


def _safe_run_id(run_id: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "-", run_id).strip("-.")
    return cleaned or "task1"


def build_contact_sheet(
    entries: list[dict],
    output_path: str | Path,
    tile_width: int = 640,
    tile_height: int = 480,
) -> Path:
    """Build an ordered one-page JPEG from 1–8 annotated frames."""
    if not entries:
        raise ValueError("at least one frame is required")

    ordered = sorted(entries, key=lambda entry: entry["capture_index"])
    count = len(ordered)
    rows = 1 if count <= 4 else 2
    cols = math.ceil(count / rows)
    header_height = 36
    tiles = []

    for entry in ordered:
        image = cv2.imread(str(entry["path"]))
        if image is None:
            image = np.zeros((tile_height, tile_width, 3), dtype=np.uint8)
        else:
            image = cv2.resize(image, (tile_width, tile_height))

        header = np.full((header_height, tile_width, 3), 24, dtype=np.uint8)
        obstacle_id = entry.get("obstacle_id")
        title = f"Obstacle {obstacle_id}" if obstacle_id is not None else f"Capture {entry['capture_index']}"
        cv2.putText(
            header,
            title,
            (12, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        tiles.append(np.vstack((header, image)))

    blank = np.zeros((header_height + tile_height, tile_width, 3), dtype=np.uint8)
    while len(tiles) < rows * cols:
        tiles.append(blank.copy())

    grid_rows = []
    for row_index in range(rows):
        start = row_index * cols
        grid_rows.append(np.hstack(tiles[start:start + cols]))
    sheet = np.vstack(grid_rows)

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), sheet):
        raise OSError(f"could not write collage to {output}")
    return output


class Task1CollageCollector:
    """Accumulate frames across short-lived Pi detection connections."""

    def __init__(self, output_dir: str | Path):
        self.output_dir = Path(output_dir)
        self._sessions: dict[str, dict] = {}
        self._lock = threading.Lock()

    def record(
        self,
        *,
        run_id: str,
        expected_images: int,
        frame_path: str | Path,
        obstacle_id: int | None,
        capture_index: int,
    ) -> Path | None:
        if expected_images <= 0:
            raise ValueError("expected_images must be positive")

        with self._lock:
            session = self._sessions.setdefault(
                run_id,
                {"expected_images": expected_images, "entries": []},
            )
            session["expected_images"] = expected_images
            session["entries"].append(
                {
                    "path": str(frame_path),
                    "obstacle_id": obstacle_id,
                    "capture_index": capture_index,
                }
            )

            if len(session["entries"]) < expected_images:
                return None

            entries = session["entries"][:expected_images]
            del self._sessions[run_id]

        output = self.output_dir / f"task1_collage_{_safe_run_id(run_id)}.jpg"
        return build_contact_sheet(entries, output)
