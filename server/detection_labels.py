"""Formatting and drawing helpers for image-recognition annotations."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import cv2
import numpy as np


BULLSEYE_ID = 0


def image_description(image_id: int, names: Mapping[int, str]) -> str:
    """Return the human-readable MDP description for an official image ID."""
    raw_name = names.get(image_id, "Unknown").strip()

    if 11 <= image_id <= 19:
        return f"Number {raw_name.title()}"
    if 20 <= image_id <= 35:
        return f"Alphabet {raw_name.upper()}"
    if 36 <= image_id <= 39:
        direction = raw_name.lower().removesuffix(" arrow").strip().title()
        return f"{direction} Arrow"
    return raw_name.title()


def annotation_lines(image_id: int, names: Mapping[int, str]) -> tuple[str, str]:
    return (
        f"Image ID: {image_id}",
        f"Description: {image_description(image_id, names)}",
    )


def _label_layout(lines: Sequence[str], available_width: int, available_height: int):
    """Choose the largest readable font whose two-line panel fits the image."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    padding = 7
    gap = 5

    for scale in (0.9, 0.85, 0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45, 0.4):
        thickness = 2
        sizes = [cv2.getTextSize(line, font, scale, thickness)[0] for line in lines]
        panel_width = max(width for width, _ in sizes) + 2 * padding
        panel_height = sum(height for _, height in sizes) + gap * (len(lines) - 1) + 2 * padding
        if panel_width <= available_width and panel_height <= available_height:
            return font, scale, thickness, sizes, panel_width, panel_height, padding, gap

    scale = 0.4
    thickness = 2
    sizes = [cv2.getTextSize(line, font, scale, thickness)[0] for line in lines]
    panel_width = min(available_width, max(width for width, _ in sizes) + 2 * padding)
    panel_height = min(
        available_height,
        sum(height for _, height in sizes) + gap * (len(lines) - 1) + 2 * padding,
    )
    return font, scale, thickness, sizes, panel_width, panel_height, padding, gap


def draw_detection_annotations(
    image: np.ndarray,
    boxes: Sequence[Sequence[float]],
    image_ids: Sequence[int],
    names: Mapping[int, str],
) -> np.ndarray:
    """Draw boxes with a large two-line label outside each boundary."""
    height, width = image.shape[:2]
    if width < 2 or height < 2:
        return image

    for box, raw_image_id in zip(boxes, image_ids):
        image_id = int(raw_image_id)
        x1, y1, x2, y2 = (int(round(value)) for value in box)
        x1 = max(0, min(width - 2, x1))
        y1 = max(0, min(height - 2, y1))
        x2 = max(x1 + 1, min(width - 1, x2))
        y2 = max(y1 + 1, min(height - 1, y2))

        color = (0, 165, 255) if image_id == BULLSEYE_ID else (0, 220, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

        lines = annotation_lines(image_id, names)
        available_width = max(1, width - 2)
        available_height = max(1, height - 2)
        layout = _label_layout(lines, available_width, available_height)
        font, scale, thickness, sizes, panel_width, panel_height, padding, gap = layout

        offset = 6
        panel_x1 = max(0, min(x1, width - panel_width))
        if y1 >= panel_height + offset:
            panel_y1 = y1 - panel_height - offset
        elif y2 + panel_height + offset < height:
            panel_y1 = y2 + offset
        elif x2 + panel_width + offset < width:
            panel_x1 = x2 + offset
            panel_y1 = max(0, min(y1, height - panel_height))
        elif x1 >= panel_width + offset:
            panel_x1 = x1 - panel_width - offset
            panel_y1 = max(0, min(y1, height - panel_height))
        else:
            # A box that nearly fills the frame leaves no truly external
            # space; keep the label at the nearest image edge as a fallback.
            panel_y1 = 0 if y1 > height - y2 else height - panel_height

        panel_x2 = min(width - 1, panel_x1 + panel_width)
        panel_y2 = min(height - 1, panel_y1 + panel_height)
        cv2.rectangle(image, (panel_x1, panel_y1), (panel_x2, panel_y2), color, -1)

        baseline_y = panel_y1 + padding
        for line, (_, text_height) in zip(lines, sizes):
            baseline_y += text_height
            if baseline_y > panel_y2:
                break
            cv2.putText(
                image,
                line,
                (panel_x1 + padding, baseline_y),
                font,
                scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )
            baseline_y += gap

    return image
