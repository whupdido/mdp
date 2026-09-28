"""Small, repository-local readers for STM32 turn-radius calibration."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path


_CALIBRATION_DEFINITIONS = (
    ("FL", "TURN_RADIUS_FL_MM"),
    ("FR", "TURN_RADIUS_FR_MM"),
    ("BL", "TURN_RADIUS_BL_MM"),
    ("BR", "TURN_RADIUS_BR_MM"),
)
_NUMERIC_LITERAL = re.compile(
    r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)[fF]?"
)


@dataclass(frozen=True, slots=True)
class CalibrationFailure:
    """One calibration macro that could not provide a usable value."""

    command: str
    macro: str
    reason: str


@dataclass(frozen=True, slots=True)
class TurnRadiusCalibration:
    """Immutable command-keyed result of reading the four radius macros."""

    values_mm: tuple[tuple[str, float], ...]
    failures: tuple[CalibrationFailure, ...]

    def value_mm_for(self, command: str) -> float | None:
        normalized = command.strip().upper()
        for candidate, value in self.values_mm:
            if candidate == normalized:
                return value
        return None


def default_calibration_path() -> Path:
    """Return the checkout-local calibration header without using CWD."""

    repository_root = Path(__file__).resolve().parents[1]
    return repository_root / "stm32" / "Core" / "Inc" / "calib.h"


def _parse_numeric_value(raw_value: str) -> float | None:
    value = raw_value.split("//", 1)[0].split("/*", 1)[0].strip()
    match = _NUMERIC_LITERAL.fullmatch(value)
    if match is None:
        return None
    parsed = float(match.group(1))
    if not math.isfinite(parsed) or parsed <= 0.0:
        return parsed
    return parsed


def load_turn_radius_calibration(
    path: Path | None = None,
) -> TurnRadiusCalibration:
    """Read valid turn radii from ``calib.h`` and report per-field failures."""

    header_path = Path(path) if path is not None else default_calibration_path()
    command_by_macro = {macro: command for command, macro in _CALIBRATION_DEFINITIONS}
    values: dict[str, float] = {}
    failures: dict[str, CalibrationFailure] = {}

    try:
        lines = header_path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        reason = f"could not read {header_path}: {exc}"
        return TurnRadiusCalibration(
            values_mm=(),
            failures=tuple(
                CalibrationFailure(command, macro, reason)
                for command, macro in _CALIBRATION_DEFINITIONS
            ),
        )

    for line_number, line in enumerate(lines, start=1):
        parts = line.strip().split(None, 2)
        if len(parts) != 3 or parts[0] != "#define" or parts[1] not in command_by_macro:
            continue

        macro = parts[1]
        command = command_by_macro[macro]
        parsed = _parse_numeric_value(parts[2])
        if parsed is None:
            failures[command] = CalibrationFailure(
                command, macro, f"malformed numeric value on line {line_number}"
            )
            values.pop(command, None)
        elif parsed <= 0.0:
            failures[command] = CalibrationFailure(
                command, macro, f"value must be positive on line {line_number}"
            )
            values.pop(command, None)
        else:
            values[command] = parsed
            failures.pop(command, None)

    for command, macro in _CALIBRATION_DEFINITIONS:
        if command not in values and command not in failures:
            failures[command] = CalibrationFailure(command, macro, "macro not found")

    return TurnRadiusCalibration(
        values_mm=tuple(
            (command, values[command])
            for command, _macro in _CALIBRATION_DEFINITIONS
            if command in values
        ),
        failures=tuple(
            failures[command]
            for command, _macro in _CALIBRATION_DEFINITIONS
            if command in failures
        ),
    )


__all__ = [
    "CalibrationFailure",
    "TurnRadiusCalibration",
    "default_calibration_path",
    "load_turn_radius_calibration",
]
