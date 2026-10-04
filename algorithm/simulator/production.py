"""Production Task 1 profile construction and launch diagnostics."""

from __future__ import annotations

from dataclasses import replace
from enum import Enum
from pathlib import Path
from typing import Callable

from algorithm.calibration import default_calibration_path, load_turn_radius_calibration
from algorithm.config import PlanningConfig, task1_robot_config
from algorithm.enums import RoutingMode

from .profiles import CandidatePolicy, candidate_profile, turn_profile
from .scenarios import config_fingerprint, config_record


class TurnProfile(str, Enum):
    ONLY_90 = "90"
    ONLY_30 = "30"
    ANGLES_45_90 = "45/90"
    PRODUCTION_60_90 = "60/90"


TURN_ANGLES = {
    TurnProfile.ONLY_90: (90.0,),
    TurnProfile.ONLY_30: (30.0,),
    TurnProfile.ANGLES_45_90: (45.0, 90.0),
    TurnProfile.PRODUCTION_60_90: (60.0, 90.0),
}


def build_production_config(
    *, calibration_path: Path | None = None,
    turn: TurnProfile = TurnProfile.PRODUCTION_60_90,
    candidates: CandidatePolicy = CandidatePolicy.CONTROL_20,
) -> tuple[PlanningConfig, bool, dict[str, object]]:
    """Create production effective config plus live-calibration metadata."""
    calibration = load_turn_radius_calibration(calibration_path)
    base = task1_robot_config(calibration_path, emit=lambda _message: None)
    config = candidate_profile(base, candidates)
    config = turn_profile(config, TURN_ANGLES[TurnProfile(turn)])
    values = {name: value for name, value in calibration.values_mm}
    metadata = {
        "source": str(calibration_path or default_calibration_path()),
        "live": not calibration.failures,
        "turn_radii_mm": values,
        "failures": [{"command": failure.command, "macro": failure.macro, "reason": failure.reason}
                     for failure in calibration.failures],
    }
    return config, not calibration.failures, metadata


def configuration_banner(config: PlanningConfig, *, live_calibration: bool,
                         diagnostic: bool = False, production_config: PlanningConfig | None = None,
                         routing_mode: RoutingMode = RoutingMode.FULL_OPTIMIZATION,
                         production_routing_mode: RoutingMode = RoutingMode.FULL_OPTIMIZATION) -> str:
    def lines(label: str, value: PlanningConfig) -> list[str]:
        radii = {primitive.command: primitive.radius_cm for primitive in value.motion.primitives
                 if primitive.radius_cm is not None}
        policy = "/".join(str(int(distance)) for distance in value.observation_standoff_distances_cm)
        fingerprint_mode = production_routing_mode if value is production_config else routing_mode
        return [
            label,
            f"  Search turns: {', '.join(f'{angle:g} degrees' for angle in value.search_turn_angles_deg)}",
            "  Objective: ESTIMATED_TIME",
            f"  Candidate policy: centered {policy} cm; tiers={value.candidate_activation_tiers}",
            "  Turn radii cm: " + " / ".join(f"{command} {radii.get(command, 0):.1f}"
                                             for command in ("FL", "FR", "BL", "BR")),
            f"  Calibration: {'LIVE' if live_calibration else 'FALLBACK / PARTIAL'}",
            f"  Robot: {value.robot.length_cm:g} x {value.robot.width_cm:g} cm; rear axle to body center "
            f"forward {value.robot.rear_axle_to_body_center_forward_cm:g}, left "
            f"{value.robot.rear_axle_to_body_center_left_cm:g} cm; margin {value.robot.safety_margin_cm:g} cm",
            f"  Camera offset: forward {value.camera.forward_offset_cm:g}, lateral {value.camera.left_offset_cm:g} cm",
            f"  Arena: {value.arena_size_cm:g} x {value.arena_size_cm:g} cm",
            f"  Planning bounds: local {value.local_planning_timeout_s:g}s, total {value.overall_planning_timeout_s:g}s, "
            f"nodes {value.max_expanded_nodes}, adaptive {value.adaptive_initial_expansions}/{value.adaptive_max_expansions}",
            f"  Config fingerprint: {config_fingerprint(value, routing_mode=fingerprint_mode)}",
        ]
    output = ["PRODUCTION SIMULATOR", f"Mode: {'DIAGNOSTIC' if diagnostic else 'PRODUCTION'}"]
    if diagnostic and production_config is not None:
        output.extend(lines("PRODUCTION CONFIG", production_config))
        output.extend(lines("ACTIVE DIAGNOSTIC CONFIG", config))
    else:
        output.extend(lines("PRODUCTION CONFIG", config))
    return "\n".join(output)


__all__ = ["TURN_ANGLES", "TurnProfile", "build_production_config", "configuration_banner"]
