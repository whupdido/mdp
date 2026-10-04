"""Immutable diagnostic profiles for Task 1 planner comparisons."""

from __future__ import annotations

from dataclasses import replace
from enum import Enum

from algorithm.config import PlanningConfig


class CandidatePolicy(str, Enum):
    CONTROL_20 = "20"
    CENTERED_20_30 = "20/30"
    CENTERED_20_10_30 = "20/10/30"
    LAZY_20_THEN_30 = "lazy-20-to-30"


def turn_profile(config: PlanningConfig, angles: tuple[float, ...]) -> PlanningConfig:
    """Create an in-memory turn-search copy without mutating production."""
    return replace(config, search_turn_angles_deg=tuple(angles))


def candidate_profile(config: PlanningConfig, policy: CandidatePolicy) -> PlanningConfig:
    """Return the centered candidate policy as a frozen config copy."""
    policy = CandidatePolicy(policy)
    distances = {
        CandidatePolicy.CONTROL_20: (20.0,),
        CandidatePolicy.CENTERED_20_30: (20.0, 30.0),
        CandidatePolicy.CENTERED_20_10_30: (20.0, 10.0, 30.0),
        CandidatePolicy.LAZY_20_THEN_30: (20.0, 30.0),
    }[CandidatePolicy(policy)]
    tiers = ((0,), (1,)) if policy is CandidatePolicy.LAZY_20_THEN_30 else ((0,), *(
        (index,) for index in range(1, len(distances))
    ))
    return replace(
        config,
        observation_lateral_offsets_cm=(0.0,),
        observation_standoff_distances_cm=distances,
        guaranteed_max_candidates_per_target=len(distances),
        candidate_activation_tiers=tiers,
    )


def modest_turn_cost_profile(config: PlanningConfig) -> PlanningConfig:
    """Add the requested 0.25 x FW010 time cost for turns/steering changes."""
    fw010_s = (0.6 + 0.2 + 0.05 + max(0.0, 10.0 - config.motion.straight_deceleration_cm) /
               config.motion.straight_speed_cm_s)
    penalty = 0.25 * fw010_s
    return replace(config, motion=replace(
        config.motion,
        turn_penalty_s=penalty,
        steering_change_penalty_s=penalty,
        direction_change_penalty_s=0.0,
    ))


def strong_turn_cost_profile(config: PlanningConfig) -> PlanningConfig:
    """Reference-only 30:1 turn penalty; never used by production config."""
    fw010_s = (0.6 + 0.2 + 0.05 + max(0.0, 10.0 - config.motion.straight_deceleration_cm) /
               config.motion.straight_speed_cm_s)
    return replace(config, motion=replace(config.motion, turn_penalty_s=29.0 * fw010_s))


__all__ = ["CandidatePolicy", "candidate_profile", "modest_turn_cost_profile", "strong_turn_cost_profile", "turn_profile"]
