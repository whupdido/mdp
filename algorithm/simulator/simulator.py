"""Compatibility imports for the original simulator module path.

The Pygame renderer intentionally lives in :mod:`algorithm.simulator.renderer`
so importing this module remains safe in headless planning environments.
"""

from .headless import HeadlessSimulator, PlaybackState, SimulationState, SimulationStep

__all__ = ["HeadlessSimulator", "PlaybackState", "SimulationState", "SimulationStep"]
