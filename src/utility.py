"""Utility helpers for the DARAM-G v2.1 mission.

This module groups together small pieces of logic that are reused across the
finite state machine.  The goal is to keep :mod:`main` focused on the
state transitions while keeping command generation, filtering and the shared
value objects here.

The real project contains several perception modules (YOLO based object
tracking, lane detection and simple colour based area extraction).  The
helpers implemented in this file only depend on the output of those modules,
so they can easily be reused in both simulation and on-board deployments.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence


# ---------------------------------------------------------------------------
# Data containers
# ---------------------------------------------------------------------------


@dataclass(slots=True)
class VelocityCommand:
    """Simple container for a differential drive velocity command.

    Attributes
    ----------
    linear : float
        Forward velocity in metres per second.
    angular : float
        Angular velocity in radians per second.
    stamp : Optional[float]
        Timestamp used when command filtering or prediction is required.  The
        finite state machine does not depend on the timestamp directly but it
        is very convenient when commands are published on ROS topics.
    """

    linear: float = 0.0
    angular: float = 0.0
    stamp: Optional[float] = None

    def clamp(self, max_linear: float, max_angular: float) -> "VelocityCommand":
        """Return a clamped copy of the command."""

        return VelocityCommand(
            linear=max(-max_linear, min(self.linear, max_linear)),
            angular=max(-max_angular, min(self.angular, max_angular)),
            stamp=self.stamp,
        )

    def is_zero(self, linear_eps: float = 1e-3, angular_eps: float = 1e-3) -> bool:
        """Check whether both linear and angular components are close to zero."""

        return abs(self.linear) <= linear_eps and abs(self.angular) <= angular_eps


@dataclass(slots=True)
class BoundingBox:
    """Normalised bounding box expressed in image coordinates."""

    center_x: float
    center_y: float
    width: float
    height: float


@dataclass(slots=True)
class YoloTarget:
    """Description of the best YOLO detection relevant for the mission."""

    label: str
    confidence: float
    box: BoundingBox
    distance: Optional[float] = None


@dataclass(slots=True)
class LaneObservation:
    """Lane perception output in the rover coordinate frame."""

    lateral_error: float
    heading_error: float


@dataclass(slots=True)
class ColorObservation:
    """Colour blob detection result."""

    label: str
    area_ratio: float
    centroid_x: float


# ---------------------------------------------------------------------------
# Numerical helpers
# ---------------------------------------------------------------------------


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp *value* to the inclusive range ``[minimum, maximum]``."""

    return max(minimum, min(maximum, value))


def _normalise_offset(center_x: float) -> float:
    """Normalise image x-coordinates to the ``[-1, 1]`` range."""

    return clamp(center_x, 0.0, 1.0) * 2.0 - 1.0


# ---------------------------------------------------------------------------
# Command generation helpers
# ---------------------------------------------------------------------------


def search_command(search_speed: float = 0.12, turn_rate: float = 0.8) -> VelocityCommand:
    """Generate a constant motion used while the rover is searching for a target."""

    return VelocityCommand(linear=search_speed, angular=turn_rate)


def align_to_target(
    detection: YoloTarget,
    align_gain: float = 1.2,
    max_angular_speed: float = 1.5,
) -> VelocityCommand:
    """Create an angular command that centres the YOLO detection."""

    offset = _normalise_offset(detection.box.center_x)
    angular = -clamp(offset * align_gain, -max_angular_speed, max_angular_speed)
    return VelocityCommand(linear=0.0, angular=angular)


def approach_target(
    detection: YoloTarget,
    desired_distance: float = 0.45,
    max_speed: float = 0.35,
    min_speed: float = 0.05,
) -> VelocityCommand:
    """Create a linear command that approaches the detected target."""

    if detection.distance is None:
        # When depth is unavailable we rely on the bounding box height as an
        # extremely crude proxy.  The heuristic assumes that taller bounding
        # boxes mean the object is closer.
        distance_ratio = clamp(1.0 - detection.box.height, 0.0, 1.0)
        distance_error = distance_ratio - desired_distance
    else:
        distance_error = detection.distance - desired_distance

    direction = -1.0 if distance_error < 0 else 1.0
    magnitude = clamp(abs(distance_error), 0.0, 1.0)
    linear = clamp(magnitude * max_speed, min_speed, max_speed) * direction
    return VelocityCommand(linear=linear, angular=0.0)


def lane_follow_command(
    observation: LaneObservation,
    base_speed: float = 0.25,
    kp: float = 0.9,
    kd: float = 0.15,
) -> VelocityCommand:
    """Create a command that keeps the rover on top of a detected lane."""

    angular = clamp(observation.lateral_error * kp + observation.heading_error * kd, -1.6, 1.6)
    return VelocityCommand(linear=base_speed, angular=angular)


def colour_alignment_command(
    observation: ColorObservation,
    gain: float = 1.0,
    base_speed: float = 0.18,
) -> VelocityCommand:
    """Steer the rover so the colour blob is centred in front of it."""

    angular = -clamp(_normalise_offset(observation.centroid_x) * gain, -1.2, 1.2)
    return VelocityCommand(linear=base_speed, angular=angular)


def stop_command() -> VelocityCommand:
    """Return a zero velocity command."""

    return VelocityCommand()


# ---------------------------------------------------------------------------
# Command filtering utilities
# ---------------------------------------------------------------------------


@dataclass(slots=True)
class RollingAverage:
    """Simple rolling average used to smooth velocity commands."""

    window: int
    _buffer: list[VelocityCommand] = field(default_factory=list)

    def push(self, command: VelocityCommand) -> None:
        if self.window <= 1:
            self._buffer = [command]
            return
        self._buffer.append(command)
        if len(self._buffer) > self.window:
            self._buffer.pop(0)

    def mean(self) -> VelocityCommand:
        if not self._buffer:
            return VelocityCommand()
        linear = sum(cmd.linear for cmd in self._buffer) / len(self._buffer)
        angular = sum(cmd.angular for cmd in self._buffer) / len(self._buffer)
        return VelocityCommand(linear=linear, angular=angular, stamp=self._buffer[-1].stamp)


def blend_commands(commands: Sequence[VelocityCommand]) -> VelocityCommand:
    """Average multiple velocity commands together."""

    if not commands:
        return VelocityCommand()
    linear = sum(cmd.linear for cmd in commands) / len(commands)
    angular = sum(cmd.angular for cmd in commands) / len(commands)
    stamp = commands[-1].stamp
    return VelocityCommand(linear=linear, angular=angular, stamp=stamp)


def are_all_zero(commands: Iterable[VelocityCommand], eps: float = 1e-3) -> bool:
    """Return ``True`` when every command in *commands* is almost zero."""

    return all(cmd.is_zero(eps, eps) for cmd in commands)


__all__ = [
    "VelocityCommand",
    "BoundingBox",
    "YoloTarget",
    "LaneObservation",
    "ColorObservation",
    "RollingAverage",
    "search_command",
    "align_to_target",
    "approach_target",
    "lane_follow_command",
    "colour_alignment_command",
    "stop_command",
    "blend_commands",
    "are_all_zero",
]
