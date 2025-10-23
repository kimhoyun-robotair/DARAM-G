"""Mission finite state machine for the DARAM-G Mk.II Increment-I demo.

The real robot executes a mission composed of four large stages:

``search``
    The rover scans the environment and looks for a target with a YOLO based
    detector.  Once the detector locks on the object the rover aligns itself
    with the target.
``approach``
    The rover moves towards the object until it is close enough for the
    gripper to pick it up.
``rotate``
    After grabbing the target, the rover performs a 180° rotation so the lane
    detector and the colour tracker point towards the return track.
``return``
    The rover follows the lane and finishes the mission once a specific colour
    area has been detected (the drop zone).

The original project exposes several ROS topics and services.  For the
purposes of this kata we keep the implementation framework agnostic and rely on
simple python callables.  The concrete project can plug the ROS publishers and
subscribers on top of the :class:`FiniteStateMachine` class.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Optional, Protocol

from .utility import (
    ColorObservation,
    LaneObservation,
    RollingAverage,
    VelocityCommand,
    YoloTarget,
    align_to_target,
    approach_target,
    colour_alignment_command,
    lane_follow_command,
    search_command,
    stop_command,
)


# ---------------------------------------------------------------------------
# Protocols used for dependency injection
# ---------------------------------------------------------------------------


class YoloDetector(Protocol):
    """Return the most relevant YOLO detection or ``None`` if nothing was found."""

    def observe(self) -> Optional[YoloTarget]:  # pragma: no cover - interface
        ...


class LaneDetector(Protocol):
    """Return lane information used for line following."""

    def observe(self) -> Optional[LaneObservation]:  # pragma: no cover - interface
        ...


class ColorDetector(Protocol):
    """Return colour blob information for the drop zone."""

    def observe(self) -> Optional[ColorObservation]:  # pragma: no cover - interface
        ...


class GripperController(Protocol):
    """Very small abstraction over the physical gripper."""

    def open(self) -> None:  # pragma: no cover - interface
        ...

    def close(self) -> None:  # pragma: no cover - interface
        ...


class CommandPublisher(Protocol):
    """Publish commands to the actuators."""

    def publish(self, command: VelocityCommand) -> None:  # pragma: no cover - interface
        ...


# ---------------------------------------------------------------------------
# Configuration and runtime context
# ---------------------------------------------------------------------------


class MissionState(Enum):
    """High level mission stages."""

    SEARCH = auto()
    ALIGN = auto()
    APPROACH = auto()
    GRAB = auto()
    ROTATE = auto()
    RETURN = auto()
    FINISHED = auto()


@dataclass(slots=True)
class FSMConfig:
    """Tunable parameters for the mission."""

    confidence_threshold: float = 0.45
    alignment_tolerance: float = 0.12
    distance_tolerance: float = 0.08
    max_linear_speed: float = 0.4
    max_angular_speed: float = 1.6
    smoothing_window: int = 5
    grip_duration: float = 1.8
    rotation_duration: float = 3.5
    rotation_speed: float = 1.1
    return_colour: str = "home"
    return_area_threshold: float = 0.18
    fallback_search_turn_rate: float = 0.6


@dataclass(slots=True)
class MissionContext:
    """Runtime context stored by the finite state machine."""

    state: MissionState = MissionState.SEARCH
    last_detection: Optional[YoloTarget] = None
    grip_started_at: Optional[float] = None
    rotate_started_at: Optional[float] = None
    last_command: VelocityCommand = field(default_factory=VelocityCommand)


# ---------------------------------------------------------------------------
# Finite state machine implementation
# ---------------------------------------------------------------------------


class FiniteStateMachine:
    """Coordinates perception modules and produces motion commands."""

    def __init__(
        self,
        yolo: YoloDetector,
        lane: LaneDetector,
        colour: ColorDetector,
        gripper: Optional[GripperController],
        publisher: Optional[CommandPublisher] = None,
        config: Optional[FSMConfig] = None,
        *,
        time_source: Callable[[], float] = time.monotonic,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.yolo = yolo
        self.lane = lane
        self.colour = colour
        self.gripper = gripper
        self.publisher = publisher
        self.config = config or FSMConfig()
        self.time_source = time_source
        self.context = MissionContext()
        self.filter = RollingAverage(self.config.smoothing_window)
        self.logger = logger or logging.getLogger(__name__)
        self._gripper_closed = False

        if self.gripper is not None:
            try:
                self.gripper.open()
            except Exception:  # pragma: no cover - defensive
                self.logger.exception("Failed to open the gripper during initialisation")

    # ------------------------------------------------------------------
    # Observation helpers
    # ------------------------------------------------------------------

    def _observe_yolo(self) -> Optional[YoloTarget]:
        detection = self.yolo.observe()
        if detection and detection.confidence < self.config.confidence_threshold:
            return None
        return detection

    def _observe_lane(self) -> Optional[LaneObservation]:
        return self.lane.observe()

    def _observe_colour(self) -> Optional[ColorObservation]:
        return self.colour.observe()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _transition(self, new_state: MissionState) -> None:
        if self.context.state is new_state:
            return
        self.logger.debug("Transition %s -> %s", self.context.state.name, new_state.name)
        self.context.state = new_state

    def _issue_command(self, command: VelocityCommand, stamp: float) -> VelocityCommand:
        clamped = command.clamp(self.config.max_linear_speed, self.config.max_angular_speed)
        clamped.stamp = stamp
        self.filter.push(clamped)
        filtered = self.filter.mean()
        self.context.last_command = filtered
        if self.publisher is not None:
            self.publisher.publish(filtered)
        return filtered

    # ------------------------------------------------------------------
    # Mission execution
    # ------------------------------------------------------------------

    def step(self) -> MissionState:
        """Advance the state machine by one step and publish the command."""

        now = self.time_source()
        state = self.context.state
        command = stop_command()

        if state is MissionState.SEARCH:
            command = self._state_search(now)
        elif state is MissionState.ALIGN:
            command = self._state_align(now)
        elif state is MissionState.APPROACH:
            command = self._state_approach(now)
        elif state is MissionState.GRAB:
            command = self._state_grab(now)
        elif state is MissionState.ROTATE:
            command = self._state_rotate(now)
        elif state is MissionState.RETURN:
            command = self._state_return(now)
        elif state is MissionState.FINISHED:
            command = stop_command()

        self._issue_command(command, now)
        return self.context.state

    # ------------------------------------------------------------------
    # Individual state implementations
    # ------------------------------------------------------------------

    def _state_search(self, now: float) -> VelocityCommand:
        detection = self._observe_yolo()
        if detection is None:
            self.context.last_detection = None
            return search_command(turn_rate=self.config.fallback_search_turn_rate)

        self.context.last_detection = detection
        self._transition(MissionState.ALIGN)
        return align_to_target(detection)

    def _state_align(self, now: float) -> VelocityCommand:
        detection = self._observe_yolo()
        if detection is None:
            self._transition(MissionState.SEARCH)
            return search_command(turn_rate=self.config.fallback_search_turn_rate)

        self.context.last_detection = detection
        command = align_to_target(detection)
        offset = abs(detection.box.center_x - 0.5)  # bounding boxes use normalised coordinates
        if offset <= self.config.alignment_tolerance:
            self._transition(MissionState.APPROACH)
        return command

    def _state_approach(self, now: float) -> VelocityCommand:
        detection = self._observe_yolo()
        if detection is None:
            self._transition(MissionState.SEARCH)
            return search_command(turn_rate=self.config.fallback_search_turn_rate)

        self.context.last_detection = detection
        approach = approach_target(detection)
        alignment = align_to_target(detection)
        command = VelocityCommand(
            linear=approach.linear,
            angular=alignment.angular,
        )

        if detection.distance is not None and detection.distance <= self.config.distance_tolerance:
            self._transition(MissionState.GRAB)
            self.context.grip_started_at = now
            return stop_command()

        # When we have no depth information we rely on the command magnitude to
        # decide when to attempt a grab.  Close to the object, the approach
        # command becomes almost zero.
        if detection.distance is None and abs(approach.linear) <= 0.02:
            self._transition(MissionState.GRAB)
            self.context.grip_started_at = now
            return stop_command()

        return command

    def _state_grab(self, now: float) -> VelocityCommand:
        if self.gripper is not None and not self._gripper_closed:
            try:
                self.gripper.close()
            except Exception:  # pragma: no cover - defensive
                self.logger.exception("Failed to close the gripper")
            finally:
                self._gripper_closed = True

        if self.context.grip_started_at is None:
            self.context.grip_started_at = now

        elapsed = now - self.context.grip_started_at
        if elapsed >= self.config.grip_duration:
            self._transition(MissionState.ROTATE)
            self.context.rotate_started_at = now
            return VelocityCommand(angular=self.config.rotation_speed)

        return stop_command()

    def _state_rotate(self, now: float) -> VelocityCommand:
        if self.context.rotate_started_at is None:
            self.context.rotate_started_at = now

        if now - self.context.rotate_started_at >= self.config.rotation_duration:
            self._transition(MissionState.RETURN)
            return stop_command()

        return VelocityCommand(angular=self.config.rotation_speed)

    def _state_return(self, now: float) -> VelocityCommand:
        colour = self._observe_colour()
        if colour and colour.label == self.config.return_colour and colour.area_ratio >= self.config.return_area_threshold:
            self._transition(MissionState.FINISHED)
            return stop_command()

        lane = self._observe_lane()
        commands = []
        if lane is not None:
            commands.append(lane_follow_command(lane))
        if colour is not None:
            commands.append(colour_alignment_command(colour))

        if commands:
            return VelocityCommand(
                linear=sum(cmd.linear for cmd in commands) / len(commands),
                angular=sum(cmd.angular for cmd in commands) / len(commands),
            )

        # Nothing detected, keep moving slowly forward and scan.
        return search_command(search_speed=0.15, turn_rate=0.0)

    # ------------------------------------------------------------------
    # Convenience runner
    # ------------------------------------------------------------------

    def run(
        self,
        *,
        rate_hz: float = 20.0,
        stop_condition: Optional[Callable[[MissionState], bool]] = None,
        max_steps: Optional[int] = None,
    ) -> MissionState:
        """Repeatedly call :meth:`step`.

        Parameters
        ----------
        rate_hz:
            Frequency at which :meth:`step` is invoked.
        stop_condition:
            Optional callable that receives the current state and returns
            ``True`` when the loop has to stop.
        max_steps:
            Hard limit on the number of steps for unit testing.
        """

        interval = 1.0 / max(rate_hz, 1e-6)
        step_count = 0
        while True:
            state = self.step()
            step_count += 1
            if stop_condition is not None and stop_condition(state):
                break
            if max_steps is not None and step_count >= max_steps:
                break
            time.sleep(interval)
        return self.context.state


# ---------------------------------------------------------------------------
# Example stubs used for unit tests or documentation
# ---------------------------------------------------------------------------


@dataclass
class _CallableDetector:
    callback: Callable[[], Optional[object]]

    def observe(self) -> Optional[object]:
        return self.callback()


class DummyPublisher:
    """Publisher that simply stores the last command."""

    def __init__(self) -> None:
        self.history: list[VelocityCommand] = []

    def publish(self, command: VelocityCommand) -> None:
        self.history.append(command)


__all__ = [
    "FiniteStateMachine",
    "MissionState",
    "FSMConfig",
    "MissionContext",
    "YoloDetector",
    "LaneDetector",
    "ColorDetector",
    "GripperController",
    "CommandPublisher",
    "DummyPublisher",
]
