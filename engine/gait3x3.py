import time

from .base_leg import BaseLeg
from .stepper import Stepper
from .py_kinematics.point3d import Point3D
from .base_gait import BaseGait


class Gait3x3(BaseGait):
    def __init__(self, config: dict, legs: list[BaseLeg]):
        self._steps_per_second = config['steps_per_second']
        self._step_length = config['step_length']
        self._prev_tick = None
        self._direction = Point3D()
        self._normalized_step_direction = Point3D()
        self._leg_a_tip_pos = Point3D()
        self._leg_b_tip_pos = Point3D()
        self._stationary = True
        self._zero_direction_since = None
        self._cycle_length = 100
        self._step_cycle_pos = 0
        step_height = config['step_height']

        self._steppers = {}
        for i, leg in enumerate(legs):
            self._steppers[leg] = Stepper(
                step_height,
                cycle_length=self._cycle_length,
                leg_group_count=2,
                leg_phase_offset=(self._cycle_length / 2) * (leg.config['order'] + (0 if leg.config['side'] == 'left' else 1)))

    def set_direction(self, direction: Point3D):
        if direction.z != 0:
            raise ValueError(f"Direction z component must be 0, got: {direction.z}")

        if direction.magnitude() > 1.0001:
            raise ValueError(f"Direction magnitude should be <= 1, got {direction} -> {direction.magnitude()}")

        self._direction = direction
        self._normalized_step_direction = self._direction.normalized() * self._step_length

    def tick(self):
        now = time.time()
        if self._prev_tick is None:
            self._prev_tick = now
            return

        if self._direction.magnitude() > 0:
            self._zero_direction_since = None
            self._stationary = False
        elif self._zero_direction_since is None:
            self._zero_direction_since = now

        if self._zero_direction_since is not None  and now - self._zero_direction_since > 1:
            # If direction is zero for more than 1 second, reset cycle position to avoid weird leg positions
            self._step_cycle_pos = 0
            self._stationary = True
        else:
            dt = now - self._prev_tick
            self._prev_tick = now

            cycle_pos_delta = dt * self._steps_per_second * self._direction.magnitude() * self._cycle_length
            self._step_cycle_pos = self._normalize_cycle_pos(self._step_cycle_pos + cycle_pos_delta)

    def get_tip_offset(self, leg: BaseLeg) -> Point3D:
        if self._stationary:
            return Point3D()
        else:
            return self._steppers[leg].step(self._normalized_step_direction, self._step_cycle_pos)

    def _normalize_cycle_pos(self, cycle_pos: float) -> float:
        while cycle_pos >= self._cycle_length:
            cycle_pos -= self._cycle_length

        return cycle_pos

    def _is_leg_a(self, leg: BaseLeg) -> bool:
        return leg.config['side'] == 'left' and leg.config['order'] % 2 == 0 \
            or leg.config['side'] == 'right' and leg.config['order'] % 2 == 1
