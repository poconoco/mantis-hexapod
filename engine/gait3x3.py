import time

from .base_leg import BaseLeg
from .stepper import Stepper
from .py_kinematics.point3d import Point3D
from .base_gait import BaseGait


class Gait3x3(BaseGait):
    def __init__(self, config: dict):
        self._steps_per_second = config['steps_per_second']
        self._step_length = config['step_length']
        self._prev_tick = None
        self._direction = Point3D()
        self._stepper = Stepper(config['step_height'])
        self._leg_a_tip_pos = Point3D()
        self._leg_b_tip_pos = Point3D()
        self._stationary = True
        self._zero_direction_since = None

        # Defines the step cycle position:
        # 0    - leg is on the ground in the middle of the step stroke
        # 0.5  - leg is moved half-step to push the body forward (e.g. leg is moved back)
        #        at this point, leg starts to lift and move forward
        # 1    - leg is at the highest posiotion and in the middle of the step stroke
        #        here we flip position to -1 so it is cyclic
        # -1   - phisically same position as 1, leg starts to descend and continues to move forward
        # -0.5 - leg is touched the ground, it's half-step forward, and starts to move backward (to push body forward)
        # 0    - then cycle repeats
        self._step_cycle_pos = 0

    def set_direction(self, direction: Point3D):
        if direction.z != 0:
            raise ValueError(f"Direction z component must be 0, got: {direction.z}")

        if direction.magnitude() > 1:
            raise ValueError(f"Direction magnitude should be <= 1, got {direction} -> {direction.magnitude()}")

        self._direction = direction

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

            cycle_pos_delta = dt * self._steps_per_second * self._direction.magnitude()
            self._step_cycle_pos = self._get_next_cycle_pos(self._step_cycle_pos, cycle_pos_delta)

        if self._stationary:
            self._leg_a_tip_pos = Point3D()
            self._leg_b_tip_pos = Point3D()
        else:
            step_direction = self._direction.normalized() * self._step_length

            self._leg_a_tip_pos = self._stepper.step(step_direction, self._step_cycle_pos)
            self._leg_b_tip_pos = self._stepper.step(step_direction, self._normalize_cycle_pos(self._step_cycle_pos + 1))

    def get_tip_offset(self, leg: BaseLeg) -> Point3D:
        if self._is_leg_a(leg):
            return self._leg_a_tip_pos
        else:
            return self._leg_b_tip_pos

    def _get_next_cycle_pos(self, current_pos: float, delta: float) -> float:
        return self._normalize_cycle_pos(current_pos + delta)
        
    def _normalize_cycle_pos(self, cycle_pos: float) -> float:
        if cycle_pos > 1:
            return cycle_pos - 2
    
        return cycle_pos

    def _is_leg_a(self, leg: BaseLeg) -> bool:
        return leg.config['side'] == 'left' and leg.config['order'] % 2 == 0 \
            or leg.config['side'] == 'right' and leg.config['order'] % 2 == 1
