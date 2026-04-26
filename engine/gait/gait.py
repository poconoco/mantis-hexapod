import time

from enum import Enum

from ..leg import Leg
from .stepper_linear import StepperLinear
from .stepper_circular import StepperCircular
from ..py_kinematics.point3d import Point3D


class Gait:
    class Type(Enum):
        GAIT_3_3 = 1
        GAIT_4_2 = 2
        GAIT_5_1 = 3
        def next(self):
            cls = self.__class__
            members = list(cls)
            current_index = members.index(self)
            next_index = (current_index + 1) % len(members)
            return members[next_index]
        
    def __init__(self, config: dict, legs: list[Leg], type: Type, circular: bool):
        self._steps_per_second = config['steps_per_second']
        self._step_length = config['step_length']
        self._step_height = config['step_height']
        self._prev_tick = None
        self._direction = Point3D()
        self._direction_full_step = Point3D()
        self._leg_a_tip_pos = Point3D()
        self._leg_b_tip_pos = Point3D()
        self._stationary = True
        self._zero_direction_since = None
        self._cycle_length = 100
        self._step_cycle_pos = None
        self._rest_cycle_pos = None
        self._type = None
        self._circular = None
        self._stationary_timeout = 2  # timeout in seconds after which all legs will be down, and step cycle reset
        self._legs = legs
        self._steppers = {}

        self.set_gait(Gait.Type.GAIT_5_1, circular)

    def set_gait(self, type: Type, circular: bool):
        self._type = type
        self._circular = circular
        for i, leg in enumerate(self._legs):
            if type == Gait.Type.GAIT_3_3:
                leg_group_count = 2
                phase_index = leg.order + (0 if leg.side == 'left' else 1)
                self._step_cycle_pos = 0
            elif type == Gait.Type.GAIT_4_2:
                leg_group_count = 3
                if leg.side == 'left' and leg.order == 0 \
                      or leg.side == 'right' and leg.order == 2:
                    phase_index = 0
                elif leg.side == 'left' and leg.order == 1 \
                      or leg.side == 'right' and leg.order == 1:
                    phase_index = 1
                elif leg.side == 'left' and leg.order == 2 \
                      or leg.side == 'right' and leg.order == 0:
                    phase_index = 2
                else:
                    raise ValueError(f'Unexpected leg: side={leg.side}, order={leg.order}')
            elif type == Gait.Type.GAIT_5_1:
                leg_group_count = 6
                phase_index = leg.order + (0 if leg.side == 'left' else 3)

            else:
                raise ValueError(f'Unexpected gait type: {type}')

            leg_phase_offset = (self._cycle_length / leg_group_count) * phase_index

            if self._circular:
                stepper_class = StepperCircular
            else:
                stepper_class = StepperLinear

            stepper = stepper_class(
                self._step_height,
                cycle_length=self._cycle_length,
                leg_group_count=leg_group_count,
                leg_phase_offset=leg_phase_offset,
                rest_offset=leg.origin + leg.relative_rest_pos
            )

            self._steppers[leg] = stepper

        self._rest_cycle_pos = stepper.down_phase_length / 2
        self._step_cycle_pos = self._rest_cycle_pos

    def get_type(self):
        return self._type
    
    def is_circular(self):
        return self._circular

    def set_direction(self, direction: Point3D):
        if direction.z != 0:
            raise ValueError(f"Direction z component must be 0, got: {direction.z}")

        if self._circular:
            self._direction = direction
        else:
            self._direction = direction.cube_to_sphere()

        self._direction_full_step = self._direction.normalized() * self._step_length

    def get_direction(self):
        return self._direction

    def tick(self, dt_factor: float = 1.0):
        now = time.time()
        if self._prev_tick is None:
            self._prev_tick = now
            return

        if self._direction.magnitude() > 0:
            self._zero_direction_since = None
            self._stationary = False
        elif self._zero_direction_since is None:
            self._zero_direction_since = now

        if self._zero_direction_since is not None  and now - self._zero_direction_since > self._stationary_timeout:
            # If direction is zero for more than stationary timeout, reset cycle position to avoid weird leg positions
            self._step_cycle_pos = self._rest_cycle_pos
            self._stationary = True
        else:
            dt = (now - self._prev_tick) * dt_factor
            self._prev_tick = now

            if self._circular:
                # In circular mode, the step speed only depends on Y input
                dt_multiplier = abs(self._direction.y)
            else:
                # While in linear mode, it depends on combined X+Y magnitude
                dt_multiplier = self._direction.magnitude()

            cycle_pos_delta = dt * self._steps_per_second * dt_multiplier * self._cycle_length
            self._step_cycle_pos = self._normalize_cycle_pos(self._step_cycle_pos + cycle_pos_delta)

    def get_tip_offset(self, leg: Leg) -> Point3D:
        if self._stationary:
            return Point3D()
        else:
            return self._steppers[leg].step(self._direction, self._direction_full_step, self._step_cycle_pos)

    def _normalize_cycle_pos(self, cycle_pos: float) -> float:
        while cycle_pos >= self._cycle_length:
            cycle_pos -= self._cycle_length

        return cycle_pos

    def _is_leg_a(self, leg: Leg) -> bool:
        return leg.config['side'] == 'left' and leg.config['order'] % 2 == 0 \
            or leg.config['side'] == 'right' and leg.config['order'] % 2 == 1
