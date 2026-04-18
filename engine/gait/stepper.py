import math

from abc import ABC, abstractmethod

from ..py_kinematics.point3d import Point3D


class Stepper(ABC):
    def __init__(
        self, 
        step_height: float, 
        cycle_length: float, 
        leg_group_count: int, 
        leg_phase_offset: float, 
        rest_offset: Point3D
    ):
        self._step_height = step_height
        self.cycle_length = cycle_length
        self.leg_group_count = leg_group_count
        self.up_phase_length = self.cycle_length / self.leg_group_count
        self.down_phase_length = self.cycle_length - self.up_phase_length

        self._leg_phase_offset = leg_phase_offset
        self.rest_offset = rest_offset


    def step(self, steer_direction: Point3D, direction_full_step: Point3D, global_cycle_pos: float) -> Point3D:
        leg_cycle_pos = self._normalize_cycle_pos(global_cycle_pos + self._leg_phase_offset)

        if self._is_in_down_phase(leg_cycle_pos):
            m = self._get_get_down_phase_multiplier(leg_cycle_pos)
            return self.offset_step(steer_direction, direction_full_step * -1,  m)
        elif self._is_in_up_phase(leg_cycle_pos):
            m = self._get_up_phase_multiplier(leg_cycle_pos)
            angle = (math.pi / 2) - math.pi * m
            step = self.offset_step(steer_direction, direction_full_step, m)
            step.z = math.sin(angle) * self._step_height
            return step
        else:
            raise ValueError(f'Invalid cycle_pos, leg: {leg_cycle_pos}, global: {global_cycle_pos}')

    @abstractmethod
    def offset_step(
        self, 
        steer_direction: Point3D, 
        direction_full_step: Point3D, 
        multiplier: float
    ) -> Point3D:
        pass

    def _normalize_cycle_pos(self, cycle_pos: float):
        while cycle_pos >= self.cycle_length:
            cycle_pos -= self.cycle_length
        
        return cycle_pos

    def _is_in_down_phase(self, cycle_pos: float) -> bool:
        return cycle_pos < self.down_phase_length
    
    def _is_in_up_phase(self, cycle_pos: float) -> bool:
        return cycle_pos >= self.down_phase_length
    
    def _get_get_down_phase_multiplier(self, cycle_pos: float) -> float:
        return (cycle_pos / self.down_phase_length) - 0.5

    def _get_up_phase_multiplier(self, cycle_pos: float) -> float:
        return ((cycle_pos - self.down_phase_length) / self.up_phase_length) - 0.5
