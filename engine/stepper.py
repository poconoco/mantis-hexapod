import math

from .py_kinematics.point3d import Point3D


class Stepper:
    def __init__(self, step_height: float, cycle_length: float, leg_group_count: int, leg_phase_offset: float):
        self._step_height = step_height
        self.cycle_length = cycle_length
        self.leg_group_count = leg_group_count
        self.up_phase_length = self.cycle_length / self.leg_group_count
        self.down_phase_length = self.cycle_length - self.up_phase_length

        self._leg_phase_offset = leg_phase_offset


    def step(self, direction: Point3D, global_cycle_pos: float) -> Point3D:
        leg_cycle_pos = self._normalize_cycle_pos(global_cycle_pos + self._leg_phase_offset)

        if self._is_in_down_phase(leg_cycle_pos):
            m = self._get_get_down_phase_multiplier(leg_cycle_pos)
            return Point3D(- direction.x * m, - direction.y * m, 0)
        elif self._is_in_up_phase(leg_cycle_pos):
            m = self._get_up_phase_multiplier(leg_cycle_pos)
            angle = (math.pi / 2) - math.pi * m
            z = math.sin(angle) * self._step_height
            return Point3D(
                direction.x * m,
                direction.y * m,
                z
            )
        else:
            raise ValueError(f'Invalid cycle_pos, leg: {leg_cycle_pos}, global: {global_cycle_pos}')

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
