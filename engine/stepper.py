import math

from .base_leg import BaseLeg
from .py_kinematics.point3d import Point3D

class Stepper:
    def __init__(self, step_height: float):
        self._step_height = step_height

    def step(self, direction: Point3D, cycle_pos: float) -> Point3D:


        # Leg is on the ground in this part of the cycle
        if -0.5 <= cycle_pos and cycle_pos <= 0.5:
            return Point3D(- direction.x * cycle_pos, - direction.y * cycle_pos, 0)

        # Leg is lifted, so calculating half circle movement        
        if cycle_pos > 0.5 and cycle_pos <= 1:
            normalized_lifted_cycle_pos = cycle_pos - 1
        elif cycle_pos < -0.5 and cycle_pos >= -1:
            normalized_lifted_cycle_pos = cycle_pos + 1
        else:
            raise ValueError(f"Invalid cycle_pos: {cycle_pos}")

        angle = (math.pi / 2) - math.pi * normalized_lifted_cycle_pos
        
        z = math.sin(angle) * self._step_height

        return Point3D(
            direction.x * normalized_lifted_cycle_pos,
            direction.y * normalized_lifted_cycle_pos,
            z
        )
