from .stepper import Stepper
from ..py_kinematics.point3d import Point3D


class StepperLinear(Stepper):
    def offset_step(
        self, 
        steer_direction: Point3D, 
        direction_full_step: Point3D, 
        multiplier: float
    ) -> Point3D:
        
        return Point3D(direction_full_step.x * multiplier, direction_full_step.y * multiplier, 0)
