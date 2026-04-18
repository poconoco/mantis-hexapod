import math

from .stepper import Stepper
from ..py_kinematics.point3d import Point3D
from ..py_kinematics.orientation3d import Orientation3D


MIN_TURN_RADIUS = 100


class StepperCircular(Stepper):
    def __init__(
        self, 
        step_height: float, 
        cycle_length: float, 
        leg_group_count: int, 
        leg_phase_offset: float, 
        rest_offset: Point3D
    ):
        super().__init__(step_height, cycle_length, leg_group_count, leg_phase_offset, rest_offset)

        # just to make it externally available for visualization, otherwise it could be
        # local var in offset_step()
        self.cx = 0

    def offset_step(
        self, 
        steer_direction: Point3D, 
        direction_full_step: Point3D, 
        multiplier: float
    ) -> Point3D:
        if steer_direction.x == 0:
            return Point3D(0, direction_full_step.y * multiplier, 0)

        turn_radius = (1 / steer_direction.x) * MIN_TURN_RADIUS
        self.cx = turn_radius

        angle = calculate_turn_angle(turn_radius, direction_full_step.y) * multiplier * -1
       
        return self.rest_offset.rotate_around_z(self.cx, 0, angle) - self.rest_offset


def calculate_turn_angle(turn_radius: float, chord_length: float) -> float:
    if turn_radius == 0.0:
        return 0.0

    ratio = chord_length / (2.0 * turn_radius)
    ratio = max(-1.0, min(ratio, 1.0))

    return 2.0 * math.asin(ratio)
