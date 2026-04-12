from .base_leg import BaseLeg
from .tail import Tail
from .base_gait import BaseGait
from .py_kinematics.point3d import Point3D
from .py_kinematics.orientation3d import Orientation3D

class Body:
    legs: list[BaseLeg]
    tail: Tail
    orientation: Orientation3D
    translation: Point3D

    def __init__(self, legs: list[BaseLeg], tail: Tail, gait: BaseGait):
        self.legs = legs
        self.tail = tail
        self.gait = gait
        
        self.orientation = Orientation3D()
        self.translation = Point3D()

    def tick(self):
        self.gait.tick()

        for leg in self.legs:
            tip = leg.origin + leg.relative_rest_pos
            tip = tip.rotate(self.orientation)
            tip += self.gait.get_tip_offset(leg)
            tip += self.translation

            leg.tick(tip - leg.origin)

        self.tail.tick(self.orientation.pitch * 2)

    def set_walk_direction(self, direction: Point3D):
        self.gait(direction)

    def detach(self):
        for leg in self.legs:
            leg.detach()

        self.tail.detach()
