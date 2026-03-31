from .base_leg import BaseLeg
from .tail import Tail
from .py_kinematics.point3d import Point3D
from .py_kinematics.orientation3d import Orientation3D

class Body:
    legs: list[BaseLeg]
    tail: Tail
    orientation: Orientation3D
    translation: Point3D

    def __init__(self, legs: list[BaseLeg], tail: Tail):
        self.legs = legs
        self.tail = tail
        
        self.orientation = Orientation3D()
        self.translation = Point3D()

    def tick(self):
        for leg in self.legs:
            # TODO: apply orientation
            leg.tick(self.translation + leg.relative_rest_pos)
