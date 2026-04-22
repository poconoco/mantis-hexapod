import math
import time

from .leg import Leg
from .tail import Tail
from .gait.gait import Gait
from .py_kinematics.point3d import Point3D
from .py_kinematics.orientation3d import Orientation3D
from .py_kinematics.orientation_smoother import OrientationSmoother
from .py_kinematics.smoother3d import Smoother3D


class Body:
    legs: list[Leg]
    tail: Tail
    orientation: Orientation3D
    translation: Point3D

    def __init__(self, legs: list[Leg], tail: Tail, gait: Gait, config: dict):
        self.legs = legs
        self.tail = tail
        self.gait = gait

        self._prev_tick = None
        self._orientation = OrientationSmoother(Orientation3D(), config['tilt_speed_factor'])
        self._translation = Smoother3D(Point3D(), config['translation_speed'], config['translation_acceleration'])

    def tick(self):
        now = time.time()
        if self._prev_tick is None:
            self._prev_tick = now
            return
        dt = now - self._prev_tick        
        
        self.gait.tick()
        self._translation.tick(dt)
        self._orientation.tick(dt)

        translation = self._translation.get()
        orientation = self._orientation.get()

        for leg in self.legs:
            tip = leg.origin + leg.relative_rest_pos
            tip = tip.rotate(orientation)
            tip += self.gait.get_tip_offset(leg)
            tip += translation

            leg.tick(tip - leg.origin)

        self.tail.tick(- orientation.pitch * 2 - 60)

    def set_walk_direction(self, direction: Point3D):
        self.gait(direction)

    def set_translation(self, translation: Point3D):
        self._translation.set_target(translation)

    def set_orientation(self, orientation: Orientation3D):
        self._orientation.set_target(orientation)

    def detach(self):
        for leg in self.legs:
            leg.detach()

        self.tail.detach()
