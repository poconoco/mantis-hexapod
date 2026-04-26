import math
import time

from .leg import Leg
from .tail import Tail
from .gait.gait import Gait
from .py_kinematics.point3d import Point3D
from .py_kinematics.orientation3d import Orientation3D
from .py_kinematics.orientation_smoother import OrientationSmoother
from .py_kinematics.smoother3d import Smoother3D
from engine import leg


class Body:
    legs: list[Leg]
    tail: Tail
    orientation: Orientation3D
    translation: Point3D

    def __init__(self, legs: list[Leg], tail: Tail, gait: Gait, config: dict):
        self.legs = legs
        self.tail = tail
        self.gait = gait
        self._stance_mode = False

        self._prev_tick = None
        self._base_orientation = Orientation3D()
        self._orientation = OrientationSmoother(Orientation3D(), config['tilt_speed_factor'])
        self._translation = Smoother3D(Point3D(), config['translation_speed'], config['translation_acceleration'])
        self._stand_left_pos = Point3D()
        self._stand_right_pos = Point3D()
        self._stand_pos_incline = config['stand_pos_incline']
        self._stand_pos_offset = Point3D.from_dict(config['stand_pos_offset'])

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
            tip = tip.rotate(orientation, leg.origin)
            tip -= translation

            if self._stance_mode:
                if leg.order == 2:  # order 2 are front legs
                    stand_pos = self._stand_left_pos if leg.side == 'left' else self._stand_right_pos
                    tip += leg.stand_pos_offset + stand_pos
                tip -= self._stand_pos_offset
            else:
                # Translation anf gait are only applied in non-stand mode
                tip -= self.gait.get_tip_offset(leg)

            leg.tick(tip - leg.origin)

        tail_pitch = orientation.pitch * -2 - 60
        if self._stance_mode:
            tail_pitch -= self._stand_pos_incline

        self.tail.tick(tail_pitch)

    def set_stance_mode(self, stance_mode: bool):
        if self._stance_mode == stance_mode:
            return

        self._stance_mode = stance_mode
        self.set_orientation()

    def is_stance_mode(self) -> bool:
        return self._stance_mode

    def set_stance_mode_front_positions(self, left_pos: Point3D, right_pos: Point3D):
        self._stand_left_pos = left_pos
        self._stand_right_pos = right_pos

    def set_walk_direction(self, direction: Point3D):
        self.gait(direction)

    def set_translation(self, translation: Point3D):
        self._translation.set_target(translation)

    def set_orientation(self, orientation: Orientation3D | None = None):
        if orientation is not None:
             self._base_orientation = orientation

        if self._stance_mode:
            target_orientation = Orientation3D.copy(self._base_orientation)
            target_orientation.pitch -= self._stand_pos_incline
        else:
            target_orientation = self._base_orientation

        self._orientation.set_target(target_orientation)

    def detach(self):
        for leg in self.legs:
            leg.detach()

        self.tail.detach()
