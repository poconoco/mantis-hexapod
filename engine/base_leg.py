from abc import ABC, abstractmethod

from .py_kinematics.point3d import Point3D
from .py_kinematics.smoother3d import Smoother3D


class BaseLeg(ABC):

    def __init__(self, leg_config: dict, initial_point: Point3D):
        self._prev_tick = None
        self._mirror_x = leg_config.get('mirror_x', False)

        self.origin = Point3D(
            leg_config['origin']['x'],
            leg_config['origin']['y'],
            leg_config['origin']['z']
        )

        self.relative_rest_pos = Point3D(
            leg_config['relative_rest_pos']['x'],
            leg_config['relative_rest_pos']['y'],
            leg_config['relative_rest_pos']['z']
        )

        x_mul = -1 if self._mirror_x else 1

        self._smoother = Smoother3D(
            Point3D(initial_point.x * x_mul, initial_point.y, initial_point.z),
            leg_config['speed'],
            leg_config['acceleration'],
            1.0
        )

    @abstractmethod
    def tick(self, reach_to: Point3D):
        pass

    @abstractmethod
    def detach(self):
        pass
