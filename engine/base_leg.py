from abc import ABC, abstractmethod

from .py_kinematics.point3d import Point3D
from .py_kinematics.smoother3d import Smoother3D


class BaseLeg(ABC):

    def __init__(self, leg_config: dict, initial_point: Point3D | None):
        self.config = leg_config
        self._prev_tick = None
        self._mirror_x = self.config.get('mirror_x', False)

        if initial_point is None:
            initial_point = Point3D.from_dict(self.config['relative_rest_pos'])

        self.origin = Point3D(
            self.config['origin']['x'],
            self.config['origin']['y'],
            self.config['origin']['z']
        )

        self.relative_rest_pos = Point3D(
            self.config['relative_rest_pos']['x'],
            self.config['relative_rest_pos']['y'],
            self.config['relative_rest_pos']['z']
        )

        x_mul = -1 if self._mirror_x else 1

        self._smoother = Smoother3D(
            Point3D(initial_point.x * x_mul, initial_point.y, initial_point.z),
            self.config['speed'],
            self.config['acceleration'],
        )

    @abstractmethod
    def tick(self, reach_to: Point3D):
        pass

    @abstractmethod
    def detach(self):
        pass
