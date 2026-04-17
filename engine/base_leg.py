from abc import ABC, abstractmethod

from .py_kinematics.point3d import Point3D
from .py_kinematics.smoother3d import Smoother3D


class BaseLeg(ABC):

    def __init__(self, leg_config: dict):
        self.config = leg_config
        self._prev_tick = None
        self._mirror_x = self.config.get('mirror_x', False)

        self.side = self.config['side']
        self.order = self.config['order']
        self.origin = Point3D.from_dict(self.config['origin'])
        self.relative_rest_pos = Point3D.from_dict(self.config['relative_rest_pos'])
        self.current_abs_pos = self.origin + self.relative_rest_pos
        self.smoothed_abs_pos = self.current_abs_pos

        x_mul = -1 if self._mirror_x else 1

        self._smoother = Smoother3D(
            Point3D(
                self.relative_rest_pos.x * x_mul, 
                self.relative_rest_pos.y, 
                self.relative_rest_pos.z
            ),
            self.config['speed'],
            self.config['acceleration'],
        )

    @abstractmethod
    def tick(self, reach_to: Point3D):
        pass

    @abstractmethod
    def detach(self):
        pass
