from abc import ABC, abstractmethod

from .py_kinematics.point3d import Point3D


class BaseLeg(ABC):

    @abstractmethod
    def tick(self, reach_to: Point3D):
        pass

    @abstractmethod
    def detach(self):
        pass
