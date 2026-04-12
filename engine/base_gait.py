from abc import ABC, abstractmethod

from .py_kinematics.point3d import Point3D
from .base_leg import BaseLeg


class BaseGait(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def set_direction(self, direction: Point3D):
        pass

    @abstractmethod
    def tick(self):
        pass


    @abstractmethod
    def get_tip_offset(self, leg: BaseLeg) -> Point3D:
        pass
