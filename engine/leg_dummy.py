
from .base_leg import BaseLeg
from .py_kinematics.point3d import Point3D


class LegDummy(BaseLeg):
    def __init__(self, leg_config: dict):
        super().__init__(leg_config)
        self.current_abs_pos = self.origin + self.relative_rest_pos

    def tick(self, reach_to: Point3D):
        self.current_abs_pos = self.origin + self.relative_rest_pos + reach_to

    def detach(self):
        pass
