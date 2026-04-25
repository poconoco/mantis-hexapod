from abc import ABC, abstractmethod

from .py_kinematics.point3d import Point3D
from .py_kinematics.smoother3d import Smoother3D
from .servo_utils.servo import BaseServo, Servo
from .servo_utils.pca_pool import PcaPool


class Leg(ABC):

    def __init__(self, leg_config: dict):
        self.config = leg_config
        self._prev_tick = None
        self._mirror_x = self.config.get('mirror_x', False)

        self.side = self.config['side']
        self.order = self.config['order']
        self.origin = Point3D.from_dict(self.config['origin'])
        self.relative_rest_pos = Point3D.from_dict(self.config['relative_rest_pos'])
        self.stand_pos_offset = Point3D.from_dict(self.config.get('stand_pos_offset', {'x': 0, 'y': 0, 'z': 0}))
        self.abs_pos = self.origin + self.relative_rest_pos
        self.smoothed_abs_pos = self.abs_pos

        self._smoother = Smoother3D(self.relative_rest_pos,
            self.config['speed'],
            self.config['acceleration'],
        )

    @abstractmethod
    def tick(self, reach_to: Point3D):
        pass

    @abstractmethod
    def detach(self):
        pass


def create_servo(pca_pool: PcaPool, config: dict):
    if pca_pool:
        return Servo(pca_pool, config)
    else:
        return BaseServo()


def create_leg(leg_config: dict, pca_pool: PcaPool) -> Leg:
    from .leg_3dof import Leg3DOF
    from .leg_4dof import Leg4DOF
    
    if leg_config['dof'] == 3:
        return Leg3DOF(leg_config, pca_pool, create_servo)
    elif leg_config['dof'] == 4:
        return Leg4DOF(leg_config, pca_pool, create_servo)
    else:
        raise ValueError(f'Unsupported DOF: {leg_config["dof"]}')
