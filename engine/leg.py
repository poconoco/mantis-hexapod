from .utils.pca_pool import PcaPool
from .base_leg import BaseLeg
from .leg3dof import Leg3DOF
from .leg4dof import Leg4DOF
from .utils.servo import BaseServo, Servo


def create_servo(pca_pool: PcaPool, config: dict):
    if pca_pool:
        return Servo(pca_pool, config)
    else:
        return BaseServo()


def create_leg(leg_config: dict, pca_pool: PcaPool) -> BaseLeg:
    if leg_config['dof'] == 3:
        return Leg3DOF(leg_config, pca_pool, create_servo)
    elif leg_config['dof'] == 4:
        return Leg4DOF(leg_config, pca_pool, create_servo)
    else:
        raise ValueError(f'Unsupported DOF: {leg_config["dof"]}')
