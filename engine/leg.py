from .py_kinematics.point3d import Point3D
from .utils.pca_pool import PcaPool
from .base_leg import BaseLeg
from .leg3dof import Leg3DOF
from .leg4dof import Leg4DOF


def create_leg(leg_config: dict, pca_pool: PcaPool, initial_point: Point3D) -> BaseLeg:
    if leg_config['dof'] == 3:
        return Leg3DOF(leg_config, pca_pool, initial_point)
    elif leg_config['dof'] == 4:
        return Leg4DOF(leg_config, pca_pool, initial_point)
    else:
        raise ValueError(f'Unsupported DOF: {leg_config["dof"]}')
