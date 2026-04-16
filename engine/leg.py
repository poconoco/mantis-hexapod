from .utils.pca_pool import PcaPool
from .base_leg import BaseLeg
from .leg3dof import Leg3DOF
from .leg4dof import Leg4DOF
from .leg_dummy import LegDummy


def create_leg(leg_config: dict, pca_pool: PcaPool) -> BaseLeg:
    if pca_pool is None:
        print(f'Creating leg dummy')
        return LegDummy(leg_config)
    elif leg_config['dof'] == 3:
        return Leg3DOF(leg_config, pca_pool)
    elif leg_config['dof'] == 4:
        return Leg4DOF(leg_config, pca_pool)
    else:
        raise ValueError(f'Unsupported DOF: {leg_config["dof"]}')
