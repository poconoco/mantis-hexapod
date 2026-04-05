from .utils.pca_pool import PcaPool
from .utils.pca_pool import PcaPool
from .utils.servo import Servo
from .py_kinematics.smoother3d import Smoother3D
from .py_kinematics.point3d import Point3D


class Tail:
    def __init__(self, tail_config: dict, pca_pool: PcaPool):
        self._servo = Servo(pca_pool, tail_config['servo'])

        # I'm too lazy to write 1D smoother for angle-only smoothing, so just reuse 3D smoother
        self._smoother = Smoother3D(
            Point3D(0, 0, 0),
            10, # Speed
            10, # Acceleration
        )

    def tick(self):
        pass

    def detach(self):
        pass
