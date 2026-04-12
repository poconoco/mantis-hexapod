import time

from .utils.pca_pool import PcaPool
from .utils.pca_pool import PcaPool
from .utils.servo import Servo
from .py_kinematics.smoother3d import Smoother3D
from .py_kinematics.point3d import Point3D


class Tail:
    def __init__(self, tail_config: dict, pca_pool: PcaPool):
        self._servo = Servo(pca_pool, tail_config['servo'])
        self._prev_tick = None

        # I'm too lazy to write 1D smoother for angle-only smoothing, so just reuse 3D smoother
        # but use only x
        self._smoother = Smoother3D(
            Point3D(0, 0, 0),
            100, # Speed
            200, # Acceleration
        )

    def tick(self, angle: float):
        now = time.time()
        if self._prev_tick is None:
            self._prev_tick = now
            return

        self._smoother.set_target(Point3D(angle, 0, 0))
        self._smoother.tick(now - self._prev_tick)
        self._prev_tick = now

        smoothed_angle = self._smoother.get().x
        self._servo.move(smoothed_angle)

    def detach(self):
        self._servo.detach()
