import time

from .py_kinematics.point3d import Point3D
from .py_kinematics.ik3dof import IK3DOF
from .servo_utils.pca_pool import PcaPool
from .base_leg import BaseLeg


class Leg3DOF(BaseLeg):
    def __init__(self, leg_config: dict, pca_pool: PcaPool, create_servo_func):
        super().__init__(leg_config)

        if leg_config['dof'] != 3:
            raise ValueError(f'Expected 3 DOF for leg, got {leg_config["dof"]}')

        self._ik = IK3DOF()
        self._ik.coxa_h_offset = leg_config['coxa']['h_offset']
        self._ik.coxa_v_offset = leg_config['coxa']['v_offset']

        self._ik.femur_length = leg_config['femur']['length']
        self._ik.tibia_length = leg_config['tibia']['length']

        self._ik.coxa_angle_for_perpendicular = leg_config['coxa']['angle_for_perpendicular']
        self._ik.femur_angle_for_horizontal = leg_config['femur']['angle_for_horizontal']

        # angle_physical_delta is used for femur that are L-shaped, and it's easier to
        # calibrate angles for the straight part of the tibia, but the real tibia is diagonal
        # between the ends of L
        self._ik.tibia_angle_for_femur_parallel = leg_config['tibia']['angle_for_femur_parallel'] - \
            leg_config['tibia'].get('angle_physical_delta', 0.0) * leg_config['tibia'].get('multiplier', 1.0)

        self._coxa_servo = create_servo_func(pca_pool, leg_config['coxa']['servo'])
        self._femur_servo = create_servo_func(pca_pool, leg_config['femur']['servo'])
        self._tibia_servo = create_servo_func(pca_pool, leg_config['tibia']['servo'])

    def tick(self, reach_to: Point3D):
        now = time.time()
        if self._prev_tick is None:
            self._prev_tick = now
            return

        self._smoother.set_target(reach_to)
        self._smoother.tick(now - self._prev_tick)
        self._prev_tick = now

        smoothed_point = self._smoother.get()

        self.abs_pos = self.origin + self.relative_rest_pos + reach_to
        self.smoothed_abs_pos = self.origin + self.relative_rest_pos + smoothed_point

        smoothed_point = smoothed_point.copy()
        if self._mirror_x:
            smoothed_point.x *= -1
        coxa_angle, femur_angle, tibia_angle = self._ik.get_angles(smoothed_point)

        self._coxa_servo.move(coxa_angle)
        self._femur_servo.move(femur_angle)
        self._tibia_servo.move(tibia_angle)

    def detach(self):
        self._coxa_servo.detach()
        self._femur_servo.detach()
        self._tibia_servo.detach()

