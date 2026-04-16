import time

from .py_kinematics.point3d import Point3D
from .py_kinematics.ik4dof import IK4DOF
from .utils.pca_pool import PcaPool
from .utils.servo import Servo
from .base_leg import BaseLeg


class Leg4DOF(BaseLeg):
    def __init__(self, leg_config: dict, pca_pool: PcaPool):
        super().__init__(leg_config)

        if leg_config['dof'] != 4:
            raise ValueError(f'Expected 4 DOF for leg, got {leg_config["dof"]}')

        self._ik = IK4DOF()
        self._ik.coxa1_h_offset = leg_config['coxa']['h_offset']
        self._ik.coxa1_v_offset = leg_config['coxa']['v_offset']

        self._ik.coxa2_length = leg_config['coxa']['length2']
        self._ik.femur_length = leg_config['femur']['length']
        self._ik.tibia_length = leg_config['tibia']['length']

        coxa2_angle_physical_delta = leg_config['coxa'].get('angle2_physical_delta', 0.0)
        angle_physical_delta = leg_config['tibia'].get('angle_physical_delta', 0.0)

        self._ik.coxa1_for_perpendicular = leg_config['coxa']['angle1_for_perpendicular']
        self._ik.coxa2_angle_for_horizontal = leg_config['coxa']['angle2_for_horizontal'] - coxa2_angle_physical_delta

        self._ik.coxa2_min_angle = -90 + coxa2_angle_physical_delta
        self._ik.coxa2_max_angle = 90 + coxa2_angle_physical_delta

        self._ik.femur_angle_for_coxa2_parallel = leg_config['femur']['angle_for_coxa2_parallel'] + coxa2_angle_physical_delta
        self._ik.tibia_angle_for_femur_parallel = leg_config['tibia']['angle_for_femur_parallel'] - angle_physical_delta

        self._coxa1_servo = Servo(pca_pool, leg_config['coxa']['servo1'])
        self._coxa2_servo = Servo(pca_pool, leg_config['coxa']['servo2'])
        self._femur_servo = Servo(pca_pool, leg_config['femur']['servo'])
        self._tibia_servo = Servo(pca_pool, leg_config['tibia']['servo'])

    def tick(self, reach_to: Point3D):
        now = time.time()
        if self._prev_tick is None:
            self._prev_tick = now
            return

        x_mul = -1 if self._mirror_x else 1

        self._smoother.set_target(Point3D(reach_to.x * x_mul, reach_to.y, reach_to.z))
        self._smoother.tick(now - self._prev_tick)
        self._prev_tick = now

        smoothed_point = self._smoother.get()

        coxa1_angle, coxa2_angle, femur_angle, tibia_angle = self._ik.get_angles(smoothed_point)

        #print(f'reach_to: {reach_to}, smoothed_point: {smoothed_point}')

        #print(f'Angles: C1 {int(coxa1_angle)}, C2 {int(coxa2_angle)}, F {int(femur_angle)}, T {int(tibia_angle)}')

        self._coxa1_servo.move(coxa1_angle)
        self._coxa2_servo.move(coxa2_angle)
        self._femur_servo.move(femur_angle)
        self._tibia_servo.move(tibia_angle)

    def detach(self):
        self._coxa1_servo.detach()
        self._coxa2_servo.detach()
        self._femur_servo.detach()
        self._tibia_servo.detach()
