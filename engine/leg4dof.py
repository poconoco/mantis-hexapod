import time

from .py_kinematics.point3d import Point3D
from .py_kinematics.ik4dof import IK4DOF
from .py_kinematics.smoother3d import Smoother3D
from .utils.pca_pool import PcaPool
from .utils.servo import Servo


class Leg4DOF:
    def __init__(self, leg_config: dict, pca_pool: PcaPool, initial_point: Point3D):
        self.prev_tick = None
        self.ik = IK4DOF()
        self.ik.coxa1_h_offset = leg_config['coxa']['h_offset']
        self.ik.coxa1_v_offset = leg_config['coxa']['v_offset']

        self.ik.coxa2_length = leg_config['coxa']['length2']
        self.ik.femur_length = leg_config['femur']['length']
        self.ik.tibia_length = leg_config['tibia']['length']

        coxa2_angle_physical_delta = leg_config['coxa'].get('angle2_physical_delta', 0.0)
        self.ik.coxa1_for_perpendicular = leg_config['coxa']['angle1_for_perpendicular']
        self.ik.coxa2_angle_for_horizontal = leg_config['coxa']['angle2_for_horizontal'] + \
            coxa2_angle_physical_delta

        self.ik.coxa2_min_angle = 0 + coxa2_angle_physical_delta
        self.ik.coxa2_max_angle = 180 + coxa2_angle_physical_delta

        self.ik.femur_angle_for_coxa2_parallel = leg_config['femur']['angle_for_coxa2_parallel']

        self.ik.tibia_angle_for_femur_parallel = leg_config['tibia']['angle_for_femur_parallel'] + \
            leg_config['tibia'].get('angle_physical_delta', 0.0)

        self.mirror_x = leg_config.get('mirror_x', False)

        if self.mirror_x:
            initial_point.x *= -1

        self.smoother = Smoother3D(
            initial_point,
            leg_config['speed'],
            leg_config['acceleration'],
            1.0
        )

        self.coxa1_servo = Servo(pca_pool, leg_config['coxa']['servo1'])
        self.coxa2_servo = Servo(pca_pool, leg_config['coxa']['servo1'])
        self.femur_servo = Servo(pca_pool, leg_config['femur']['servo'])
        self.tibia_servo = Servo(pca_pool, leg_config['tibia']['servo'])

    def tick(self, reach_to: Point3D):
        now = time.time()
        if self.prev_tick is None:
            self.prev_tick = now
            return

        if self.mirror_x:
            reach_to.x *= -1

        self.smoother.set_target(reach_to)
        self.smoother.tick(now - self.prev_tick)
        self.prev_tick = now

        smoothed_point = self.smoother.get()
        #print(f'Smoothed point: {smoothed_point}')

        coxa1_angle, coxa2_angle, femur_angle, tibia_angle = self.ik.get_angles(smoothed_point)

        print(f'C1: {coxa1_angle}, C2: {coxa2_angle}, F:{femur_angle}, T:{tibia_angle}')

        self._set_servo_angle(self.coxa1_servo, coxa1_angle)
        #self._set_servo_angle(self.coxa2_servo, coxa2_angle)
        #self._set_servo_angle(self.femur_servo, femur_angle)
        #self._set_servo_angle(self.tibia_servo, tibia_angle)

    def detach(self):
        self.coxa_servo.detach()
        self.femur_servo.detach()
        self.tibia_servo.detach()


    def _get_servo(self, servo_config, pca):
        return servo.Servo(
            pca.channels[servo_config['channel']],
            min_pulse=servo_config['min_pulse'],
            max_pulse=servo_config['max_pulse'])

    @staticmethod
    def _set_servo_angle(servo, angle):
        if angle is not None:
            servo.move(angle)

