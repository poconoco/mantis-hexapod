import time

from .py_kinematics.point3d import Point3D
from .py_kinematics.ik3dof import IK3DOF
from .py_kinematics.smoother3d import Smoother3D
from .utils.pca_pool import PcaPool
from .utils.servo import Servo


class Leg3DOF:
    def __init__(self, leg_config: dict, pca_pool: PcaPool, initial_point: Point3D):
        self.prev_tick = None
        self.ik = IK3DOF()
        self.ik.coxa_h_offset = leg_config['coxa']['h_offset']
        self.ik.coxa_v_offset = leg_config['coxa']['v_offset']

        self.ik.femur_length = leg_config['femur']['length']
        self.ik.tibia_length = leg_config['tibia']['length']

        self.ik.coxa_angle_for_perpendicular = leg_config['coxa']['angle_for_perpendicular']
        self.ik.femur_angle_for_horizontal = leg_config['femur']['angle_for_horizontal']

        # angle_phisical_delta is used for femur that are L-shaped, and it's easier to
        # calibrate angles for the straight part of the tibia, but the real tibia is diagonal
        # between the ends of L
        self.ik.tibia_angle_for_femur_parallel = leg_config['tibia']['angle_for_femur_parallel'] + \
            leg_config['tibia'].get('angle_phisical_delta', 0.0) * leg_config['tibia'].get('multiplier', 1.0)

        #self.ik.coxa_multiplier = leg_config['coxa'].get('multiplier', 1.0)
        #self.ik.femur_multiplier = leg_config['femur'].get('multiplier', 1.0)
        #self.ik.tibia_multiplier = leg_config['tibia'].get('multiplier', 1.0)

        self.mirror_x = leg_config.get('mirror_x', False)

        if self.mirror_x:
            initial_point.x *= -1

        self.smoother = Smoother3D(
            initial_point,
            leg_config['speed'],
            leg_config['acceleration'],
            1.0
        )

        self.coxa_servo = Servo(pca_pool, leg_config['coxa']['servo'])
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

        coxa_angle, femur_angle, tibia_angle = self.ik.get_angles(smoothed_point)

        print(f'C: {coxa_angle}, F:{femur_angle}, T:{tibia_angle}')

        self._set_servo_angle(self.coxa_servo, coxa_angle)
        self._set_servo_angle(self.femur_servo, femur_angle)
        self._set_servo_angle(self.tibia_servo, tibia_angle)

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

