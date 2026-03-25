from adafruit_motor import servo

from .pca_pool import PcaPool

class Servo:
    def __init__(self, pca_pool: PcaPool, servo_config: dict):
        pca = pca_pool.get_pca(servo_config['i2c_address'])
        self.servo = servo.Servo(pca.channels[servo_config['channel']], min_pulse=servo_config['min_pulse'], max_pulse=servo_config['max_pulse'])
        self.rest_angle = servo_config.get('rest_angle', 0)
        self.angle_physical_delta = 0 #servo_config.get('angle_physical_delta', 0)
        self.multiplier = servo_config.get('multiplier', 1)

    def move(self, angle):
        servo_angle = self.rest_angle + (self.angle_physical_delta + angle) * self.multiplier
        self.servo.angle = clamp(servo_angle, 0, 180)

    def detach(self):
        self.servo.angle = None


def clamp(value, min_value, max_value):
    return min(max(value, min_value), max_value)
