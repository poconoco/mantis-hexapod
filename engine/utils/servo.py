from adafruit_motor import servo

from .pca_pool import PcaPool

class Servo:
    def __init__(self, pca_pool: PcaPool, servo_config: dict):
        pca = pca_pool.get_pca(servo_config['i2c_address'])
        self.servo = servo.Servo(pca.channels[servo_config['channel']], min_pulse=servo_config['min_pulse'], max_pulse=servo_config['max_pulse'])
        self.rest_angle = servo_config.get('rest_angle', 0)
        self.multiplier = servo_config.get('multiplier', 1)

    def move(self, angle: float):
        if angle is None:
            return

        servo_angle = self.rest_angle + angle * self.multiplier
        self.servo.angle = min(max(servo_angle, 0), 180)

    def detach(self):
        self.servo.angle = None
