from typing import Optional
from adafruit_motor import servo

from .pca_pool import PcaPool

class Servo:
    def __init__(self, pca_pool: PcaPool, servo_config: dict):
        pca = pca_pool.get_pca(servo_config['i2c_address'])
        self.servo = servo.Servo(
            pca.channels[servo_config['channel']],
            min_pulse=servo_config['min_pulse'],
            max_pulse=servo_config['max_pulse']
        )

        self.rest_angle = servo_config.get('rest_angle', 90)
        self.multiplier = servo_config.get('multiplier', 1)

    def move(self, angle: Optional[float]):
        # With default rest_angle, this servo util expects angle to be between -90 and 90, unlike the usual 0-180 range.
        # I find this more easy to use with trigonometry and kinematics. But it can be tuned with rest_angle attribute in the config.
        #
        # While the multiplier attribute can be used to scale the movement, its primary purpose is to invert the direction to
        # accomodate the servo motor orientation.

        if angle is None:
            # Kinematics might return None for positions that are not reachable, so we just keep the previous position in that case
            return

        servo_angle = self.rest_angle + angle * self.multiplier
        self.servo.angle = min(max(servo_angle, 0), 180)

    def detach(self):
        self.servo.angle = None
