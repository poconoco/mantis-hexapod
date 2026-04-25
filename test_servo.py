import sys
import yaml
import time
import board

from engine.py_kinematics.point3d import Point3D
from engine.servo_utils.pca_pool import PcaPool
from engine.servo_utils.servo import Servo
from engine.leg import create_leg

def main():

    i2c = board.I2C()
    pca_pool = PcaPool(i2c)

    with open('config.yaml', 'r') as config_file:
        config = yaml.safe_load(config_file)


    servo = Servo(pca_pool, config['legs'][5]['tibia']['servo'])
    servo.move(90-15.2 - 73)
    sys.exit()

    #left_point1 = Point3D.from_dict(config['legs'][2]['relative_rest_pos'])
    #right_point1 = Point3D.from_dict(config['legs'][3]['relative_rest_pos'])

    left_point1 = Point3D(-180, 0, -120)
    right_point1 = Point3D(180, 0, -120)

    left_leg1 = create_leg(config['legs'][4], pca_pool, left_point1)
    right_leg1 = create_leg(config['legs'][5], pca_pool, right_point1)

#    left_leg1.tick(left_point1 + Point3D(0, 0, 0))
#    left_leg1.tick(left_point1 + Point3D(0, 0, 0))

#    right_leg1.tick(right_point1 + Point3D(0, 0, 0))
#    right_leg1.tick(right_point1 + Point3D(0, 0, 0))


    print("\nPress Ctrl+C key to exit...")
    try:
        while True:
            # gait.tick()

            # left_leg1.tick(left_point1 + gait.get_tip_offset(left_leg1))
            # right_leg1.tick(right_point1 + gait.get_tip_offset(right_leg1))

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nExiting...")


if __name__ == "__main__":
    main()
