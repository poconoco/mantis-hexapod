import sys
import yaml
import time
import board

from engine.py_kinematics.point3d import Point3D
from engine.utils.pca_pool import PcaPool
from engine.utils.servo import Servo
from engine.leg import create_leg

def main():

    i2c = board.I2C()
    pca_pool = PcaPool(i2c)

    with open('config.yaml', 'r') as config_file:
        config = yaml.safe_load(config_file)


    #servo = Servo(pca_pool, config['legs'][5]['tibia']['servo'])
    #servo.move(90-15.2 - 73)
    #sys.exit()

    left_point = Point3D(-150, 130, -120)
    right_point = Point3D(150, 130, -120)

    left_leg = create_leg(config['legs'][4], pca_pool, left_point)
    right_leg = create_leg(config['legs'][5], pca_pool, right_point)

    left_leg.tick(left_point + Point3D(0, 0, 0))
    left_leg.tick(left_point + Point3D(0, 0, 0))
    right_leg.tick(right_point + Point3D(0, 0, 0))
    right_leg.tick(right_point + Point3D(0, 0, 0))

    #print("\nPress Ctrl+C key to exit...")
    #try:
        #while True:
            #left_rear_leg.tick(left_center_point + Point3D(0, 0, 0))


            #time.sleep(0.005)
    #except KeyboardInterrupt:
        #print("\nExiting...")
    #finally:
        #left_rear_leg.detach()


if __name__ == "__main__":
    main()
