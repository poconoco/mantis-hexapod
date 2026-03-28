import yaml
import time
import board

from engine.py_kinematics.point3d import Point3D
from engine.utils.pca_pool import PcaPool
from engine.utils.servo import Servo
from engine.leg import Leg3DOF

def main():

    i2c = board.I2C()
    pca_pool = PcaPool(i2c)

    with open('config.yaml', 'r') as config_file:
        config = yaml.safe_load(config_file)

    #servo = Servo(pca_pool, config['legs']['left_rear']['coxa']['servo'])
    #servo.move(0)



    left_center_point = Point3D(50, -20, -80)
    left_rear_leg = Leg3DOF(config['legs']['left_rear'], pca_pool, left_center_point)

    left_rear_leg.tick(left_center_point + Point3D(0, 0, 0))
    left_rear_leg.tick(left_center_point + Point3D(0, 0, 0))

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
