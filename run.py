import yaml
import time
import board

from engine.py_kinematics.point3d import Point3D
from engine.utils.pca_pool import PcaPool
#from engine.utils.servo import Servo
from engine.leg import Leg3DOF

def main():

    i2c = board.I2C()
    pca_pool = PcaPool(i2c)

    with open('config.yaml', 'r') as config_file:
        config = yaml.safe_load(config_file)

    #servos = (
        #Servo(pca_pool, config['legs']['left_rear']['coxa']['servo']),
        #Servo(pca_pool, config['legs']['right_rear']['coxa']['servo']),
        #Servo(pca_pool, config['legs']['left_middle']['coxa']['servo']),
        #Servo(pca_pool, config['legs']['right_middle']['coxa']['servo']),
        #Servo(pca_pool, config['legs']['left_front']['coxa']['servo1']),
        #Servo(pca_pool, config['legs']['left_front']['coxa']['servo2']),
        #Servo(pca_pool, config['legs']['right_front']['coxa']['servo1']),
        #Servo(pca_pool, config['legs']['right_front']['coxa']['servo2']),

        #Servo(pca_pool, config['legs']['left_rear']['femur']['servo']),
        #Servo(pca_pool, config['legs']['right_rear']['femur']['servo']),
        #Servo(pca_pool, config['legs']['left_middle']['femur']['servo']),
        #Servo(pca_pool, config['legs']['right_middle']['femur']['servo']),
        #Servo(pca_pool, config['legs']['left_front']['femur']['servo']),
        #Servo(pca_pool, config['legs']['right_front']['femur']['servo']),

        #Servo(pca_pool, config['legs']['left_rear']['tibia']['servo']),
        #Servo(pca_pool, config['legs']['right_rear']['tibia']['servo']),
        #Servo(pca_pool, config['legs']['left_middle']['tibia']['servo']),
        #Servo(pca_pool, config['legs']['right_middle']['tibia']['servo']),
        #Servo(pca_pool, config['legs']['left_front']['tibia']['servo']),
        #Servo(pca_pool, config['legs']['right_front']['tibia']['servo']),

        #Servo(pca_pool, config['tail']['servo']),
    #)

    #angle_base = 0
    #angle_delta = 0
    #increment = 0.5
    #while True:
        #angle_delta += increment
        #if angle_delta > 10 or angle_delta == 0:
            #increment = -increment

        #angle = angle_base + angle_delta

        #for servo_ in servos:
            #servo_.move(angle)

        #time.sleep(0.05)

#    servo = Servo(pca_pool, config['legs']['left_rear']['coxa']['servo'])
#    servo.move(-45)

    left_center_point = Point3D(-120, 50, -90)
    left_rear_leg = Leg3DOF(config['legs']['left_rear'], pca_pool, left_center_point)

    print("\nPress Ctrl+C key to exit...")
    try:
        while True:
            left_rear_leg.tick(left_center_point + Point3D(0, 0, 0))

            #if rc.is_connected():
                ##print('\r'+rc_state_to_str(rc), end='', flush=True)
                #rc.send(f'Live for: {round(rc.get_connected_time())}s')

                #left_leg.tick(
                    #left_center_point + Point3D(
                        #rc.get_slider_left(),
                        #-rc.get_x1(),
                        #rc.get_y1()
                    #) * scale
                #)

                #right_leg.tick(
                    #right_center_point + Point3D(
                        #rc.get_slider_right(),
                        #rc.get_x2(),
                        #rc.get_y2()
                    #) * scale
                #)
            #else:
                #left_leg.detach()
                #right_leg.detach()

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        left_rear_leg.detach()


if __name__ == "__main__":
    main()
