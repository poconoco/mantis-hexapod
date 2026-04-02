import sys
import yaml
import time
import board

from engine.py_kinematics.point3d import Point3D
from engine.utils.pca_pool import PcaPool
from engine.utils.servo import Servo
from engine.leg import create_leg
from engine.bt_remote.receivers.python.tcp_rc_receiver import TcpRcReceiver

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

    # left_leg.tick(left_point + Point3D(0, 0, 0))
    # left_leg.tick(left_point + Point3D(0, 0, 0))
    # right_leg.tick(right_point + Point3D(0, 0, 0))
    # right_leg.tick(right_point + Point3D(0, 0, 0))

    rc = TcpRcReceiver(print_debug=True)
    scale = 0.5

    print("\nPress Ctrl+C key to exit...")
    try:
        while True:
            if rc.is_connected():
                #print('\r'+rc_state_to_str(rc), end='', flush=True)
                rc.send(f'Live for: {round(rc.get_connected_time())}s')

                left_leg.tick(
                    left_point + Point3D(
                        rc.get_slider_left(),
                        -rc.get_x1(),
                        rc.get_y1()
                    ) * scale
                )

                right_leg.tick(
                    right_point + Point3D(
                        rc.get_slider_right(),
                        rc.get_x2(),
                        rc.get_y2()
                    ) * scale
                )
            else:
                left_leg.detach()
                right_leg.detach()

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        left_leg.detach()
        right_leg.detach()



if __name__ == "__main__":
    main()
