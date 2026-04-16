import sys
import yaml
import time
import board

from system_detector import DESKTOP
from engine.py_kinematics.point3d import Point3D
from engine.py_kinematics.orientation3d import Orientation3D
from engine.utils.pca_pool import PcaPool
from engine.utils.servo import Servo
from engine.leg import create_leg
from engine.tail import Tail
from engine.gait3x3 import Gait3x3
from engine.body import Body
from engine.bt_remote.receivers.python.tcp_rc_receiver import TcpRcReceiver

def main():

    if not DESKTOP:
        i2c = board.I2C()
        pca_pool = PcaPool(i2c)
    else:
        pca_pool = None

    with open('config.yaml', 'r') as config_file:
        config = yaml.safe_load(config_file)

    run_body(config, pca_pool)
    sys.exit()

    #servo = Servo(pca_pool, config['legs'][5]['tibia']['servo'])
    #servo.move(90-15.2 - 73)
    #sys.exit()

    #left_point1 = Point3D.from_dict(config['legs'][2]['relative_rest_pos'])
    #right_point1 = Point3D.from_dict(config['legs'][3]['relative_rest_pos'])

    left_point1 = Point3D(-180, 0, -120)
    right_point1 = Point3D(180, 0, -120)

    gait = Gait3x3(config['gait'])
    gait.set_direction(Point3D(0, 0.5, 0))

    left_leg1 = create_leg(config['legs'][4], pca_pool, left_point1)
    right_leg1 = create_leg(config['legs'][5], pca_pool, right_point1)

#    left_leg1.tick(left_point1 + Point3D(0, 0, 0))
#    left_leg1.tick(left_point1 + Point3D(0, 0, 0))

#    right_leg1.tick(right_point1 + Point3D(0, 0, 0))
#    right_leg1.tick(right_point1 + Point3D(0, 0, 0))


    print("\nPress Ctrl+C key to exit...")
    try:
        while True:
            gait.tick()

            left_leg1.tick(left_point1 + gait.get_tip_offset(left_leg1))
            right_leg1.tick(right_point1 + gait.get_tip_offset(right_leg1))

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nExiting...")



def run_body(config: dict, pca_pool: PcaPool):
    legs = []
    for leg_config in config['legs']:
        leg = create_leg(leg_config, pca_pool)
        legs.append(leg)

    tail = Tail(config['tail'], pca_pool)
    gait = Gait3x3(config['gait'])
    body = Body(legs, tail, gait)

    rc = TcpRcReceiver(print_debug=True)
    translation_scale = 0.5
    orientation_scale = 0.2

    print("\nPress Ctrl+C key to exit...")
    try:
        while True:
            if rc.is_connected():
                #print('\r'+rc_state_to_str(rc), end='', flush=True)
                rc.send(f'Live for: {round(rc.get_connected_time())}s')

                gait.set_direction(
                    Point3D(
                        normalize_rc_input(rc.get_x1()),
                        normalize_rc_input(rc.get_y1()),
                        0
                    ).cube_to_sphere()
                )

                body.translation = Point3D(
                    - rc.get_x2(),
                    - rc.get_y2(),
                    - rc.get_slider_right()
                ) * translation_scale

                body.orientation = Orientation3D(
                    rc.get_pitch() * orientation_scale,
                    - rc.get_roll() * orientation_scale,
                    - rc.get_slider_right() * orientation_scale
                )

            body.tick()

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        body.detach()


def normalize_rc_input(rc_input):
    normalized = rc_input / 127
    return min(max(normalized, -1), 1)


if __name__ == "__main__":
    main()
