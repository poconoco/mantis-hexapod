import sys
import yaml
import time
import board

from system_detector import DESKTOP
from visualizer import Visualizer
from engine.py_kinematics.point3d import Point3D
from engine.py_kinematics.orientation3d import Orientation3D
from engine.servo_utils.pca_pool import PcaPool
from engine.servo_utils.servo import Servo
from engine.leg import create_leg
from engine.tail import Tail
from engine.gait.gait import Gait
from engine.body import Body
from engine.bt_remote.receivers.python.udp_rc_receiver import UdpRcReceiver

def main():

    if not DESKTOP:
        i2c = board.I2C()
        pca_pool = PcaPool(i2c)
        visualizer = None
    else:
        pca_pool = None
        visualizer = Visualizer()

    with open('config.yaml', 'r') as config_file:
        config = yaml.safe_load(config_file)

    run_body(config, pca_pool, visualizer)
    sys.exit()

    #servo = Servo(pca_pool, config['legs'][5]['tibia']['servo'])
    #servo.move(90-15.2 - 73)
    #sys.exit()

    #left_point1 = Point3D.from_dict(config['legs'][2]['relative_rest_pos'])
    #right_point1 = Point3D.from_dict(config['legs'][3]['relative_rest_pos'])

    left_point1 = Point3D(-180, 0, -120)
    right_point1 = Point3D(180, 0, -120)

    left_leg1 = create_leg(config['legs'][4], pca_pool, left_point1)
    right_leg1 = create_leg(config['legs'][5], pca_pool, right_point1)

    gait = Gait(config['gait'], [left_leg1, right_leg1], Gait.Type.GAIT_3_3)
    gait.set_direction(Point3D(0, 0.5, 0))

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


class VisualizerStopException(Exception):
    pass


def run_body(config: dict, pca_pool: PcaPool, visualizer: Visualizer):
    legs = []
    for leg_config in config['legs']:
        leg = create_leg(leg_config, pca_pool)
        legs.append(leg)

    gait_type = Gait.Type.GAIT_5_1
    gait_circular = True
    tail = Tail(config['tail'], pca_pool)
    gait = Gait(config['gait'], legs, gait_type, gait_circular)
    body = Body(legs, tail, gait, config['body'])

    rc = UdpRcReceiver(print_debug=True)
    translation_scale = 0.5
    stance_mode_front_legs_translation_scale = 0.5
    orientation_scale = 0.2
    prev_switch_e_state = False

    if DESKTOP:
        print("\nClose matplotly window to exit...")
    else:
        print("\nPress Ctrl+C key to exit...")

    try:
        while True:
            if rc.is_connected():
                rc.send(f'Gait: {gait.get_type().name}\n({("circular" if gait.is_circular() else "linear")})')

                new_switch_e_state = rc.get_switch_e()
                if not new_switch_e_state and prev_switch_e_state:
                    # Button e clicked
                    gait_type = gait_type.next()
                prev_switch_e_state = new_switch_e_state

                gait_circular = not rc.get_switch_a()

                if gait_type != gait.get_type() or gait_circular != gait.is_circular():
                    gait.set_gait(gait_type, gait_circular)

                stance_mode = rc.get_switch_b()
                body.set_stance_mode(stance_mode)
                if stance_mode:
                    body.set_stance_mode_front_positions(
                        Point3D(
                            rc.get_slider_left(),
                            rc.get_x1(),
                            rc.get_y1(),
                        ) * stance_mode_front_legs_translation_scale,
                        Point3D(
                            rc.get_slider_right(),
                            rc.get_x2(),
                            rc.get_y2(),
                        ) * stance_mode_front_legs_translation_scale
                    )
                else:
                    gait.set_direction(
                        Point3D(
                            normalize_rc_input(rc.get_x1()),
                            normalize_rc_input(rc.get_y1()),
                            0
                        )
                    )

                    body.set_translation(
                        Point3D(
                            - rc.get_x2(),
                            - rc.get_y2(),
                            - rc.get_slider_right()
                        ) * translation_scale
                    )

                body.set_orientation(
                    Orientation3D(
                        rc.get_pitch() * orientation_scale,
                        - rc.get_roll() * orientation_scale,
                        0
                    )
                )

            elif DESKTOP:
                gait.set_direction(Point3D(0, 0.5, 0))

            body.tick()
            if visualizer:
                if not visualizer.update(body):
                    raise VisualizerStopException()

            time.sleep(0.0005)
    except KeyboardInterrupt:
        print('\nExiting...')
    except VisualizerStopException as e:
        print('Visualizer stopped')
    finally:
        body.detach()
        if visualizer:
            visualizer.finish()


def normalize_rc_input(rc_input):
    normalized = rc_input / 127
    return min(max(normalized, -1), 1)


if __name__ == "__main__":
    main()
