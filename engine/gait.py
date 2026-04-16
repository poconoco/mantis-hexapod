import math
import time

from .py_kinematics.point3d import Point3D


class LegCycle:
    def __init__(self):
        self._cycle_shift = 0
        self._turn = False
        self._turn_angle = 0.0
        self._turn_cx = 0.0
        self._turn_cy = 0.0
        self._step_height = 50  # TODO: read from config

        self._start = None
        self._stop = None

        self.result_offset = Point3D(0, 0, 0)
        self.result_rotation_angle = 0.0
        self.result_rotation_center = Point3D(0, 0, 0)

        self.set_cycle_middle(127)

    def set_cycle_shift(self, shift: int):
        self._cycle_shift = shift

    def set_cycle_middle(self, middle: int):
        self._cycle_middle = middle
        self._inv_cycle_middle = 1.0 / float(self._cycle_middle)

        self._cycle_after_middle = 255 - self._cycle_middle
        self._inv_cycle_after_middle = 1.0 / float(self._cycle_after_middle)

    def set_step(self, 
                 start, 
                 stop, 
                 turn: bool, 
                 turn_angle: float, 
                 turn_cx: float, 
                 turn_cy: float):
        self._start = start
        self._stop = stop
        self._turn = turn
        self._turn_angle = turn_angle
        self._turn_cx = turn_cx
        self._turn_cy = turn_cy

    def set_cycle_pos(self, pos: int):
        norm1pos, height = self._cycle(self._shift_cycle_pos(pos))

        if not self._turn:
            point_x = self._start.x + (self._stop.x - self._start.x) * norm1pos
            point_y = self._start.y + (self._stop.y - self._start.y) * norm1pos
            point_z = self._start.z + (self._stop.z - self._start.z) * norm1pos + height
            
            self.result_offset = Point3D(point_x, point_y, point_z)
            self.result_rotation_angle = None
            self.result_rotation_center = None
        else:
            self.result_offset = Point3D(self._start.x, self._start.y, self._start.z + height)
            self.result_rotation_angle = self._turn_angle * norm1pos
            self.result_rotation_center = Point3D(self._turn_cx, self._turn_cy, 0)

    def _shift_cycle_pos(self, pos: int) -> int:
        return (pos + self._cycle_shift) & 0xFF

    def _cycle(self, cycle_pos: int) -> tuple[float, float]:
        if cycle_pos <= self._cycle_middle:
            norm1pos = float(cycle_pos) * self._inv_cycle_middle
            height = 0.0
        else:
            norm1pos = float(self._cycle_after_middle - (cycle_pos - self._cycle_middle)) * self._inv_cycle_after_middle
            height = self._step_height * (1.0 - math.fabs(0.5 - norm1pos) * 2.0)
            
        return norm1pos, height


class Gait:
    def __init__(self, leg_cycles: list[LegCycle]):
        self._leg_cycles = leg_cycles

        self._step_per_second = SmoothFloat(0.0, 0.0)
        self._prev_tick = None
        self._current_cycle_pos = 0.0
        
        self._turn = SmoothFloat(0.0, 0.0)
        self._direction_x = SmoothFloat(0.0, 0.0)
        self._direction_y = SmoothFloat(0.0, 0.0)
        self._direction_z = SmoothFloat(0.0, 0.0)
        self._do_turn = False

        self.set_gait_6x1()

    def set_gait_2x3(self):
        self._leg_cycles[0].set_cycle_shift(0)
        self._leg_cycles[1].set_cycle_shift(127)
        self._leg_cycles[2].set_cycle_shift(127)
        self._leg_cycles[3].set_cycle_shift(0)
        self._leg_cycles[4].set_cycle_shift(0)
        self._leg_cycles[5].set_cycle_shift(127)
        
        for i in range(6):
            self._leg_cycles[i].set_cycle_middle(127)

    def set_gait_3x2(self):
        self._leg_cycles[0].set_cycle_shift(0)
        self._leg_cycles[5].set_cycle_shift(0)
        
        self._leg_cycles[1].set_cycle_shift(int((255 / 3) * 1))
        self._leg_cycles[4].set_cycle_shift(int((255 / 3) * 1))
        
        self._leg_cycles[2].set_cycle_shift(int((255 / 3) * 2))
        self._leg_cycles[3].set_cycle_shift(int((255 / 3) * 2))
        
        for i in range(6):
            self._leg_cycles[i].set_cycle_middle(int(255 - (255 / 3)))

    def set_gait_6x1(self):
        self._leg_cycles[0].set_cycle_shift(0)
        self._leg_cycles[1].set_cycle_shift(int((255 / 6) * 1))
        self._leg_cycles[2].set_cycle_shift(int((255 / 6) * 2))
        self._leg_cycles[3].set_cycle_shift(int((255 / 6) * 3))
        self._leg_cycles[4].set_cycle_shift(int((255 / 6) * 4))
        self._leg_cycles[5].set_cycle_shift(int((255 / 6) * 5))
        
        for i in range(6):
            self._leg_cycles[i].set_cycle_middle(int(255 - (255 / 6)))

    def set_step(self, dir_point, turn: bool, turn_value: float):
        self._direction_x.set_target(dir_point.x)
        self._direction_y.set_target(dir_point.y)
        self._direction_z.set_target(dir_point.z)
        self._turn.set_target(turn_value)
        self._do_turn = turn

    def set_speed(self, steps_per_second: float):
        self._step_per_second.set_target(steps_per_second)

    def tick(self):
        now = time.time()
        if self._prev_tick is None:
            self._prev_tick = now
            return
        
        dt_ms = now - self._prev_tick * 1000.0
        self._prev_tick = now

        speed_step_delta = 0.01 * float(dt_ms)
        direction_step_delta = 0.1 * float(dt_ms)
        turn_step_delta = 0.01 * float(dt_ms)

        self._direction_x.tick(direction_step_delta)
        self._direction_y.tick(direction_step_delta)
        self._direction_z.tick(direction_step_delta)
        self._turn.tick(turn_step_delta)

        self._tick_step()

        cycle_delta = float(dt_ms) * (255.0 * 0.001) * self._step_per_second.get_current(speed_step_delta)
        self._current_cycle_pos += cycle_delta

        # Keep within the 255 byte boundary
        while self._current_cycle_pos >= 255.0:
            self._current_cycle_pos -= 255.0

        curr_cycle_pos_byte = int(self._current_cycle_pos)

        for i in range(6):
            self._leg_cycles[i].set_cycle_pos(curr_cycle_pos_byte)

    def _tick_step(self):
        direction = Point3D(
            self._direction_x.get_current(),
            self._direction_y.get_current(),
            self._direction_z.get_current()
        )

        cx = 0.0
        cy = 0.0 
        angle = 0.0

        if self._do_turn:
            angle = self._turn.get_current() * 0.7

            cx = distance_to_horde(math.fabs(direction.y), angle)
            cy = 0.0

            if direction.y < 0:
                angle *= -1.0

        step_start = direction * 0.5
        step_stop = step_start * (-1.0)

        for i in range(6):
            self._leg_cycles[i].set_step(
                step_start,
                step_stop,
                self._do_turn,
                angle,
                cx,
                cy
            )


class SmoothFloat:
    def __init__(self, target: float, current: float):
        self._target = target
        self._current = current

    def set_target(self, target: float):
        self._target = target

    def get_current(self) -> float:
        return self._current

    def tick(self, step_delta_orig: float):
        dist = abs(self._current - self._target)
        
        if dist < step_delta_orig * 2.0:
            step_delta = step_delta_orig * 0.1
        elif dist < step_delta_orig * 5.0:
            step_delta = step_delta_orig * 0.2
        else:
            step_delta = step_delta_orig

        if self._current >= self._target:
            self._current = max(self._target, self._current - step_delta)
        else:
            self._current = min(self._target, self._current + step_delta)


def distance_to_horde(horde_length: float, horde_angle: float) -> float:
    # Prevent ZeroDivisionError crash in Python
    if horde_angle == 0.0:
        return 9999999999.0
        
    return (horde_length * 0.5) / math.tan(horde_angle * 0.5)
