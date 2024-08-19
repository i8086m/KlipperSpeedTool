# Copyright (C)
# 2016-2024  Kevin O'Connor <kevin@koconnor.net>
# 2024 i8086m "IEOX" <i8086m@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math


# Common suffixes: _d is distance (in mm), _v is velocity (in
#   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
#   seconds), _r is ratio (scalar between 0.0 and 1.0)

# Class to track each move request

avgspeed = []

class Move:
    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
        self.junction_deviation = toolhead.junction_deviation
        velocity = min(speed, toolhead.max_velocity)
        self.axes_d = axes_d = [end_pos[i] - start_pos[i] for i in (0, 1)]
        self.move_d = move_d = math.sqrt(sum([d * d for d in axes_d]))
        if move_d < .000000001:
            return
        else:
            inv_move_d = 1. / move_d
        self.axes_r = [d * inv_move_d for d in axes_d]
        self.min_move_t = move_d / velocity
        # Junction speeds are tracked in velocity squared.  The
        # delta_v2 is the maximum amount of this squared-velocity that
        # can change in this move.
        self.max_start_v2 = 0.
        self.max_cruise_v2 = velocity ** 2
        self.delta_v2 = 2.0 * move_d * self.accel
        self.max_smoothed_v2 = 0.
        self.smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel

    def limit_speed(self, speed, accel):
        speed2 = speed ** 2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)

    def calc_junction(self, prev_move):
        # Allow extruder to calculate its maximum junction
        # extruder_v2 = self.toolhead.extruder.calc_junction(prev_move, self)
        # Find max velocity using "approximated centripetal velocity"
        axes_r = self.axes_r
        prev_axes_r = prev_move.axes_r
        junction_cos_theta = -(axes_r[0] * prev_axes_r[0]
                               + axes_r[1] * prev_axes_r[1])
        if junction_cos_theta > 0.999999:
            return
        junction_cos_theta = max(junction_cos_theta, -0.999999)
        sin_theta_d2 = math.sqrt(0.5 * (1.0 - junction_cos_theta))
        R_jd = sin_theta_d2 / (1. - sin_theta_d2)
        # Approximated circle must contact moves no further away than mid-move
        tan_theta_d2 = sin_theta_d2 / math.sqrt(0.5 * (1.0 + junction_cos_theta))
        move_centripetal_v2 = .5 * self.move_d * tan_theta_d2 * self.accel
        prev_move_centripetal_v2 = (.5 * prev_move.move_d * tan_theta_d2
                                    * prev_move.accel)
        # Apply limits
        self.max_start_v2 = min(
            R_jd * self.junction_deviation * self.accel,
            R_jd * prev_move.junction_deviation * prev_move.accel,
            move_centripetal_v2, prev_move_centripetal_v2,
            self.max_cruise_v2, prev_move.max_cruise_v2,
            prev_move.max_start_v2 + prev_move.delta_v2)
        self.max_smoothed_v2 = min(
            self.max_start_v2
            , prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2)

    def set_junction(self, start_v2, cruise_v2, end_v2):
        def split_move(start, end, full_time, start_time, end_time):
            from_x = start[0] + (start_time / full_time) * (end[0] - start[0])
            from_y = start[1] + (start_time / full_time) * (end[1] - start[1])
            to_x = start[0] + (end_time / full_time) * (end[0] - start[0])
            to_y = start[1] + (end_time / full_time) * (end[1] - start[1])
            return (from_x, from_y), (to_x, to_y)

        # Determine accel, cruise, and decel portions of the move distance
        half_inv_accel = .5 / self.accel
        accel_d = (cruise_v2 - start_v2) * half_inv_accel
        decel_d = (cruise_v2 - end_v2) * half_inv_accel
        cruise_d = self.move_d - accel_d - decel_d

        # Determine move velocities
        start_v = math.sqrt(start_v2)
        cruise_v = math.sqrt(cruise_v2)
        end_v = math.sqrt(end_v2)

        # Determine time spent in each portion of move (time is the
        # distance divided by average velocity)
        accel_t = accel_d / ((start_v + cruise_v) * 0.5)
        cruise_t = cruise_d / cruise_v
        decel_t = decel_d / ((end_v + cruise_v) * 0.5)
        full_t = accel_t + cruise_t + decel_t

        avg_v = self.move_d / full_t
        avgspeed.append(avg_v)
        print(avg_v)

        # print(f'Move from ({self.start_pos[0]}, {self.start_pos[1]}) to ({self.end_pos[0]}, {self.end_pos[1]})')

        accel_move = split_move(self.start_pos, self.end_pos, full_t, 0, accel_t)
        cruise_move = split_move(self.start_pos, self.end_pos, full_t, accel_t, accel_t + cruise_t)
        decel_move = split_move(self.start_pos, self.end_pos, full_t, accel_t + cruise_t, full_t)

        return [(accel_move, (start_v, cruise_v)), (cruise_move, (cruise_v, cruise_v)), (decel_move, (cruise_v, end_v))]


# Class to track a list of pending move requests and to facilitate
# "look-ahead" across moves to reduce acceleration between moves.
class LookAheadQueue:
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.queue = []
        self.output = []

    def reset(self):
        del self.queue[:]

    def add_to_output(self, res):
        self.output = res + self.output

    def flush(self, lazy=False):
        update_flush_count = lazy
        queue = self.queue
        flush_count = len(queue)
        # Traverse queue from last to first move and determine maximum
        # junction speed assuming the robot comes to a complete stop
        # after the last move.
        delayed = []
        next_end_v2 = next_smoothed_v2 = peak_cruise_v2 = 0.
        for i in range(flush_count - 1, -1, -1):
            move = queue[i]
            reachable_start_v2 = next_end_v2 + move.delta_v2
            start_v2 = min(move.max_start_v2, reachable_start_v2)
            reachable_smoothed_v2 = next_smoothed_v2 + move.smooth_delta_v2
            smoothed_v2 = min(move.max_smoothed_v2, reachable_smoothed_v2)
            if smoothed_v2 < reachable_smoothed_v2:
                # It's possible for this move to accelerate
                if (smoothed_v2 + move.smooth_delta_v2 > next_smoothed_v2
                        or delayed):
                    # This move can decelerate or this is a full accel
                    # move after a full decel move
                    if update_flush_count and peak_cruise_v2:
                        flush_count = i
                        update_flush_count = False
                    peak_cruise_v2 = min(move.max_cruise_v2, (
                            smoothed_v2 + reachable_smoothed_v2) * .5)
                    if delayed:
                        # Propagate peak_cruise_v2 to any delayed moves
                        if not update_flush_count and i < flush_count:
                            mc_v2 = peak_cruise_v2
                            for m, ms_v2, me_v2 in reversed(delayed):
                                mc_v2 = min(mc_v2, ms_v2)
                                res = m.set_junction(min(ms_v2, mc_v2), mc_v2, min(me_v2, mc_v2))
                                self.add_to_output(res)

                        del delayed[:]
                if not update_flush_count and i < flush_count:
                    cruise_v2 = min((start_v2 + reachable_start_v2) * .5
                                    , move.max_cruise_v2, peak_cruise_v2)
                    res = move.set_junction(min(start_v2, cruise_v2), cruise_v2, min(next_end_v2, cruise_v2))
                    self.add_to_output(res)
            else:
                # Delay calculating this move until peak_cruise_v2 is known
                delayed.append((move, start_v2, next_end_v2))
            next_end_v2 = start_v2
            next_smoothed_v2 = smoothed_v2
        if update_flush_count or not flush_count:
            return

        # Remove processed moves from the queue
        del queue[:flush_count]

    def add_move(self, move):
        self.queue.append(move)
        if len(self.queue) == 1:
            return
        move.calc_junction(self.queue[-2])


# Main code to track events (and their timing) on the printer toolhead
class ToolHead:
    def __init__(self, max_velocity=500, max_accel=5000, mcr=0.5, scv=5.0):
        self.lookahead = LookAheadQueue(self)
        self.commanded_pos = [0.0, 0.0]  # Current position

        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self.min_cruise_ratio = mcr
        self.square_corner_velocity = scv

        self.junction_deviation = self.max_accel_to_decel = 0.0
        self._calc_junction_deviation()

    def flush_lookahead(self):
        self.lookahead.flush()

    # Movement commands
    def get_position(self):
        return list(self.commanded_pos)

    def set_position(self, new_pos):
        self.flush_lookahead()
        self.commanded_pos[:] = new_pos

    def move(self, new_pos, speed):
        move = Move(self, self.commanded_pos, new_pos, speed)
        if not move.move_d:
            return
        self.commanded_pos[:] = move.end_pos
        self.lookahead.add_move(move)

    def _calc_junction_deviation(self):  # вычислить перекрёсточного девианта
        scv2 = self.square_corner_velocity ** 2
        self.junction_deviation = scv2 * (math.sqrt(2.) - 1.) / self.max_accel
        self.max_accel_to_decel = self.max_accel * (1. - self.min_cruise_ratio)

    def set_max_velocity(self, max_velocity):
        self.max_velocity = max_velocity
        self._calc_junction_deviation()

    def set_accel(self, accel):
        self.square_corner_velocity = accel
        self._calc_junction_deviation()

    def set_scv(self, scv):
        self.square_corner_velocity = scv
        self._calc_junction_deviation()

    def set_mcr(self, mcr):
        self.min_cruise_ratio = mcr
        self._calc_junction_deviation()
