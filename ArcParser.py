import math

# Coordinates created by this are converted into G1 commands.
#
# supports XY, XZ & YZ planes with remaining axis as helical

# Enum
ARC_PLANE_X_Y = 0
ARC_PLANE_X_Z = 1
ARC_PLANE_Y_Z = 2

# Enum
X_AXIS = 0
Y_AXIS = 1
Z_AXIS = 2
E_AXIS = 3


class ArcParser:

    def __init__(self):
        # self.printer = config.get_printer()
        self.mm_per_arc_segment = 0.1

        self.plane = ARC_PLANE_X_Y

    def G3(self, i=0.0, j=-15.0, x=0, y=0, old_x=0, old_y=0):
        clockwise = True
        # Parse parameters
        asTarget = [x, y, 0]

        I = i
        J = j
        asPlanar = (I, J)
        axes = (X_AXIS, Y_AXIS, Z_AXIS)

        # Build linear coordinates to move
        return self.planArc((old_x, old_y, 0), asTarget, asPlanar, clockwise, *axes)

    def planArc(self, currentPos, targetPos, offset, clockwise, alpha_axis, beta_axis, helical_axis):
        # todo: sometimes produces full circles

        # Radius vector from center to current location
        r_P = -offset[0]
        r_Q = -offset[1]

        # Determine angular travel
        center_P = currentPos[alpha_axis] - r_P
        center_Q = currentPos[beta_axis] - r_Q
        rt_Alpha = targetPos[alpha_axis] - center_P
        rt_Beta = targetPos[beta_axis] - center_Q
        angular_travel = math.atan2(r_P * rt_Beta - r_Q * rt_Alpha,
                                    r_P * rt_Alpha + r_Q * rt_Beta)
        if angular_travel < 0.:
            angular_travel += 2. * math.pi
        if clockwise:
            angular_travel -= 2. * math.pi

        if (angular_travel == 0.
                and currentPos[alpha_axis] == targetPos[alpha_axis]
                and currentPos[beta_axis] == targetPos[beta_axis]):
            # Make a circle if the angular rotation is 0 and the
            # target is current position
            angular_travel = 2. * math.pi

        # Determine number of segments
        linear_travel = targetPos[helical_axis] - currentPos[helical_axis]
        radius = math.hypot(r_P, r_Q)
        flat_mm = radius * angular_travel
        if linear_travel:
            mm_of_travel = math.hypot(flat_mm, linear_travel)
        else:
            mm_of_travel = math.fabs(flat_mm)
        segments = max(1., math.floor(mm_of_travel / self.mm_per_arc_segment))

        # Generate coordinates
        theta_per_segment = angular_travel / segments
        linear_per_segment = linear_travel / segments

        asE = None
        asF = None

        e_per_move = e_base = 0.
        if asE is not None:
            e_per_move = (asE - e_base) / segments

        out = []
        for i in range(1, int(segments) + 1):
            dist_Helical = i * linear_per_segment
            c_theta = i * theta_per_segment
            cos_Ti = math.cos(c_theta)
            sin_Ti = math.sin(c_theta)
            r_P = -offset[0] * cos_Ti + offset[1] * sin_Ti
            r_Q = -offset[0] * sin_Ti - offset[1] * cos_Ti

            c = [None, None, None]
            c[alpha_axis] = center_P + r_P
            c[beta_axis] = center_Q + r_Q
            c[helical_axis] = currentPos[helical_axis] + dist_Helical

            if i == segments:
                c = targetPos
            # Convert coords into G1 commands
            g1_params = {'X': c[0], 'Y': c[1], 'Z': c[2]}
            if e_per_move:
                g1_params['E'] = e_base + e_per_move

            if asF is not None:
                g1_params['F'] = asF
            # g1_gcmd = self.gcode.create_gcode_command("G1", "G1", g1_params)

            out.append((g1_params["X"], g1_params["Y"]))

        return out

