import numpy as np
import quaternion as qt


def decompose_yr(quat):
    rotmat = qt.as_rotation_matrix(quat)
    yaw = np.arctan2(rotmat[..., 1, 0], rotmat[..., 0, 0])

    zero = np.zeros_like(yaw)
    yaw_quat = qt.from_rotation_vector(np.c_[zero, zero, yaw])
    rest_rotmat = qt.as_rotation_matrix(yaw_quat.conj() * quat)
    lean = np.arctan2(rest_rotmat[..., 1, 2], rest_rotmat[..., 2, 2])
    return np.r_[yaw, lean]


def get_body_xquat(data, name):
    """Quaternion-typed version of MjData.get_body_quat."""
    return qt.from_float_array(data.get_body_xquat(name))


def unwrap_rotation(last, quat, axis):
    return np.unwrap([last, qt.as_rotation_vector(quat)[axis]])[-1]


# class Termination(Enum):
#
#
# class Path:
#
#     def __init__(self, t_end: float, steering_angle: float, force: float,
#                  termination: Termination):
#         self.t_end = t_end
#
#     def x(self, t):
#
#
#
# def PiecewisePath:
#     def __init__(self, paths):


class Turn:
    def __init__(self, steering_angle, p_gain):
        self.steering_angle = steering_angle
        self.p_gain

    def action(self, state):
        lateral_force = self.p_gain * (self.steering_angle - state[4])
        return np.r_[0, lateral_force]


class Accel:
    def __init__(self, force, p_gain):
        self.force = force

    def action(self, state):
        lateral_force = self.p_gain * - state[4]
        return np.r_[self.force, lateral_force]


class LinearSegment:
    def __init__(self, p1, p2):
        self.p1, self.p2 = p1, p2
        self.delta = self.p2 - self.p1
        self.length = np.linalg.norm(p2 - p1)

    def x(self, s):
        """Find point as a function of the arc length parameterization."""
        if s < 0 or s > self.length:
            raise ValueError('Arclength outside of range [0, {self.length}], '
                             f'got {s}')
        return self.p1 + s / self.length * self.delta


class CircularSegment:
    def __init__(self, p1, p2, center):
        start_v = p1 - center
        end_v = p2 - center
        self.start_angle = np.arctan2(start_v[1], start_v[2])
        self.end_angle = np.arctan2(end_v[1], end_v[2])
        self.center = center
        self.radius = np.linalg.norm(p1 - center)
        self.angle = abs(self.start_angle - self.end_angle)
        self.delta = self.end_angle - self.start_angle
        self.length = self.radius * self.angle

    def x(self, s):
        if s < 0 or s > self.length:
            raise ValueError('Arclength outside of range [0, {self.length}], '
                             f'got {s}')
        pos = np.exp(self.start_angle + self.delta * s)
        return self.center + [self.radius * pos.real, self.radius * pos.imag]


class Path:

    """
    Paths in a plane resticted to linear and circular segments.
    """

    def __init__(self, points, tangents):
        """
        Construct a path from a list of points and tangents.

        Arguments:
            points: list of points which the path will go through
            tangents: the tangent vector of the path at each point in
                points. Must have same length of points.
        """
        if len(points) != len(tangents):
            raise ValueError("point and tangents must have equal length "
                             f"({len(points)} != {len(tangents)}")
        self.n_segments = max(len(points) - 1, 0)
        self.segments = []

        for i in range(self.n_segments):
            p1, p2 = points[i:i+2]
            t1, t2 = tangents[i:i+2]
            self.segments.append(
                self.compute_segment(p1, p2, t1, t2))

    def compute_segment(self, p1, p2, t1, t2):
        # x1 = p1 + t1 * u1
        # x2 = p2 + t2 * u2
        # x1 = x2 -> p1 + t1 * u1 = p2 + t2 * u2
        # p2 - p1 = t1 * u1 - t2 * u2
        # p2 - p1 = [t1, -t2].T * [u1, u2]
        #            (2x2) * (2x1)
        #            [[t1.x, -t2.x],  [u1]
        #             [t1.y, -t2.y]]   u2]
        A = np.c_[t1, t2].T
        if np.isclose(np.linalg.det(A), 0):
            return LinearSegment(p1, p2)

        u = np.linalg.pinv(A) @ (p2 - p1)
        center = p1 + t1 * u[0]
        return CircularSegment(p1, p2, center)


class PathController:
    def __init__(self, bic, path):
        self.state = np.zeros(7, dtype=float)
        self.bic = bic
        self.path = path

    def extract_state(self, sim):
        """
        Converts current simulation data into controller state.

        State Data Format:
            7-array
            0: x of rear wheel
            1: y of rear wheel
            2: yaw of rear wheel
            3: lean angle
            4: steering angle
            5: rear wheel angle
            6: front wheel angle
        """
        # TODO: Compute rear contact point?
        # geom_id = sim.model.geom_name2id('rear wheel')
        # rear_radius = sim.model.geom_size[geom_id][0]
        data = sim.data
        head_angle = self.bic.head_angle
        rear = get_body_xquat(data, 'rear frame')
        r_wheel = get_body_xquat(data, 'rear wheel')
        front = get_body_xquat(data, 'front frame')
        f_wheel = get_body_xquat(data, 'front wheel')

        self.state[:2] = data.get_body_xpos('rear wheel')[:2]

        quat = (rear *
                qt.from_rotation_vector(np.deg2rad([0, head_angle, 0])))
        self.state[2:4] = np.unwrap([self.state[2:4], decompose_yr(quat)],
                                    axis=0)[-1]

        # Steering, rear wheel, and front wheel angles.
        for i, quat, axis in [(4, rear.conj() * front, 2),
                              (5, rear.conj() * r_wheel, 1),
                              (6, front.conj() * f_wheel, 1)]:
            self.state[i] = unwrap_rotation(self.state[i], quat, axis)

        return self.state

    def action(self, data):
        return np.zeros(3, dtype=np.float32)
