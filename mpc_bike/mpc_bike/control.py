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
