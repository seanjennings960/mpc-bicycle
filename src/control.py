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


class PathController:
    def __init__(self, bic):
        self.state = np.zeros(7, dtype=float)
        self.bic = bic

    def extract_state(self, sim):
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
        # self.state[4] = qt.as_rotation_vector(rear.conj() * front)[2]
        # self.state[5] = qt.as_rotation_vector(rear.conj() * rear_wheel)[1]
        # self.state[6] = qt.as_rotation_vector(front.conj() * front_wheel)[1]
        for i, quat, axis in [(4, rear.conj() * front, 2),
                              (5, rear.conj() * r_wheel, 1),
                              (6, front.conj() * f_wheel, 1)]:
            self.state[i] = unwrap_rotation(self.state[i], quat, axis)

        return self.state

    def action(self, data):
        return np.zeros(3, dtype=np.float32)
