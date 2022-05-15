import quaternion as qt
import numpy as np
import pytest
from mpc_bike.path import Path


ZERO = np.zeros(50)
EPS = 1e-6


def _gen_diag_line():
    diag = np.sqrt(2) / 2
    t0 = [diag, -diag]
    length = np.sqrt(2)

    rot45 = qt.from_rotation_vector([0, 0, np.deg2rad(-45)])
    diag_line = qt.from_vector_part([1, 1, 0]) + (
        rot45 * qt.from_vector_part(
            np.c_[np.linspace(0, length), ZERO, ZERO]) * rot45.conj())

    return ([[1, 1], [2, 0]], t0,
            qt.as_vector_part(diag_line)[:, :2], length)


def _gen_circular_segment():
    points = [[0, 0], [1, 1]]
    t0 = [0, 1]
    length = np.pi / 2
    positions = np.array([
        [-np.cos(i), np.sin(i)] for i in np.linspace(0, length)
    ])
    # Add center of circle
    positions += [1, 0]
    return points, t0, positions, length


def _gen_partial_circle():
    #              .
    #           ..   slope = np.sqrt(3) / 3 = (1/2) / (sqrt(3) / 2)
    #         .
    #          \     H = np.sqrt(3) / 2
    #      60deg\
    #    . -------x
    #        R=1
    points = [[-1, 0], [-1/2, np.sqrt(3) / 2]]
    t0 = [0, 1]
    length = np.pi / 3
    positions = np.array([
        [-np.cos(i), np.sin(i)] for i in np.linspace(0, length)
    ])
    return points, t0, positions, length


def _gen_multipart():
    points = [[0, 0], [3, 0], [4, 1]]
    t0 = [1, 0]
    length = 3 + np.pi / 2

    s = np.linspace(0, length)
    seg1 = np.c_[s, ZERO]
    seg2 = np.array([
        [3 + np.sin(t), 1 - np.cos(t)] for t in (s - 3)
    ])
    positions = np.where(s < 3, seg1.T, seg2.T).T

    return points, t0, positions, length


@pytest.mark.parametrize(
    ('points', 't0', 'position', 'length'),
    [
        ([[0, 0], [0, 1]], [0, 1],
            np.c_[ZERO, np.linspace(0, 1)], 1),
        ([[0, 0], [-2, 0]], [-1, 0],
            np.c_[np.linspace(0, -2), ZERO], 2),
        _gen_diag_line(),
        _gen_circular_segment(),
        _gen_partial_circle(),
        _gen_multipart(),
    ]
)
def test_linear_path(points, t0, position, length):
    path = Path(points, t0)
    assert np.isclose(path.length, length)
    path_x = np.concatenate([path.x(i)[np.newaxis, ...]
                             for i in np.linspace(0, length)])
    assert np.all(np.isclose(path_x, position))

    for i, (p1, p2) in enumerate(zip(points[:-1], points[1:])):
        seg = path.segments[i]
        # Make sure segment endpoints hit each of the given points.
        assert np.all(np.isclose(p1, seg.x(0)))
        assert np.all(np.isclose(p2, seg.x(seg.length)))

    with pytest.raises(ValueError):
        path.x(length + EPS)

    with pytest.raises(ValueError):
        path.x(-EPS)
