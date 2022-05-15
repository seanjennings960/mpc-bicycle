import bisect
import numpy as np

ROT90 = np.array([[0, -1], [1, 0]])


class Segment:
    # TODO: Separate these again into CircularSegment and LinearSegment.
    # Move __init__ code to a factory method.

    def __init__(self, p1, p2, t):
        # Compute the center of the circle. Compare
        # the tangent and secant (p1 -> p2) vectors to find angle
        # radius and center.
        self.p1 = p1
        self.t = t
        sec = p2 - p1
        sec_norm = np.linalg.norm(sec)
        phi = np.arccos(np.dot(sec, t) / sec_norm)

        if not np.isclose(phi, 0):
            self.angle = 2 * phi
            self.dir = np.sign(np.cross(t, sec))
            self.radius = sec_norm / (2 * np.sin(phi))
            # n_hat is center -> p1
            n_hat = - self.dir * ROT90 @ t
            self.center = p1 - self.radius * n_hat
            self.start_angle = np.arctan2(n_hat[1], n_hat[0])
            self.length = self.radius * self.angle
        else:
            self.angle = None
            self.dir = None
            self.radius = None
            self.center = None
            self.start_angle = None
            self.length = sec_norm

    def x(self, s):
        if s < 0 or s > self.length:
            raise ValueError(f'Arclength outside of range [0, {self.length}], '
                             f'got {s}')
        if self.angle is None:
            # Degenerate linear case.
            return self.p1 + s * self.t
        angle = self.start_angle + self.dir * s / self.radius
        z = self.radius * np.exp(complex(0, angle))
        return self.center + [z.real, z.imag]


class Path:

    """
    Paths in a plane resticted to linear and circular segments.
    """

    def __init__(self, points, t0):
        """
        Construct a path from a list of points and tangents.

        Arguments:
            points: list of points which the path will go through
            tangents: the tangent vector of the path at each point in
                points. Must have same length of points.
        """
        points = np.asarray(points)
        t0 = np.asarray(t0)

        assert points.ndim == 2, 'Points must both be 2D'
        assert points.shape[1] == 2, ('Points must both have '
                                      'length 2 along axis 1.')
        assert t0.shape == (2,), 't0 must be length 2'

        assert np.all(np.isclose(np.linalg.norm(t0), 1)), (
            't0 must have norm 1.')

        self.n_segments = max(len(points) - 1, 0)
        self.segments = [Segment(points[i], points[i + 1], t0)
                         for i in range(self.n_segments)]

        self.lengths = [seg.length for seg in self.segments]
        self.length = np.sum(self.lengths)
        self._lengths_cumsum = np.r_[0, np.cumsum(self.lengths)]

    def x(self, s):
        if s < 0 or s > self.length:
            raise ValueError(
                f'Path arclength must be between [0, {self.length}, (length)]')

        i = min(bisect.bisect(self._lengths_cumsum, s) - 1,
                self.n_segments - 1)
        # print('s', s)
        # print('length_cumsum', self._lengths_cumsum)
        # print('i', i)
        s_new = s - self._lengths_cumsum[i]
        # print('s_new', s_new)
        return self.segments[i].x(s_new)


class Angle:
    def __init__(self, deg):
        self.deg = deg
        self.rad = np.deg2rad(deg)
        self.z = np.exp(complex(0, self.rad))
        self.vector = np.array([self.z.real, self.z.imag])


TEST_PATHS = [
    Path([[0, 0], [3, 0], [7, 0.1]], [1, 0])
]
