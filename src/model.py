#!/usr/bin/env python3
"""
MuJoCo Model of bicycle including important geometric factors of fork geometry
which affect stability. Parameterization follows the notation used by
Meijaard, etc al. (see docs/linearized_bicycle_equations.pdf).
"""

from mujoco_py import load_model_from_xml
import numpy as np

# TODO:
# ~~1. Make mass properties of bicycle more readily configurable.
# ~~2. Site actuation at the bike seat!
# ~~3. Convert to a more concise state to use for control.
#       ^^^ TOOK way longer than expected
# ~~4. Wrap XML into Bicycle class to make things prettier.
# 5. Path class -> piecewise:
#       a. accelerate in straight line to constant velocity
#       b. feedforward impulse to one side to get wheel to turn
#       c. maintain curve
#       d. Impulse base to left to straighten

# Default model parameters.
R_BACK = 0.3  # m
R_FRONT = 0.35  # m
WHEELBASE = 1.02  # m
TRAIL = 0.08  # m
HEAD_ANGLE = 12  # deg
CM_X = 0.3
CM_Z = 0.9


class Bicycle:

    """Geometric model for bicycle."""

    def __init__(self, r_b=R_BACK, r_f=R_FRONT, w=WHEELBASE, c=TRAIL,
                 lbda=HEAD_ANGLE, cm_x=CM_X, cm_z=CM_Z):
        self.r_b = r_b
        self.r_f = r_f
        self.w = w
        self.c = c
        self.lbda = np.deg2rad(lbda)
        self.cm_x, self.cm_z = cm_x, cm_z
        self.compute_derived()
        self.model = load_model_from_xml(self.xml())

    def compute_derived(self):
        cos = np.cos(self.lbda)
        sin = np.sin(self.lbda)
        tan = np.tan(self.lbda)
        self.l_b = ((self.r_b / tan + self.w + self.c) * cos
                    - self.r_b / sin)

        # Coordinates in global coordinate system.
        fx = self.w - self.l_b * cos
        fz = -self.l_b * sin - self.r_b + self.r_f
        self.fork_x = cos * fx + sin * fz
        self.fork_z = -sin * fx + cos * fz

        # Messy manual frame handling to convert the center of mass
        # expressed relative to the rear contact, into the frame
        # of at the rear wheel center and rotated by lambda. (This frame is
        # used so that l_b can be used conveniently.)
        self.body_x = cos * self.cm_x + sin * (self.cm_z - self.r_b)
        self.body_z = -sin * self.cm_x + cos * (self.cm_z - self.r_b)

    @property
    def r_back(self):
        """Radius of rear wheel."""
        return self.r_b

    @property
    def r_front(self):
        """Radius of front wheel."""
        return self.r_f

    @property
    def wheelbase(self):
        """Distance between wheel centers."""
        return self.w

    @property
    def trail(self):
        """Distance between fork axis and wheel contact."""
        return self.r_b

    @property
    def l_back(self):
        """Length of back frame segment."""
        return self.l_b

    @property
    def head_angle(self):
        """Angle of fork from ground."""
        return np.rad2deg(self.lbda)

    def xml(self):
        return f"""
        <?xml version="1.0" ?>
        <mujoco>
            <option timestep="0.005" />
            <option collision="predefined"/>
            <asset>
                <texture builtin="checker" height="100" name="texplane"
                    rgb1="0 0 0" rgb2="0.8 0.8 0.8" type="2d" width="100"/>
                <material name="geom" texture="texplane" texuniform="true"/>
            </asset>

            <worldbody>
                <body name="rear wheel" pos="0 0 {self.r_back}">
                    <joint damping="0" name="slide0" pos="0 0 0" type="free"/>
                    <geom name="rear wheel" mass="2" pos="0 0 0" rgba="1 0 0 1"
                        size="{self.r_back} 0.01" axisangle="1 0 0 90"
                        type="cylinder"/>
                    <camera euler="0 0 0" fovy="40" name="rgb" pos="0 0 2.5"/>
                    <!--
                    <inertial pos="0 0 0" mass="2"
                        fullinertia="0.0603 0.12 0.0603 0 0 0"/>
                    -->
                <body name="rear frame" pos="0 0 0"
                        axisangle="0 -1 0 {self.head_angle}">
                    <joint axis="0 1 0" name="pedal" type="hinge"/>
                    <geom pos="{self.l_back / 2} 0 0" mass="40"
                        axisangle="0 1 0 90"
                        rgba="0 1 0 1" size="0.05 {self.l_back / 2}"
                        type="cylinder"/>
                    <!--
                    <inertial pos="{self.body_x} 0 {self.body_z}" mass="40"
                        fullinertia="9.2 11 2.8 0 2.4 0"/>
                    -->
                    <site name="seat" pos="0.5 0 0.3"
                        axisangle="0 1 0 {self.head_angle}" size="0.05"/>
                <body name="front frame" pos="{self.l_back} 0 0">
                    <joint axis="0 0 1" name="steering" type="hinge"/>
                    <geom pos="0 0 {self.fork_z / 2}" mass="4"
                        rgba="0 0 1 1" size="0.05 {abs(self.fork_z) / 2}"
                        type="cylinder"/>
                    <inertial pos="0 0 {self.fork_z / 2}" mass="4"
                        fullinertia="0.05892 0.06 0.00708 0 -0.00756 0"/>
                <body name="front wheel" pos="{self.fork_x} 0 {self.fork_z}"
                        axisangle="0 1 0 {self.head_angle}">
                    <joint axis="0 1 0" name="front bearing" type="hinge"/>
                    <geom name="front wheel" pos="0 0 0" axisangle="1 0 0 90"
                        mass="3"
                        size="{self.r_front} 0.01" type="cylinder"/>
                    <!--
                    <inertial pos="{self.fork_x} 0 {self.fork_z}" mass="3"
                        fullinertia="0.1405 0.28 0.1405 0 0 0"/>
                    -->
                </body>
                </body>
                </body>
                </body>
                <body name="floor" pos="0 0 -.1">
                    <geom name="floor" condim="3" size="100.0 100.0 0.02"
                        rgba="0 1 0 1" type="plane" material="geom"/>
                </body>
            </worldbody>
            <contact>
                <pair geom1="rear wheel" geom2="floor"/>
                <pair geom1="front wheel" geom2="floor"/>
            </contact>
            <actuator>
                <!--
                <motor gear="1.0" joint="pedal"/>
                <motor gear="1.0" joint="steering"/>
                -->
                <general name="forward push" site="seat" gear="1 0 0 0 0 0"/>
                <general name="side push" site="seat" gear="0 -1 0 0 0 0"/>
            </actuator>
        </mujoco>
        """
