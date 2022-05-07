#!/usr/bin/env python3
"""
Example of how bodies interact with each other. For a body to be able to
move it needs to have joints. In this example, the "robot" is a red ball
with X and Y slide joints (and a Z slide joint that isn't controlled).
On the floor, there's a cylinder with X and Y slide joints, so it can
be pushed around with the robot. There's also a box without joints. Since
the box doesn't have joints, it's fixed and can't be pushed around.
"""
from mujoco_py import load_model_from_xml, MjSim, MjViewer
import numpy as np
import os

R_BACK = 0.3  # m
R_FRONT = 0.35  # m
WHEELBASE = 1.02  # m
TRAIL = 0.08  # m
HEAD_ANGLE = 12  # deg


class Bicycle:

    def __init__(self, r_b, r_f, w, c, lbda):
        self.r_b = r_b
        self.r_f = r_f
        self.w = w
        self.c = c
        self.lbda = np.deg2rad(lbda)
        self.compute_derived()

    def compute_derived(self):
        cos = np.cos(self.lbda)
        sin = np.sin(self.lbda)
        tan = np.tan(self.lbda)
        self.l_b = ((self.r_b / tan + self.w + self.c) * cos
                    - self.r_b / sin)

        # Coordinates in global coordinate system.
        fx = self.w - self.l_b * cos
        fz = -self.l_b * sin - self.r_b + self.r_f
        print('fx: ', fx)
        print('fz: ', fz)
        self.fork_x = cos * fx + sin * fz
        self.fork_z = -sin * fx + cos * fz

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


def generate_xml_code(bic):
    print(bic.r_back)
    print(bic.fork_z)
    print(bic.fork_x)
    return f"""
    <?xml version="1.0" ?>
    <mujoco>
        <option timestep="0.005" />
        <option collision="predefined"/>
        <worldbody>
            <body name="rear wheel" pos="0 0 {bic.r_back}">
                <joint damping="0" name="slide0" pos="0 0 0" type="free"/>
                <geom name="rear wheel" mass="1.0" pos="0 0 0" rgba="1 0 0 1"
                    size="{bic.r_back} 0.01" axisangle="1 0 0 90" type="cylinder"/>
                <camera euler="0 0 0" fovy="40" name="rgb" pos="0 0 2.5"></camera>
            <body name="rear frame" pos="0 0 0" axisangle="0 -1 0 {bic.head_angle}">
                <joint axis="0 1 0" name="pedal" type="hinge"/>
                <geom mass="1.0" pos="{bic.l_back / 2} 0 0" axisangle="0 1 0 90"
                    rgba="0 1 0 1" size="0.05 {bic.l_back / 2}" type="cylinder"/>
            <body name="front frame" pos="{bic.l_back} 0 0">
                <joint axis="0 0 1" name="steering" type="hinge"/>
                <geom mass="1.0" pos="0 0 {bic.fork_z / 2}"
                    rgba="0 0 1 1" size="0.05 {abs(bic.fork_z) / 2}" type="cylinder"/>
            <body name="front wheel" pos="{bic.fork_x} 0 {bic.fork_z}"
                    axisangle="0 1 0 {bic.head_angle}">
                <joint axis="0 1 0" name="front bearing" type="hinge"/>
                <geom name="front wheel" mass="1.0" pos="0 0 0" axisangle="1 0 0 90"
                    size="{bic.r_front} 0.01" type="cylinder"/>
            </body>
            </body>
            </body>
            </body>
            <body name="floor" pos="0 0 -.1">
                <geom name="floor" condim="3" size="4.0 4.0 0.02" rgba="0 1 0 1" type="box"/>
            </body>
        </worldbody>
        <contact>
            <pair geom1="rear wheel" geom2="floor"/>
            <pair geom1="front wheel" geom2="floor"/>
        </contact>
        <actuator>
            <motor gear="2000.0" joint="pedal"/>
        </actuator>
    </mujoco>
    """

def main():
    xml = generate_xml_code(
        Bicycle(
            R_BACK, R_FRONT,
            WHEELBASE, TRAIL,
            HEAD_ANGLE
        )
    )
    model = load_model_from_xml(xml)
    sim = MjSim(model)
    viewer = MjViewer(sim)
    t = 0
    sim.data.ctrl[0] = 0.0000
    while True:
        # if t > 1000:
        #     sim.data.ctrl[0] *= -1.01
        # sim.data.ctrl[1] = math.sin(t / 10.) * 0.01
        t += 1
        sim.step()
        viewer.render()


if __name__ == '__main__':
    main()
