#!/usr/bin/env python3
"""
MuJoCo Model of bicycle including important geometric factors of fork geometry
which affect stability. Parameterization follows the notation used by
Meijaard, etc al. (see docs/linearized_bicycle_equations.pdf).
"""


from pynput import keyboard
from mujoco_py import const, load_model_from_xml, MjSim, MjViewer
import numpy as np

# TODO:
# 1. Make mass properties of bicycle more readily configurable.
# 2. Site actuation at the bike seat!
# 3. Convert to a more concise state to use for control.
# 4. Wrap XML into Bicycle class to make things prettier.
# 5. Path class -> piecewise:
#       a. accelerate in straight line to constant velocity
#       b. feedforward impulse to one side to get wheel to turn
#       c. maintain curve
#       d. Impulse base to left to straighten

R_BACK = 0.3  # m
R_FRONT = 0.35  # m
WHEELBASE = 1.02  # m
TRAIL = 0.08  # m
HEAD_ANGLE = 12  # deg


class Bicycle:

    """Geometric model for bicycle."""

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
                <geom name="floor" condim="3" size="100.0 100.0 0.02" rgba="0 1 0 1" type="plane"
                    material="geom"/>
            </body>
        </worldbody>
        <contact>
            <pair geom1="rear wheel" geom2="floor"/>
            <pair geom1="front wheel" geom2="floor"/>
        </contact>
        <actuator>
            <motor gear="2000.0" joint="pedal"/>
            <motor gear="2000.0" joint="steering"/>
        </actuator>
    </mujoco>
    """

class ControlOutput:

    """Convert arrow key inputs into very simple control output."""

    def __init__(self, magn_forward=0.001, magn_lr=0.0001):
        self.value = np.zeros(2, dtype=np.float64)
        self.magn_forward = magn_forward
        self.magn_lr = magn_lr

    def on_press(self, key):
        if self.value[0] == 0:
            if key == keyboard.Key.up:
                # print('we up')
                self.value[0] = self.magn_forward
            elif key == keyboard.Key.down:
                # print('we down')
                self.value[0] = - self.magn_forward

        if self.value[1] == 0:
            if key == keyboard.Key.right:
                # print('we right')
                self.value[1] = self.magn_lr
            elif key == keyboard.Key.left:
                # print('we left')
                self.value[1] = - self.magn_lr

    def on_release(self, key):
        if key == keyboard.Key.up or key == keyboard.Key.down:
            # print('we back to the ground')
            self.value[0] = 0
        if key == keyboard.Key.right or key == keyboard.Key.left:
            # print('goddamn aint we deft')
            self.value[1] = 0


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
    output = ControlOutput()

    # Setup camera to follow the bike.
    viewer.cam.type = const.CAMERA_TRACKING
    viewer.cam.trackbodyid = model.body_names.index('rear frame')
    viewer.cam.distance = 10

    with keyboard.Listener(on_press=output.on_press,
                           on_release=output.on_release) as l:
        while True:
            sim.data.ctrl[:] = output.value
            sim.step()
            viewer.render()

if __name__ == '__main__':
    main()
