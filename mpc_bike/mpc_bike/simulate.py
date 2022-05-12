import click
import numpy as np
from pynput import keyboard
from mujoco_py import const, MjSim, MjViewer

from mpc_bike.model import Bicycle
from mpc_bike.control import PathController


class ControlOutput:

    """Convert arrow key inputs into very simple control output."""

    def __init__(self, magn_forward=200., magn_lr=20.0):
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


class BikeViewer(MjViewer):
    def __init__(self, sim):
        super().__init__(sim)
        # Setup camera to follow the bike.
        self.cam.type = const.CAMERA_TRACKING
        self.cam.trackbodyid = sim.model.body_names.index('rear frame')
        self.cam.distance = 10


def keyboard_control(sim, viewer, bic):
    TIMESTEPS = 4000
    output = ControlOutput()
    control = PathController(bic)
    states = np.zeros((TIMESTEPS, *control.state.shape), dtype=np.float32)
    actions = np.zeros((TIMESTEPS, 2), dtype=np.float32)
    with keyboard.Listener(on_press=output.on_press,
                           on_release=output.on_release):
        for i in range(TIMESTEPS):
            states[i] = control.extract_state(sim)
            sim.data.ctrl[:] = actions[i] = output.value
            sim.step()
            if viewer is not None:
                viewer.render()

    np.savez_compressed('data.npz', states=states, actions=actions)


def path_control(sim, viewer):
    control = PathController()
    while True:
        state = control.extract_state(sim)
        sim.data.ctrl[:] = control.action(state)

        import pdb
        pdb.set_trace()


@click.command()
@click.option('--mode', type=click.Choice(['keyboard', 'path']),
              default='keyboard')
@click.option('--gui/--headless', default=True)
def main(mode, gui):
    bic = Bicycle()
    model = bic.model
    sim = MjSim(model)
    # sim.userdata['bic'] = bic
    viewer = BikeViewer(sim) if gui else None
    if mode == 'keyboard':
        keyboard_control(sim, viewer, bic)
    elif mode == 'path':
        path_control(sim, viewer)


if __name__ == '__main__':
    main()
