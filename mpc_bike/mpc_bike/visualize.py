import click
import numpy as np
import matplotlib.pyplot as plt

from mpc_bike.path import TEST_PATHS


@click.command()
@click.option('-n', '--path-num', type=int, required=True)
def main(path_num):
    # data = np.load('data.npy')
    path = TEST_PATHS[path_num]
    s = np.linspace(0, path.length)
    x = np.array([path.x(s_i) for s_i in s])
    plt.figure()
    plt.plot(x[:, 0], x[:, 1])
    # plt.plot(data[:, 0], data[:, 1])
    # plt.plot(data[:, 2])
    # plt.plot(data[:, 3:])
    plt.gca().set_aspect('equal', 'box')
    plt.show()


if __name__ == '__main__':
    main()
