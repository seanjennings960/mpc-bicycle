import numpy as np
import matplotlib.pyplot as plt


def main():
    data = np.load('data.npy')
    plt.figure()
    # plt.plot(data[:, 0], data[:, 1])
    # plt.plot(data[:, 2])
    plt.plot(data[:, 3:])
    plt.show()


if __name__ == '__main__':
    main()
