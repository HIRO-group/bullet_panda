#!/home/anuj/projects/research/catkin_ws/src/poke/venv/bin/python

from panda_robot import PandaArm

import time
import numpy as np


def square_trajectory(p, reverse=False):
    ee_vels_x = np.array([0, 0.1, 0, -0.1, 0])
    ee_vels_y = np.array([0.1, 0, -0.1, 0, 0.1])
    times = [1.5, 3, 3, 3, 1.5]
    if reverse:
        ee_vels_y = -ee_vels_y

    for x_vel, y_vel, t in zip(ee_vels_x, ee_vels_y, times):
        p.apply_vel_vec([x_vel, y_vel, 0, 0, 0, 0], t, log=True, datapath='data/robot_data.npy')


if __name__ == '__main__':
    p = PandaArm()
    time.sleep(2.0)

    square_trajectory(p)
    time.sleep(2.0)

    square_trajectory(p, reverse=True)
    time.sleep(2.0)

