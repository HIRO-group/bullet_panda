#!/home/anuj/projects/research/catkin_ws/src/poke/venv/bin/python

from __future__ import division

import pybullet as p
import time

from signal import signal, SIGINT

from itertools import product
import json
from numpy import save

from utils import let_object_settle, shutdown_handler, print_log, change_object_dynamics, setup_block_test_environment

signal(SIGINT, shutdown_handler)

cube_id, cube_start_pos, cube_start_orientation = setup_block_test_environment()

masses = [5, 10, 15]
friction_coeffs = [0.3, 0.5, 0.7]

trial_combinations = list(product(masses, friction_coeffs))
trials = {i: trial_combinations[i] for i in range(len(trial_combinations))}
trials[-1] = ('mass', 'mu')
with open('data/trial_params_friction.json', 'w') as f:
    json.dump(trials, f)

force_mag = 0
i = 0
data = [[] for _ in range(len(trials.keys())-1)]

for n, params in trials.items():
    if n == -1:
        continue
    m, mu = params
    change_object_dynamics(cube_id, m, mu)

    for _ in range(150):
        force_vec = [force_mag, 0, 0]
        print_log(cube_id, force_vec)

        p.applyExternalForce(cube_id, -1, force_vec, posObj=[0, 0, 0], flags=p.LINK_FRAME)
        p.stepSimulation()
        time.sleep(1/240)

        total_normal_force = 0
        total_lateral_friction_force = [0, 0, 0]
        pts = p.getContactPoints()
        for pt in pts:
            total_normal_force += pt[9]
            total_lateral_friction_force[0] += pt[11][0] * pt[10] + pt[13][0] * pt[12]
            total_lateral_friction_force[1] += pt[11][1] * pt[10] + pt[13][1] * pt[12]
            total_lateral_friction_force[2] += pt[11][2] * pt[10] + pt[13][2] * pt[12]

        data[i].append([force_mag, total_lateral_friction_force, total_normal_force])
        force_mag += 1

    p.resetBasePositionAndOrientation(cube_id, cube_start_pos, cube_start_orientation)
    let_object_settle()
    time.sleep(2.0)

    force_mag = 0.0
    i += 1

save('data/friction_data.npy', data)

p.disconnect()
