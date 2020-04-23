#!/home/anuj/projects/research/catkin_ws/src/poke/venv/bin/python

from __future__ import division

import pybullet as p
import time
import pybullet_data

from signal import signal, SIGINT

from itertools import product
import json
from numpy import save

from utils import shutdown_handler, is_object_moving
from utils import print_log, change_object_dynamics, setup_block_test_environment, let_object_settle


def run_trial(trial_num, object_id, force, mass, friction, verbose=True):
    change_object_dynamics(object_id, mass, friction)

    start_x = p.getBasePositionAndOrientation(object_id)[0][0]

    if verbose:
        print('\n*************TRIAL %d**************' % trial_num)
        print_log(object_id, force)

    # log1_id = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
    #                               'data/pybullet_logs/log_object_dynamics_%s.bin' % str(trial_num).zfill(5))
    # log2_id = p.startStateLogging(p.STATE_LOGGING_CONTACT_POINTS,
    #                               'data/pybullet_logs/log_contact_dynamics_%s.bin' % str(trial_num).zfill(5))

    p.applyExternalForce(object_id, -1, force, posObj=[0, 0, 0], flags=p.LINK_FRAME)
    p.stepSimulation()
    while is_object_moving(object_id):
        p.stepSimulation()
        time.sleep(1/240)

    return p.getBasePositionAndOrientation(object_id)[0][0] - start_x  # return total x-displacement

    # p.stopStateLogging(log1_id)
    # p.stopStateLogging(log2_id)


signal(SIGINT, shutdown_handler)

cube_id, cube_start_pos, cube_start_orientation = setup_block_test_environment()

let_object_settle()

masses = [0.1, 0.4, 0.7]
friction_coeffs = [0.1, 0.4, 0.7]
force_mags = [50, 75, 100]

trial_combinations = list(product(masses, friction_coeffs, force_mags))
trials = {i: trial_combinations[i] for i in range(len(trial_combinations))}
trials[-1] = ('mass', 'mu', 'force_x')
with open('data/trial_params_dynamics.json', 'w') as f:
    json.dump(trials, f)

data = []
for n, params in trials.items():
    if n == -1:
        continue
    m, mu, force_mag = params
    displacement = run_trial(n, int(cube_id), force=[force_mag, 0, 0], mass=m, friction=mu)
    p.resetBasePositionAndOrientation(cube_id, cube_start_pos, cube_start_orientation)
    let_object_settle()
    time.sleep(2.0)
    data.append(displacement)

save('data/dynamics_data.npy', data)

p.disconnect()
