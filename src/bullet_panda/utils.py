from __future__ import division

import pybullet as p
import pybullet_data
from sys import exit
from numpy.linalg import norm
import struct
import time


def shutdown_handler(signal_received, frame):
    print("Shutting down pybullet and exiting gracefully...")
    p.disconnect()
    exit(0)


def is_object_moving(object_id, vel_thresh=3e-4):
    linear_vel, angular_vel = p.getBaseVelocity(object_id)
    return (norm(linear_vel) > vel_thresh) or (norm(angular_vel) > vel_thresh)


def print_log(object_id, force):
    shape_data = p.getVisualShapeData(object_id)[0]
    dynamics_data = p.getDynamicsInfo(object_id, -1)
    print('***********************************')
    print('Object: %s' % shape_data[4])
    print('Dimensions: (%f, %f, %f)' % shape_data[3])
    print('Mass: %f' % dynamics_data[0])
    print('Lateral Friction: %f' % dynamics_data[1])
    print('Inertia Diagonal: (%f, %f, %f)' % dynamics_data[2])
    print('Force: (%f, %f, %f)' % tuple(force))
    print('***********************************')


def change_object_dynamics(object_id, mass, friction):
    dimensions = p.getVisualShapeData(object_id)[0][3]
    inertia = [(1 / 12) * mass * (dimensions[1] ** 2 + dimensions[2] ** 2),
               (1 / 12) * mass * (dimensions[0] ** 2 + dimensions[2] ** 2),
               (1 / 12) * mass * (dimensions[0] ** 2 + dimensions[1] ** 2)]
    p.changeDynamics(object_id, -1, mass=mass, lateralFriction=friction, localInertiaDiagonal=inertia)


# https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/kuka_grasp_block_playback.py
def read_pybullet_log(filename, verbose=True):
    f = open(filename, 'rb')
    print('Opened %s' % filename)

    keys = f.readline().decode('utf8').rstrip('\n').split(',')
    fmt = f.readline().decode('utf8').rstrip('\n')

    # The byte number of one record
    sz = struct.calcsize(fmt)
    # The type number of one record
    ncols = len(fmt)

    if verbose:
        print('Keys: ', keys)
        print('Format: ', fmt)
        print('Size: ', sz)
        print('Columns: ', ncols)

    # Read data
    whole_file = f.read()
    # split by alignment word
    chunks = whole_file.split(b'\xaa\xbb')
    log = list()
    for chunk in chunks:
        if len(chunk) == sz:
            values = struct.unpack(fmt, chunk)
            record = list()
            for i in range(ncols):
                record.append(values[i])
            log.append(record)

    return log


def let_object_settle():
    # let object fall and settle
    for _ in range(250):
        p.stepSimulation()
        time.sleep(1 / 240)


def setup_block_test_environment():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    p.loadURDF("plane.urdf")
    cube_start_pos = [0, 0, 1]
    cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    cube_id = p.loadURDF("../../assets/cube.urdf", cube_start_pos, cube_start_orientation)

    let_object_settle()

    return cube_id, cube_start_pos, cube_start_orientation
