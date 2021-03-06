import time

import numpy as np
from bullet_robot import BulletRobot

import logging
from robot_config import ROBOT_CONFIG
from os.path import abspath

from numpy import matmul
from numpy.linalg import pinv


class PandaArm(BulletRobot):

    """
    Bullet simulation interface for the Franka Panda Emika robot
    
    Available methods (for usage, see documentation at function definition):
        - exec_position_cmd
        - exec_position_cmd_delta
        - move_to_joint_position
        - move_to_joint_pos_delta
        - exec_velocity_cmd
        - exec_torque_cmd
        - inverse_kinematics
        - untuck
        - tuck
        - q_mean
        - state
        - angles
        - joint_limits
        - joint_velocities
        - joint_efforts
        - ee_velocity
        - n_joints
        - joint_names

        - jacobian*
        - ee_pose*
        - ee_velocity*
        - inertia*
        - inverse_kinematics*
        - joint_ids*
        - get_link_pose*
        - get_link_velocity*
        - get_joint_state*
        - set_joint_angles*
        - get_movable_joints*
        - get_all_joints*
        - get_joint_by_name*
        - set_default_pos_ori*
        - set_pos_ori*
        - set_ctrl_mode*

        *documentation for these methods in parent class (BulletRobot). Refer bullet_robot.py       


    """

    def __init__(self, robot_description=None, config=ROBOT_CONFIG, uid=None, *args, **kwargs):

        """
        @param robot_description: path to description file (urdf, .bullet, etc.)
        @param config           : optional config file for specifying robot information 
        @param uid              : optional server id of bullet 

        @type robot_description : str
        @type config            : dict
        @type uid               : int
        """
        self._ready = False
        self.data = []

        if robot_description is None:
            robot_description = abspath('../../assets') + '/panda_arm.urdf'

        self._joint_names = ['panda_joint%s' % (s,) for s in range(1, 8)]

        self._bullet_robot = BulletRobot(robot_description, uid=uid)

        all_joint_dict = self._bullet_robot.get_joint_dict()
        self._joint_ids = [all_joint_dict[joint_name] for joint_name in self._joint_names]

        self._tuck = [-0.017792060227770554, -0.7601235411041661, 0.019782607023391807, -2.342050140544315,
                      0.029840531355804868, 1.5411935298621688, 0.7534486589746342]

        self._untuck = self._tuck

        lower_limits = self._bullet_robot.get_joint_limits()['lower'][self._joint_ids]
        upper_limits = self._bullet_robot.get_joint_limits()['upper'][self._joint_ids]

        self._jnt_limits = [{'lower': x[0], 'upper': x[1]} for x in zip(lower_limits, upper_limits)]

        self.move_to_joint_position(self._tuck)

        self._ready = True

    def exec_position_cmd(self, cmd):
        """
        Execute position command. Use for position controlling.

        @param cmd  : joint position values
        @type cmd   : [float] len: self._nu

        """
        self._bullet_robot.set_joint_positions(cmd, self._joint_ids)

    def exec_position_cmd_delta(self, cmd):
        """
        Execute position command by specifying difference from current positions. Use for position controlling.

        @param cmd  : joint position delta values
        @type cmd   : [float] len: self._nu

        """
        self._bullet_robot.set_joint_positions(self.angles() + cmd, self._joint_ids)

    def move_to_joint_position(self, cmd):
        """
        Same as exec_position_cmd. (Left here for maintaining structure of PandaArm class from panda_robot package)

        @param cmd  : joint position values
        @type cmd   : [float] len: self._nu

        """
        self.exec_position_cmd(cmd)

    def move_to_joint_pos_delta(self, cmd):
        """
        Same as exec_position_cmd_delta.
        (Left here for maintaining structure of PandaArm class from panda_robot package)

        @param cmd  : joint position delta values
        @type cmd   : [float] len: self._nu

        """
        self.exec_position_cmd_delta(cmd)

    def exec_velocity_cmd(self, cmd):
        """
        Execute velocity command. Use for velocity controlling.

        @param cmd  : joint velocity values
        @type cmd   : [float] len: self._nu

        """
        self._bullet_robot.set_joint_velocities(cmd, self._joint_ids)

    def exec_torque_cmd(self, cmd):
        """
        Execute torque command. Use for torque controlling.

        @param cmd  : joint torque values
        @type cmd   : [float] len: self._nu

        """
        self._bullet_robot.set_joint_torques(cmd, self._joint_ids)

    def inverse_kinematics(self, position, orientation=None):
        """
        @return Joint positions for given end-effector pose obtained using bullet IK.
        @rtype: np.ndarray

        @param position: target end-effector position (X,Y,Z) in world frame
        @param orientation: target end-effector orientation in quaternion format (w, x, y , z) in world frame

        @type position: [float] * 3
        @type orientation: [float] * 4
 
        """
        return self._bullet_robot.inverse_kinematics(position, orientation)[0]

    def set_sampling_rate(self, sampling_rate=100):
        """
        (Left here for maintaining structure of PandaArm class from panda_robot package)
        """
        pass

    def untuck(self):
        """
        Send robot to tuck position.
        """
        self.exec_position_cmd(self._untuck)

    def tuck(self):
        """
        Send robot to tuck position.
        """
        self.exec_position_cmd(self._tuck)

    def q_mean(self):
        """
        @return Mean joint positions.
        @rtype: [float] * self._nq
        """
        return self._bullet_robot.q_mean()[self._joint_ids]

    def state(self):
        """
        @return Current robot state, as a dictionary, containing 
                joint positions, velocities, efforts, zero jacobian,
                joint space inertia tensor, end-effector position, 
                end-effector orientation, end-effector velocity (linear and angular)
        @rtype: dict: {'position': np.ndarray,
                       'velocity': np.ndarray,
                       'effort': np.ndarray,
                       'jacobian': np.ndarray,
                       'inertia': np.ndarray,
                       'ee_point': np.ndarray,
                       'ee_ori': np.ndarray,
                       'ee_vel': np.ndarray,
                       'ee_omg': np.ndarray }
        """
        joint_angles = self.angles()
        joint_velocities = self.joint_velocities()
        joint_efforts = self.joint_efforts()

        state = {}
        state['position'] = joint_angles
        state['velocity'] = joint_velocities
        state['effort'] = joint_efforts
        state['jacobian'] = self._bullet_robot.jacobian(None)
        state['inertia'] = self._bullet_robot.inertia(None)

        state['ee_point'], state['ee_ori'] = self._bullet_robot.ee_pose()

        state['ee_vel'], state['ee_omg'] = self._bullet_robot.ee_velocity()

        return state

    def angles(self):
        """
        @return Current joint positions.
        @rtype: [float] * self._nq
        """
        return self._bullet_robot.angles()[self._joint_ids]

    def joint_limits(self):
        """
        @return Joint limits
        @rtype: dict {'lower': ndarray, 'upper': ndarray}
        """
        return self._jnt_limits

    def joint_velocities(self):
        """
        @return Current joint velocities.
        @rtype: [float] * self._nq
        """
        return self._bullet_robot.joint_velocities()[self._joint_ids]

    def joint_efforts(self):
        """
        @return Current joint efforts.
        @rtype: [float] * self._nq
        """
        return self._bullet_robot.joint_efforts()[self._joint_ids]

    def joint_names(self):
        """
        @return Name of all joints
        @rtype: [str] * self._nq
        """
        return self._joint_names

    def apply_vel_vec(self, vec, t, log=False, **kwargs):
        now = time.time()

        while True:

            if log:
                self.data.append(self.state())
            if time.time() - now > t:
                break
            j = self.state()['jacobian']
            j_inv = pinv(j)
            self.exec_velocity_cmd(matmul(j_inv, vec))

        self._bullet_robot.set_ctrl_mode()
        if log:
            np.save(kwargs['datapath'], self.data)
