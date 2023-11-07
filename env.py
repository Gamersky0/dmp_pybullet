import time
import math
import random

import numpy as np
import pybullet as p
import pybullet_data

from utilities import Models, Camera
from collections import namedtuple
from attrdict import AttrDict
from tqdm import tqdm


class FailToReachTargetError(RuntimeError):
    pass


class ClutteredPushGrasp:

    SIMULATION_STEP_DELAY = 1 / 240.

    def __init__(self, robot_l, robot_r, models: Models, camera=None, vis=False) -> None:
        self.robot_l = robot_l
        self.robot_r = robot_r
        self.vis = vis
        if self.vis:
            self.p_bar = tqdm(ncols=0, disable=False)
        self.camera = camera

        # define environment
        self.physicsClient = p.connect(p.GUI if self.vis else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)
        self.planeID = p.loadURDF("plane.urdf")

        self.robot_l.load()
        self.robot_r.load()
        self.robot_l.step_simulation = self.step_simulation
        self.robot_r.step_simulation = self.step_simulation

    def step_simulation(self):
        """
        Hook p.stepSimulation()
        """
        p.stepSimulation()
        if self.vis:
            time.sleep(self.SIMULATION_STEP_DELAY)
            self.p_bar.update(1)

    def step(self):
        """
        action: (x, y, z, roll, pitch, yaw, gripper_opening_length) for End Effector Position Control
                (a1, a2, a3, a4, a5, a6, a7, gripper_opening_length) for Joint Position Control
        control_method:  'end' for end effector position control
                         'joint' for joint position control
        """
        # assert control_method in ('joint', 'end')
        # self.robot_l.move_ee(action[:-1], control_method)
        # self.robot_l.move_gripper(action[-1])
        for i in range(120):  # Wait for a few steps
            print("i: ", i)
            self.step_simulation()
        return



    def get_observation(self):
        return

    def reset(self):
        self.robot_l.reset()
        self.robot_r.reset()
        return

    def close(self):
        p.disconnect(self.physicsClient)
