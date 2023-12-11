import os
import numpy as np
import pybullet as p
from tqdm import tqdm
from env import ClutteredPushGrasp
from robot import Panda, UR5Robotiq85, UR5Robotiq140
from utilities import YCBModels, Camera
import time
import math


def user_control_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', '**', 'textured-decmp.obj'),
    )
    camera = Camera((1, 1, 1),
                    (0, 0, 0),
                    (0, 0, 1),
                    0.1, 5, (320, 320), 40)
    camera = None
    # robot = Panda((0, 0.5, 0), (0, 0, 0))
    # robot = UR5Robotiq140((0, 0.5, 0), (0, 0, 0))
    robot_l = UR5Robotiq85((0, 0, 0), (0, 0, 0))
    robot_r = UR5Robotiq85((0, -0.8, 0), (0, 0, 0))

    env = ClutteredPushGrasp(robot_l, robot_r, ycb_models, camera, vis=True)

    env.reset()

    # # env.SIMULATION_STEP_DELAY = 0
    env.step()

if __name__ == '__main__':
    user_control_demo()
