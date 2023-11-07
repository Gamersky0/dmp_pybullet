
from tqdm import tqdm
from env import ClutteredPushGrasp
from robot import Panda, UR5Robotiq85, UR5Robotiq140
from utilities import YCBModels, Camera

import pybullet as p
import pandas as pd
import numpy as np
import time
import math
from datetime import datetime
import pybullet_data
import keyboard
from robot_movement import *
from environment import *
from trajectorytools import *
import sys
filename = 'signal_trajectory_1008.csv'

if __name__ == '__main__':
    args = sys.argv[1:]

    ENABLE_MOVE = True
    for arg in args:    
        if arg.lower() == "notmove":
            ENABLE_MOVE = False

    # robot_l = UR5Robotiq85((0, 0, 0), (0, 0, 0))
    # robot_r = UR5Robotiq85((0, -0.8, 0), (0, 0, 0))
    robot_l = UR5Robotiq85((0, 0, 0), (0, 0, -math.pi / 2))
    robot_r = UR5Robotiq85((0, -0.6, 0), (0, 0, -math.pi / 2))
    env = Environment(robot_l, robot_r)
    # robot_l.reset()
    # robot_r.reset() # 卸力

    # data = p.getMeshData(env.clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    # # print("data=",data)
    # print(data[0])
    # print(data[1][0])
    # print(data[1][11])
    # p.createSoftBodyAnchor(env.clothId, 132, robot_l.id, -1, [0, 0, 0])
    # p.createSoftBodyAnchor(env.clothId, 143,robot_r.id, -1, [0, 0, 0])
    # p.createSoftBodyAnchor(env.clothId, 0, -1, -1)
    # p.createSoftBodyAnchor(env.clothId, 11, -1, -1)
    # p.createSoftBodyAnchor(env.clothId, 132, -1, -1)
    # p.createSoftBodyAnchor(env.clothId, 143, -1, -1)


    if ENABLE_MOVE:
        print("Using move module")

        traj = Trajectory(filename)

        ENABLE_TRAJ_PLOT = False
        if ENABLE_TRAJ_PLOT:
            traj.plot_in_bullet(traj.data_l)
            traj.plot_in_bullet(traj.data_r)

        # 同时开始，同时结束
        robot_move(env, robot_l, robot_r, traj.data_l, traj.data_r)
    else:
        print("Using not-move Module")

    keyboard.wait()
    # time.sleep(5)
