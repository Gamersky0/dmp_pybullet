
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
    ONLY_BOX_MOVE = False
    TIME_FEEDBACK = False
    for arg in args:    
        if arg.lower() == "notmove":
            ENABLE_MOVE = False
        if arg.lower() == "box":
            ONLY_BOX_MOVE = True
        if arg.lower() == "time":
            TIME_FEEDBACK = True

    robot_l = UR5Robotiq85((0, 0, 0), (0, 0, -math.pi / 2))
    robot_r = UR5Robotiq85((0, -0.6, 0), (0, 0, -math.pi / 2))
    env = Environment(robot_l, robot_r)
    # robot_l.reset()
    # robot_r.reset() # 卸力

    if TIME_FEEDBACK == False:
        traj = Trajectory(filename)

        ENABLE_TRAJ_PLOT = True
        if ENABLE_TRAJ_PLOT:
            traj.plot_in_bullet(traj.data_l)
            traj.plot_in_bullet(traj.data_r)
    else:
        traj = Trajectory(None) # 实时反馈

    if ENABLE_MOVE:
        print("Using move module")
        # TODO: 从示教轨迹到机械臂轨迹转换
        # TODO：目前只能同时开始，同时结束
        pipeline(env, robot_l, robot_r, traj.data_l, traj.data_r)
    else:
        print("Using not-move Module")

    keyboard.wait()
    # time.sleep(5)
