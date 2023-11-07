
from tqdm import tqdm
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
import sys


ENABLE_TRAJ_SCALE_DEBUG = False
ENABLE_DMP_POSE_DEBUG = False

class Trajectory:
    def __init__(self, filename):
        self.filename = filename
        self.data_raw = None
        self.data_scale = None
        self.read_datafile()
        self.trajectory_scale()
        self.data_l = self.data_scale
        self.data_r = self.map_to_right(self.data_l)

    def read_datafile(self):
        self.data_raw = pd.read_csv(self.filename)
        self.rows, self.cols = self.data_raw.shape
        if self.rows > 0 and self.cols > 0:
            print("Success reading data scv, data.shape: ", self.data_raw.shape)
        else:
            print("Fail to read csv !!!")
            exit()

    def trajectory_scale(self):
        data = self.data_raw
        # map the data to where robot-arm can reach out

        # get max_value & min_value in every dimension
        max_vals = data.max(axis = 0)
        min_vals = data.min(axis = 0)
        
        xmax = max_vals[0]
        xmin = min_vals[0]
        ymax = max_vals[1]
        ymin = min_vals[1]
        zmax = max_vals[2]
        zmin = min_vals[2]
        range_x = xmax - xmin
        range_y = ymax - ymin
        range_z = zmax - zmin


        # TODO: x方向为机械臂操作方向，变化幅度最大，之后需要改
        if range_y > range_x:
            print("++++++++ exist swap col +++++++++")
            # data.iloc[:, [1, 0] + list(range(2, len(data.columns)))] = data.iloc[:, [0, 1] + list(range(2, len(data.columns)))].values
            temp = range_x
            range_x = range_y
            range_y = temp

        if ENABLE_TRAJ_SCALE_DEBUG:
            print("trajctory value diff: ", range_x, range_y, range_z)

        for index, row in data.iterrows():
            # 在 x 的 scale 上进行缩放
            row[0] = (row[0] - xmin) / range_x / 1.1
            row[1] = (row[1] - ymin) / range_x / 1.1
            row[2] = (row[2] - zmin) / range_x / 1.1

            # TODO: 暂时修改
            row[0] = -0.5 * row[0] - 0.15
            row[2] = row[2] + 0.1  # 抬高一些
        self.data_scale = data

    def plot_in_bullet(self, data):
        hasprevPose = False
        for index, row in data.iterrows():
            pos_x = row[0]
            pos_y = row[1]
            pos_z = row[2]
            curPose = [pos_x, pos_y, pos_z]
            if hasprevPose == True:
                if ENABLE_DMP_POSE_DEBUG:
                    print("prevPose: ", prevPose, "curPose: ", curPose)
                # p.addUserDebugLine(prevPose, curPose, [0, 0, 0.3], 1, trailDuration)
                p.addUserDebugLine(prevPose, curPose, [0, 0, 0.3], 1)
            else:
                hasprevPose = True
            prevPose = curPose
        return

    def map_to_right(self, trajectory_l):
        trajectory_r = trajectory_l.copy()
        trajectory_r.iloc[:, 1] = trajectory_l.iloc[:, 1] - 0.8
        return trajectory_r
