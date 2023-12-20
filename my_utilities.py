import pybullet as p
import time
import math
from my_utilities import *
import pybullet_data
import keyboard

def polt_axis(robot):
    # 获取坐标轴的起始点和终点

    start_pos = [0, 0, 0]  # 坐标轴的起始点坐标
    axis_length = 1  # 坐标轴的长度
    axis_colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]  # 每个坐标轴的颜色，分别为X、Y、Z轴

    # Render X-axis
    end_pos = [start_pos[0] + axis_length, start_pos[1], start_pos[2]]  # X轴的终点坐标
    p.addUserDebugLine(start_pos, end_pos, axis_colors[0], lineWidth=2)

    # Render Y-axis
    end_pos = [start_pos[0], start_pos[1] + axis_length, start_pos[2]]  # Y轴的终点坐标
    p.addUserDebugLine(start_pos, end_pos, axis_colors[1], lineWidth=2)

    # Render Z-axis
    end_pos = [start_pos[0], start_pos[1], start_pos[2] + axis_length]  # Z轴的终点坐标
    p.addUserDebugLine(start_pos, end_pos, axis_colors[2], lineWidth=2)

    return