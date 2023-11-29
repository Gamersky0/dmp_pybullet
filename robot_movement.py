import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
import keyboard

ENABLE_TRAJ_FOLLOW_DEBUG = False

# trailDuration is duration (in seconds) after debug lines will be removed automatically
# use 0 for no-removal
trailDuration = 150
useNullSpace = 1
useOrientation = 1
ikSolver = 0
# If we set useSimulation = 0, it sets the arm pose to be the IK result directly without using dynamic control.
# This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 1


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

def move_to_ready_position(robot, ready_position):
    print("Begin move to ready position.")

    TrackIndex = robot.eef_id
    numJoints = p.getNumJoints(robot.id)
    # print("numJoints: ", numJoints) # 19 for ur
    hasPrevPose = False
    
    p.stepSimulation()
    x = ready_position.iloc[0]
    y = ready_position.iloc[1]
    z = ready_position.iloc[2]
    print("ready_position: ", ready_position)
    # robot.move_ee(robot.arm_rest_poses, 'end')
    robot.move_ee([x, y, z, 0,  -math.pi, -math.pi], 'end')

    ls = p.getLinkState(robot.id, TrackIndex)
    if (hasPrevPose):
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    prevPose1 = ls[4]
    hasPrevPose = True

    print("Success move to ready position.")
    return

# Cloth & Box Init
def cloth_box_init(env):
    # Init Object
    env.init_Object()
    data = p.getMeshData(env.clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    # print("Index number: ", data[0])
    # print(data[1][0])
    # print(data[1][11])
    boxId_l = p.loadURDF("cube.urdf", data[1][0], globalScaling = 0.001, useMaximalCoordinates = True)
    boxId_r = p.loadURDF("cube.urdf", data[1][11], globalScaling = 0.001, useMaximalCoordinates = True)
    p.changeDynamics(boxId_l, -1, mass=0, linearDamping=0.2, angularDamping=0.2)
    p.changeDynamics(boxId_r, -1, mass=0, linearDamping=0.2, angularDamping=0.2)
    p.createSoftBodyAnchor(env.clothId, 0, boxId_l, -1, [0, 0, 0.005])
    p.createSoftBodyAnchor(env.clothId, 11, boxId_r, -1, [0, 0, 0.005])
    return boxId_l, boxId_r

# Box Move Test in 11.27
def box_move_test(boxId_l, boxId_r):
    # 定义轨迹参数
    start_x = -0.5  # 起始位置
    end_x = -1.5  # 结束位置
    speed = 0.1  # 移动速度
    z = 0.35  # z 坐标不变

    # 运动循环
    while True:
        # 获取当前时间
        current_time = time.time()

        # 计算物体在轨迹上的位置
        t = (current_time * speed) % 2  # 在 0 到 2 之间循环
        if t > 1:
            x = end_x - (t - 1) * (end_x - start_x)
        else:
            x = start_x + t * (end_x - start_x)

        # 设置物体的刚体运动状态
        p.resetBasePositionAndOrientation(boxId_l, [x, -0.07286397145073759, z], [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(boxId_r, [x, -0.7295659481960693, z], [0, 0, 0, 1])

        # 控制循环速率
        time.sleep(1 / 60.0)
    return

# 两条轨迹，一个个轨迹点进行跟随

def robot_move(env, robot_l, robot_r, data_l, data_r, only_box_move):
    TrackIndex = robot_l.eef_id
    p.setRealTimeSimulation(useRealTimeSimulation)

    orientation_l = data_l.iloc[:, 3:]
    orientation_r = data_r.iloc[:, 3:]
    trajectory_l = data_l.iloc[:, :3]
    trajectory_r = data_r.iloc[:, :3]

    ready_position_l = trajectory_l.iloc[0]
    ready_position_r = trajectory_r.iloc[0]
    
    # move to init position
    robot_l.move_ee(robot_l.arm_rest_poses, 'joint')
    robot_r.move_ee(robot_r.arm_rest_poses, 'joint')
    robot_l.move_gripper(0.01)
    robot_r.move_gripper(0.01)

    # DEBUG:
    time.sleep(3)

    boxId_l, boxId_r = cloth_box_init(env)

    # 固定刚体 TODO
    # currentPos, currentOrn = p.getBasePositionAndOrientation(boxId_l)
    # cid = p.createConstraint(boxId_l, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], currentPos, [0,0,0])
    # constraint_r = p.createConstraint(boxId_r, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], data[1][0], [0, 0, 0])

    # 获得刚体位置
    currentPos_l, currentOrn_l = p.getBasePositionAndOrientation(boxId_l)
    currentPos_r, currentOrn_r = p.getBasePositionAndOrientation(boxId_r)
    print("boxId_l currentPos_l: ", currentPos_l)
    print("boxId_r currentPos_r: ", currentPos_r)

    # # Box Move Test in 11.27
    # box_move_test(boxId_l, boxId_r)


    # announcement
    prevPose_tl = [0, 0, 0] # target l
    prevPose_tr = [0, 0, 0] # target r
    prevPose_al = [0, 0, 0] # actual l
    prevPose_ar = [0, 0, 0] # actual r
    hasPrevPose = 0

    count = 0
    for index, row in trajectory_l.iterrows():
        count += 1
        # if ENABLE_TRAJ_FOLLOW_DEBUG:
        #     print("count: ", count)

        if (useSimulation and useRealTimeSimulation == 0):
            p.stepSimulation()

        for i in range(1):
            pos_l = [row[0], row[1], row[2]]
            if ENABLE_TRAJ_FOLLOW_DEBUG:
                print("index: ", index)
            pos_r = [trajectory_r.iloc[index, 0], trajectory_r.iloc[index, 1], trajectory_r.iloc[index, 2]]

            # end effector points down, not up (in case useOrientation==1)
            # orn_l = [0, math.pi / 2,  0]
            # orn_r = [0, math.pi / 2,  0]
            # orn_l = [0, 0, -math.pi / 2]
            # orn_r = [0, 0, -math.pi / 2]
            orn_l = [math.pi, 0, -math.pi / 2]
            orn_r = [math.pi, 0, -math.pi / 2]
            # orn_l = [orientation_l.iloc[index, 0], orientation_l.iloc[index, 1], orientation_l.iloc[index, 2]]
            # orn_r = [orientation_r.iloc[index, 0], orientation_r.iloc[index, 1], orientation_r.iloc[index, 2]]            

            action_l = list(pos_l) + list(orn_l)
            action_r = list(pos_r) + list(orn_r)


            if only_box_move:
                p.resetBasePositionAndOrientation(boxId_l, list(pos_l), [0, 0, 0, 1])
                p.resetBasePositionAndOrientation(boxId_r, list(pos_r), [0, 0, 0, 1])

            if (useNullSpace == 1) and (useOrientation == 1): # only use this
                robot_l.move_ee(action_l, 'end')
                robot_r.move_ee(action_r, 'end')
                # jointPoses_l = p.calculateInverseKinematics(armId_l, kukaEndEffectorIndex, pos_l, orientation_l, ll, ul, jr, rp)
                # jointPoses_r = p.calculateInverseKinematics(armId_r, kukaEndEffectorIndex, pos_r, orientation_r, ll, ul, jr, rp)

        ls_l = p.getLinkState(robot_l.id, TrackIndex)
        ls_r = p.getLinkState(robot_r.id, TrackIndex)
        if (hasPrevPose):
            p.addUserDebugLine(prevPose_tl, pos_l, [0, 0, 0.3], 1, trailDuration) # target move
            p.addUserDebugLine(prevPose_al, ls_l[4], [1, 0, 0], 1, trailDuration) # actual move
            p.addUserDebugLine(prevPose_tr, pos_r, [0, 0, 0.3], 1, trailDuration) # target move
            p.addUserDebugLine(prevPose_ar, ls_r[4], [1, 0, 0], 1, trailDuration) # actual move
        prevPose_tl = pos_l
        prevPose_al = ls_l[4]
        prevPose_tr = pos_r
        prevPose_ar = ls_r[4]
        hasPrevPose = 1

    # p.disconnect()

    return

# TODO: 类函数