import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data

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

# 两条轨迹，一个个轨迹点进行跟随

def robot_move(env, robot_l, robot_r, data_l, data_r):
    TrackIndex = robot_l.eef_id
    p.setRealTimeSimulation(useRealTimeSimulation)

    orientation_l = data_l.iloc[:, 3:]
    orientation_r = data_r.iloc[:, 3:]
    trajectory_l = data_l.iloc[:, :3]
    trajectory_r = data_r.iloc[:, :3]

    ready_position_l = trajectory_l.iloc[0]
    ready_position_r = trajectory_r.iloc[0]
    
    # DEBUG:
    time.sleep(1)

    # move to init position
    robot_l.move_ee(robot_l.arm_rest_poses, 'joint')
    robot_r.move_ee(robot_r.arm_rest_poses, 'joint')
    robot_l.move_gripper(0)
    robot_r.move_gripper(0)

    # DEBUG:
    time.sleep(3)

    # Init Object
    env.init_Object()
    data = p.getMeshData(env.clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    print("Index number: ", data[0])
    print(data[1][0])
    print(data[1][11])
    p.createSoftBodyAnchor(env.clothId, 0, robot_l.id, TrackIndex, [0, 0, 0])
    p.createSoftBodyAnchor(env.clothId, 11, robot_r.id, TrackIndex, [0, 0, 0])

    # # move to ready position
    # pos_l = data[1][0]
    # pos_r = data[1][11]
    # orn_l = [math.pi, 0, 0]
    # orn_r = [math.pi, 0, 0]
    # # orn_l = [orientation_l.iloc[index,
    # action_l = list(pos_l) + list(orn_l)
    # action_r = list(pos_r) + list(orn_r)

    # robot_l.move_ee(action_l, 'end')
    # robot_r.move_ee(action_r, 'end')

    # time.sleep(3)

    Track_LinkState_L = p.getLinkState(robot_l.id, TrackIndex)
    Track_LinkState_R = p.getLinkState(robot_l.id, TrackIndex)
    print("Track_LinkState_L:", Track_LinkState_L)
    print("Track_LinkState_R:", Track_LinkState_R)

    # offset = Track_LinkState_L[0] - data[1][0]


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

        # p.getCameraImage(320,
        #                 200,
        #                 flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
        #                 renderer=p.ER_BULLET_HARDWARE_OPENGL)

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
            orn_l = [math.pi, 0, 0]
            orn_r = [math.pi, 0, 0]
            # orn_l = [orientation_l.iloc[index, 0], orientation_l.iloc[index, 1], orientation_l.iloc[index, 2]]
            # orn_r = [orientation_r.iloc[index, 0], orientation_r.iloc[index, 1], orientation_r.iloc[index, 2]]            

            action_l = list(pos_l) + list(orn_l)
            action_r = list(pos_r) + list(orn_r)

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