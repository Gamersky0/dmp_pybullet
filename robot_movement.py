import pybullet as p
import time
import math
from datetime import datetime
from my_utilities import *
import pybullet_data
import keyboard


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
ENABLE_TRAJ_FOLLOW_DEBUG = False

class Movement():
    def __init__(self, env, robot_l, robot_r, traj_l, traj_r):
        self.env = env
        self.robot_l = robot_l
        self.robot_r = robot_r
        self.traj_l = traj_l
        self.traj_r = traj_r
        self.clothId = None
        self.boxId_l = None
        self.boxId_r = None

    def move_to_init_position(self):
        self.robot_l.move_ee(self.robot_l.arm_rest_poses, 'joint')
        self.robot_r.move_ee(self.robot_r.arm_rest_poses, 'joint')
        self.robot_l.move_gripper(0.01)
        self.robot_r.move_gripper(0.01)

    # Cloth & Box Init
    def init_cloth_and_box(self):
        # Init Object
        self.env.init_Object()
        self.clothId = self.env.clothId
        data = p.getMeshData(self.clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
        # print("Index number: ", data[0])
        # print(data[1][0])
        # print(data[1][11])
        self.boxId_l = p.loadURDF("cube.urdf", data[1][0], globalScaling = 0.001, useMaximalCoordinates = True)
        self.boxId_r = p.loadURDF("cube.urdf", data[1][11], globalScaling = 0.001, useMaximalCoordinates = True)
        p.changeDynamics(self.boxId_l, -1, mass=0, linearDamping=0.2, angularDamping=0.2)
        p.changeDynamics(self.boxId_r, -1, mass=0, linearDamping=0.2, angularDamping=0.2)
        p.createSoftBodyAnchor(self.clothId, 0, self.boxId_l, -1, [0, 0, 0.005])
        p.createSoftBodyAnchor(self.clothId, 11, self.boxId_r, -1, [0, 0, 0.005])

    # Option1
    # Follow Trajectory in fixed trajectory
    def trajectory_follow_fixed(self):
        TrackIndex = self.robot_l.eef_id

        orientation_l = self.traj_l.iloc[:, 3:]
        orientation_r = self.traj_l.iloc[:, 3:]
        trajectory_l = self.traj_l.iloc[:, :3]
        trajectory_r = self.traj_r.iloc[:, :3]

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

                p.resetBasePositionAndOrientation(self.boxId_l, list(pos_l), [0, 0, 0, 1])
                p.resetBasePositionAndOrientation(self.boxId_r, list(pos_r), [0, 0, 0, 1])

                if (useNullSpace == 1) and (useOrientation == 1): # only use this
                    self.robot_l.move_ee(action_l, 'end')
                    self.robot_r.move_ee(action_r, 'end')
                    # jointPoses_l = p.calculateInverseKinematics(armId_l, kukaEndEffectorIndex, pos_l, orientation_l, ll, ul, jr, rp)
                    # jointPoses_r = p.calculateInverseKinematics(armId_r, kukaEndEffectorIndex, pos_r, orientation_r, ll, ul, jr, rp)

            ls_l = p.getLinkState(self.robot_l.id, TrackIndex)
            ls_r = p.getLinkState(self.robot_r.id, TrackIndex)
            if (hasPrevPose):
                p.addUserDebugLine(prevPose_tl, pos_l, [0, 0, 0.3], 1, trailDuration) # l target move
                p.addUserDebugLine(prevPose_al, ls_l[4], [1, 0, 0], 1, trailDuration) # l actual move
                p.addUserDebugLine(prevPose_tr, pos_r, [0, 0, 0.3], 1, trailDuration) # r target move
                p.addUserDebugLine(prevPose_ar, ls_r[4], [1, 0, 0], 1, trailDuration) # r actual move
            prevPose_tl = pos_l
            prevPose_al = ls_l[4]
            prevPose_tr = pos_r
            prevPose_ar = ls_r[4]
            hasPrevPose = 1

        return

    # Option2
    # Follow Trajectory with couple feedback
    def trajectory_follow_with_couple(self):
        return


# TODO: 固定 or 实时
def pipeline(env, robot_l, robot_r, traj_l, traj_r):
    p.setRealTimeSimulation(useRealTimeSimulation)
    m = Movement(env, robot_l, robot_r, traj_l, traj_r)

    # STEP 1: move to init position 非阻塞
    m.move_to_init_position()
    time.sleep(2)

    # STEP 2: init object
    m.init_cloth_and_box()
    time.sleep(1)

    # TODO：机械臂坐标系问题
    # TODO: create a thread to attach object with robot
    # STEP 3: catch object
    # catch_object(robot_l, robot_r, boxId_l, boxId_r)
    # time.sleep(1)

    # STEP 4: move to ready position

    # STEP 5: follow trajectory
    if traj_l == None and traj_r == None:
        m.trajectory_follow_with_couple()
    else:
        m.trajectory_follow_fixed()

    time.sleep(10)
    return

def move_to_init_position(robot_l, robot_r):
    robot_l.move_ee(robot_l.arm_rest_poses, 'joint')
    robot_r.move_ee(robot_r.arm_rest_poses, 'joint')
    robot_l.move_gripper(0.01)
    robot_r.move_gripper(0.01)

def move_to_ready_position(robot, ready_position):
    print("Begin move to ready position.")

    TrackIndex = robot.eef_id

    # DEBUG：
    # numJoints = p.getNumJoints(robot.id)
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

# Catch Object in 11.29
def catch_object(robot_l, robot_r, boxId_l, boxId_r):
    # 获得刚体位置
    currentPos_l, currentOrn_l = p.getBasePositionAndOrientation(boxId_l)
    currentPos_r, currentOrn_r = p.getBasePositionAndOrientation(boxId_r)
    print("抓点位置 boxId_l currentPos_l: ", currentPos_l)
    print("抓点位置 boxId_r currentPos_r: ", currentPos_r)
    currentPos_l = list(currentPos_l)
    currentPos_r = list(currentPos_r)
    currentPos_l[2] += 0.1
    currentPos_r[2] += 0.1
    currentPos_l = tuple(currentPos_l)
    currentPos_r = tuple(currentPos_r)


    orn_l = [math.pi, 0, -math.pi / 2]
    orn_r = [math.pi, 0, -math.pi / 2]
    action_l = list(currentPos_l) + list(orn_l)
    action_r = list(currentPos_r) + list(orn_r)

    if (useNullSpace == 1) and (useOrientation == 1): # only use this
        robot_l.move_ee(action_l, 'end')
        robot_r.move_ee(action_r, 'end')

    # TODO: 到达位置后闭合夹具
    
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

def robot_move(env, robot_l, robot_r, data_l, data_r, only_box_move):
    TrackIndex = robot_l.eef_id
    p.setRealTimeSimulation(useRealTimeSimulation)

    orientation_l = data_l.iloc[:, 3:]
    orientation_r = data_r.iloc[:, 3:]
    trajectory_l = data_l.iloc[:, :3]
    trajectory_r = data_r.iloc[:, :3]

    ready_position_l = trajectory_l.iloc[0]
    ready_position_r = trajectory_r.iloc[0]
    
    # STEP 1: move to init position 非阻塞
    move_to_init_position(robot_l, robot_r)
    time.sleep(2)

    # STEP 2: init object
    boxId_l, boxId_r = cloth_box_init(env)
    time.sleep(1)

    # TODO：机械臂坐标系问题
    # TODO: create a thread to attach object with robot
    # STEP 3: catch object
    # catch_object(robot_l, robot_r, boxId_l, boxId_r)
    # time.sleep(1)

    # STEP 4: move to ready position
    # move_to_ready_position(robot_l, robot_r, boxId_l, boxId_r)

    # STEP 5:
    # follow trajectory without couple

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
            p.addUserDebugLine(prevPose_tl, pos_l, [0, 0, 0.3], 1, trailDuration) # l target move
            p.addUserDebugLine(prevPose_al, ls_l[4], [1, 0, 0], 1, trailDuration) # l actual move
            p.addUserDebugLine(prevPose_tr, pos_r, [0, 0, 0.3], 1, trailDuration) # r target move
            p.addUserDebugLine(prevPose_ar, ls_r[4], [1, 0, 0], 1, trailDuration) # r actual move
        prevPose_tl = pos_l
        prevPose_al = ls_l[4]
        prevPose_tr = pos_r
        prevPose_ar = ls_r[4]
        hasPrevPose = 1

    # p.disconnect()
    return