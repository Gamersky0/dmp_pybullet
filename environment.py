import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
from tqdm import tqdm
from env import ClutteredPushGrasp
from robot import Panda, UR5Robotiq85, UR5Robotiq140
from utilities import YCBModels, Camera
import numpy as np
# deprecated
class RobotArm:
    def __init__(self, arm_name, orn):
        self.name = arm_name
        self.orn = orn
        self.id = -1
        self.numArmJoints = -1
        self.numGripperJoints  = -1
        self.init_Arm()
        self.init_Gripper()
        self.init_Pose()

    def move2readyposition(self):
        print(f"move2readyposition not add yet.")     

    def init_Arm(self):
        if self.name.lower() == 'none':
            return None
        elif self.name.lower() == "kuka":
            # self.id = p.loadURDF("kuka_iiwa/model.urdf")
            self.id = p.loadURDF("dh_robotics_ag95_description/urdf/ur5_robotiq_85.urdf")
            if self.orn == "left":
                p.resetBasePositionAndOrientation(self.id, [0, 0, 0], [0, 0, 0, 1]) # position & orientation reset
            if self.orn == "right":
                p.resetBasePositionAndOrientation(self.id, [0, -0.8, 0], [0, 0, 0, 1]) # position & orientation reset
            self.numArmJoints = p.getNumJoints(self.id)
        else:
            print(f"Error! None support arm: ", self.name)
            return None

    def init_Pose(self):
        return
        print("NumJoints of {} arm: {}".format(self.name, self.numArmJoints))
        if (self.numArmJoints != 7):
            print("Error! NumJoints != 7")
            exit()
        for i in range(self.numArmJoints):
            p.resetJointState(self.id, i, rp[i])

    def init_Gripper(self):
        if self.orn == "right":
            return
        # AG-95
        self.gripper_id = p.loadURDF("dh_robotics_ag95_description/urdf/test.urdf")
        # self.gripper_id = p.loadURDF("dh_robotics_ag95_description/urdf/robotiq_85_gripper_simple.urdf")
        self.numGripperJoints = p.getNumJoints(self.gripper_id)
        p.resetBasePositionAndOrientation(self.gripper_id, [-1, -1, 0], [0, 0, 0, 1]) # position & orientation reset

        print("numJoints of robot_numJoints: ", self.numArmJoints)
        print("numJoints of gripper_numJoints: ", self.numGripperJoints)

        for i in range(self.numGripperJoints):
            p.changeDynamics(self.gripper_id, i, lateralFriction=0.2)
            p.resetJointState(self.gripper_id, i, 1)

        # 将夹具链接到机械臂的末端链接
        # joint_id = p.createConstraint(parentBodyUniqueId = self.id, 
        #                               parentLinkIndex = 6, 
        #                               childBodyUniqueId = self.gripper_id, 
        #                               childLinkIndex = -1,
        #                               jointType = p.JOINT_FIXED, 
        #                               jointAxis = [0, 0, 0],
        #                               parentFramePosition = [0, 0, 0], 
        #                               childFramePosition = [0, 0, 0],
        #                               parentFrameOrientation = [0, 0, 0, 1],
        #                               childFrameOrientation = [0, 0, 0, 1]
        #                               )

        # print(f"Gripper not add yet.")     

    def combine_arm_gripper(self):
        print(f"Not add yet.")

class Environment:
    def __init__(self, robot_l, robot_r):
        self.robot_l = robot_l
        self.robot_r = robot_r
        self.init_PyBullet_Environment()
        self.init_VisualizerCamera()
        self.init_Plane()
        # self.init_Table()
        # self.init_Object()
        self.robot_l.load()
        self.robot_r.load()

    def init_PyBullet_Environment(self):
        clid = p.connect(p.SHARED_MEMORY)
        if (clid < 0):
            p.connect(p.GUI)
            #p.connect(p.SHARED_MEMORY_GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

        p.setGravity(0, 0, -9.8)

    def init_Plane(slef):
        p.loadURDF("plane.urdf", [0, 0, 0], flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

    def init_Table(self):
        # load table
        table_pos = [-0.5, -0.4, -1]
        scaling_factor = 1.5
        tableId = p.loadURDF("table/table.urdf", table_pos, useFixedBase=True, globalScaling = scaling_factor)
        return tableId

    def init_Object(self):
        self.clothId = p.loadSoftBody("manipulation/towel.obj", # .obj格式 或 VTK格式
                                # basePosition = [-1, -0.4, 0.5],
                                basePosition = [-1, -0.4, 0.001],
                                scale = 1.5,
                                mass = 0.5, 
                                collisionMargin = 0.01,
                                useNeoHookean = 0, 
                                useBendingSprings=0,
                                useMassSpring=1, 
                                springElasticStiffness=50, 
                                springDampingStiffness=.1, 
                                springDampingAllDirections = 1, 
                                useSelfCollision = 0, 
                                frictionCoeff = .5, 
                                useFaceContact=1
                                )
        p.changeVisualShape(self.clothId, -1, rgbaColor=[0.4, 0.6, 1, 1], flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
        p.resetBasePositionAndOrientation(self.clothId, [-1, -0.4, 0.01], p.getQuaternionFromEuler([0, 0, -np.pi/2.0]))

        p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

    def init_VisualizerCamera(self):
        # camera visualizer settings
        default_cameraDistance = 2
        default_cameraYaw = 50
        default_cameraPitch = 200
        default_cameraTargetPosition = [0,-0.4,0]
        p.resetDebugVisualizerCamera(cameraDistance = default_cameraDistance, 
                                    cameraYaw = default_cameraYaw, 
                                    cameraPitch = default_cameraPitch, 
                                    cameraTargetPosition = default_cameraTargetPosition)




if __name__ == '__main__':
    robot_l = UR5Robotiq85((0, 0, 0), (0, 0, 0))
    robot_r = UR5Robotiq85((0, -0.8, 0), (0, 0, 0))
    Env = Environment(robot_l, robot_r)
    time.sleep(5)
    print("Success initing environment, exit")