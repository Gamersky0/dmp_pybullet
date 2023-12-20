import pybullet as p
from time import sleep

physicsClient = p.connect(p.GUI)
import pybullet_data

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

gravZ=-30
p.setGravity(0, 0, gravZ)

planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])

planeId = p.loadURDF("plane.urdf", [0,0,0],planeOrn)


# clothId_second = p.loadSoftBody("test/kubni/towel.obj", # .obj格式 或 VTK格式
#                          basePosition = [0, 0, 0],
#                          scale = 10,
#                          mass = 1., 
#                          collisionMargin = 0.01,
#                          useNeoHookean = 0, 
#                          useBendingSprings=1,
#                          useMassSpring=1, 
#                          springElasticStiffness=300, 
#                          springDampingStiffness=.1, 
#                          springDampingAllDirections = 1, 
#                          useSelfCollision = 0, 
#                          frictionCoeff = .5, 
#                          useFaceContact=1
#                          )

# self.blanket = p.loadSoftBody(os.path.join(self.directory, 'clothing', 'blanket_2089v.obj'), 
#                               scale=0.8, 
#                               mass=0.15, 
#                               useBendingSprings=0, 
#                               useMassSpring=1, 
#                               springElasticStiffness=1, 
#                               springDampingStiffness=0.01, 
#                               springDampingAllDirections=1, 
#                               springBendingStiffness=0, 
#                               useSelfCollision=1, 
#                               collisionMargin=0.0001, frictionCoeff=0.1, useFaceContact=1, physicsClientId=self.id)

clothId_second = p.loadSoftBody("manipulation/towel.obj", # .obj格式 或 VTK格式
                         basePosition = [0,0,1],
                         mass = 0.5, 
                         collisionMargin = 0.01,
                         useNeoHookean = 0, 
                         useBendingSprings=0,
                         useMassSpring=1, 
                         springElasticStiffness=100, 
                         springDampingStiffness=.1, 
                         springDampingAllDirections = 1, 
                         useSelfCollision = 0, 
                         frictionCoeff = .5, 
                         useFaceContact=1
                         )
p.changeVisualShape(clothId_second, -1, rgbaColor=[0.4, 0.6, 1, 1], flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
# p.createSoftBodyAnchor(clothId_second, 0, -1, -1)
# p.createSoftBodyAnchor(clothId_second, 11, -1, -1)
p.createSoftBodyAnchor(clothId_second, 132, -1, -1)
p.createSoftBodyAnchor(clothId_second, 143, -1, -1)


clothId = p.loadSoftBody("cloth_z_up.obj", 
                         basePosition = [0,0,2], 
                         scale = 0.5, 
                         mass = 1., 
                         collisionMargin = 0.01,
                         useNeoHookean = 0, 
                         useBendingSprings=1,
                         useMassSpring=1, 
                         springElasticStiffness=300, 
                         springDampingStiffness=.1, 
                         springDampingAllDirections = 1, 
                         useSelfCollision = 0, 
                         frictionCoeff = .5, 
                         useFaceContact=1
                         )

p.changeVisualShape(clothId, -1, rgbaColor=[0.4, 0.6, 1, 1], flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

p.createSoftBodyAnchor(clothId  ,24,-1,-1)
p.createSoftBodyAnchor(clothId ,20,-1,-1)


p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

# debug = True
debug = False
if debug:
  data = p.getMeshData(clothId_second, -1, flags=p.MESH_DATA_SIMULATION_MESH)
  print("--------------")
  print("data=",data)
  print(data[0])
  print(data[1])
  text_uid = []
  for i in range(data[0]):
      pos = data[1][i]
      uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1])
      text_uid.append(uid)

while p.isConnected():
  p.getCameraImage(320,200)
  
  if debug:
    data = p.getMeshData(clothId_second, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    for i in range(data[0]):
      pos = data[1][i]
      uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1], replaceItemUniqueId=text_uid[i])

  p.setGravity(0, 0, gravZ)
  p.stepSimulation()
  #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
  #sleep(1./240.)
  
