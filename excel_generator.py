import pandas as pd
import re

data_string = "[jointInfo(id=0, name='world_arm_joint', type=4, damping=0.0, friction=0.0, lowerLimit=0.0, upperLimit=-1.0, maxForce=0.0, maxVelocity=0.0, controllable=False), jointInfo(id=1, name='shoulder_pan_joint', type=0, damping=0.0, friction=0.0, lowerLimit=-3.14159265359, upperLimit=3.14159265359, maxForce=150.0, maxVelocity=3.15, controllable=True), jointInfo(id=2, name='shoulder_lift_joint', type=0, damping=0.0, friction=0.0, lowerLimit=-3.14159265359, upperLimit=3.14159265359, maxForce=150.0, maxVelocity=3.15, controllable=True), jointInfo(id=3, name='elbow_joint', type=0, damping=0.0, friction=0.0, lowerLimit=-3.14159265359, upperLimit=3.14159265359, maxForce=150.0, maxVelocity=3.15, controllable=True), jointInfo(id=4, name='wrist_1_joint', type=0, damping=0.0, friction=0.0, lowerLimit=-3.14159265359, upperLimit=3.14159265359, maxForce=28.0, maxVelocity=3.2, controllable=True), jointInfo(id=5, name='wrist_2_joint', type=0, damping=0.0, friction=0.0, lowerLimit=-3.14159265359, upperLimit=3.14159265359, maxForce=28.0, maxVelocity=3.2, controllable=True), jointInfo(id=6, name='wrist_3_joint', type=0, damping=0.0, friction=0.0, lowerLimit=-3.14159265359, upperLimit=3.14159265359, maxForce=28.0, maxVelocity=3.2, controllable=True), jointInfo(id=7,name='ee_fixed_joint', type=4, damping=0.0, friction=0.0, lowerLimit=0.0, upperLimit=-1.0, maxForce=0.0, maxVelocity=0.0, controllable=False), jointInfo(id=8, name='robotiq_85_base_joint', type=4, damping=0.0, friction=0.0, lowerLimit=0.0, upperLimit=-1.0, maxForce=0.0, maxVelocity=0.0, controllable=False), jointInfo(id=9, name='finger_joint', type=0, damping=0.0, friction=0.0, lowerLimit=0.0, upperLimit=0.8, maxForce=1000.0, maxVelocity=2.0, controllable=True), jointInfo(id=10, name='left_outer_finger_joint', type=4, damping=0.0, friction=0.0, lowerLimit=0.0, upperLimit=-1.0, maxForce=0.0, maxVelocity=0.0, controllable=False), jointInfo(id=11, name='left_inner_finger_joint', type=0, damping=0.0, friction=0.0,lowerLimit=-0.8757, upperLimit=0.0, maxForce=1000.0, maxVelocity=2.0, controllable=True), jointInfo(id=12, name='left_inner_finger_pad_joint', type=4, damping=0.0, friction=0.0, lowerLimit=0.0, upperLimit=-1.0, maxForce=0.0, maxVelocity=0.0, controllable=False), jointInfo(id=13, name='left_inner_knuckle_joint', type=0, damping=0.0, friction=0.0, lowerLimit=0.0, upperLimit=0.8757, maxForce=1000.0, maxVelocity=2.0, controllable=True), jointInfo(id=14, name='right_outer_knuckle_joint', type=0, damping=0.0, friction=0.0, lowerLimit=0.0, upperLimit=0.81, maxForce=1000.0, maxVelocity=2.0, controllable=True), jointInfo(id=15, name='right_outer_finger_joint', type=4, damping=0.0, friction=0.0, lowerLimit=0.0, upperLimit=-1.0, maxForce=0.0, maxVelocity=0.0, controllable=False), jointInfo(id=16, name='right_inner_finger_joint', type=0, damping=0.0, friction=0.0, lowerLimit=-0.8757, upperLimit=0.0, maxForce=1000.0, maxVelocity=2.0, controllable=True), jointInfo(id=17, name='right_inner_finger_pad_joint', type=4, damping=0.0, friction=0.0, lowerLimit=0.0, upperLimit=-1.0, maxForce=0.0, maxVelocity=0.0, controllable=False), jointInfo(id=18, name='right_inner_knuckle_joint', type=0, damping=0.0, friction=0.0, lowerLimit=0.0, upperLimit=0.8757, maxForce=1000.0, maxVelocity=2.0, controllable=True)]"

# Extracting the required information using regex
joint_ids = re.findall("id=(.*?),", data_string)
joint_names = re.findall("name='(.*?)'", data_string)
joint_types = re.findall("type=(.*?),", data_string)
dampings = re.findall("damping=(.*?),", data_string)
frictions = re.findall("friction=(.*?),", data_string)
lower_limits = re.findall("lowerLimit=(.*?),", data_string)
upper_limits = re.findall("upperLimit=(.*?),", data_string)
max_forces = re.findall("maxForce=(.*?),", data_string)
max_velocities = re.findall("maxVelocity=(.*?),", data_string)
controllables = re.findall("controllable=(.*?)\)", data_string)

# Creating a dictionary
data = {
    "Id": joint_ids,
    "Name": joint_names,
    "Type": joint_types,
    "Damping": dampings,
    "Friction": frictions,
    "Lower Limit": lower_limits,
    "Upper Limit": upper_limits,
    "Max Force": max_forces,
    "Max Velocity": max_velocities,
    "Controllable": controllables,
}

# Creating a DataFrame
df = pd.DataFrame(data)

# Saving DataFrame to CSV
df.to_csv("joint_info.csv", index=False)
