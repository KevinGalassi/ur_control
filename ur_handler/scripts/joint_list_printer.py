#! /usr/bin/env python3



import moveit_commander

# Initialize MoveIt commander
moveit_commander.roscpp_initialize([])

# Create a RobotCommander object to retrieve information about the robot
robot = moveit_commander.RobotCommander()

# Specify the name of the MoveGroup to control
group_name = "rightarm_robot"

# Create a MoveGroupCommander object for the specified MoveGroup
group = moveit_commander.MoveGroupCommander(group_name)

# Get the joint names for the MoveGroup
joint_names = group.get_active_joints()

# Print the joint names
print("Joint names for MoveGroup {}:".format(group_name))
for name in joint_names:
    print("- {}".format(name))

# Shutdown MoveIt commander
moveit_commander.roscpp_shutdown()
