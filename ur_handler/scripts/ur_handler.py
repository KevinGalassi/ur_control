#! /usr/bin/env python3

import rospy
import moveit_commander
import rospy
import math
import actionlib
import copy
from termcolor import cprint

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint 
from geometry_msgs.msg import Pose, Point, Quaternion


class URHandler(object) :

    def __init__(self, ee_frame="panda_object_link", robot_name='',  simulation=False) :

        print('UR Mover Start Initialization')
        
        self.simulation = simulation
        self.EE_frame = ee_frame

        self.group_name = robot_name
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)


        
        self.cart_vel = 0.005
        self.init_offset = 0.1

        print('UR Mover Start Initialization completed')






    def homing(self, open_gripper_cmd = False) :

        print('homing:')

        joint_goal = self.move_group.get_current_joint_values()

        joint_goal = [-1.5353, -0.767, -1.767, -2.367, 1.567, 1.567]


        self.move_group.go(joint_goal, wait=True)
        
        self.move_group.stop()

        print('homing: OK')


    def go_to_pose_goal(self, target_pose, EE_frame):

        self.move_group.set_pose_target(target_pose, EE_frame)
        rv  = self.move_group.go(wait = True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return rv

    def go_to_joint_goal(self, target_joint):

        self.move_group.set_joint_value_target(target_joint)
        self.move_group.go(wait = True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return 
    

    def plan_pose_goal(self, target_pose, EE_frame):
        self.move_group.set_pose_target(target_pose, EE_frame)
        traj = self.move_group.plan()
        self.move_group.clear_pose_targets()
        return traj


    def get_current_pose(self, EE_frame):
        return self.move_group.get_current_pose(EE_frame).pose


    def set_cartesian_velocity(self, velocity, init_offset = 0.1):
        self.cart_vel = velocity
        self.init_offset = init_offset


    def move_cartesian(self, target_pose, EE_frame, debug=False) :
        
        waypoints = []
        if isinstance(target_pose, Pose) :
            waypoints.append(copy.deepcopy(self.get_current_pose(EE_frame)))
            waypoints.append(copy.deepcopy(target_pose))
        elif isinstance(target_pose, list):
            waypoints = [pose for pose in target_pose]
            #waypoints.insert(0, self.get_current_pose(EE_frame))
        else :
            return 0.0

        self.move_group.set_end_effector_link(EE_frame)

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
        if fraction > 0.5 :
            #new_plan = self.velocity_scaling(plan, waypoints, self.cart_vel, self.init_offset)
            #if debug :
            self.move_group.execute(plan, wait = True)
  
        else :
            print(waypoints)
            return fraction
        

    def velocity_scaling(self, trajectory_cartesian, waypoints, velocity, init_offset) :

        distance = 0.0
        print(len(waypoints))
        print(waypoints)
        for i in range(len(waypoints) - 1) :
            p0 = waypoints[i].position
            p1 = waypoints[i+1].position
            flag_dist = math.sqrt(math.pow(p1.x - p0.x,2) + 
                                  math.pow(p1.y - p0.y,2) + 
                                  math.pow(p1.z - p1.z,2))
            distance += flag_dist

        overall_time = distance/velocity
        scaling_factor = overall_time/(trajectory_cartesian.joint_trajectory.points[(len(trajectory_cartesian.joint_trajectory.points)-1)].time_from_start.to_sec())

        print("Overall distance : {}, velocity : {}".format(distance, velocity))
        print("Overall time : {}".format(overall_time))
        print("Total old time: {}".format(trajectory_cartesian.joint_trajectory.points[len(trajectory_cartesian.joint_trajectory.points) - 1].time_from_start.to_sec()))
        print("scaling_factor : {}".format(scaling_factor))

        # Scaling
        new_trajectory = RobotTrajectory()
        new_trajectory.joint_trajectory.joint_names = [name for name in trajectory_cartesian.joint_trajectory.joint_names]

        for i in range(len(trajectory_cartesian.joint_trajectory.points)) :
            new_point = JointTrajectoryPoint()
            for k in range(7) :
                new_point.positions.append(copy.deepcopy(trajectory_cartesian.joint_trajectory.points[i].positions[k]))
                new_point.velocities.append(copy.deepcopy(trajectory_cartesian.joint_trajectory.points[i].velocities[k] / scaling_factor))
                new_point.accelerations.append(copy.deepcopy(trajectory_cartesian.joint_trajectory.points[i].accelerations[k] / (scaling_factor*scaling_factor)))
            new_point.time_from_start = trajectory_cartesian.joint_trajectory.points[i].time_from_start * scaling_factor
            new_trajectory.joint_trajectory.points.append(copy.deepcopy(new_point))

        for i in range(len(new_trajectory.joint_trajectory.points)) :
            new_trajectory.joint_trajectory.points[i].time_from_start = new_trajectory.joint_trajectory.points[i].time_from_start + rospy.Duration(init_offset)

        print("Final execution time : {}".format(new_trajectory.joint_trajectory.points[(len(new_trajectory.joint_trajectory.points)-1)].time_from_start.to_sec()))

        traj_point = JointTrajectoryPoint()
        traj_point.time_from_start = rospy.Duration(0)

        traj_point.positions = [joint for joint in new_trajectory.joint_trajectory.points[0].positions]
        traj_point.velocities = [0 for i in range(7)]
        traj_point.accelerations = [0 for i in range(7)]

        for i in range(1,len(new_trajectory.joint_trajectory.points)) :
            if new_trajectory.joint_trajectory.points[i].time_from_start <= new_trajectory.joint_trajectory.points[i-1].time_from_start :
                new_trajectory.joint_trajectory.points[i].time_from_start = new_trajectory.joint_trajectory.points[i-1].time_from_start + rospy.Duration(0.00001)

        new_trajectory.joint_trajectory.points.insert(0, traj_point)
        new_trajectory.joint_trajectory.points[len(new_trajectory.joint_trajectory.points)-1].time_from_start += rospy.Duration(init_offset)

        #print('out_trajectory')
        #print(new_trajectory)
        return new_trajectory


    def set_vel_acc(self, velocity, acc) :
        self.move_group.set_max_velocity_scaling_factor(velocity)
        self.move_group.set_max_acceleration_scaling_factor(acc)
        return

    # Testint

    def test(self) :

        # Home
        print('Homing')
        self.homing()

        print('choming completed')

        tg_pose = Pose() 
        tg_pose.position = Point(0.0, 0.2, 0.15397)
        tg_pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)

        print('Going to pose')
        self.go_to_pose_goal(tg_pose, "rightarm_tool0")
        print('Pose reached')

        print('home')   
        self.homing()
        print('home completed')


if __name__ == '__main__' :

    rospy.init_node("robot_mover")

    EE_frame = "robotiq_hande_tip_link_joint"
    robot_name = 'rightarm'

    robot = URHandler(EE_frame, robot_name)

    robot.test()
    
    print('rip')
