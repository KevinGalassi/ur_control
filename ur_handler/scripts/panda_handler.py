#! /usr/bin/env python3

import rospy
import moveit_commander
import rospy
import math
import actionlib
import copy
from termcolor import cprint
import arrow

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint 
from geometry_msgs.msg import Pose, Quaternion

from franka_gripper.msg import GraspAction, GraspActionGoal, GraspEpsilon
from franka_gripper.msg import MoveAction, MoveActionGoal
from franka_gripper.msg import StopAction, StopActionGoal

from franka_msgs.msg import FrankaState, ErrorRecoveryActionGoal, ErrorRecoveryAction

class PandaHandler(object) :

    def __init__(self, ee_frame="panda_object_link", simulation=False) :

        print('Panda Mover Start Initialization')
        
        self.simulation = simulation
        self.EE_frame = ee_frame

        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.group_hand = "panda_hand"
        self.hand_group = moveit_commander.MoveGroupCommander(self.group_hand)

        self.move_group.set_max_velocity_scaling_factor(0.2)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        
        self.cart_vel = 0.005
        self.init_offset = 0.1

        if not self.simulation:

            self.grasp_client = actionlib.SimpleActionClient("franka_gripper/grasp", GraspAction)
            self.gripper_move_client = actionlib.SimpleActionClient("franka_gripper/move", MoveAction)
            self.gripper_stop_client = actionlib.SimpleActionClient("franka_gripper/stop", StopAction)

            print('Recovery Client : Waiting')
            self.err_recovery_client = actionlib.SimpleActionClient("/franka_control/error_recovery", ErrorRecoveryAction)
            self.err_recovery_client.wait_for_server()
            cprint('Recovery Client : OK\n', "green")

        #rospy.sleep(2)
        print('Panda Mover Initialization completed')


    def check_errors(self, msg):
        members = [v for v in msg.__class__.__dict__.keys() if not v.startswith('__') and not v.startswith('_') and "violation" in v]
        members.append("cartesian_reflex")
        for v in members:
            if getattr(msg, v) == True: return True 
        return False

    def recovery(self):
        recovery_goal = ErrorRecoveryActionGoal()
        self.err_recovery_client.send_goal(recovery_goal)
        self.err_recovery_client.wait_for_result(rospy.Duration(3))
        self.homing()


    def go_to_pose_goal_error_recovery(self, goal_pose, EE_frame, n = 2):
        # move to next pose
        rv = self.go_to_pose_goal(goal_pose, EE_frame)

        if self.simulation:
            return rv

        for i in range(n):
            err_msg = rospy.wait_for_message("/franka_state_controller/franka_states", FrankaState)
            if self.check_errors(err_msg.current_errors) or rv == False:
                cprint('Failed! retrying pose', 'yellow')
                self.recovery()
                rv = self.go_to_pose_goal(goal_pose, EE_frame)
            else:
                break
        
        return rv



    def homing(self, open_gripper_cmd = False) :

        print('homing:')

        if open_gripper_cmd and not self.simulation: 
            stop_goal = StopActionGoal()
            self.gripper_stop_client.send_goal(stop_goal.goal)
            self.open_gripper()

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -0.785
        joint_goal[2] = 0
        joint_goal[3] = -2.356
        joint_goal[4] = 0
        joint_goal[5] = 1.571
        joint_goal[6] = 0.785

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
            #    t1 = arrow.utcnow()
            self.move_group.execute(plan, wait = True)
            if debug :
                print('Total execution time : {}'.format((arrow.utcnow() -t1).total_seconds()))
            return True
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


    def grasp(self,width=0.0) :

        grasp_goal = GraspActionGoal()
        eps = GraspEpsilon()

        eps.inner = 0.001
        eps.outer = 0.005

        grasp_goal.goal.width = width
        grasp_goal.goal.epsilon = eps
        grasp_goal.goal.speed = 0.15
        grasp_goal.goal.force = 2.0

        self.grasp_client.send_goal(grasp_goal.goal)
        

        result = self.grasp_client.wait_for_result()
        
        #print("Grasp completed : {}".format(result.error))

        return result

    def open_gripper(self, width = 0.07):
        
        print('Open gripper command received')
        open_goal = MoveActionGoal()

        open_goal.goal.width = width
        open_goal.goal.speed = 0.10

        self.gripper_stop_client.send_goal(StopActionGoal())
        self.gripper_stop_client.wait_for_result()
        
        self.gripper_move_client.send_goal(open_goal.goal)
        result = self.gripper_move_client.wait_for_result()
        print('Gripper op completed')
        return result

    def close_gripper(self):
        
        open_goal = MoveActionGoal()

        open_goal.goal.width = 0.001
        open_goal.goal.speed = 0.10

        self.gripper_move_client.send_goal(open_goal.goal)
        result = self.gripper_move_client.wait_for_result()

        return result


    def test(self) :

        while not rospy.is_shutdown() :
            #homing
            self.homing()

            print('Ready to take command')

            start_pose = Pose()

            start_pose.position.x = 0.30
            start_pose.position.z = 0.20
            start_pose.position.z = 0.40
            start_pose.orientation = Quaternion(1.0, 0.0, 0.0, 0.0)
            # move to first post
            self.go_to_pose_goal(start_pose, self.EE_frame)

            # Close gripper 
            self.grasp()

            # Move to goal pose

            goal_pose = Pose()
            goal_pose.position.x = 0.20
            goal_pose.position.z = 0.40
            goal_pose.position.z = 0.30
            start_pose.orientation = Quaternion(1.0, 0.0, 0.0, 0.0)

            self.go_to_pose_goal(goal_pose, self.EE_frame)



if __name__ == '__main__' :

    rospy.init_node("robot_mover")

    EE_frame = "panda_hand"

    robot = PandaHandler(EE_frame)

    robot.test()
    
    print('rip')
