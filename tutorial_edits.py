#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def all_close(goal, actual, tolerance):

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveFrankaGazeboRobot(object):
    def __init__(self):
        super(MoveFrankaGazeboRobot, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_franka_gazebo_robot", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL
        
         # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    
    def go_to_joint_state(self):

        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -0.5
        joint_goal[1] = -1.1
        joint_goal[2] = 0.5
        joint_goal[3] = -2.5
        joint_goal[4] = -1.5
        joint_goal[5] = 3  # 1/6 of a turn
        joint_goal[6] = 0.5

        move_group.go(joint_goal, wait=True)


        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    
    def go_to_pose_goal(self):

        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
    
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.6
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        move_group.set_pose_target(pose_goal) 
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
        
   
    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  
        return plan, fraction

      
    def display_trajectory(self, plan): # this whole constructor is to display the trajectoey path in RVIZ as well. 

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

       
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)


    def execute_plan(self, plan):
        
        move_group = self.move_group
        move_group.execute(plan, wait=True)
        
        
    



def main():
    try:
        
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveFrankaGazeboRobot()
        
        input(
                "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        tutorial.go_to_joint_state()
        
        input(
                "============ Press `Enter` to execute a movement using a pose goal ..."
        )
        tutorial.go_to_pose_goal()
        
        input(
                "============ Press `Enter` to plan and display a cartesian path, ..."
        )
        cartesian_plan, fraction = tutorial.plan_cartesian_path()
        input(
                "============ Press `Enter` to  display a saved trajectory (this will replay the cartesian path)  ..."
        )
        tutorial.display_trajectory(cartesian_plan)
        input(
                "============ Press `X` to  execute the saved path  ..."
        )
        tutorial.execute_plan(cartesian_plan)
        
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
if __name__ == "__main__":
    main()

