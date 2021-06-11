#!/usr/bin/env python

import argparse
import sys

from copy import copy

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    GripperCommandActionGoal
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface
import baxter_tools

from baxter_moveit.msg import BaxterTrajectory
from baxter_interface import CHECK_VERSION

limb = ""

class Trajectory(object):

    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)
        gripper_topic = "/robot/end_effector/" + limb + "_gripper/gripper_action/goal"
        self.gripper_publisher = rospy.Publisher(gripper_topic, GripperCommandActionGoal, queue_size=10)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
            
    def open_gripper(self): 
    	gripperCommandMsg = GripperCommandActionGoal()
    	gripperCommandMsg.goal.command.position = 100.0
    	self.gripper_publisher.publish(gripperCommandMsg)
    	
    def close_gripper(self): 
    	gripperCommandMsg = GripperCommandActionGoal()
    	gripperCommandMsg.goal.command.position = 0.0
    	self.gripper_publisher.publish(gripperCommandMsg)

def trajectory_callback(msg, args):

    rospy.sleep(5)
    limb = args
        
    for i in range(len(msg.trajectory)):
    	
    	traj = Trajectory(limb)
    	# Start with open gripper
    	if i == 0:
    		traj.open_gripper()
    		
    	rospy.on_shutdown(traj.stop)
    	# Command Current Joint Positions first
    	limb_interface = baxter_interface.limb.Limb(limb)
    	    
    	t = 1
    	for point in msg.trajectory[i].joint_trajectory.points:
    		p = point.positions
    		traj.add_point(p, t)
    		t += 1
    	
    	traj.start()
    	traj.wait(t + 1)
    	# Close gripper on grasping
    	if i == 1:
    		rospy.sleep(1)
    		traj.close_gripper()
    	# Reopen gripper on release
    	elif i == len(msg.trajectory) - 1:
    		rospy.sleep(1)
    		traj.open_gripper()
    	    
    print("Exiting - Joint Trajectory Action Complete")
    
def main():
    
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    rospy.Subscriber("baxter_joint_trajectory", BaxterTrajectory, trajectory_callback, (limb))
    rospy.spin()


if __name__ == "__main__":
    main()
