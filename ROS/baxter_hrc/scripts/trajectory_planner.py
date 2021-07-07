#!/usr/bin/env python

from __future__ import print_function

import rospy
import argparse
import sys
import copy
import math
import moveit_commander
from moveit_commander.conversions import pose_to_list

from std_msgs.msg import String
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Pose, PoseStamped

from baxter_hrc.msg import PlannedTrajectory
from baxter_hrc.srv import ActionService, ActionServiceRequest, ActionServiceResponse

# Global variables
trajectory_publisher = rospy.Publisher('baxter_joint_trajectory', PlannedTrajectory, queue_size=10)
height_offset = 0.0
height_obstacle = 0.0
       
# Plan straight line trajectory
def plan_cartesian_trajectory(move_group, limb, destination_pose, start_joint_angles):
    current_joint_state = JointState()
    joint_names = [limb + '_' + joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)
    move_group.set_goal_tolerance(10e-6)
    (plan, fraction) = move_group.compute_cartesian_path([destination_pose], 0.2, 0.0)

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        exit(1)

    return plan

# Plan trajectory to given pose
def plan_to_pose(move_group, limb, destination_pose, start_joint_angles):
    current_joint_state = JointState()
    joint_names = [limb + '_' + joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)
    move_group.set_pose_target(destination_pose)
    move_group.set_goal_tolerance(10e-6)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        exit(1)

    return plan[1]
    
# Plan joint space trajectory
def plan_return_to_home(move_group, limb, final_joint_config, start_joint_config):
    current_joint_state = JointState()
    joint_names = [limb + '_' + joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_config

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)
    move_group.set_joint_value_target(final_joint_config)
    move_group.set_goal_tolerance(10e-3)
    
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        exit(1)

    return plan[1]

# PICK AND PLACE ACTION
def pick_and_place(req):

    op = req.operation
    limb = req.arm
    
    response = ActionServiceResponse()
    response.operation = op
    response.arm = limb

    group_name = limb + "_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Initial joint configuration
    current_robot_joint_configuration = [math.radians(req.joints.angles[i]) for i in range(7)]
    initial_joint_configuration = copy.deepcopy(current_robot_joint_configuration)
    
    # Pre grasp - position gripper directly above target object
    pre_grasp_traj = plan_cartesian_trajectory(move_group, limb, req.pick_pose, current_robot_joint_configuration)

    previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(pre_grasp_traj)

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= height_offset
    grasp_traj = plan_cartesian_trajectory(move_group, limb, pick_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = grasp_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(grasp_traj)

    # Pick Up - raise gripper back to the pre grasp position
    pick_up_traj = plan_cartesian_trajectory(move_group, limb, req.pick_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = pick_up_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(pick_up_traj)
    
    # Move gripper to desired placement position
    move_traj = plan_cartesian_trajectory(move_group, limb, req.place_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = move_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(move_traj)

    # Place - Descend and leave object in the desired position
    place_pose = copy.deepcopy(req.place_pose)
    place_pose.position.z -= height_offset
    place_traj = plan_cartesian_trajectory(move_group, limb, place_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = place_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(place_traj)
        
    # Return to home pose
    return_home_traj = plan_return_to_home(move_group, limb, initial_joint_configuration, previous_ending_joint_angles)
    response.trajectories.append(return_home_traj)

    move_group.clear_pose_targets()
    
    jointsMsg = PlannedTrajectory()
    jointsMsg.operation = op
    jointsMsg.arm = limb
    jointsMsg.trajectory = response.trajectories
    trajectory_publisher.publish(jointsMsg)

    return response
    
# TOOL HANDOVER ACTION
def tool_handover(req):

    op = req.operation
    limb = req.arm
    
    response = ActionServiceResponse()
    response.operation = op
    response.arm = limb

    group_name = limb + "_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Initial joint configuration
    current_robot_joint_configuration = [math.radians(req.joints.angles[i]) for i in range(7)]
    initial_joint_configuration = copy.deepcopy(current_robot_joint_configuration)

    # Pre grasp - position gripper directly above target object
    pre_grasp_traj = plan_cartesian_trajectory(move_group, limb, req.pick_pose, current_robot_joint_configuration)

    previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(pre_grasp_traj)

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= height_offset
    grasp_traj = plan_cartesian_trajectory(move_group, limb, pick_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = grasp_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(grasp_traj)

    # Pick Up - raise gripper back to the pre grasp position
    lift_pose = copy.deepcopy(req.pick_pose)
    lift_pose.position.z += 0.20
    lift_up_traj = plan_cartesian_trajectory(move_group, limb, lift_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = lift_up_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(lift_up_traj)
    
    # Handover - move gripper to desired handover position
    handover_traj = plan_to_pose(move_group, limb, req.place_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = handover_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(handover_traj)
    
    # Return to home pose
    return_home_traj = plan_return_to_home(move_group, limb, initial_joint_configuration, previous_ending_joint_angles)
    response.trajectories.append(return_home_traj)

    move_group.clear_pose_targets()
    
    jointsMsg = PlannedTrajectory()
    jointsMsg.operation = op
    jointsMsg.arm = limb
    jointsMsg.trajectory = response.trajectories
    trajectory_publisher.publish(jointsMsg)

    return response

# COMPONENT HANDOVER ACTION
def component_handover(req):

    op = req.operation
    limb = req.arm

    response = ActionServiceResponse()
    response.operation = op
    response.arm = limb

    group_name = limb + "_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Initial joint configuration
    current_robot_joint_configuration = [math.radians(req.joints.angles[i]) for i in range(7)]
    initial_joint_configuration = copy.deepcopy(current_robot_joint_configuration)

    # Pre grasp - position gripper directly above target object
    pre_grasp_traj = plan_cartesian_trajectory(move_group, limb, req.pick_pose, current_robot_joint_configuration)

    previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(pre_grasp_traj)

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= height_offset
    grasp_traj = plan_cartesian_trajectory(move_group, limb, pick_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = grasp_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(grasp_traj)

    # Pick Up - raise gripper back to the pre grasp position
    lift_pose = copy.deepcopy(req.pick_pose)
    lift_up_traj = plan_cartesian_trajectory(move_group, limb, lift_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = lift_up_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(lift_up_traj)

    # Handover - move gripper to desired handover position
    handover_traj = plan_cartesian_trajectory(move_group, limb, req.place_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = handover_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(handover_traj)

    # Put back - take object back to its original pose
    put_back_pose = copy.deepcopy(req.pick_pose)
    put_back_pose.position.z -= 0.9*height_offset
    put_back_traj = plan_cartesian_trajectory(move_group, limb, put_back_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = put_back_traj.joint_trajectory.points[-1].positions
    response.trajectories.append(put_back_traj)

    # Return to home pose
    return_home_traj = plan_return_to_home(move_group, limb, initial_joint_configuration, previous_ending_joint_angles)
    response.trajectories.append(return_home_traj)

    move_group.clear_pose_targets()

    jointsMsg = PlannedTrajectory()
    jointsMsg.operation = op
    jointsMsg.arm = limb
    jointsMsg.trajectory = response.trajectories
    trajectory_publisher.publish(jointsMsg)

    return response

# Dispatcher callback handling different operations upon request
def dispatcher(req):
    if(req.operation == "pick_and_place" or req.operation == "put_back"):
    	return pick_and_place(req)
    elif(req.operation == "tool_handover"):
    	return tool_handover(req)
    elif(req.operation == "component_handover"):
    	return component_handover(req)
    else:
    	return None

# Spawn obstacles for robot planning scene
def spawn_obstacles(scene):
    
    global height_obstacle
    p1 = PoseStamped()
    p1.header.frame_id = "world"
    p1.pose.position.x = 0
    p1.pose.position.y = 0.
    p1.pose.position.z = 0.4
    scene.add_box("table", p1, (2, 2, height_obstacle))

    p2 = PoseStamped()
    p2.header.frame_id = "world"
    p2.pose.position.x = -0.5
    p2.pose.position.y = 0
    p2.pose.position.z = 1
    scene.add_box("rear_wall", p2, (0.25, 3, 3))

    p2 = PoseStamped()
    p2.header.frame_id = "world"
    p2.pose.position.x = 1
    p2.pose.position.y = 0
    p2.pose.position.z = 1
    scene.add_box("front_wall", p2, (0.1, 2, 2))

    print("Adding obstacles to scene..")
    
def moveit_server():

    moveit_commander.roscpp_initialize(sys.argv) 
    s = rospy.Service('baxter_hrc_trajectory', ActionService, dispatcher)
        
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)

    # Add collision objects to the scene
    spawn_obstacles(scene)
    rospy.sleep(1)
    
    print("Ready to plan")
    rospy.spin()

def main():
    rospy.init_node('moveit_trajectory_planner')
    
    # Parse arguments from launch file
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-o', '--offset', required=True, type=float,
        help='height offset value'
    )
    required.add_argument(
        '-t', '--obstacle', required=True, type=float,
        help='obstacle height offset value'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    global height_offset
    height_offset = args.offset
    
    global height_obstacle
    height_obstacle = args.obstacle

    moveit_server()


if __name__ == "__main__":
    main()
    

    
