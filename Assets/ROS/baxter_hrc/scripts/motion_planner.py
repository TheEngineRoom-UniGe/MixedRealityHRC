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


class MotionPlanner:

    def __init__(self, limb, offset):
        self.limb = limb
        self.height_offset = offset

        group_name = self.limb + "_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.publisher = rospy.Publisher('baxter_moveit_trajectory', PlannedTrajectory, queue_size=10)
        
    # Plan straight line trajectory
    def plan_cartesian_trajectory(self, destination_pose, start_joint_angles):
        current_joint_state = JointState()
        joint_names = [self.limb + '_' + joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_angles

        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = current_joint_state
        self.move_group.set_start_state(moveit_robot_state)
        self.move_group.set_goal_tolerance(10e-6)
        (plan, fraction) = self.move_group.compute_cartesian_path([destination_pose], 0.2, 0.0)

        if not plan:
            exception_str = """
                Trajectory could not be planned for a destination of {} with starting joint angles {}.
                Please make sure target and destination are reachable by the robot.
            """.format(destination_pose, destination_pose)
            exit(1)

        return plan

    # Plan trajectory to given pose
    def plan_to_pose(self, destination_pose, start_joint_angles):
        current_joint_state = JointState()
        joint_names = [self.limb + '_' + joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_angles

        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = current_joint_state
        self.move_group.set_start_state(moveit_robot_state)
        self.move_group.set_pose_target(destination_pose)
        self.move_group.set_goal_tolerance(10e-6)
        plan = self.move_group.plan()

        if not plan:
            exception_str = """
                Trajectory could not be planned for a destination of {} with starting joint angles {}.
                Please make sure target and destination are reachable by the robot.
            """.format(destination_pose, destination_pose)
            exit(1)

        return plan[1]
    
    # Plan joint space trajectory
    def plan_return_to_home(self, final_joint_config, start_joint_config):
        current_joint_state = JointState()
        joint_names = [self.limb + '_' + joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_config

        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = current_joint_state
        self.move_group.set_start_state(moveit_robot_state)
        self.move_group.set_joint_value_target(final_joint_config)
        self.move_group.set_goal_tolerance(10e-3)
        
        plan = self.move_group.plan()

        if not plan:
            exception_str = """
                Trajectory could not be planned for a destination of {} with starting joint angles {}.
                Please make sure target and destination are reachable by the robot.
            """.format(destination_pose, destination_pose)
            exit(1)

        return plan[1]

    # Dispatcher callback handling different operations upon request
    def dispatcher(self, req):
        if(req.operation == "pick_and_place" or req.operation == "put_back"):
            return self.pick_and_place(req)
        elif(req.operation == "tool_handover"):
            return self.tool_handover(req)
        elif(req.operation == "component_handover"):
            return self.component_handover(req)
        else:
            return None

    # Plan pick and place action
    def pick_and_place(self, req):
        op = req.operation

        response = ActionServiceResponse()
        response.operation = op
        response.arm = self.limb

        # Initial joint configuration
        current_robot_joint_configuration = [math.radians(req.joints.angles[i]) for i in range(7)]
        initial_joint_configuration = copy.deepcopy(current_robot_joint_configuration)

        # Pre grasp - position gripper directly above target object
        pre_grasp_traj = self.plan_cartesian_trajectory(req.pick_pose, current_robot_joint_configuration)

        previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(pre_grasp_traj)

        # Grasp - lower gripper so that fingers are on either side of object
        pick_pose = copy.deepcopy(req.pick_pose)
        pick_pose.position.z -= self.height_offset
        grasp_traj = self.plan_cartesian_trajectory(pick_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = grasp_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(grasp_traj)

        # Pick Up - raise gripper back to the pre grasp position
        pick_up_traj = self.plan_cartesian_trajectory(req.pick_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = pick_up_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(pick_up_traj)

        # Move gripper to desired placement position
        move_traj = self.plan_cartesian_trajectory(req.place_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = move_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(move_traj)

        # Place - Descend and leave object in the desired position
        place_pose = copy.deepcopy(req.place_pose)
        place_pose.position.z -= self.height_offset
        place_traj = self.plan_cartesian_trajectory(place_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = place_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(place_traj)
            
        # Return to home pose
        return_home_traj = self.plan_return_to_home(initial_joint_configuration, previous_ending_joint_angles)
        response.trajectories.append(return_home_traj)

        self.move_group.clear_pose_targets()

        jointsMsg = PlannedTrajectory()
        jointsMsg.operation = op
        jointsMsg.arm = self.limb
        jointsMsg.trajectory = response.trajectories
        self.publisher.publish(jointsMsg)

        return response
  
    # Plan tool handover action
    def tool_handover(self, req):
        op = req.operation
        
        response = ActionServiceResponse()
        response.operation = op
        response.arm = self.limb

        # Initial joint configuration
        current_robot_joint_configuration = [math.radians(req.joints.angles[i]) for i in range(7)]
        initial_joint_configuration = copy.deepcopy(current_robot_joint_configuration)

        # Pre grasp - position gripper directly above target object
        pre_grasp_traj = self.plan_cartesian_trajectory(req.pick_pose, current_robot_joint_configuration)

        previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(pre_grasp_traj)

        # Grasp - lower gripper so that fingers are on either side of object
        pick_pose = copy.deepcopy(req.pick_pose)
        pick_pose.position.z -= self.height_offset
        grasp_traj = self.plan_cartesian_trajectory(pick_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = grasp_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(grasp_traj)

        # Pick Up - raise gripper back to the pre grasp position
        lift_pose = copy.deepcopy(req.pick_pose)
        lift_pose.position.z += 0.20  # HARDCODED VALUE FOR NOW
        lift_up_traj = self.plan_cartesian_trajectory(lift_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = lift_up_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(lift_up_traj)
        
        # Handover - move gripper to desired handover position
        handover_traj = self.plan_to_pose(req.place_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = handover_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(handover_traj)
        
        # Return to home pose
        return_home_traj = self.plan_return_to_home(initial_joint_configuration, previous_ending_joint_angles)
        response.trajectories.append(return_home_traj)

        self.move_group.clear_pose_targets()
        
        jointsMsg = PlannedTrajectory()
        jointsMsg.operation = op
        jointsMsg.arm = self.limb
        jointsMsg.trajectory = response.trajectories
        self.publisher.publish(jointsMsg)

        return response

    # Plan Component handover action
    def component_handover(self, req):
        op = req.operation

        response = ActionServiceResponse()
        response.operation = op
        response.arm = self.limb

        # Initial joint configuration
        current_robot_joint_configuration = [math.radians(req.joints.angles[i]) for i in range(7)]
        initial_joint_configuration = copy.deepcopy(current_robot_joint_configuration)

        # Pre grasp - position gripper directly above target object
        pre_grasp_traj = self.plan_cartesian_trajectory(req.pick_pose, current_robot_joint_configuration)

        previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(pre_grasp_traj)

        # Grasp - lower gripper so that fingers are on either side of object
        pick_pose = copy.deepcopy(req.pick_pose)
        pick_pose.position.z -= self.height_offset
        grasp_traj = self.plan_cartesian_trajectory(pick_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = grasp_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(grasp_traj)

        # Pick Up - raise gripper back to the pre grasp position
        lift_pose = copy.deepcopy(req.pick_pose)
        lift_up_traj = self.plan_cartesian_trajectory(lift_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = lift_up_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(lift_up_traj)

        # Handover - move gripper to desired handover position
        handover_traj = self.plan_cartesian_trajectory(req.place_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = handover_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(handover_traj)

        # Put back - take object back to its original pose
        put_back_pose = copy.deepcopy(req.pick_pose)
        put_back_pose.position.z -= 0.9*self.height_offset
        put_back_traj = self.plan_cartesian_trajectory(put_back_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = put_back_traj.joint_trajectory.points[-1].positions
        response.trajectories.append(put_back_traj)

        # Return to home pose
        return_home_traj = self.plan_return_to_home(initial_joint_configuration, previous_ending_joint_angles)
        response.trajectories.append(return_home_traj)

        self.move_group.clear_pose_targets()

        jointsMsg = PlannedTrajectory()
        jointsMsg.operation = op
        jointsMsg.arm = self.limb
        jointsMsg.trajectory = response.trajectories
        self.publisher.publish(jointsMsg)

        return response


# Spawn obstacles for robot planning scene
def spawn_obstacles(scene, obstacle):
    
    p1 = PoseStamped()
    p1.header.frame_id = "world"
    p1.pose.position.x = 0
    p1.pose.position.y = 0.
    p1.pose.position.z = 0.4
    scene.add_box("table", p1, (2, 2, obstacle))

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
    
def main():
    # Initialize node
    rospy.init_node('motion_planner_service_node')
    moveit_commander.roscpp_initialize(sys.argv) 

    # Parse argument from launch file
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                    description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, type=str,
        help='limb parameter [left, right]'
    )
    required.add_argument(
        '-o', '--offset', required=True, type=float,
        help='height offset value'
    )
    required.add_argument(
        '-t', '--obstacle', required=True, type=float,
        help='obstacle height offset value'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    # Parsed arguments
    limb = args.limb
    offset = args.offset
    obstacle = args.obstacle

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)

    # Add collision objects to the scene
    spawn_obstacles(scene, obstacle)
    rospy.sleep(1)

    # Define motion planner object with argument parsed
    motion_planner = MotionPlanner(limb, offset)
    s = rospy.Service('baxter_hrc_motion_planner', ActionService, motion_planner.dispatcher)
        
    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    main()
    

    
