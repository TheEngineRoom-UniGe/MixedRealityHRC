#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from baxter_hrc.msg import PlannedTrajectory, NextAction
from baxter_hrc.srv import ActionService, JointStateService
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.init_node(ros_node_name, anonymous=True)

    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
    	'baxter_joint_states': RosService('baxter_joint_states', JointStateService),
        'left_group/baxter_hrc_motion_planner': RosService('left_group/baxter_hrc_motion_planner', ActionService),
        'right_group/baxter_hrc_motion_planner': RosService('right_group/baxter_hrc_motion_planner', ActionService),
        'next_action': RosSubscriber('next_action', NextAction, tcp_server)
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()
