#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from panda_moveit.msg import PandaMoveitJoints, PandaTrajectory
from panda_moveit.srv import TrajectoryService, JointConfigService
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import JointState


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.init_node(ros_node_name, anonymous=True)

    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
    	'panda_joint_states': RosService('panda_joint_states', JointConfigService),
        'panda_moveit_trajectory': RosService('panda_moveit_trajectory', TrajectoryService)
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()
