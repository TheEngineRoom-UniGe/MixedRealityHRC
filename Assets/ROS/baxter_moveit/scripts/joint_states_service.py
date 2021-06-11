#!/usr/bin/env python  
import rospy

from sensor_msgs.msg import JointState
from baxter_moveit.srv import JointConfigService, JointConfigServiceRequest, JointConfigServiceResponse

def get_joint_states(req):
	
	response = JointConfigServiceResponse()
	joint_state_msg = rospy.wait_for_message("robot/joint_states", JointState)
	
	response.joint_state_msg = joint_state_msg
	return response

if __name__ == '__main__':

    rospy.init_node('baxter_joint_states_service')
    s = rospy.Service('baxter_joint_states', JointConfigService, get_joint_states)
    rospy.spin()
