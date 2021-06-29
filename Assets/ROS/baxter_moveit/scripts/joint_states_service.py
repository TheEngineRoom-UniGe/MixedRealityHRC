#!/usr/bin/env python  
import rospy
from rospy.exceptions import ROSException

from sensor_msgs.msg import JointState
from baxter_moveit.srv import JointConfigService, JointConfigServiceRequest, JointConfigServiceResponse

def get_joint_states(req):

    response = JointConfigServiceResponse()
    try:
        joint_state_msg = rospy.wait_for_message("robot/joint_states", JointState, timeout=3.0)
        while len(joint_state_msg.position) < 2:
            joint_state_msg = rospy.wait_for_message("robot/joint_states", JointState)

        response.joint_state_msg = joint_state_msg
        return response
    except ROSException:
        # Fixed starting position in case real robot is not connected
        response.joint_state_msg.position = [0, 0, 0, 0.75, 0, -0.55, 0, 1.26, 0, 0, 0.75, 0, -0.55, 0, 1.26, 0, 0]
        return response

if __name__ == '__main__':

    rospy.init_node('baxter_joint_states_service')
    s = rospy.Service('baxter_joint_states', JointConfigService, get_joint_states)
    rospy.spin()
