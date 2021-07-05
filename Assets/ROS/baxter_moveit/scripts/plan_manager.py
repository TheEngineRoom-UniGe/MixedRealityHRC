#!/usr/bin/env python

from __future__ import print_function

import rospy
import argparse
import sys
import copy
import math
import rospkg
import logging
import cv2
import serial
import threading
import time

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Quaternion, Pose, PoseStamped

from baxter_moveit.msg import BaxterTrajectory, BaxterAction
from baxter_moveit.srv import TrajectoryService, TrajectoryServiceRequest, TrajectoryServiceResponse

from cv_bridge import CvBridge

# Separate thread that listens to serial port for user input
class SerialReaderTask:

    def __init__(self):
        self.running = True
        self.has_received_command = False
        self.t_received_command = 0.0

    def handle_data(self, data):
        if(int(data)== 1 and abs(time.time() - self.t_received_command) > 10.0):
            self.has_received_command = True
            self.t_received_command = time.time()

    def terminate(self):
        self.running = False

    def run(self, ser):
        while self.running:
            reading = ser.read(1).decode()
            self.handle_data(reading)


def logging_callback(msg):
    logging.info(msg)

def main():

    rospy.init_node('plan_manager', disable_signals=True)
    
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('baxter_moveit')
    logging.basicConfig(filename=pkg_path + "/logs/test.log",
                                filemode='a',
                                format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
                                datefmt='%H:%M:%S',
                                level=logging.INFO,
                                force=True)
                                
    # Start thread that listens to serial port                            
    serial_port = serial.Serial('/dev/ttyACM0')
    reader_task = SerialReaderTask()
    thread = threading.Thread(target=reader_task.run, args=(serial_port,))
    thread.start()
    
    action_pub = rospy.Publisher('baxter_action', BaxterAction, queue_size=10)
    image_pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=10)
    rospy.Subscriber("left_group/baxter_joint_trajectory", BaxterTrajectory, logging_callback)
    rospy.Subscriber("right_group/baxter_joint_trajectory", BaxterTrajectory, logging_callback)
        
    # Read plan steps from file
    with open(pkg_path + '/config/plan.txt') as f:
        plan_steps = f.readlines()
        
    # Publish start screen image on robot's display
    img = cv2.imread(pkg_path + "/images/start.png")
    img_msg = CvBridge().cv2_to_imgmsg(img)
    rospy.sleep(0.25)
    image_pub.publish(img_msg)
        
    # Prepare dictionary of images file names and step indices
    images_dict = {
        0: "step1",
        1: "step2",
        2: "step3",
        3: "step4",
        4: "step5",
        5: "step6",
        6: "step7",
        8: "step8",
        9: "step9",
        10: "step10",
    }    

    # Wait for user's tap before sending actions
    print("Press joystick to start collaboration ..")
    global has_received_command
    while(not reader_task.has_received_command):
        try:
             rospy.sleep(0.25)
        except KeyboardInterrupt:
            reader_task.terminate()
            thread.join()
            sys.exit(1)
    reader_task.has_received_command = False

    logging.info("First action started")
    t_start = rospy.Time.now()
    
    for i,step in enumerate(plan_steps):
        instruction = step.split()

        operation = instruction[0]
        id = int(instruction[1])

        # If row contains two instructions, plan for both arms
        if(len(instruction) > 2):
            operation2 = instruction[2]
            id2 = int(instruction[3])

            baxter_action_msg = BaxterAction()
            baxter_action_msg.operation = operation
            baxter_action_msg.id = id
            action_pub.publish(baxter_action_msg)

            rospy.sleep(0.1)

            baxter_action_msg.operation = operation2
            baxter_action_msg.id = id2
            action_pub.publish(baxter_action_msg)

        else:
            baxter_action_msg = BaxterAction()
            baxter_action_msg.operation = operation
            baxter_action_msg.id = id
            action_pub.publish(baxter_action_msg)

        # If certain step of plan is reached, publish new image with instructions
        if(i in images_dict):
            img = cv2.imread(pkg_path + "/images/" + images_dict[i] + ".png")
            img_resized = cv2.resize(img, (1000, 600), interpolation = cv2.INTER_AREA)
            img_msg = CvBridge().cv2_to_imgmsg(img_resized)
            image_pub.publish(img_msg)

        print("Press joystick for next action ..")
        while(not reader_task.has_received_command):
           try:
                rospy.sleep(0.25)
           except KeyboardInterrupt:
               reader_task.terminate()
               thread.join()
               sys.exit(1)
        reader_task.has_received_command = False

        elapsed = rospy.Time.now().secs - t_start.secs
        logging.info("Action " + str(i+1) + " took: " + str(elapsed) + " seconds")
        t_start = rospy.Time.now()

    # Publish end screen image to display at the end of planning steps
    img = cv2.imread(pkg_path + "/images/end.png")
    img_msg = CvBridge().cv2_to_imgmsg(img)
    image_pub.publish(img_msg)
    logging.info("Last action completed")
    
    reader_task.terminate()
    thread.join()
    
    rospy.sleep(0.5)

if __name__ == "__main__":
    main()
    

    
