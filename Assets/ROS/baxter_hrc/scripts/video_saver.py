#! /usr/bin/python

import rospy
import rospkg
import argparse
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VideoSaver():

    def __init__(self, path):
        self.count = 1
        self.path = path
        self.bridge = CvBridge()

    def save_frame(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            cv2.imwrite(self.path + 'frame_' + str(self.count) + '.jpeg', cv2_img)
            self.count += 1


def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f',
        '--folder',
        required=True,
        help='Subject name'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('video_saver')

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('baxter_hrc')

    folder_path = pkg_path + "/data/videos/" + args.folder
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    rospy.sleep(1)
    vs = VideoSaver(folder_path + "/")
    image_topic = "/usb_cam/image_raw"
    rospy.Subscriber(image_topic, Image, vs.save_frame)
    
    rospy.spin()

if __name__ == '__main__':
    main()
