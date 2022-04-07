#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('camera/image',
                                          Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                                           Twist, queue_size=10)

        self.twist = Twist()

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([26, 43, 46])
        upper_yellow = np.array([34, 255, 255])

        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])

        mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask2 = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = image.shape
        search_top = 2*h/3
        mask1[0:search_top, 0:w] = 0
        mask2[0:search_top, 0:w] = 0

        # mask_pair = np.concatenate((mask1, mask2), axis=1)
        mask_add = mask1 + mask2

        cv2.imshow("masks", mask_add)

        M1 = cv2.moments(mask1)
        M2 = cv2.moments(mask2)

        if M1['m00'] > 0:
            if M1['m00'] == 0:
                cx1 = 0
                cy1 = 0
            else:
                cx1 = int(M1['m10']/M1['m00'])
                cy1 = int(M1['m01']/M1['m00'])
            if M2['m00'] == 0:
                cx2 = 0
                cy2 = 0
            else:
                cx2 = int(M2['m10']/M2['m00'])
                cy2 = int(M2['m01']/M2['m00'])

            fpt_x = (cx1 + cx2)/2
            fpt_y = (cy1 + cy2)/2 + 2*h/3

            cv2.circle(image, (cx1, cy1), 10, (0, 255, 255), -1)
            cv2.circle(image, (cx2, cy2), 10, (255, 255, 255), -1)
            cv2.circle(image, (fpt_x, fpt_y), 10, (128, 128, 128), -1)

            err = w/2 - fpt_x

            self.twist.linear.x = 0.3
            self.twist.angular.z = (err*90.0/160)/15
            self.cmd_vel_pub.publish(self.twist)
        
        cv2.imshow("image", image)
        cv2.waitKey(1)


rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
