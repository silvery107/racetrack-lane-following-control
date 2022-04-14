#!/usr/bin/env python
from __future__ import print_function
from time import time
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from controllers import PIDController, NonholomonicController
from moving_window_filter import MovingWindowFilter
import warnings
warnings.simplefilter('ignore', np.RankWarning)

K = np.array([265, 0, 160, 0, 265, 120, 0, 0, 1]).reshape((3,3))
R = np.array([1, 0, 0, 0, 0, -1, 0, -1, 0]).reshape((3,3))
t = np.array([0, 0.6, 0.7]).reshape((3,1))
n = np.array([0, -1, 0]).reshape((3,1))
d = 0.115
H = K.dot(R - t.dot(n.T)/d).dot(np.linalg.inv(K))
H /= H[2,2]
# H = np.array([-0.434, -1.33, 229, 0.0, -2.88, 462, 0.0, -0.00833, 1.0]).reshape((3,3))
print(H)


# pid = PIDController(2.5, 0, 0.0) # 2.5, 0, 0.0
controller = NonholomonicController(0.029, 2.5, 0.5) # 0.031, 2.5, 0.5

DTYPE = np.float32
distCoeffs = np.zeros(5, dtype=DTYPE)
kernel = cv2.getGaussianKernel(5, 2)
rot_90 = np.array([0,1,-1,0], dtype=DTYPE).reshape((2,2))
filter_x = MovingWindowFilter(1, dim=1)
filter_y = MovingWindowFilter(1, dim=1)
filter_t = MovingWindowFilter(2, dim=1)

ARUCO_TAG = cv2.aruco.DICT_6X6_50
this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_TAG)
this_aruco_parameters = cv2.aruco.DetectorParameters_create()

def rad2deg(rad):
    return rad/np.pi*180

def check_coord(coord):
    if coord[0]<0 and coord[1]<0:
        return -coord
    elif coord[0]<0 and coord[1]>0:
        return -coord
    else:
        return coord

def get_lane_param(mask):
    lane_param = {"curvature":0.0, "theta":0.0}
    mask_eroded = cv2.erode(mask, kernel, iterations=2)
    contours, _ = cv2.findContours(mask_eroded, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        line_param = cv2.fitLine(contours[0], distType=cv2.DIST_L2, param=0, reps=0.01, aeps=0.01)
        new_coord = rot_90.dot(np.array([line_param[0], line_param[1]],dtype=DTYPE).reshape((2,1)))
        new_coord = check_coord(new_coord)
        lane_param["theta"] = np.arctan2(new_coord[1], new_coord[0])
        
        cnt = contours[0].squeeze(axis=1)
        if cnt.shape[0]>2:
            poly_param = np.polyfit(cnt[:,0], cnt[:,1], 2)
            lane_param["curvature"] = (2*poly_param[0]) / \
                (1 + (2*poly_param[0]*cnt[0, 0] + poly_param[1])**2)**1.5

    return lane_param

class Follower:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('camera/image',
                                          Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                                           Twist, queue_size=10)

        self.twist = Twist()

        self.stop_flag = False
        self.stop_once = False
        self.timer = 0.0

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        img_bird_view = cv2.warpPerspective(image, H, (image.shape[1], image.shape[0]))
        # cv2.imshow("BEV", img_bird_view)

        hsv = cv2.cvtColor(img_bird_view, cv2.COLOR_BGR2HSV)
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([26, 43, 46], dtype=DTYPE)
        upper_yellow = np.array([34, 255, 255], dtype=DTYPE)

        lower_white = np.array([0, 0, 200], dtype=DTYPE)
        upper_white = np.array([180, 30, 255], dtype=DTYPE)

        mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
        left_lane_param = get_lane_param(mask1)

        mask2 = cv2.inRange(hsv, lower_white, upper_white)
        right_lane_param = get_lane_param(mask2)

        theta = filter_t.calculate_average((left_lane_param["theta"]+right_lane_param["theta"])/2)
        curvature = (left_lane_param["curvature"]+right_lane_param["curvature"])/2

        h, w, d = image.shape
        search_top = int(1*h/3)
        mask1[0:search_top, 0:w] = 0
        mask2[0:search_top, 0:w] = 0
        mask1[:, 0:int(w/6)] = 0
        mask2[:, 5*int(w/6):] = 0

        mask_add = mask1 + mask2
        # cv2.imshow("masks", mask_add)
        (corners, ids, _) = cv2.aruco.detectMarkers(
            image, this_aruco_dictionary, parameters=this_aruco_parameters)

        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, K, distCoeffs)

            if tvecs.squeeze()[-1] < 1.5 and not self.stop_flag and not self.stop_once:
                self.stop_flag = True
                self.timer = time()

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

            fpt_x = int((cx1 + cx2)/2)
            fpt_y = int((cy1 + cy2)/2)

            cv2.circle(img_bird_view, (cx1, cy1), 10, (0, 255, 255), -1)
            cv2.circle(img_bird_view, (cx2, cy2), 10, (255, 255, 255), -1)
            cv2.circle(img_bird_view, (fpt_x, fpt_y), 10, (128, 128, 128), -1)

            # err = w/2 - fpt_x

            # dx = fpt_y/10.0
            # dy = (w/2 - fpt_x)
            dx = filter_x.calculate_average(fpt_y/10.0)
            dy = filter_y.calculate_average(w/2 - fpt_x)

            v, omega = controller.apply(dx, dy, theta)

            print("dx:", dx)
            print("dy:", dy)
            print("theta:", rad2deg(theta))
            print("omega:", omega)
            print("veloc:", v)
            # print("curvature: %+5.4f"%curvature)
            # print()

            cv2.line(img_bird_view, (int(w/2-dy*5), h), (int(w/2-dy*5), h-int(dx*5)), (0, 0, 255), 2)
            cv2.line(img_bird_view, (int(w/2), h-2), (int(w/2-dy*5), h-2), (0, 0, 255), 2)
            
            if self.stop_flag:
                v = 0.0
                # omega = 0.0
                print("stop time: %.2f"%(time()-self.timer))
                if time()-self.timer>10:
                    self.stop_flag = False
                    self.stop_once = True
            
            # print("pid: %+.3f"%pid.apply(curvature))
            self.twist.linear.x = v
            self.twist.angular.z = omega#(err*90.0/160)/15

            self.cmd_vel_pub.publish(self.twist)
            
        
        img_pair = np.concatenate((image, img_bird_view), axis=1)
        cv2.imshow("image", img_pair)
        cv2.waitKey(1)

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
