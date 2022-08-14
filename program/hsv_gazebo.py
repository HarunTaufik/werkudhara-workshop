 #! /usr/bin/env python3

import rospy
from time import sleep
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Pose
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Range
import cv2
import numpy as np


class drone(object):

    global alt

    def __init__(self):
        self.rate = rospy.Rate(10)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.move_msg = Twist()
        self.takeoff_pub = rospy.Publisher("/drone/takeoff", Empty, queue_size=1)
        self.takeoff_msg = Empty()
        self.land_pub = rospy.Publisher("/drone/land", Empty, queue_size=1)
        self.land_msg = Empty()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/drone/front_camera/image_raw", Image, self.front_cam_cb)
        self.image_sub = rospy.Subscriber("/drone/down_camera/image_raw", Image, self.below_cam_cb)
        self.sonar_sub = rospy.Subscriber("/drone/sonar", Range, self.sonar_cb)
        self.alt_sub = rospy.Subscriber("/drone/gt_pose", Pose, self.alt_cb)

    def publish_vel(self, pesan):
        while True:
            connections = self.vel_pub.get_num_connections()
            if connections > 0:
                self.vel_pub.publish(pesan)
                break
            else:
                self.rate.sleep()

    # won't take off sometimes bcs of RNG
    def takeoff(self):
        print("Taking off...")
        self.takeoff_pub.publish(self.takeoff_msg)
        print(".")
        sleep(1)
        self.takeoff_pub.publish(self.takeoff_msg)
        print(".")
        sleep(1)
        vz = 1
        self.move(0, 0, vz)
        sleep(1.2)
        vz = 0
        self.move(0, 0, vz)
        sleep(1)
        vyaw = 0
        self.yaw(vyaw)

    def land(self):
        self.land_pub.publish(self.land_msg)
        print("Landing...")
        sleep(1)

    def nothing(x):
        pass
    cv2.namedWindow("Trackbar")
    cv2.createTrackbar("L-H", "Trackbar",0,180, nothing)
    cv2.createTrackbar("L-S", "Trackbar",122,255, nothing)
    cv2.createTrackbar("L-v", "Trackbar",0,255, nothing)
    cv2.createTrackbar("U-H", "Trackbar",180,180, nothing)
    cv2.createTrackbar("U-S", "Trackbar",255,255, nothing)
    cv2.createTrackbar("U-V", "Trackbar",255,255, nothing)

    def sonar_cb(self, sonarData):
        global snr
        snr = sonarData.range

    def move(self, x, y, z):
        self.move_msg.linear.x = x
        self.move_msg.linear.y = y
        self.move_msg.linear.z = z
        self.publish_vel(self.move_msg)

    def yaw(self, spd):
        self.move_msg.angular.z = spd
        self.publish_vel(self.move_msg)

    def stop(self):
        self.move_msg.linear.x = 0
        self.move_msg.linear.y = 0
        self.move_msg.linear.z = 0
        self.move_msg.angular.z = 0
        self.publish_vel(self.move_msg)

    def forward(self):
        if True:
            print("move forward!")
            vx = 4
            self.move(vx, 0, 0)
            sleep(2)
            vx = 0
            self.move(vx, 0, 0)
            sleep(1)

    def front_cam_cb(self, data):
        global front_cam
        front_cam = self.bridge.imgmsg_to_cv2(data, "bgr8")
        front_cam = cv2.add(front_cam, np.array([-50.0]))
        # cv2.imshow("Cam Depan", front_cam)
        cv2.waitKey(1) & 0xFF

    def below_cam_cb(self, data):
        global below_cam
        below_cam = self.bridge.imgmsg_to_cv2(data, "bgr8")
        below_cam = cv2.add(below_cam, np.array([-50.0]))
        # cv2.imshow("Cam Bawah", below_cam)
        cv2.waitKey(1) & 0xFF

    def alt_cb(self, data):
        global alt
        alt = data.position.z

    def gate(self):
        global front_cam
        global snr
        global alt
        
        font = cv2.FONT_HERSHEY_COMPLEX
        while True:
            # _,frame = cap.read()
            frame = front_cam
            below_frame = below_cam
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv_baw = cv2.cvtColor(below_frame, cv2.COLOR_BGR2HSV)
            L_H = cv2.getTrackbarPos("L-H", "Trackbar")
            L_S = cv2.getTrackbarPos("L-S", "Trackbar")
            L_V = cv2.getTrackbarPos("L-V", "Trackbar")
            U_H = cv2.getTrackbarPos("U-H", "Trackbar")
            U_S = cv2.getTrackbarPos("U-S", "Trackbar")
            U_V = cv2.getTrackbarPos("U-V", "Trackbar")
            lower_red = np.array([L_H,L_S,L_V])
            upper_red = np.array([U_H,U_S,U_V])
            mask = cv2.inRange(hsv, lower_red,upper_red)
            mask_baw = cv2.inRange(hsv_baw, lower_red,upper_red)
            
            #cv2.imshow("frame", frame)
            cv2.imshow("frame bawah", below_frame)
            #cv2.imshow("mask", mask)
            cv2.imshow("mask_baw", mask_baw)
            if cv2.waitKey(2) & 0xFF == 27:
                break
            # cap.release()
        cv2.destroyAllWindows()

    def line_follow(self):
        global below_cam


if __name__ == "__main__":
    rospy.init_node("Werkudhara")
    d = drone()
    print("start")
    d.takeoff()
    #d.forward()
    d.nothing()
    d.gate()
