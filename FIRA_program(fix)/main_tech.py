#! /usr/bin/env python3

import rospy
from time import sleep
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Range, Imu
import cv2
import numpy as np

class drone():
    def __init__(self):
        #rospy.init_node("a")
        self.rate = rospy.Rate(10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_msg = Twist()
        self.takeoff_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self.takeoff_msg = Empty()
        self.land_pub = rospy.Publisher('/drone/land', Empty, queue_size=1)
        self.land_msg = Empty()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/drone/down_camera/image_raw', Image, self.below_cam_cb)
        self.image2_sub = rospy.Subscriber('/drone/front_camera/image_raw', Image, self.front_cam_cb)
        self.sonar_sub = rospy.Subscriber('drone/sonar', Range, self.sonar_cb)
        self.alt_sub = rospy.Subscriber('/drone/gt_pose', Pose, self.alt_cb)
        self.compass_sub = rospy.Subscriber('/drone/imu', Imu, self.compass_cb)
        self.compass = 0
        self.alt = 0
        self.sonar = 0
        self.front_cam = 0
        self.below_cam = 0
        
    
    def publish_vel(self, pesan):
        while True:
            connections = self.vel_pub.get_num_connections()
            if connections > 0:
                self.vel_pub.publish(pesan)
                break
            else:
                self.rate.sleep()
    
    def takeoff(self):
        print("Taking off...")
        i = 1
        for i in range(3):
            self.takeoff_pub.publish(self.takeoff_msg)
            print(".")
            i = i+1
            sleep(0.5)
    
    def land(self):
        self.land_pub.publish(self.land_msg)
        print("Landing...")
        sleep(1)

    def velocity(self, x, y):
        self.move_msg.linear.x = x
        self.move_msg.linear.y = y
        self.publish_vel(self.move_msg)
    
    def velocity_z(self, z):
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

    def below_cam_cb(self, data):
        global below_cam
        below_cam = self.bridge.imgmsg_to_cv2(data, "bgr8")
        below_cam = cv2.add(below_cam, np.array([-50.0]))
        #cv2.imshow("camera bawah", below_cam)
        cv2.waitKey(1) & 0xFF

    def front_cam_cb(self, data):
        global front_cam
        front_cam = self.bridge.imgmsg_to_cv2(data, "bgr8")
        front_cam = cv2.add(front_cam, np.array([-50.0]))
        #cv2.imshow("Image window", front_cam)
        cv2.waitKey(1) & 0xFF

    def sonar_cb(self, msg):
        self.sonar = msg.range

    def alt_cb(self, data):
        self.alt = data.position.z	
    
    def compass_cb(self, data):
        self.compass = data.orientation.z
    
    def set_ang(self, objective):
        self.stop()
        rospy.sleep(0.3)
        while True:
            diff = objective - self.compass
            if abs(diff) < 0.02 :
                vyaw = 0
                self.stop()
                break
            else :
                vyaw = diff*25
            self.yaw(vyaw)
        
    def set_alt(self, objective):
        self.stop()
        rospy.sleep(0.3)
        while True:
            diff_alt = objective - self.alt
            c = self.compass
            vyaw = -c*50
            if abs(c) < 0.02 :
                vyaw = 0
            self.yaw(vyaw)
            if abs(diff_alt) < 0.03 :
                self.velocity_z(0)
                break
            else :
                self.velocity_z(diff_alt)
    
    
    
    def gate_detection(self) :
        global front_cam
        diff = 0
        mxluas = 0
        geser = 0
        frame = front_cam
        height, wide = frame.shape[:2]
        mask = cv2.inRange(frame, (137, 140, 0), (162, 255, 255))
        contours_g,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if len(contours_g) > 0 :
            area = [cv2.contourArea(c) for c in contours_g]
            idx = area.index(max(area))
            cv2.drawContours(frame, contours_g[idx], -1, (0,255,0), 1)
            (x1,y1),r = cv2.minEnclosingCircle(contours_g[idx])
            diff = x1 - int(wide/2)
            mxluas = int(max(area))
            geser = x1
        #cv2.imshow("frame", frame)
        return diff, mxluas, geser
    
    def drone_chopper(self):
        global front_cam
        diff = 0
        mxluas = 0
        ang = 0
        chopper_x = 0
        chopper_y = 0
        frame = front_cam
        height, wide = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 135, 0), (88, 255, 255))
        contours_g,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if len(contours_g) > 0 :
            area = [cv2.contourArea(c) for c in contours_g]
            idx = area.index(max(area))
            cv2.drawContours(frame, contours_g[idx], -1, (0,255,0), 1)
            (x2,y2),r = cv2.minEnclosingCircle(contours_g[idx])
            diff = x2 - int(wide/2)
            mxluas = int(max(area))
            box = cv2.minAreaRect(contours_g[idx])
            (x2,y2),(w,h),ang = box
            if ang < -45 :
                ang = 90 + ang
                if w < h and ang > 0:
                    ang = (90-ang)*-1
                if w > h and ang < 0:
                    ang = 90 + ang
            #cv2.putText(frame, str(ang), (10,10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1)
            chopper_x = x2
            chopper_y = y2
        #cv2.imshow("frame_", frame)
        return diff, mxluas, ang, chopper_x, chopper_y
            
    