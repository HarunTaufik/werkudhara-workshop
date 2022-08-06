#! /usr/bin/env python3

import rospy
from time import sleep
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
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
        self.image_sub = rospy.Subscriber("/drone/front_camera/image_raw", Image, self.cam_dep_cb)
        self.image_sub = rospy.Subscriber("/drone/down_camera/image_raw", Image, self.cam_bawah_cb)

    def publish_vel(self, pesan):
        while True:
            connections = self.vel_pub.get_num_connections()
            if connections > 0:
                self.vel_pub.publish(pesan)
                break
            else:
                self.rate.sleep()

#won't take off sometimes bcs of RNG
    def takeoff(self):
        print("Taking off...")
        self.takeoff_pub.publish(self.takeoff_msg)
        print(".")
        sleep(1)
        self.takeoff_pub.publish(self.takeoff_msg)
        print(".")
        sleep(1)
        self.takeoff_pub.publish(self.takeoff_msg)
        print(".")
        sleep(1)
        print("naik dikit")
        vz = 0.5
        self.gerak(0, 0, vz)
        sleep(2.5)
        vz = 0
        self.gerak(0, 0, vz)
        

    def land(self):
        self.land_pub.publish(self.land_msg)
        print("Landing...")
        sleep(1)

    def gerak(self, x, y, z):
        self.move_msg.linear.x = x
        self.move_msg.linear.y = y
        self.move_msg.linear.z = z
        self.publish_vel(self.move_msg)

    def yaw(self, spd):
        self.move_msg.angular.z = spd
        self.publish_vel(self.move_msg)

    def diam(self):
        self.move_msg.linear.x = 0
        self.move_msg.linear.y = 0
        self.move_msg.linear.z = 0
        self.move_msg.angular.z = 0
        self.publish_vel(self.move_msg)

    def maju_depan(self):
        if True:
            print("maju")
            vx = 1
            self.gerak(vx, 0, 0)
            sleep(4)
            vx = 0
            self.gerak(vx, 0, 0)

    def cam_dep_cb(self, data):
        global cam_depan
        cam_depan = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cam_depan = cv2.add(cam_depan, np.array([-50.0]))
        # cv2.imshow("Cam Depan", cam_depan)
        cv2.waitKey(1) & 0xFF

    def cam_bawah_cb(self, data):
        global cam_bawah
        cam_bawah = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cam_bawah = cv2.add(cam_bawah, np.array([-50.0]))
        # cv2.imshow("Cam Bawah", cam_bawah)
        #cv2.waitKey(1) & 0xFF

    def gawang(self):
        global cam_depan
        while True:
            # _,frame = cap.read()
            frame = cam_depan
            # crop = frame[327:445, 137:530]
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv_a = np.array([180, 153, 77])
            hsv_b = np.array([0, 0, 0])
            orenji_a = np.array([0, 122, 0])
            orenji_b = np.array([180, 255, 255])
            mask = cv2.inRange(hsv, hsv_b, hsv_a)
            blade = cv2.inRange(hsv, orenji_a, orenji_b)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel)
            cv2.line(frame, (280, 175), (360, 175), (255, 0, 0), 2)
            cv2.line(frame, (320, 135), (320, 215), (255, 0, 0), 2)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            for cnt in contours:
                luas = [cv2.contourArea(contour) for contour in contours]
                luas.append(0)
                idx = luas.index(max(luas))
                area = cv2.contourArea(cnt)
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                # e = approx.ravel()[0]
                # f = approx.ravel()[1]
                if area > 400:
                    cv2.drawContours(frame, [approx], 0, (0, 0, 255), 5)
                    if len(approx) == 4:
                        (a, b), radius = cv2.minEnclosingCircle(contours[idx])
                        cv2.circle(frame, (int(a), int(b)), 1, (0, 255, 0), 5)

            conturs, _ = cv2.findContours(blade, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for cnts in conturs:
                pisau = [cv2.contourArea(cnts) for cnts in conturs]
                pisau.append(0)
                indx = pisau.index(max(pisau))
                cv2.drawContours(frame, conturs, -1, (0, 255, 0), 1)
                (c, d), radius = cv2.minEnclosingCircle(conturs[indx])
                cv2.circle(frame, (int(c), int(d)), 1, (0, 255, 0), 5)
                # bilek
                if c > 320 and d < 240:
                    self.gerak(0, 0, 0)
                elif c > 320 and d > 240:
                    self.gerak(0, 0, 0)
                elif c < 320 and d > 240:
                    print("maju")
                    self.gerak(2, 0, 0)
                elif c < 320 and d < 240:
                    self.gerak(0, 0, 0)

            cv2.imshow("frame", frame)
            if cv2.waitKey(100) & 0xFF == 27:
                break
            # cap.release()
        cv2.destroyAllWindows()

    def line_follow(self):
        global cam_bawah


if __name__ == "__main__":
    rospy.init_node("kendali")
    d = drone()
    print("start")
    d.takeoff()
    d.maju_depan()
    d.gawang()

