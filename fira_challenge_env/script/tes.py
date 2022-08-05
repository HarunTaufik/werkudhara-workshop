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
        cv2.waitKey(1) & 0xFF

    def gawang(self):
        global cam_depan

    def line_follow(self):
        global cam_bawah
        while True:
            # _, frame = cap.read()
            frame = cam_bawah
            frame2 = cam_depan
            mask = cv2.inRange(frame, (0, 0, 0), (50, 50, 50))
            contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)
            if len(contours) > 0:
                area = [cv2.contourArea(contour) for contour in contours]
                area.append(0)
                idx = area.index(max(area))
                # cv2.drawContours(frame, contours[idx], -1, (0, 255, 0), 2)
                blackbox = cv2.minAreaRect(contours[idx])
                (x_min, y_min), (w_min, h_min), ang = blackbox
                if ang < -45:
                    ang = 90 + ang
                if w_min < h_min and ang > 0:
                    ang = (90 - ang) * -1
                if w_min > h_min and ang < 0:
                    ang = 90 + ang

                setpoint = 320
                error = int(x_min - setpoint)
                ang = int(ang)
                box = cv2.boxPoints(blackbox)
                box = np.int0(box)
                # cv2.drawContours(frame, [box], 0, (0, 0, 255), 3)
                # cv2.putText(frame,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                # cv2.putText(frame,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                # cv2.line(frame, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)
                print(ang, " | ", error)
                if ang > 10 and ang < 50:
                    vyaw = -1
                elif ang < -10 and ang > -50:
                    vyaw = 1
                else:
                    vyaw = 0
                if error > 10:
                    vy = -0.15
                elif error < -10:
                    vy = 0.15
                elif error > 40:
                    vy = -0.3
                elif error < -40:
                    vy = 0.3
                else:
                    vy = 0
                if ang < -25 and ang > -50:
                    vx = 0.05
                elif ang > 25 and ang < 50:
                    vx = 0.05
                else:
                    vx = 0.3
                # self.gerak(vx, vy, 0)
                # self.yaw(vyaw)
            cv2.imshow("bawah", frame)
            cv2.imshow("depan", frame2)
            if cv2.waitKey(1) & 0xFF == 27:
                break
        # cap.release()
        cv2.destroyAllWindows()
        # print("The video was successfully saved")


if __name__ == "__main__":
    rospy.init_node("kendali")
    d = drone()
    print("start")
    d.takeoff()
    d.line_follow()
