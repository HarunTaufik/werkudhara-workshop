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
        vyaw = 5
        self.yaw(vyaw)

    def land(self):
        self.land_pub.publish(self.land_msg)
        print("Landing...")
        sleep(1)

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
        lap = 0
        while lap < 4:
            # _,frame = cap.read()
            frame = front_cam
            below_frame = below_cam
            # crop = frame[327:445, 137:530]
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv_below = cv2.cvtColor(below_frame, cv2.COLOR_BGR2HSV)
            hsv_a = np.array([180, 153, 77])
            hsv_b = np.array([0, 0, 0])
            orange_a = np.array([0, 135, 0])
            orange_b = np.array([255, 255, 255])
            mask = cv2.inRange(hsv, hsv_b, hsv_a)
            radian = cv2.inRange(hsv, orange_a, orange_b)
            gatebaw = cv2.inRange(hsv_below, (130, 80, 0), (195, 255, 255))
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

            conturs, _ = cv2.findContours(radian, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for cnts in conturs:
                radian_cnt = [cv2.contourArea(cnts) for cnts in conturs]
                radian_cnt.append(0)
                indx = radian_cnt.index(max(radian_cnt))
                cv2.drawContours(frame, conturs, -1, (0, 255, 0), 1)
                (c, d), radius = cv2.minEnclosingCircle(conturs[indx])
                cv2.circle(frame, (int(c), int(d)), 1, (0, 255, 0), 5)
                # bilek
                # diperjelas makin gajelas
                while snr < 1:
                    print("repositioning sonar -x")
                    vx = -0.5
                    self.move(vx, 0, 0)
                    sleep(1)
                    vx = 0
                    self.move(vx, 0, 0)
                while 3 > snr > 1.5:
                    print("repositioning sonar x")
                    vx = 0.5
                    self.move(vx, 0, 0)
                    sleep(1)
                    vx = 0
                    self.move(vx, 0, 0)
                while alt < 1.7:
                    print("repositioning alt -z")
                    vz = 0.1
                    self.move(0, 0, vz)
                    sleep(1)
                    vz = 0
                    self.move(0, 0, vz)
                while 4 > alt > 2:
                    print("repositioning alt z")
                    vz = -0.2
                    self.move(0, 0, vz)
                    sleep(1)
                    vz = 0
                    self.move(0, 0, vz)
                sleep(0.5)
                if lap < 3:
                    if c > 320 and d < 175:
                        vx = 0
                        self.move(vx, 0, 0)
                    elif c > 320 and d > 175:
                        vx = 0
                        self.move(vx, 0, 0)
                    elif c < 320 and d > 175:
                        vx = 0
                        self.move(vx, 0, 0)
                    elif c < 320 and d < 175:
                        print("Move!")
                        sleep(1)
                        vx = 3
                        self.move(vx, 0, 0)
                        sleep(2)
                        vx = 0
                        self.move(vx, 0, 0)
                        sleep(0.5)
                        lap = lap + 1
                if lap > 0 and lap < 3:
                    print("passed")
                    vz = -1
                    self.move(0, 0, vz)
                    sleep(1)
                    vz = 0
                    self.move(0, 0, vz)
                    sleep(2)
                    vx = -7
                    self.move(vx, 0, 0)
                    sleep(2)
                    vx = 0
                    self.move(vx, 0, 0)
                    sleep(1)
                    vz = 1
                    self.move(0, 0, vz)
                    sleep(1)
                    vz = 0
                    self.move(0, 0, vz)
                    print("lap : ", lap)
                    sleep(2)
                if lap == 3:
                    vx = 1
                    self.move(vx, 0, 0)
                    sleep(3)
                    # contours_baw, _ = cv2.findContours(gatebaw, 1, cv2.CHAIN_APPROX_NONE)

                    self.land()
                # atumalaca

                # if len(contours_baw) > 0:
                # landpad = landpad + 1
                # h_area = [cv2.contourArea(contour) for contour in contours_baw]
                # h_area.append(0)
                # idex = h_area.index(max(h_area))
                # cv2.drawContours(below_frame, contours_baw[idex], -1, (0,255,0), 2)

            cv2.imshow("frame", frame)
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
    d.forward()
    d.gate()
