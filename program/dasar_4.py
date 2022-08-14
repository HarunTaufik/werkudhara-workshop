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
        self.image_sub = rospy.Subscriber('/drone/down_camera/image_raw', Image, self.cam_bawah_cb)
        self.image2_sub = rospy.Subscriber('/drone/front_camera/image_raw', Image, self.cam_depan_cb)
        self.sonar_sub = rospy.Subscriber('drone/sonar', Range, self.sonar_cb)
        self.alt_sub = rospy.Subscriber('/drone/gt_pose', Pose, self.alt_cb)
        self.compass_sub = rospy.Subscriber('/drone/imu', Imu, self.compass_cb)
        self.compass = 0
        self.alt = 0
        self.sonar = 0
        self.cam_depan = 0
        self.cam_bawah = 0
        
    
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

    def gerak(self, x, y):
        self.move_msg.linear.x = x
        self.move_msg.linear.y = y
        self.publish_vel(self.move_msg)
    
    def gerak_z(self, z):
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

    def cam_bawah_cb(self, data):
        global cam_bawah
        cam_bawah = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cam_bawah = cv2.add(cam_bawah, np.array([-50.0]))
        #cv2.imshow("camera bawah", cam_bawah)
        cv2.waitKey(1) & 0xFF

    def cam_depan_cb(self, data):
        global cam_depan
        cam_depan = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cam_depan = cv2.add(cam_depan, np.array([-50.0]))
        cv2.imshow("Image window", cam_depan)
        cv2.waitKey(1) & 0xFF

    def sonar_cb(self, msg):
        self.sonar = msg.range

    def alt_cb(self, data):
        self.alt = data.position.z	
    
    def compass_cb(self, data):
        self.compass = data.orientation.z
    
    def set_ang(self, tujuan):
        self.diam()
        rospy.sleep(0.3)
        while True:
            selisih = tujuan - self.compass
            if abs(selisih) < 0.02 :
                vyaw = 0
                self.diam()
                print("berhasil memposisikan")
                break
            else :
                vyaw = selisih*25
            self.yaw(vyaw)
        
    def set_alt(self, tujuan):
        self.diam()
        rospy.sleep(0.3)
        while True:
            selisih_alt = tujuan - self.alt
            c = self.compass
            vyaw = -c*50
            if abs(c) < 0.02 :
                vyaw = 0
            self.yaw(vyaw)
            print("alt :", self.alt)
            if abs(selisih_alt) < 0.03 :
                self.gerak_z(0)
                print("mencapai alt target")
                break
            else :
                self.gerak_z(selisih_alt)
    
    def deteksi_garis(self):
        global cam_bawah
        error = 0
        ang = 0
        area = 0
        mxarea = 0
        garis = 0
        frame = cam_bawah
        tinggi, lebar = frame.shape[:2]
        tengah = lebar/2
        mask = cv2.inRange(frame, (0,0,0), (45,45,45))
        contours, _ = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0 :
            area = [cv2.contourArea(contour) for contour in contours]
            area.append(0)
            idx = area.index(max(area))
            mxarea = cv2.contourArea(contours[idx])
            if mxarea < 60000 :
                box = cv2.minAreaRect(contours[idx])
                (x,y),(w,h),ang = box
                if ang < -45 :
                    ang = 90 + ang
                if w < h and ang > 0:
                    ang = (90-ang)*-1
                if w > h and ang < 0:
                    ang = 90 + ang
                tengah = lebar/2
                error = int(x - tengah) 
                ang = int(ang)
                box = cv2.boxPoints(box)
                box = np.int0(box)
                garis = 1
            else :
                garis = 0
            
        print(error)
        return error, ang, mxarea, garis, tengah
    
    def deteksi_gerbang(self, sumber):
        global cam_depan
        global cam_bawah
        selisih_x = 0
        selisih_y = 0
        mxarea = 0
        vertikal = 0
        kotak = 0
        if sumber == 1 :
            frame = cam_depan
        elif sumber == 2 :
            frame = cam_bawah
        tinggi, lebar = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (130, 80, 0), (195, 255, 255))
        mask = cv2.dilate(mask, None, iterations=1)
        mask = cv2.erode(mask, None, iterations=1)
        contours_g,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        for cnt in contours_g:
            luas = cv2.contourArea(cnt)
            area = [cv2.contourArea(c) for c in contours_g]
            idx = area.index(max(area))
            mxarea = int(max(area))
            if luas > 30:
                sudut = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
                if 3 < len(sudut) < 7 :
                    kotak = 1
                    (a,b,w,h) = cv2.boundingRect(sudut)
                    cv2.circle(frame,(a,b),3,(255,0,0),2)
                    rasio = w/float(h)
                    (x,y), radius = cv2.minEnclosingCircle(contours_g[idx])
                    selisih_x = (lebar/2) - int(x)
                    selisih_y = (tinggi/2) - int(y)
                    if rasio < 2.6 :
                        vertikal = 0
                    elif rasio > 2.5 and luas > 2400 :
                        roi = mask[(int(x-5)):(int(x+5)), (int(y-5)):(int(y+5))]
                        contours_roi,_ = cv2.findContours(roi, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                        vertikal = 1
                    else :
                        vertikal = 0
            else :
                if sumber == 2 :
                    kotak = 0
                    (x,y), radius = cv2.minEnclosingCircle(contours_g[idx])
                    selisih_x = (lebar/2) - int(x)
                    selisih_y = (tinggi/2) - int(y)
        return selisih_x, selisih_y, mxarea, vertikal, kotak
    
    def deteksi_gerbang_tech(self) :
        global cam_depan
        selisih = 0
        mxluas = 0
        frame = cam_depan
        tinggi, lebar = frame.shape[:2]
        mask = cv2.inRange(frame, (137, 140, 0), (162, 255, 255))
        contours_g,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if len(contours_g) > 0 :
            luas = [cv2.contourArea(c) for c in contours_g]
            idx = luas.index(max(luas))
            cv2.drawContours(frame, contours_g[idx], -1, (0,255,0), 1)
            (x,y),r = cv2.minEnclosingCircle(contours_g[idx])
            selisih = x - int(lebar/2)
            mxluas = int(max(luas))
        #cv2.imshow("frame", frame)
        return selisih, mxluas
    
    def deteksi_jarum(self):
        global cam_depan
        selisih = 0
        mxluas = 0
        ang = 0
        frame = cam_depan
        tinggi, lebar = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (5, 0, 0), (33, 255, 255))
        contours_g,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if len(contours_g) > 0 :
            luas = [cv2.contourArea(c) for c in contours_g]
            idx = luas.index(max(luas))
            cv2.drawContours(frame, contours_g[idx], -1, (0,255,0), 1)
            (x,y),r = cv2.minEnclosingCircle(contours_g[idx])
            selisih = x - int(lebar/2)
            mxluas = int(max(luas))
            box = cv2.minAreaRect(contours_g[idx])
            (x,y),(w,h),ang = box
            if ang < -45 :
                ang = 90 + ang
                if w < h and ang > 0:
                    ang = (90-ang)*-1
                if w > h and ang < 0:
                    ang = 90 + ang
            #cv2.putText(frame, str(ang), (10,10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1)
            print(str(ang))
        #cv2.imshow("frame_", frame)
        return selisih, mxluas, ang
            
    
    def deteksi_gerbangbawah(self):
        global cam_bawah
        selisih_x = 0
        selisih_y = 0
        area = 0
        vertikal = 0
        kotak = 0
        frame = cam_bawah
        tinggi, lebar = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (130, 80, 0), (195, 255, 255))
        mask = cv2.dilate(mask, None, iterations=1)
        mask = cv2.erode(mask, None, iterations=1)
        contours_g,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        for cnt in contours_g:
            luas = cv2.contourArea(cnt)
            area = [cv2.contourArea(c) for c in contours_g]
            idx = area.index(max(area))
            if luas > 5:
                sudut = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
                cv2.drawContours(frame, [sudut], -1, (0,255,0), 1)
                if 3 < len(sudut) < 7 :
                    kotak = 1
                    (a,b,w,h) = cv2.boundingRect(sudut)
                    rasio = w/float(h)
                    (x,y), radius = cv2.minEnclosingCircle(contours_g[idx])
                    cv2.circle(frame,(int(x),int(y)),3,(0,0,255),2)
                    selisih_x = (lebar/2) - int(x)
                    selisih_y = (tinggi/2) - int(y)
                else :
                    kotak = 0
                    (x,y), radius = cv2.minEnclosingCircle(contours_g[idx])
                    selisih_x = (lebar/2) - int(x)
                    selisih_y = (tinggi/2) - int(y)
        #cv2.imshow("frame",frame)
        return selisih_x, selisih_y, area, vertikal, kotak