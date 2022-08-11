#! /usr/bin/env python3

from curses.ascii import alt
from cv2 import boundingRect
import rospy
from time import sleep
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Range
import cv2
import numpy as np

class drone(object):

	global alt

	def __init__(self):
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

	def move(self, x, y):
		self.move_msg.linear.x = x
		self.move_msg.linear.y = y
		self.publish_vel(self.move_msg)
	
	def move_z(self, z):
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
		#cv2.imshow("kamera bawah", cam_bawah)
		cv2.waitKey(1) & 0xFF

	def cam_depan_cb(self, data):
		global cam_depan
		cam_depan = self.bridge.imgmsg_to_cv2(data, "bgr8")
		cam_depan = cv2.add(cam_depan, np.array([-50.0]))
		#cv2.imshow("kamera depan", cam_depan)
		cv2.waitKey(1) & 0xFF
	
	def sonar_cb(self, msg):
		global snr
		snr = msg.range

	def alt_cb(self, data):
		global alt
		alt = data.position.z
	
	def cek_camB(self):
		global cam_bawah
		while True:
			frame = cam_bawah
			if cv2.waitKey(1) & 0xFF == 27:
				break
			cv2.imshow("camB", frame)
		cv2.destroyAllWindows()

	def misi(self):
		global cam_bawah
		global cam_depan
		vx = 0
		vy = 0
		vz = 0
		dep_en = 1
		landpad = 0
		while landpad < 2:
			#_, frame = cap.read()
			frame_baw = cam_bawah
			frame_dep = cam_depan
			height, widht = frame_baw.shape[:2]
			hsv = cv2.cvtColor(frame_dep, cv2.COLOR_BGR2HSV)
			hsv_baw = cv2.cvtColor(frame_baw, cv2.COLOR_BGR2HSV)
			mask_gateV = cv2.inRange(hsv_baw, (130, 80, 0), (195, 255, 255))
			mask_baw = cv2.inRange(frame_baw, (0,0,0), (45,45,45))
			mask_land = cv2.inRange(hsv_baw, (111, 50, 0), (128, 255, 74))
			contours_baw, _ = cv2.findContours(mask_baw, 1, cv2.CHAIN_APPROX_NONE)
			contours_gatebaw, _ = cv2.findContours(mask_gateV, 1, cv2.CHAIN_APPROX_NONE)
			if len(contours_baw) > 0:	 
				area = [cv2.contourArea(contour) for contour in contours_baw]
				area.append(0)
				idx = area.index(max(area))
				cv2.drawContours(frame_baw, contours_baw[idx], -1, (0,255,0), 2)
				blackbox = cv2.minAreaRect(contours_baw[idx])
				(x_min, y_min), (w_min, h_min), ang = blackbox
				if ang < -45 :
					ang = 90 + ang
				if w_min < h_min and ang > 0:	  
					ang = (90-ang)*-1
				if w_min > h_min and ang < 0:
					ang = 90 + ang
				setpoint = widht/2
				error = int(x_min - setpoint) 
				ang = int(ang)	 
				box = cv2.boxPoints(blackbox)
				box = np.int0(box)
				cv2.drawContours(frame_baw,[box],0,(0,0,255),3)	 
				#cv2.putText(frame,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
				#cv2.putText(frame,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
				cv2.line(frame_baw, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
				#print(ang," | ",error)
				vyaw = -(ang*(2))
				if abs(ang) < 5 or abs(ang) > 60 :
					vyaw = 0
				vy = -(error/280)
				if abs(error) < 3:
					vy = 0
				if abs(ang)>10:
					vx = 0.8 - (vx*ang/35)
				elif abs(ang)<11:
					vx = 0.8

				if landpad == 1:
					if len(contours_gatebaw) > 0:	 
						gateb = [cv2.contourArea(cont) for cont in contours_gatebaw]
						gateb.append(0)
						idx = gateb.index(max(gateb))
						if int(max(gateb)) > 10000:
							vx = 0.3
							approxb = cv2.approxPolyDP(contours_gatebaw[idx], 0.02*cv2.arcLength(contours_gatebaw[idx], True), True)
							if 3 < len(approxb) < 5 :
								(a,b,w,h) = cv2.boundingRect(approxb)
								r = w/h
								if (not int(widht*0.25) < int(a) < int(widht*0.75)) and (r < 1) :
									print("turun")
									self.diam()
									rospy.sleep(1.2)
									self.move(-0.08,0)
									#self.yaw(0)
									self.move_z(-1.6)
									rospy.sleep(0.7)
									self.diam()
								elif r > 1.5 :
									print("aaaaaaa")
									self.move(-0.08, 0.05)
					elif int(max(area)) > 100000 :
						landpad = 2
				
			else :
				vx = 0.1
				vyaw = 0
				print("-")
				
			self.move(vx,vy)
			self.yaw(vyaw)
			#print("area = ", area)
			
			height2, widht2 = frame_dep.shape[:2]
			hsv_b = np.array([130, 80, 0])
			hsv_a = np.array([195, 255, 255])
			mask_dep = cv2.inRange(hsv, hsv_b, hsv_a)
			mask_dep = cv2.dilate(mask_dep, None, iterations=1)
			mask_dep = cv2.erode(mask_dep, None, iterations=1)
			contours_dep,_ = cv2.findContours(mask_dep, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
			if dep_en == 1 :
				for cnt in contours_dep:
					luas = [cv2.contourArea(contour)for contour in contours_dep]
					luas.append(0)
					idx2 = luas.index(max(luas))
					area = cv2.contourArea(cnt)
					approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
					x = approx.ravel()[0]
					y = approx.ravel()[1]
					if area > 20:
						cv2.drawContours(frame_dep, [approx], 0, (0, 255, 255), 1)
						if 3 < len(approx) < 7:
							(a,b,w,h) = cv2.boundingRect(approx)
							rasio = w/float(h)
							selisih_x = (widht2/2) - x
							if rasio < 2.6:
								roi = mask_dep[(int(b-5)):(int(b+5)), (int(a-5)):(int(a+5))]
								contours_roi,_ = cv2.findContours(roi, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
								if len(contours_roi) > 0:
									for mgt in contours_roi:
										luas_mgt = cv2.contourArea(mgt)
										if luas_mgt > 90 :
											vz = 0
											print("bukan gate")
								else:
									(c,d), radius = cv2.minEnclosingCircle(contours_dep[idx2])
									cv2.circle(frame_dep, (int(c), int(d)), 1, (0, 150, 0), 3)
									cv2.putText(frame_dep, "Gerbang", (int(c + 3), int(d)), cv2.FONT_HERSHEY_COMPLEX, (0.5), (0, 150, 0))
									selisih = (height2/2) - int(d)
									if abs(selisih) < 3 and ang < 5:
										vz = 0
										vx = 1.2
										print(">>>")
									elif abs(selisih) < 3:
										vz = 0
									elif abs(selisih) > 130 :
										vx = 0.03
									elif abs(selisih) > 200 :
										vx = 0
										vyaw  = 0
									else :
										vz = selisih/175
							if rasio > 2.5:
								#print("gate vertikal")
								if area > 2400 :
									roi = mask_dep[(int(b-15)):(int(b+15)), (int(a-15)):(int(a+15))]
									contours_roi,_ = cv2.findContours(roi, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
									if len(contours_roi) > 0:
										for mgt in contours_roi:
											luas_mgt = cv2.contourArea(mgt)
											if luas_mgt > 0 :
												print("Gerbang Vertikal")
												self.diam()
												self.move_z(1.8)
												rospy.sleep(0.3)
												self.diam()
												rospy.sleep(0.5)
												dep_en = 0
												landpad = 1
												#self.land()
												#break
						else:
							(c,d), radius = cv2.minEnclosingCircle(contours_dep[idx2])
							#cv2.putText(frame_dep, "target", (int(a), int(b)), cv2.FONT_HERSHEY_COMPLEX, (0.5), (100, 255, 255))
							selisih = (height2/2) - int(d)
							vz = selisih/300
							if abs(selisih) < 10 :
								vz = 0
								
							#print("ungu")

					self.move_z(vz)
					#print(vz)
			#print(vx, " | ", vy, " | ", vz)
			print("alt = ", alt)
			if cv2.waitKey(1) & 0xFF == 27:
				break
		#cap.release()
			#cv2.imshow("bawah", frame_baw)
			#cv2.imshow("depan", frame_dep)
		cv2.destroyAllWindows()

if __name__ == "__main__":
	rospy.init_node("drone_controll")
	d = drone()
	d.diam()
	d.takeoff()
	d.diam()
	d.yaw(-1)
	d.move_z(0.3)
	rospy.sleep(0.4)
	d.diam()
	rospy.sleep(0.3)
	d.misi()
	d.diam()
	print("Home")
	d.move_z(-0.7)
	rospy.sleep(1.5)
	d.land()