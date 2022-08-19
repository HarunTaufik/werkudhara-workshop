#! /usr/bin/env python3

import rospy
import Dasar_2
from rospy import sleep

max_spd = 1
vx=vy=vz=vyaw = 0
mulai = 0

 
if __name__ == '__main__' :
    rospy.init_node("Werkudhara_node")
    print("max_spd :", max_spd, end='\n')
    drone = Dasar_2.drone()
    drone.diam()
    drone.takeoff_time = rospy.get_rostime()
    drone.takeoff()
    drone.diam()
    drone.set_alt(1.3)
    sleep(0.3)
    drone.yaw(-0.7)
    sleep(0.6)
    drone.yaw(0)
    drone.gerak(1,0)
    sleep(2.5)
    drone.yaw(1)
    sleep(0.3)
    drone.yaw(0.5)
    sleep(1.5)
    drone.set_alt(1)
    sleep(0.3)
    drone.gerak(1,0)
    sleep(0.7)
    drone.yaw(-2)
    sleep(0)
    drone.gerak(1,0)
    sleep(1)

    '''print("Step 1")
    drone.gerak(1, 0)
    drone.gerak_z(0)
    sleep(1)

    print("Step 2")
    drone.gerak(1, 0.5)
    drone.gerak_z(0)
    sleep(0.5)
    
    print("Step 3")
    drone.gerak(1, 0)
    drone.gerak_z(1)
    sleep(1)

    print("Step 4")
    drone.gerak(1, 0.2)
    drone.gerak_z(0)
    sleep(1)
         
    print("Step 5")
    drone.gerak(0, 0)
    drone.gerak_z(0)
    sleep(5)
    
    print('')'''
    drone.diam()
    print("Home")
    drone.land()
    flight_duration =  rospy.get_rostime() - drone.takeoff_time
    #print("vx :",format(vx, '.3f'),"|| vy :",format(vy, '.3f'), "|| vz :",format(vz, '.3f')," ", error, " dur : ", format(flight_duration.to_sec(), '.3f') , "          ", end='\r')
    print("")
    print("")
       