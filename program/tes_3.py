#! /usr/bin/env python3

import rospy
import dasar
from rospy import sleep

max_spd = 1.2
vx=vy=vz=vyaw = 0
mulai = 0

if __name__ == '__main__' :
    rospy.init_node("Werkudhara_node")
    drone = dasar.drone()
    drone.takeoff()
    drone.diam()
    #drone.set_alt(0.7)
    while mulai == 0:
        gerbang = drone.deteksi_gerbang(1)
        if abs(gerbang[0]) > 2 :
            drone.yaw((gerbang[0])/80)
            drone.gerak_z((gerbang[1]/200))
        elif abs(gerbang[0]) < 3 :
            drone.yaw(0)
            print("dapat")
            drone.diam()
            drone.gerak(0.1,0)
            break
    vertikal = gerbang[3]
    boleh_land = 0
    while True :
        if vertikal == 0:
            garis = drone.deteksi_garis()
            gerbang = drone.deteksi_gerbang(1)
            error = 0
            error = garis[0]
            ang = garis[1]
            area = garis[2]
            grs = garis[3]
            selisih_z = gerbang[1]
            vertikal = gerbang[3]
            
            #mengikuti garis
            vyaw = -(ang*(2))
            if abs(ang) < 5 or abs(ang) > 60 :
                vyaw = 0
            vy = -(error/280)
            if abs(error) < 3:
                vy = 0
            if abs(ang)>10:
                vx = max_spd*0.7 - (vx*ang/35)
            elif abs(ang) < 11 and abs(error) < 5:
                vx = max_spd*0.7
            
            #penyesuaian altitude untuk gerbang secara horizontal + adaptasi kecepatan dan yaw rate
            if vertikal == 0 :
                if abs(selisih_z) < 3 and 0 < ang < 5 and 0 < abs(error) < 4:
                    vz = 0
                    vx = max_spd
                    print(">>>")
                elif abs(selisih_z) < 3:
                    vz = 0
                elif abs(selisih_z) > 130 :
                    vx = max_spd*0.05
                elif abs(selisih_z) > 200 :
                    vx = 0
                else :
                    vz = selisih_z/175
            drone.gerak(vx,vy)
            drone.gerak_z(vz)
            drone.yaw(vyaw)
            
            #melewati gerbang secara vertikal
            if vertikal == 1 :
                drone.set_alt(3)
                drone.gerak(0.2,0)
                sleep(1)

            if boleh_land > 0 :
                if area > 65000 :
                    break
                    

        if vertikal == 1 :
            #drone.gerak(0.1, 0)
            gerbang2 = drone.deteksi_gerbangbawah()
            selisih2_x = gerbang2[0]
            selisih2_y = gerbang2[1]
            kotak = gerbang2[4]
            #print("V")
            #print("v")
            if kotak == 1:
                print("aptx4869")
                vx = selisih2_y/400
                if abs(selisih2_x) < 4 :
                    vx = 0
                vy = selisih2_x/400
                if abs(selisih2_y) < 4 :
                    vy = 0
                if abs(selisih2_x) < 4 and abs(selisih2_y) < 4 :
                    drone.set_alt(1)
                    boleh_land = 1
                    vertikal = 0
                    
            else:
                print("bbbbb")
                vx = selisih2_y/300
                if abs(selisih2_y) < 4 :
                    vx = 0
                vy = selisih2_x/300
                if abs(selisih2_x) < 4 :
                    vy = 0
                if abs(selisih2_x) < 4 and abs(selisih2_y) < 4 :
                    drone.gerak_z(0.01)
                    
            print("x :",vx,"y :",vy)
            drone.gerak(vx,vy)
        
        #
    print("Home")
    drone.land()