#! /usr/bin/env python3

import rospy
import Dasar
from rospy import sleep

max_spd = 1.2
vx=vy=vz=vyaw = 0
mulai = 0

 
if __name__ == '__main__' :
    rospy.init_node("Werkudhara_node")
    drone = Dasar.drone()
    drone.takeoff()
    drone.diam()
    drone.set_alt(0.8)
    
    #drone.set_alt(0.7)
    while mulai == 0:
        gerbang = drone.deteksi_gerbang(1)
        if abs(gerbang[0]) > 2 :
            drone.yaw((gerbang[0])/100)
            #drone.gerak_z((gerbang[1]/300))
        elif abs(gerbang[0]) < 3 :
            drone.yaw(0)
            print("dapat",end='\n')
            drone.diam()
            drone.gerak(0.05,0)
            break
    jalur = 0
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
            luasH = garis[4]
            selisih_z = gerbang[1]
            vertikal = gerbang[3]
            
            #mengikuti garis
            vyaw = -(ang*(2))
            #vy = -(error/325)
            if grs > 0:
                if abs(error) < 3 :
                    vy = 0
                    vx = max_spd
                elif abs(error) < 10 :
                    vy = -(error/600)
                    vx = (max_spd*0.75)-(max_spd*abs(error)/1000)
                elif abs(error) < 30 :
                    vy = -(error/500)
                    vx = (max_spd*0.75)-(max_spd*abs(error)/800)
                elif abs(error) < 50 :
                    vy = -(error/450)
                    vx = (max_spd*0.75)-(max_spd*abs(error)/700)
                elif abs(error) < 80 :
                    vy = -(error/410)
                    vx = (max_spd*0.75)-(max_spd*abs(error)/600)
                elif abs(error) < 120 :
                    vy = -(error/375)
                    vx = (max_spd*0.75)-(max_spd*abs(error)/500)
                else :
                    vy = -(error/325)
                    vx = (max_spd*0.75)-(max_spd*abs(error)/350)
            else:
                vx = 0.3
            
            if abs(ang) <= 3:
                vyaw = 0
                vx = vx
            elif abs(ang) < 10:
                vyaw = -(ang*(2))
                vx = vx-(vx*abs(ang)/600)
            elif abs(ang) < 25:
                vyaw = -(ang*(2))
                vx = vx-(vx*abs(ang)/450)
            elif abs(ang) < 50:
                vyaw = -(ang*(2))
                vx = vx-(vx*abs(ang)/350)
            elif abs(ang) <= 70:
                vyaw = -(ang*(2))
                vx = vx-(vx*abs(ang)/250)
                vy = 0
            else:
                vyaw = 0
                vx = vx*0.8

            #antisipasi drone mundur
            if vx < 0.3 : 
                vx = 0.3
            
            #penyesuaian altitude untuk gerbang secara horizontal + adaptasi kecepatan dan yaw rate
            if vertikal == 0 :
                if abs(selisih_z) < 3 :
                    vz = 0
                    vx = vx
                    #print(">>>")
                elif abs(selisih_z) > 130 :
                    vx = vx*0.6
                elif abs(selisih_z) > 200 :
                    vx = vx*0.1
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
                #rint("kotak")
                drone.gerak_z(0.05)
                vx = selisih2_y/400
                if abs(selisih2_x) < 6 :
                    vx = 0
                vy = selisih2_x/400
                if abs(selisih2_y) < 6 :
                    vy = 0
                if abs(selisih2_x) < 6 and abs(selisih2_y) < 6 :
                    drone.set_alt(0.7)
                    boleh_land = 1
                    vertikal = 0
                    
            else:
                #print("ungu")
                vx = selisih2_y/300
                if abs(selisih2_y) < 8 :
                    vx = 0
                vy = selisih2_x/300
                if abs(selisih2_x) < 8 :
                    vy = 0
                if abs(selisih2_x) < 11 and abs(selisih2_y) < 11 :
                    drone.gerak_z(0.05)
                    
            drone.gerak(vx,vy)
        print("vx :",format(vx, '.3f'),"|| vy :",format(vy, '.3f'), "|| vz :",format(vz, '.3f'),end='\r')
       
    
    print('')
    drone.diam()
    print("Home")
    drone.land()