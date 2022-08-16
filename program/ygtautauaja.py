from calendar import c
from select import select
import dasar_4, rospy, cv2

rospy.init_node("Werkudhara")
drone = dasar_4.drone()

def persiapan():
    while True:
        #drone.yaw(0)
        sonar = drone.sonar
        c = drone.compass
        a = drone.deteksi_gerbang_tech()
        selisih = a[0]
        mxluas = a[1]
        vyaw = -c*50
        if abs(c) < 0.02 :
            vyaw = 0
        print(mxluas,' | ',c)
        drone.yaw(vyaw)

        if abs(selisih) < 3 :
            vy = 0
        else :
            vy = selisih/300

        if mxluas < 38000 :
            vx = 1
        elif mxluas >= 40000 :
            vx = 0
            print("siap")
            break
        vz = 1.9 - drone.alt
        if abs(vz) < 0.05:
            vz = 0
        drone.gerak_z(vz)
        drone.gerak(vx,0)
    drone.diam()
    cv2.destroyAllWindows()
    rospy.sleep(0.3)

def gerbang():
    sonar = drone.sonar
    while True:
        j = drone.deteksi_jarum()
        c = drone.compass
        a = drone.deteksi_gerbang_tech()
        geser = a[2]
        jarum_x = j[3]
        jarum_y = j[4]
        jarum_x = 0
        jarum_y = 0

        if sonar < 1.5 :
            drone.gerak(-0.5, 0)

        s_y = geser - 320
        sr_y = -s_y/80
        drone.gerak(0,sr_y)
        rospy.sleep(0.1)
        if 315 > geser > 325 :
            drone.gerak(0,0)

        vyaw = -c*50
        if abs(c) < 0.02 :
            vyaw = 0
        drone.yaw(vyaw)
        ang = j[2]
        print(ang)
        vz = 2 - drone.alt
        if abs(vz) < 0.05:
            vz = 0
        drone.gerak_z(vz)
        if 41 > ang > 38 or -60 > ang > -79:
            drone.gerak(5,0)
            drone.yaw(0)
            rospy.sleep(2)
            drone.set_alt(4)
            drone.gerak(-2,0)
            rospy.sleep(2)
            drone.set_alt(2)
            drone.diam()
            rospy.sleep(0.3)
            break
        else :
            drone.diam()

def lap_terakhir():
    sonar = drone.sonar
    while True:
        j = drone.deteksi_jarum()
        c = drone.compass
        a = drone.deteksi_gerbang_tech()
        geser = a[2]
        jarum_x = j[3]
        jarum_y = j[4]
        jarum_x = 0
        jarum_y = 0

        s_y = geser - 320
        sr_y = -s_y/80
        drone.gerak(0,sr_y)
        rospy.sleep(0.1)
        if 315 > geser > 325 :
            drone.gerak(0,0)

        vyaw = -c*50
        if abs(c) < 0.02 :
            vyaw = 0
        drone.yaw(vyaw)
        ang = j[2]
        print(ang)
        vz = 2 - drone.alt
        if abs(vz) < 0.05:
            vz = 0
        drone.gerak_z(vz)
        if 41 > ang > 38 or -60 > ang > -79 :
            drone.gerak(5,0)
            drone.yaw(0)
            rospy.sleep(3)
            break
        else :
            drone.diam()
    

if __name__ == '__main__' :

    drone = dasar_4.drone()
    drone.diam()
    drone.takeoff()
    drone.diam()
    drone.set_alt(1.8)
    drone.set_ang(0)
    persiapan()
    gerbang()
    gerbang()
    lap_terakhir()
    drone.land()
