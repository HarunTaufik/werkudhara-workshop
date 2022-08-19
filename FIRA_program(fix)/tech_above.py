from calendar import c
from select import select
import main_tech, rospy, cv2

rospy.init_node("Werkudhara")
drone = main_tech.drone()

def setup():
    while True:
        #drone.yaw(0)
        sonar = drone.sonar
        c = drone.compass
        a = drone.gate_detection()
        diff = a[0]
        mxluas = a[1]
        vyaw = -c*50
        if abs(c) < 0.02 :
            vyaw = 0
        drone.yaw(vyaw)

        if abs(diff) < 3 :
            vy = 0
        else :
            vy = diff/300

        if mxluas < 38000 :
            vx = 1
        elif mxluas >= 40000 :
            vx = 0
            break
        vz = 1.9 - drone.alt
        if abs(vz) < 0.05:
            vz = 0
        drone.velocity_z(vz)
        drone.velocity(vx,0)
    drone.stop()
    cv2.destroyAllWindows()
    rospy.sleep(0.3)

def gerbang():
    sonar = drone.sonar
    while True:
        j = drone.drone_chopper()
        c = drone.compass
        a = drone.gate_detection()
        geser = a[2]
        jarum_x = j[3]
        jarum_y = j[4]
        jarum_x = 0
        jarum_y = 0

        if sonar < 1.5 :
            drone.velocity(-0.5, 0)

        s_y = geser - 320
        sr_y = -s_y/80
        drone.velocity(0,sr_y)
        rospy.sleep(0.1)
        if 315 > geser > 325 :
            drone.velocity(0,0)

        vyaw = -c*50
        if abs(c) < 0.02 :
            vyaw = 0
        drone.yaw(vyaw)
        ang = j[2]
        vz = 2 - drone.alt
        if abs(vz) < 0.05:
            vz = 0
        drone.velocity_z(vz)
        if 41 > ang > 38 or -60 > ang > -79:
            print("GO!!!")
            drone.velocity(5,0)
            drone.yaw(0)
            rospy.sleep(2)
            drone.set_alt(4)
            drone.velocity(-2,0)
            rospy.sleep(2)
            drone.set_alt(2)
            drone.stop()
            rospy.sleep(0.3)
            break
        else :
            drone.stop()

def last_lap():
    sonar = drone.sonar
    while True:
        j = drone.drone_chopper()
        c = drone.compass
        a = drone.gate_detection()
        geser = a[2]

        s_y = geser - 320
        sr_y = -s_y/80
        drone.velocity(0,sr_y)
        rospy.sleep(0.1)
        if 315 > geser > 325 :
            drone.velocity(0,0)

        vyaw = -c*50
        if abs(c) < 0.02 :
            vyaw = 0
        drone.yaw(vyaw)
        ang = j[2]
        vz = 2 - drone.alt
        if abs(vz) < 0.05:
            vz = 0
        drone.velocity_z(vz)
        if 41 > ang > 38 or -60 > ang > -79 :
            print("GO!!!!")
            drone.velocity(5,0)
            drone.yaw(0)
            rospy.sleep(2)
            drone.set_alt(4)
            drone.velocity(-2,0)
            rospy.sleep(2)
            drone.set_alt(2)
            drone.stop()
            drone.velocity(-2,0)
            rospy.sleep(1.5)
            break
        else :
            drone.stop()
    

if __name__ == '__main__' :

    drone = main_tech.drone()
    drone.stop()
    drone.takeoff()
    drone.stop()
    drone.set_alt(1.8)
    drone.set_ang(0)
    setup()
    gerbang()
    gerbang()
    last_lap()
    drone.land()
