import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
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
        if luas > 30:
            sudut = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            if 3 < len(sudut) < 7 :
                (a,b,w,h) = cv2.boundingRect(sudut)
                #cv2.circle(frame,(a,b),3,(255,0,0),2)
                rasio = w/float(h)
                if rasio < 2.6 :
                    (x,y), radius = cv2.minEnclosingCircle(contours_g[idx])
                elif rasio > 2.5 and luas > 2400 :
                    (x,y), radius = cv2.minEnclosingCircle(contours_g[idx])
                    tespoint = hsv[int(x),int(y)]
                    nilai_hue = tespoint[0]
                    cv2.circle(frame,(int(x),int(y)),3,(255,0,0),2)
                    if 129 < nilai_hue < 195 :
                        print("vertikal")
                    else:
                        print("A")
                
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break
cap.release()
cv2.destroyAllWindows()