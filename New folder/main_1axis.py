from queue import deque
import numpy as np
import cv2
from PID import PIDController
import cflib
import time
from cflib.crtp.radiodriver import RadioDriver
from cflib.crazyflie import Crazyflie

# bufferSize = 16
# greenLower = (29, 86, 6)
# greenUpper = (64, 255, 255)
# pts = deque(maxlen = bufferSize)

# greenLower = (20, 139, 21)
# greenUpper = (35, 255, 255)
# greenLower = (20, 117, 166)
# greenUpper = (99, 189, 255)

# greenLower = (29, 88, 161)
# greenUpper = (47, 255, 255)
greenLower = (27, 76, 98)
greenUpper = (87, 255, 255)
camera = cv2.VideoCapture(2)
camera.set(cv2.CAP_PROP_FPS , 60)

pid = PIDController(proportional = 25.0, derivative_time = 1, integral_time=100)
pid.vmin, pid.vmax = -10000, 10000
pid.setpoint = 600
baseThrust = 43000
rollTrim = 1
pitchTrim = 1.5
yawTrim = 0

cflib.crtp.init_drivers(True)
r = RadioDriver()
# cf_urls = r.scan_interface(None)
# print(cf_urls)
# uri = cf_urls[0][0]
cf = Crazyflie()
cf.open_link("radio://0/90/250K")
cf.commander.send_setpoint(0,0,0,0)
pidout = 0

while True:
    (grabbed, frame) = camera.read()

    if not grabbed:
        break

    frame = cv2.resize(frame, (600, 600))
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # print (600 - center[0])
        # print(center)

        pidout = int(pid.compute_output(600 - center[1]))
        pidout += baseThrust
        print ("Height = ", 600 - center[1], "px, ", "Thrust = ", pidout)
        cf.commander.send_setpoint(rollTrim, pitchTrim, yawTrim, pidout)

        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

    else:
        print("Out of bounds.")
        cf.commander.send_setpoint(rollTrim, pitchTrim, yawTrim, 40000)
        pass
    
    # pts.appendleft(center)
    
    # for i in range(1, len(pts)):
    #     if pts[i - 1] is None or pts[i] is None:
    #         continue
    #     thickness = int(np.sqrt(bufferSize / float(i + 1)) * 2.5)
    #     cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        cf.commander.send_setpoint(0,0,0,0)
        break

cf.close_link()
camera.release()
cv2.destroyAllWindows()