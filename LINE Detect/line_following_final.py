from __future__ import print_function
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import numpy as np 
import cv2
from math import degrees
import argparse
import imutils
#import zbar
from PIL import Image
from operator import itemgetter

def midcalculation(rho1, rho2, theta1, theta2):
    distance = abs(rho1 - rho2)
    rho = min(rho1, rho2) + distance/2
    theta = theta1
    return rho, theta


def horizontal_or_vertical(rho,theta):
    t = degrees(theta)
    if(t >= 70  and t <= 110):
        return "horizontal"
    elif(t <= 20 and t>=0):
        return "vertical"

def point_of_intersection(x1l1, x2l1, y1l1, y2l1, x1l2, x2l2, y1l2, y2l2):
    # defining lines in the form Ax + By = C
    # for line 1
    A1 = y2l1 - y1l1
    B1 = x1l1 - x2l1
    C1 = B1*y1l1 + A1*x1l1
    # for line 2
    A2 = y2l2 - y1l2
    B2 = x1l2 - x2l2
    C2 = B2*y1l2 + A2*x1l2
    determinant = A1*B2 - A2*B1
    if(determinant != 0 ):
        x = (B2*C1 - B1*C2)/determinant
        y = (A1*C2 - A2*C1)/determinant 
        return x,y

#counter = 0

vs = WebcamVideoStream(src=0).start()

#cap = cv2.VideoCapture(0)

# boundaries = [
#     ([76,160,93], [255, 255, 255])
# ]

Lower = (27, 76, 98)
Upper = (87, 255, 255)

i =0 
j = 0

vertical_array = []
horizontal_array = []
temp_vertical_array = []
final_horizontal_array = []

while (True):
    
    frame = vs.read()

    # if not grabbed:
    #     break

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, Lower, Upper)
    mask = cv2.erode(mask, None, iterations = 2)
    mask = cv2.dilate(mask, None, iterations = 2)

    edges = cv2.Canny(mask.copy(), 50, 150, apertureSize = 3)
    i, j = edges.shape
    print (i)
    print (j)
    lines = cv2.HoughLines(edges, 1, np.pi/180, 80)

    cx = i/2
    cy = j/2
    
    if lines is not None:
        #print len(lines)
        for line in range(len(lines)):
                for rho,theta in lines[line]:
                    #print (lines)
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))
                    cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)

                    # print ("lines")
                    # print (lines)
                    # print ("lines[line]")
                    # print (lines[line])
                    # print ("lines[line][0]")
                    # print (lines[line][0])
                    
                    if(horizontal_or_vertical(rho,theta) == "vertical"):
                        temp_vertical_array.append(lines[line][0])
                    elif(horizontal_or_vertical(rho,theta) == "horizontal"):
                        horizontal_array.append(lines[line][0])
                    
                    
                    # if(horizontal_or_vertical(rho,theta) == "vertical" and len(vertical_array) == 0):
                    #     #print (lines[0][line])
                    #     vertical_array.append(lines[line][0])

                    # if(len(vertical_array) == 1):
                    #     rhoo = vertical_array[0][0]
                    #     diff = abs(lines[line][0][0]) - abs(vertical_array[0][0])
                    #     if(abs(diff) <= 0.5):
                    #         print ("Line not to be considered")
                    #     elif (abs(diff) > 0.5 and abs(diff) <= 118):
                    #         vertical_array.append(lines[line][0])

        midvr = 0
        midvt = 0
        midhr = 0
        midht = 0

        va = sorted(temp_vertical_array, key = itemgetter(0))
        ha = sorted(horizontal_array, key = itemgetter(0))

        temp = 0

        for z in range(len(va)):
            if(z < len(va)-1 and (abs(abs(va[z+1][0])-abs(va[z][0])) <= 10 or (abs(abs(va[z+1][0])-abs(va[z][0])) >= 10 and abs(abs(va[z+1][0])-abs(va[z][0])) <= 200))):
                # value of something
                vertical_array.append(va[z])
                temp = z
                break

        for z in range(temp+1, len(va)):
            # print (va[z][0])
            # print (vertical_array[0][0])
            diff = abs(va[z][0]) - abs(vertical_array[0][0])
            if(abs(diff) <= 0.5):
                print ("Line not to be considered")
            elif (abs(diff) > 0.5 and abs(diff) <= 118):
                vertical_array.append(lines[line][0])

        if (len(vertical_array) <= 1):
            print ("Detect again")

        if (len(vertical_array) > 1):
            midvrho, midvtheta = midcalculation(vertical_array[0][0], vertical_array[1][0], vertical_array[0][1], vertical_array[1][1])
            midvr, midvt = midvrho, midvtheta  
            if(cx < min(abs(vertical_array[0][0]), abs(vertical_array[1][0]))):
                print ("Shift Right")

            if(cx > max(abs(vertical_array[0][0]), abs(vertical_array[1][0]))):
                print ("Shift Left")
         

        #distance_cp = (midvr - cx)#abs            
        #distance between centres                
        
        for l in range(len(ha)) :
            count = 0
            if(cy - ha[l][0] > 0  and count<2):
                final_horizontal_array.append(ha[l])
                count += 1
                
        if (len(final_horizontal_array) <= 1):
            print ("Keep moving")

        if (len(final_horizontal_array) > 1):
            midhrho, midhtheta = midcalculation(final_horizontal_array[0][0],final_horizontal_array[1][0], final_horizontal_array[0][1], final_horizontal_array[1][1])
            midhr, midht = midhrho, midhtheta

        amidv = np.cos(midvt)
        bmidv = np.sin(midvt)
        x0midv = amidv*midvr
        y0midv = bmidv*midvr
        x1l1 = int(x0midv + 1000*(-bmidv))
        y1l1 = int(y0midv + 1000*(amidv))
        x2l1 = int(x0midv - 1000*(-bmidv))
        y2l1 = int(y0midv - 1000*(amidv))

        amidh = np.cos(midht)
        bmidh = np.sin(midht)
        x0midh = amidh*midhr
        y0midh = bmidh*midhr
        x1l2 = int(x0midh + 1000*(-bmidh))
        y1l2 = int(y0midh + 1000*(amidh))
        x2l2 = int(x0midh - 1000*(-bmidh))
        y2l2 = int(y0midh - 1000*(amidh))

        if (point_of_intersection(x1l1, x2l1, y1l1, y2l1, x1l2, x2l2, y1l2, y2l2) != None):
            p_x, p_y = point_of_intersection(x1l1, x2l1, y1l1, y2l1, x1l2, x2l2, y1l2, y2l2)

            if(p_x == cx   &  p_y == cy):
                print ("Scan")

                    
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# vs.qrelease()
# cv2.destroyAllWindows() 
