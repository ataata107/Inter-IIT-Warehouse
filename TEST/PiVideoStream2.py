# import the necessary packages
# from __future__ import print_function            # This needs to be on the first line
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import numpy as np
from pyquaternion import Quaternion
from PiVideoStream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import cv2
import sys

from datetime import datetime, timedelta
from PID import PIDController



#--------------------------SET UP VIDEO THREAD ----------------------------------

# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter
print('[INFO] sampling THREADED frames from `picamera` module...')
vs = PiVideoStream().start()






#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE TRACKING  ---------------------
def tracking (vstate):
    print vstate
    
    #The vehicle process images and maintains all data ready to fly in the following state.
    #However, it does not send attitude messages as the vehicle is still under manual control.

    red1Good = red2Good = False # Set True when returned target offset is reliable.
    bearing = offset = 0
    target = None # Initialise tuple returned from video stream

    # Initialise the FPS counter.
    #fps = FPS().start()

    while vstate == "tracking":
     
        # grab the frame from the threaded video stream and return left line offset
        # We do this to know if we have a 'lock' (goodTarget) as we come off of manual control.
        target = vs.readfollow()
        bearing = target[0]
        red1Good = target[1]
        offset = target[2]
        red2Good = target[3]

        # Print location information for `vehicle` in all frames (default printer)
        #print "Global Location: %s" % vehicle.location.global_frame
        #print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
        #print "Local Location: %s" % vehicle.location.local_frame    #NEDprint "Local Location: %s" % vehicle.location.local_frame

        # print tof_sensor.get_distance()
        
        # update the FPS counter
        #fps.update()

        # Check if operator has transferred to autopilot using TX switch.

    # stop the timer and display FPS information
    #fps.stop()
    #print('Elasped time in tracking state: {:.2f}'.format(fps.elapsed()))
    #print('Approx. FPS: {:.2f}'.format(fps.fps()))

    # time.sleep(1)

    return vstate



#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE FOLLOWING---------------------
def following (vstate):
    print vstate

    #The vehicle process images and uses all data to fly in the following state.
    # It sends attitude messages until manual control is resumed.    

    maxPitch = -3 # Maximum pitch for target at 0 bearing.
    multRoll= 8 # The roll angle if offset is at edge of the near field of view.
    #multYaw = 0 # A multiplier for the rate of Yaw.  
    yaw = roll = 0
    target = None # Initialise tuple returned from video stream
  
#    altitude = vehicle.location.global_relative_frame.alt
#    print altitude

    # Initialise the FPS counter.
    #fps = FPS().start()


    while vstate =="following":
     
        # grab the frame from the threaded video stream and return left line offset
        # We do this to know if we have a 'lock' (goodTarget) as we come off of manual control.
        target = vs.readfollow()
        bearing = target[0] # Returned in degrees, +ve clockwise
        red1Good = target[1]
        offset = target[2]    # Returned as a fraction of image width.  -1 extreme left.
        red2Good = target[3]
        
        # update the FPS counter
        #fps.update()

        # Get the altitude information.
        # tofHeight =  tof_sensor.get_distance()
#       tofHeight = vehicle.location.global_relative_frame.alt
#        print tofHeight
        
        # print "Measured distance is : %d mm" % tofHeight
        
    


        # Check if operator has transferred to autopilot using TX switch.

                



    return vstate


#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE LOST---------------------
def lost(vstate):
    print vstate
 
    #The vehicle process images and uses all data to fly in the lost state.
    # The vehicle rotates in one spot until a lock is established.
    # It sends attitude messages until manual control is resumed.

    maxPitch = 0
    multRoll= 0
    multYaw = 0
    yaw = roll = 0
    
    target = None # Initialise tuple returned from video stream
    
    found = False

    # Initialise the FPS counter.
    #fps = FPS().start()


    while vstate =="lost":
     
        # grab the frame from the threaded video stream and return left line offset
        # We do this to know if we have a 'lock' (goodTarget) as we come off of manual control.
        target = vs.readfollow()
        bearing = target[0]
        red1Good = target[1]
        offset = target[2]
        red2Good = target[3]
        
        if (red1Good ==True and red2Good ==True):
            print "Found"
            found = True
        else:
            target = vs.readlost()
            bearing = target[0]
            red1Good = target[1]
            roll = target[2]
            red2Good = target[3]

            # update the FPS counter
            #fps.update()






    # stop the timer and display FPS information
    #fps.stop()
    #print('Elasped time in lost state: {:.2f}'.format(fps.elapsed()))
    #print('Approx. FPS: {:.2f}'.format(fps.fps()))
    #time.sleep(2)

    return vstate


# MAIN PROGRAM

vstate = "tracking" # Set the vehicle state to tracking in the finite state machine.




while True :

    if vstate == "tracking":
        # Enter tracking state
        vstate = tracking(vstate)
        #print "Leaving tracking..."

    elif vstate == "following":
        # Enter following state
        vstate = following(vstate)
        #print "Leaving following"

    else:
        # Enter lost state
        vstate = lost(vstate)
        #print "Leaving lost"


vstate = "tracking"

    
"""

#---------------------------- RETURN TO HOME AND CLEAN UP ----------------------------


# Initiate return to home
print "Returning to Launch"
vehicle.mode = VehicleMode("RTL")
print "Pause for 10s before closing vehicle"
time.sleep(10)

"""

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
