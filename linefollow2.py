# import the necessary packages
# from __future__ import print_function            # This needs to be on the first line
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import numpy as np
#from pyquaternion import Quaternion
from PiVideoStream import PiVideoStream
#from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
#import imutils
import cv2
import sys
#from ST_VL6180X import VL6180X
from datetime import datetime, timedelta
from PID import PIDController


#--------------------------SET UP CONNECTION TO VEHICLE----------------------------------

# Parse the arguments  
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

# Connect to the physical UAV or to the simulator on the network
if not connection_string:
    print ('Connecting to pixhawk.')
    vehicle = connect('/dev/serial0', baud=57600, wait_ready= True)
else:
    print ('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)



#--------------------------SET UP VIDEO THREAD ----------------------------------

# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter
print('[INFO] sampling THREADED frames from `picamera` module...')
vs = PiVideoStream().start()
def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """
    
    """
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """
    
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
                                                             0,
                                                             0,
                                                                 # Target system
                                                             0,
                                                                 # Target component
                                                             0b00000000,
                                                                 # Type mask: bit 1 is LSB
                                                             to_quaternion(roll_angle, pitch_angle),
                                                                 # Quaternion
                                                             0,
                                                                 # Body roll rate in radian
                                                             0,
                                                                 # Body pitch rate in radian
                                                             math.radians(yaw_rate),
                                                                 # Body yaw rate in radian
                                                             thrust)
                                                                 # Thrust
    vehicle.send_mavlink(msg)
                                                             
    if duration != 0:
        # Divide the duration into the frational and integer parts
        modf = math.modf(duration)
        
        # Sleep for the fractional part
        time.sleep(modf[0])
        
        # Send command to vehicle on 1 Hz cycle
        for x in range(0,int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))
    
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    
    return [w, x, y, z]





#-------------- FUNCTION DEFINITION TO ARM AND TAKE OFF TO GIVEN ALTITUDE ---------------
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ('Basic pre-arm checks')
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print ('Waiting for vehicle to initialise...')
        time.sleep(1)

        
    print ('Arming motors')
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print ('Waiting for arming...')
        time.sleep(1)

    print ('Taking off!')
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    while True:
        # print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            break
        time.sleep(1)


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
        if vehicle.mode == "GUIDED_NOGPS":
            # print "In Guided mode..."
            
            if ( red1Good and red2Good ) == True:
                # print "In guided mode, setting following..."
                vstate = "following"
            else:
                # print "In guided mode, setting lost..."
                vstate = "lost"

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
  
    altitude = vehicle.location.global_relative_frame.alt
    print altitude

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
        tofHeight = vehicle.location.global_relative_frame.alt
        print tofHeight
        
        # print "Measured distance is : %d mm" % tofHeight
        
    
        # adjust thrust towards target
        if tofHeight > altitude + 0.1:
            thrust = 0.45
        elif tofHeight < altitude - 0.1:
            thrust = 0.55
        else:
            thrust = 0.5


        # Check if operator has transferred to autopilot using TX switch.
        if vehicle.mode == "GUIDED_NOGPS":
            # print "In Guided mode..."
            # print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame

            if (red1Good == True and red2Good == True) :
                yaw = bearing #* multYaw  # Set maximum yaw in degrees either side
                roll = offset * multRoll # Set maximum roll in degrees either side

                # Limit the range of roll possible
                if roll > 8 :
                    roll = 8
                elif roll < -8 :
                    roll = -8
                
                bearing = abs(bearing) # Pitch depends on bearing, same either -ve or +ve
                pitch = maxPitch * ( (90-bearing)/90) # So pitch reduces to zero as bearing tends to 90.
                # print "Following Bearing: ",bearing, " Yaw: ",yaw, " Roll: ",roll, " Pitch: ", pitch, thrust
                set_attitude (pitch, roll, yaw, thrust)
            else:
                vstate = "lost"

        else:
            # print "Exited GUIDED mode, setting tracking from following..."
            vstate = "tracking"

    # stop the timer and display FPS information
    #fps.stop()
    #print('Elasped time in following state: {:.2f}'.format(fps.elapsed()))
    #print('Approx. FPS: {:.2f}'.format(fps.fps()))
    #time.sleep(3.0)

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
    altitude = vehicle.location.global_relative_frame.alt
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

            # Get the altitude information.
            # tofHeight =  tof_sensor.get_distance()
            tofHeight = vehicle.location.global_relative_frame.alt
            print tofHeight
            # print "Measured distance is : %d mm" % tofHeight


            # adjust thrust towards target - increase altitude to 20-25cms
            if tofHeight > altitude + 0.1:
                thrust = 0.45
            elif tofHeight < altitude - 0.1:
                thrust = 0.55
            else:
                thrust = 0.5

        # Check if operator has transferred to autopilot using TX switch.
        if vehicle.mode == "GUIDED_NOGPS":
            #print "In Guided mode, lost..."
            
            if found == True:
                #print "Found target - exiting lost state into following"
                vstate = "following"
                
            elif red1Good == True: # We have lock on upper roi.
                print "Lost - upper lock"
                yaw = bearing * multYaw  # Set maximum yaw in degrees either side
                roll = offset * multRoll # Set maximum roll in degrees either side
                bearing = np.absolute(bearing) # Pitch depends on bearing, same either -ve or +ve
                pitch = maxPitch * ( (90-bearing)/90) # So pitch reduces to zero as bearing tends to 90.
                set_attitude (pitch, roll, yaw, thrust) 

            else: # We have nothing at all and need to rotate to look for something.
                # Set the attitude - note angles are in degrees
                print "Lost - no lock"
                yaw = 0
                roll = 0 
                pitch = 0
                set_attitude (pitch, roll, yaw, thrust)
        else:
            #print "Exited GUIDED mode, setting tracking from following..."
            vstate = "tracking"

    # stop the timer and display FPS information
    #fps.stop()
    #print('Elasped time in lost state: {:.2f}'.format(fps.elapsed()))
    #print('Approx. FPS: {:.2f}'.format(fps.fps()))
    #time.sleep(2)

    return vstate


# MAIN PROGRAM

vstate = "tracking" # Set the vehicle state to tracking in the finite state machine.


# If on simulator, arma and take off.
if connection_string:

    print ('Basic pre-arm checks')
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print ('Waiting for vehicle to initialise...')
        time.sleep(1)

    print ('Arming motors')
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    


    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print ('Waiting for arming...')
        time.sleep(1)

    # Get airborne and hover
    arm_and_takeoff(10)
    print "Reached target altitude - currently in Guided mode on altitude hold"
    
    vehicle.mode = VehicleMode("GUIDED_NOGPS")


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
