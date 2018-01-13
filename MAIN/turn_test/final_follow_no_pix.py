from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import numpy as np
from pyquaternion import Quaternion
from PiVideoStream_final import PiVideoStream
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import cv2
import sys
from datetime import datetime, timedelta
from PID import PIDController


'''
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




#--------------------------FUNCTION DEFINITION FOR SET_ATTITUDE MESSAGE --------------------

def set_attitude (pitch, roll, yaw, thrust):
    # The parameters are passed in degrees
    # Convert degrees to radians
    #degrees = (2*np.pi)/360
    yaw = np.radians(yaw)
    pitch = np.radians(pitch)
    roll = np.radians(roll) 
    
    # Now calculate the quaternion in preparation to command the change in attitude
    # q for yaw is rotation about z axis
    qyaw = Quaternion (axis = [0, 0, 1], angle = yaw )
    qpitch = Quaternion (axis = [0, 1, 0], angle = pitch )
    qroll = Quaternion (axis = [1, 0, 0], angle = roll )

    # We have components, now to combine them into one quaternion
    q = qyaw * qpitch * qroll
    
    a = q.elements
    
    rollRate = (roll * 5)
    yawRate = (yaw * 0.5)
    #pitchRate = abs(pitch * 1)
    # print " Yaw: ",yaw, " Yaw Rate: ", yawRate, " Roll: ",roll, "Roll Rate: ", rollRate, " Pitch: ", pitch, " Thrust: ",thrust
   
    msg = vehicle.message_factory.set_attitude_target_encode(
    0,
    0,                #target system
    0,                #target component
    0b0000000,       #type mask
    [a[0],a[1],a[2],a[3]],        #q
    rollRate,                #body roll rate
    0.5,                #body pitch rate
    yawRate,                #body yaw rate
    thrust)                #thrust
    
    vehicle.send_mavlink(msg)




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
'''

def tracking (vstate):
    print (vstate)
    
    #The vehicle process images and maintains all data ready to fly in the following state.
    #However, it does not send attitude messages as the vehicle is still under manual control.

    red1Good = red2Good = False # Set True when returned target offset is reliable.
    bearing = offset = 0
    target = None # Initialise tuple returned from video stream

    
    while vstate == "tracking":
     
        # grab the frame from the threaded video stream and return left line offset
        # We do this to know if we have a 'lock' (goodTarget) as we come off of manual control.
        target = vs.readfollow()
        bearing = target[0]
        print("Bearing: %s" %bearing)
        red1Good = target[1]
        offset = target[2]
        print("Offset: %s" %offset)
        red2Good = target[3]

       
            
        if ( red1Good and red2Good ) == True:
             print "In guided mode, setting following..."
            vstate = "following"
        else:
             print "In guided mode, setting lost..."
            vstate = "lost"

    return vstate


#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE FOLLOWING---------------------

def following (vstate):
    print (vstate)

    #The vehicle process images and uses all data to fly in the following state.
    # It sends attitude messages until manual control is resumed.    

    maxPitch = -3 # Maximum pitch for target at 0 bearing.
    multRoll= 8 # The roll angle if offset is at edge of the near field of view.
    #multYaw = 0 # A multiplier for the rate of Yaw.  
    yaw = roll = 0
    target = None # Initialise tuple returned from video stream
  
#    altitude = vehicle.location.global_relative_frame.alt
 #   print (altitude)

    while vstate =="following":
     
        # grab the frame from the threaded video stream and return left line offset
        # We do this to know if we have a 'lock' (goodTarget) as we come off of manual control.
        target = vs.readfollow()
        bearing = target[0] # Returned in degrees, +ve clockwise
        print("Bearing: %s" %bearing)
        red1Good = target[1]
        offset = target[2] # Returned as a fraction of image width.  -1 extreme left.
        print("Offset: %s" %offset)
        red2Good = target[3]
        
 #       tofHeight = vehicle.location.global_relative_frame.alt
 #       print (tofHeight)
        
        # print "Measured distance is : %d mm" % tofHeight
        
    
        # adjust thrust towards target
        '''
        if tofHeight > altitude + 0.1:
            thrust = 0.45
        elif tofHeight < altitude - 0.1:
            thrust = 0.55
        else:
            thrust = 0.5
        '''
        
        # Check if operator has transferred to autopilot using TX switch.
        

        if (red1Good == True and red2Good == True) :
            yaw = bearing #* multYaw  # Set maximum yaw in degrees either side
            roll = offset * multRoll # Set maximum roll in degrees either side

            # Limit the range of roll possible
            if roll > 8 :
                roll = 8
            elif roll < -8 :
                roll = -8
                
            bearing = abs(bearing) # Pitch depends on bearing, same either -ve or +ve
            pitch = maxPitch * ( (90-bearing)/90)# So pitch reduces to zero as bearing tends to 90.
            print "Following Bearing: ",bearing, " Yaw: ",yaw, " Roll: ",roll, " Pitch: ", pitch)
        
            
        else:
            vstate = "lost"

       
        
    return vstate





#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE LOST---------------------

def lost(vstate):
    print (vstate)
 
    #The vehicle process images and uses all data to fly in the lost state.
    # The vehicle rotates in one spot until a lock is established.
    # It sends attitude messages until manual control is resumed.

    maxPitch = 0
    multRoll= 0
    multYaw = 0
    yaw = roll = 0
    
    target = None # Initialise tuple returned from video stream
#    altitude = vehicle.location.global_relative_frame.alt
    found = False

    while vstate =="lost":
     
        # grab the frame from the threaded video stream and return left line offset
        # We do this to know if we have a 'lock' (goodTarget) as we come off of manual control.
        target = vs.readfollow()
        bearing = target[0]
        print("Bearing : %s " %bearing)
        red1Good = target[1]
        offset = target[2]
        print("Offset : %s " %offset)
        red2Good = target[3]
        
        if (red1Good ==True and red2Good ==True):
            print ("Found")
            found = True
        else:
            target = vs.readlost()
            bearing = target[0]
            print("Bearing : %s " %bearing)
            red1Good = target[1]
            roll = target[2]
            print("Roll : %s " %roll)
            red2Good = target[3]

 #           tofHeight = vehicle.location.global_relative_frame.alt
 #           print (tofHeight)
            # print "Measured distance is : %d mm" % tofHeight


            # adjust thrust towards target - increase altitude to 20-25cms
            '''
            if tofHeight > altitude + 0.1:
                thrust = 0.45
            elif tofHeight < altitude - 0.1:
                thrust = 0.55
            else:
                thrust = 0.5
            '''
        # Check if operator has transferred to autopilot using TX switch.
        
        if found == True:
            #print "Found target - exiting lost state into following"
            vstate = "following"
                
        elif red1Good == True: # We have lock on upper roi.
            print ("Lost - upper lock")
            yaw = bearing * multYaw  # Set maximum yaw in degrees either side
            roll = offset * multRoll # Set maximum roll in degrees either side
            bearing = np.absolute(bearing) # Pitch depends on bearing, same either -ve or +ve
            pitch = maxPitch * ( (90-bearing)/90) # So pitch reduces to zero as bearing tends to 90.
            print "Following Bearing: ",bearing, " Yaw: ",yaw, " Roll: ",roll, " Pitch: ", pitch)
            

        else: # We have nothing at all and need to rotate to look for something.
            # Set the attitude - note angles are in degrees
            print ("Lost - no lock")
            yaw = 0
            roll = 0 
            pitch = 0
            print "Following Bearing: ",bearing, " Yaw: ",yaw, " Roll: ",roll, " Pitch: ", pitch)
            
       
        
    return vstate


# MAIN PROGRAM ********************************************************************

vstate = "tracking" # Set the vehicle state to tracking in the finite state machine.

# CAMERA INITIALISATION

vs = PiVideoStream().start()
time.sleep(2.0)

while True :
	
    #frame = vs.read()
    #frame = imutils.resize(frame, width=400)
    #cv2.imshow("Frame", frame)
   
    if vstate == "tracking":
        # Enter tracking state
        vstate = tracking(vstate)
        print "Leaving tracking..."

    elif vstate == "following":
        # Enter following state
        vstate = following(vstate)
        print "Leaving following"

    else:
        # Enter lost state
        vstate = lost(vstate)
        print "Leaving lost"

    if cv2.waitKey(1) & 0xFF == ord('q'):
	break



vstate = "tracking"

#Close vehicle object before exiting script
print ("Close vehicle object")
#vehicle.close()

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()


