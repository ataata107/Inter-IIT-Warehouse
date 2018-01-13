from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import numpy as np

import time


# Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None
#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    if vehicle.mode.name == "INITIALISING":
        print "Waiting for vehicle to initialise"
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("POSHOLD")
    

    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.channels.overrides['3']=1550
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print "Attitude: %s" % vehicle.attitude
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            vehicle.channels.overrides['3']=1500
            break
        time.sleep(1)





vehicle.channels.overrides={}
print " Ch3: %s" % vehicle.channels['3']
print " Ch4: %s" % vehicle.channels['4']
arm_and_takeoff(2)
print(vehicle.attitude.yaw)
initial=vehicle.attitude.yaw
while True:
    vehicle.channels.overrides['4']=1600
    if (vehicle.attitude.yaw>=initial+1.57):
        vehicle.channels.overrides['4']=None
        break
vehicle.channels.overrides['4']=None
print " Channel overrides: %s" % vehicle.channels.overrides 
print(vehicle.attitude.yaw)
print " Ch4: %s" % vehicle.channels['4']
print " Ch3: %s" % vehicle.channels['3']
vehicle.mode=VehicleMode("LAND")
vehicle.close()
