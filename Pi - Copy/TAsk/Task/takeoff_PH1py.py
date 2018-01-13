#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
change_attitude.py: (Copter Only)
This example shows how to move/direct Copter and send commands in GUIDED_NOGPS mode using DroneKit Python.
Caution: A lot of unexpected behaviors may occur in GUIDED_NOGPS mode.
        Always watch the drone movement, and make sure that you are in dangerless environment.
        Land the drone as soon as possible when it shows any unexpected behavior.
Tested in Python 2.7.10
"""

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


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect('127.0.0.1:14550', wait_ready=True)




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
    vehicle.channels.overrides['3'] = 1550
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print "Attitude: %s" % vehicle.attitude
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            vehicle.channels.overrides['3'] = 1500
            break
        
        time.sleep(1)
def hover(a):

    
    start_time=time.time()
    
    while (time.time()<=start_time+a):
        print " Altitude: ", vehicle.location.global_relative_frame.alt
    
        print("hover")






        
arm_and_takeoff(2)
hover(5)
while True:
    vehicle.channels.overrides['2'] = 1450
    print (vehicle.mode.name)
    if vehicle.mode=="STABILIZE":
        vehicle.channels.overrides['2'] = None
        break

print("LANDING")
vehicle.channels.overrides['3'] = None
vehicle.channels.overrides
# Close vehicle object before exiting script
vehicle.mode=VehicleMode("LAND")
vehicle.close()



