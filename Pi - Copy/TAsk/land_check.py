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
from PID import PIDController
import time


# Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

#PID
pid = PIDController(proportional = 0.15, derivative_time = 0, integral_time=0)
pid.vmin, pid.vmax = -0.05, 0.05
pid.setpoint = 2.0   #aTargetAltitude(m)
TAltitude = pid.setpoint
baseThrust = 0.55

pidout = 0


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
#---------------------------TAkeoff-------------------------------------------------
def landing_nogps(aTargetAltitude):
    c=0
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    
    ##### CONSTANTS #####
    DEFAULT_LANDING_THRUST = 0.4
    SMOOTH_LANDING_THRUST = 0.4

    print ("Mode: %s" % (vehicle.mode.name))

    vehicle.mode = VehicleMode("GUIDED_NOGPS")

    while not vehicle.mode.name is "GUIDED_NOGPS":
	c+=1
    print(vehicle.mode.name, c)
    print("LANDING!")
    
    c=0
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.mode.name is "LAND":
	c+=1
    print(vehicle.mode.name, c)
    
#    thrust = DEFAULT_LANDING_THRUST
    
    '''
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %s" % current_altitude)
        set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.4, duration = 0)
        print "Attitude: %s" % vehicle.attitude
        time.sleep(0.1)
    '''
landing_nogps(0.1)