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
baseVelocity = 0.5

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
def arm_and_takeoff_nogps(TAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    

    
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check, just comment it with your own responsibility.
    if vehicle.mode.name == "INITIALISING":
        print "Waiting for vehicle to initialise"
        time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")

    while True:
        current_altitude,Baro = true_hgt()
        print ("Mode: %s" % (vehicle.mode.name))
        print ("Altitude left %s" % (TAltitude - current_altitude))
        print(Baro)
        
    
        pidout = pid.compute_output(current_altitude)
        pidout += baseVelocity
        print ("Height = ", (current_altitude), "px, ", "Thrust = ", pidout)
        send_ned_velocity(0, 0, 0.20, duration=0)
        time.sleep(0.1)
        print "Velocity: %s" % vehicle.velocity
        if (current_altitude >= TAltitude):
            break
def hover(a):
    
    start_time=time.time()
    while (time.time()<=start_time+a):
        current_altitude,Baro = true_hgt()
        print ("Mode: %s" % (vehicle.mode.name))
        pidout = pid.compute_output(current_altitude)
        pidout += baseVelocity
        print ("Height = ", (current_altitude), "px, ", "Thrust = ", pidout)
        send_ned_velocity(0, 0, 0, duration=0)
        time.sleep(0.1)
        print "Velocity: %s" % vehicle.velocity
        

def landing_nogps():
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    

    
    print("LANDING!")
    
    print ("Mode: %s" % (vehicle.mode.name))
    thrust = DEFAULT_LANDING_THRUST
    
    
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %s" % current_altitude)
        send_ned_velocity(0, 0, -0.20, duration=0)
        print "Velocity: %s" % vehicle.velocity
        time.sleep(0.1)
    


def true_hgt():
    global hgt
    global flag
    if (hgt>1):
        flag=False
    if (flag==True):
        hgt=vehicle.location.global_relative_frame.alt
    else:
        hgt=vehicle.rangefinder.distance
    return hgt,flag   
    

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
        print "Attitude: %s" % vehicle.attitude
        print (vehicle.location.global_relative_frame.alt)




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
initial_height=vehicle.location.global_relative_frame.alt
hgt=0.1

# Take off 2.5m in GUIDED_NOGPS mode.
flag=True
arm_and_takeoff_nogps(TAltitude)
hover(5)
landing_nogps()

'''
# Hold the position for 3 seconds.
print("Hold position for 3 seconds")
set_attitude(duration = 5)


# Uncomment the lines below for testing roll angle and yaw rate.
# Make sure that there is enough space for testing this.

# set_attitude(roll_angle = 1, thrust = 0.5, duration = 3)
# set_attitude(yaw_rate = 30, thrust = 0.5, duration = 3)

# Move the drone forward and backward.
# Note that it will be in front of original position due to inertia.
print("Move forward")
set_attitude(pitch_angle = 1, thrust = 0.5, duration = 3.21)

print("Move backward")
set_attitude(pitch_angle = -1, thrust = 0.5, duration = 3)


print("Setting LAND mode...")
landing_nogps(initial_height)
time.sleep(1)
'''
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()


# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")



