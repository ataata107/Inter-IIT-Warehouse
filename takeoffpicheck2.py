# Import DroneKit-Python
from dronekit import *

# Connect to the Vehicle.

vehicle = connect('192.168.1.153:14550', wait_ready=True)

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
    vehicle.mode    = VehicleMode("GUIDED_NOGPS")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)
    
    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", true_hgt())
        #Break and return from function just below target altitude.
        if true_hgt()>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)





def true_hgt():
    global hgt
    global flag
    if (hgt>1):
        flag=False
    if (flag==True):
        hgt=vehicle.location.global_relative_frame.alt
    else:
        hgt=vehicle.rangefinder.distance
    return hgt 


hgt=0.1
flag=False

arm_and_takeoff(3)
# Close vehicle object before exiting script
vehicle.close()



