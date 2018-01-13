from dronekit import *
vehicle=connect('127.0.0.1:14551',wait_ready=True)
def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    
    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6
    
    print("Basic pre-arm checks")



    print("Arming motors")




    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt

        print(" Altitude: %s " % current_altitude)
        print(" Thrust: %s " % thrust)        
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        
        time.sleep(0.2)

def landing_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    
    ##### CONSTANTS #####
    DEFAULT_LANDING_THRUST = 0.3
    SMOOTH_LANDING_THRUST = 0.4
    
    print("LANDING!")

    thrust = DEFAULT_LANDING_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %s" % current_altitude)
        if current_altitude <= aTargetAltitude*1.05: # Trigger just above target alt.
            print("Reached target altitude")
            break
        elif current_altitude <= aTargetAltitude*1.4:
            thrust = SMOOTH_LANDING_THRUST
        
        time.sleep(0.2)



arm_and_takeoff_nogps(1.25)    
