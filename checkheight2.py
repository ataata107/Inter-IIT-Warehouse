from dronekit import *
import time
vehicle=connect('com11',wait_ready=True)
def true_hgt():
    hgt=vehicle.location.global_relative_frame.alt
    if (hgt>1):
        hgt=vehicle.rangefinder.distance
    return hgt  
while True:
    true_hgt()
    time.sleep(0.1)
