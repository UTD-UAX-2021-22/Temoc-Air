import argparse
import logging
import time
import sys
import asyncio
import threading
from Utils import AdvancedLogger, DummyVehicle, calculateVisitPath
import GeneralDroneFunctions as gd #TODO REANABLE FOR FLIGHT
#import DummyGeneralFunctions as gd
from dronekit import connect


#from cv_bridge import CvBridge
#from sensor_msgs.msg import Image, CompressedImage
#import rospy


#TODO: Make actually work
#TODO: Fallback logic 
def mainFunc():
        vehicle = connect('127.0.0.1:14550', wait_ready=True)
        homeLocation = vehicle.location.global_relative_frame

        lengthOfRuns = 145 #feet
        sideOffset = 89
        runOffset = 20
        widthOfField = 160
        cWise = True 
    

        if(sideOffset >= (widthOfField/2)):
            cWise = True
        else:
            cWise = False
        gd.ArmDrone(vehicle)
        gd.TakeOffDrone2(vehicle, gd.FeetToMeters(15))
        vehicle.airspeed = 3
        runsArr = gd.HowManyRuns(sideOffset, runOffset)
        print("Number of runs!!")
        print(len(runsArr))
        start_time= time.time()
        threading.Thread(target=gd.GoToTargetBody2(vehicle, gd.FeetToMeters(lengthOfRuns), 0, 0)).start()
        threading.Thread(target=gd.scanning(start_time, 2)).start()
        gd.IsMoving(vehicle, lengthOfRuns, homeLocation)
        for running in runsArr:
    	    #distBtwn = running
            #GoToTargetBody(FeetToMeters(lengthOfRuns), 0, 0)
            #time.sleep(10)
            dist = running
            if(cWise):
                print("cWise new")
                start_time= time.time()
                threading.Thread(target=gd.scanning(start_time, 2)).start()
                gd.SetConditionYaw(vehicle, 90, True)
                time.sleep(5)
                #GoToTargetBody(FeetToMeters(5), 0, 0)
                start_time= time.time()
                threading.Thread(target=gd.GoToTargetBody2(vehicle,dist, 0, 0)).start()
                threading.Thread(target=gd.scanning(start_time, 2)).start()
                gd.IsMoving(vehicle, lengthOfRuns,homeLocation) 
                time.sleep(5)
                gd.PrintTelemetry(vehicle)
                gd.SetConditionYaw(vehicle, 90, True)
                time.sleep(5)
                cWise = False
                start_time= time.time()
                threading.Thread(target=gd.GoToTargetBody2(vehicle, gd.FeetToMeters(lengthOfRuns), 0, 0)).start()
                threading.Thread(target=gd.scanning(start_time, 2)).start()
                print(f"Distance: {lengthOfRuns}")
                #IsMoving(vehicle, lengthOfRuns, homeLocation)
                time.sleep(10)
            else:
                print("moving")
                #SetConditionYaw(vehicle, -90, True)
                #print("first yaw")
                gd.SetConditionYaw(vehicle, 270, True)
                print("second yaw")
                time.sleep(10)
                start_time= time.time()
                threading.Thread(target=gd.GoToTargetBody2(vehicle,dist, 0, 0)).start()
                threading.Thread(target=gd.scanning(start_time, 3)).start()
                time.sleep(10)
                gd.PrintTelemetry(vehicle)
                print("moving")
                gd.SetConditionYaw(vehicle,270, True)
                time.sleep(10)
                cWise = True
                start_time= time.time()
                threading.Thread(target=gd.GoToTargetBody2(vehicle,FeetToMeters(lengthOfRuns), 0, 0)).start()
                threading.Thread(target=gd.scanning(start_time, 2)).start()
                time.sleep(10)

        gd.LandDrone(vehicle)
        vehicle.close()
        
        



if __name__ == "__main__":
    #rospy.init_node('challenge2_main')
    mainFunc()
    #rospy.spin()
    #loop.close()

