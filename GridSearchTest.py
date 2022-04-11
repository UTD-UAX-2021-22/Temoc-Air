import argparse
import logging
import time
import sys
import asyncio
from Utils import AdvancedLogger, DummyVehicle, calculateVisitPath
import GeneralDroneFunctions as gd #TODO REANABLE FOR FLIGHT
#import DummyGeneralFunctions as gd
from dronekit import connect


#from cv_bridge import CvBridge
#from sensor_msgs.msg import Image, CompressedImage
#import rospy


#TODO: Make actually work
#TODO: Fallback logic 
async def mainFunc():
        vehicle = connect('127.0.0.1:14550', wait_ready=True)
        homeLocation = vehicle.location.global_relative_frame
        await gd.GridSearch(vehicle,homeLocation)
        gd.LandDrone(vehicle)
        vehicle.close()
        exit(0)
        
        



if __name__ == "__main__":
    #rospy.init_node('challenge2_main')
    import asyncio
    loop = asyncio.get_event_loop()
    asyncio.ensure_future(mainFunc())
    loop.run_forever()
    #rospy.spin()
    #loop.close()

