from dronekit import connect
import time
# import socket
# import math
# from pymavlink import mavutil
# import argparse  # Allows to input vals from command line to use them in python
import numpy as np
# import threading
# import sys
import asyncio
from ctypes import *

# Bench Testing
dummyDrone = False # Set to True to bench test and not connect to real drone, False for actual flights
if dummyDrone == True:
    print("DUMMY DRONE")
    import DummyGeneralFunctions as general_functions    
else:
    print("REAL DRONE")
    import GeneralDroneFunctions as general_functions #TODO REANABLE FOR FLIGHT

# Set constant variables
CURRENT_CHALLENGE = 4

if CURRENT_CHALLENGE == 4:
    import c4_distance as depth_analysis
    import pyzed.sl as stereolabs

# Function that transorms the position coordinate of the left eye of the camera to the
# position at the center of the camera
def transform_pose(pose, tx) :
  transform_ = stereolabs.Transform()
  transform_.set_identity()
  
  # Translate the tracking frame by tx along the X axis
  transform_[0, 3] = tx
  
  # Pose(new reference frame) = M.inverse() * pose (camera frame) * M, where M is the transform between the two frames
  transform_inv = stereolabs.Transform()
  transform_inv.init_matrix(transform_)
  transform_inv.inverse()
  pose = transform_inv * pose * transform_


class Location:
    def __init__(self, tx, ty, tz):   
        self.lat = tx
        self.lon = ty
        self.alt = tz
    
    def set(self, tx, ty, tz):
        self.lat = tx
        self.lon = ty
        self.alt = tz

async def challenge_3(vehicle, target_meters, target_altitude, field_width):
    print("Starting Challenge 3")
    home_location = vehicle.location.global_relative_frame
    half_field = (field_width / 2) - home_location.lat

    distance_traveled = 0
    currentLocation = home_location

    general_functions.ArmDrone(vehicle)
    
    print("Rising...")
    await general_functions.TakeOffDrone(vehicle, target_altitude)
    
    print(f"Flying {target_meters} Yards")
    await general_functions.GoToTargetBody(vehicle, target_meters, 0, 0)

    while vehicle.mode.name == "GUIDED":
        currentLocation = vehicle.location.global_relative_frame
        distance_traveled = general_functions.GetDistanceInMeters(vehicle, currentLocation, home_location)

        print("Distance traveled: ", distance_traveled)

        if ((currentLocation.alt > target_altitude + 0.5) | (currentLocation.alt < target_altitude - 0.5)):
            diff_in_altitude = currentLocation.alt - target_altitude
            await general_functions.GoToTargetBody(vehicle, 0, 0, diff_in_altitude)
            
            reachedElevation = False
            while reachedElevation == False:  # While the target elevation has not been reached
                currentAltLocation = vehicle.location.global_relative_frame
                currDroneHeight = currentAltLocation.alt
                print("Current drone elevation: ", currDroneHeight)

                if currDroneHeight >= (.95 * target_altitude):  # If the drone is at the target elevation (account for timing)
                    reachedElevation = True
                time.sleep(1)
                
            await general_functions.GoToTargetBody(vehicle, target_meters - distance_traveled, 0, 0)

        if ((currentLocation.lat > half_field - 3) | (currentLocation.lat < half_field + 3)):
            diff_left_and_right = home_location.lat - currentLocation.lat #calculates distance from the center of start position
            await general_functions.GoToTargetBody(vehicle, target_meters - distance_traveled, diff_left_and_right, 0) 

        if distance_traveled >= target_meters * 0.99:
            print("Target Reached\nEnding Challenge 3")
            break

        time.sleep(1)

async def challenge_4(cam, runtime_parameters, vehicle, target_meters, target_altitude, field_width, cam_width, cam_height):
    print("Start Challenge 4")
    
    # Initialize ROS & MAVLink
    print("Initializing ROS")
    scan, laser_scan_node, point_cloud_node, pointcloud, odometry_node, pose_node = depth_analysis.initialize_ros()
    
    # translation_left_to_center = cam.get_camera_information().calibration_parameters.T[0]

    # Get the pose of the ZED and send it over to ArduPilot through a MAVROS publisher
    cam_pose = stereolabs.Pose()
    pose_node.publish(cam_pose)
    
    # Get the visual odomoetry from the ZED and send it over to ArduPilot through a MAVROS pubisher
    odometry_node.publish(cam_pose)
    
    # Get the pose of the camera relative to the camera (or world) frame
    # tracking_state = cam.get_position(cam_pose, stereolabs.REFERENCE_FRAME.CAMERA)
    # transform_pose(cam_pose.pose_data(stereolabs.Transform()), translation_left_to_center)

    home_location = vehicle.location.global_relative_frame #Location(tx, ty, tz)
    half_field = (field_width / 2) - home_location.lat
    
    sector_mat = stereolabs.Mat()
    point_cloud_mat = stereolabs.Mat()

    distance_traveled = 0
    currentLocation = home_location

    print("Rising...")
    await general_functions.TakeOffDrone(vehicle, target_altitude)
    
    print(f"Flying {target_meters} Yards")
    await general_functions.GoToTargetBody(vehicle, target_meters, 0, 0)

    while vehicle.mode.name == "GUIDED_NOGPS":
        
        # transform_pose(tracking_state.pose_data(stereolabs.Transform()), translation_left_to_center)
        # tracking_state = cam.get_position(cam_pose, stereolabs.REFERENCE_FRAME.CAMERA)
        # tx = round(cam_pose.get_translation(py_translation).get()[0], 3)
        # ty = round(cam_pose.get_translation(py_translation).get()[1], 3)
        # tz = round(cam_pose.get_translation(py_translation).get()[2], 3)
        
        # Start depth analysis here?
        depth_analysis.depth_sector(cam, sector_mat, point_cloud_mat, scan, laser_scan_node, point_cloud_node, pointcloud, runtime_parameters, cam_width, cam_height)

        currentLocation = vehicle.location.global_relative_frame #.set(tx, ty, tz)
        distance_traveled = general_functions.GetDistanceInMeters(vehicle, currentLocation, home_location)
        print("Distance traveled: ", distance_traveled)

        if ((currentLocation.alt > target_altitude + 0.5) | (currentLocation.alt < target_altitude - 0.5)):
            diff_in_altitude = currentLocation.alt - target_altitude
            await general_functions.GoToTargetBody(vehicle, 0, 0, diff_in_altitude)
            
            reachedElevation = False
            while reachedElevation == False:  # While the target elevation has not been reached
                # tracking_state = cam.get_position(cam_pose, stereolabs.REFERENCE_FRAME.WORLD)
                # transform_pose(cam_pose.pose_data(stereolabs.Transform()), translation_left_to_center)
                currentAltLocation = vehicle.location.global_relative_frame
                currDroneHeight = currentAltLocation.alt # round(cam_pose.get_translation(py_translation).get()[2], 3)
                print("Current drone elevation: ", currDroneHeight)

                if currDroneHeight >= (.95 * target_altitude):  # If the drone is at the target elevation (account for timing)
                    reachedElevation = True
                time.sleep(1)
                
            await general_functions.GoToTargetBody(vehicle, target_meters - distance_traveled, 0, 0)

        if ((currentLocation.lat > half_field - 3) | (currentLocation.lat < half_field + 3)):
            diff_left_and_right = home_location.lat - currentLocation.lat #calculates distance from the center of start position
            await general_functions.GoToTargetBody(vehicle, target_meters - distance_traveled, diff_left_and_right, 0)

        if distance_traveled >= target_meters * 0.99:
            print("Target Reached\nEnding Challenge 4")
            break

        time.sleep(1)

async def main():
    target_meters = general_functions.YardsToMeters(20) #50
    target_altitude = general_functions.FeetToMeters(3.5)
    field_width = general_functions.YardsToMeters(10) #50
    
    print("Connecting To Drone...")
    if dummyDrone == True:
        vehicle = general_functions.DummyVehicle()
    else:
        #vehicle = connect('127.0.0.1:14550', wait_ready=True, baud=1500000)     
        vehicle = connect('/dev/ttyTHS2', wait_ready=True, baud=115200)

        #OA PArameters
        vehicle.parameters["OA_DB_EXPIRE"] = 0
        vehicle.parameters["OA_DB_QUEUE_SIZE"] = 40
        vehicle.parameters["OA_BR_LOOKAHEAD"] = 7 # Tune day of comp
        vehicle.parameters["OA_MARGIN_MAX"] = 1.5 # This needs to be tuned, most likley has to be set pretty low otherwise it will go side to side constantly 
        vehicle.parameters["OA_TYPE"] = 1
        vehicle.parameters["OA_DB_DIST_MAX"] = 10
        vehicle.parameters["OA_DB_SIZE"] = 200
        vehicle.parameters["AVOID_ENABLE"] = 7
        vehicle.parameters["WPNAV_SPEED"] = 30 #cm/S
        vehicle.airspeed = 0.1 # Test Speed = 2.0 THIS DOESNT WORK
    print("Connected to Drone")

    general_functions.ServoMovement(vehicle, 90)
    
    # Start the correct challenge
    if CURRENT_CHALLENGE == 3:
        if dummyDrone == False:
            vehicle.parameters["PRX_TYPE"] = 8
            vehicle.parameters["PRX_ORIENT"] = 0
            
        await challenge_3(vehicle, target_meters, target_altitude, field_width)
        
        print("Challenge 3 Complete")
    elif CURRENT_CHALLENGE == 4:
        if dummyDrone == False:
            vehicle.parameters["PRX_TYPE"] = 2
            vehicle.parameters["PRX_ORIENT"] = 1
        print("Intializing & Opening ZED Camera...")
        
        #ZED SDK Parameters
        cam = stereolabs.Camera()
        init_parameters = stereolabs.InitParameters()
        init_parameters.camera_resolution = stereolabs.RESOLUTION.HD720
        # init_parameters.camera_fps = 60
        init_parameters.depth_mode = stereolabs.DEPTH_MODE.PERFORMANCE
        init_parameters.coordinate_system = stereolabs.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        init_parameters.coordinate_units = stereolabs.UNIT.METER

        # Opening camera
        if not cam.is_opened():
            print("Opening ZED Camera...")
            status = cam.open(init_parameters)
            if status != stereolabs.ERROR_CODE.SUCCESS:
                print(repr(status))
                cam.close()
                exit(1)
        else:
            print("ZED Camera is Already Open")
        
        # Create and set RuntimeParameters after opening the camera
        runtime_parameters = stereolabs.RuntimeParameters()
        runtime_parameters.sensing_mode = stereolabs.SENSING_MODE.STANDARD
        
        # Setting the depth confidence parameters
        runtime_parameters.confidence_threshold = 100
        runtime_parameters.textureness_confidence_threshold = 100
        
        res_params = stereolabs.Resolution()
        width = round(cam.get_camera_information().camera_resolution.width / 2)
        height = round(cam.get_camera_information().camera_resolution.height / 2)
        res_params.width = width
        res_params.height = height
        print("ZED Camera Opened & Intialized")
        
        print("Enable ZED Pos Tracking")
        # Enable positional tracking with default parameters
        tracking_parameters = stereolabs.PositionalTrackingParameters()
        tracking_parameters.enable_area_memory = True
        tracking_parameters.enable_pose_smoothing = True
        err = cam.enable_positional_tracking(tracking_parameters)
        if err != stereolabs.ERROR_CODE.SUCCESS:
            print(repr(status))
            cam.close()
            exit()

        print("Setting ZED Parameters")
        # Set sensing mode in FILL
        runtime_parameters = stereolabs.RuntimeParameters()
        runtime_parameters.sensing_mode = stereolabs.SENSING_MODE.FILL
    
        await challenge_4(cam, runtime_parameters, vehicle, target_meters, target_altitude, field_width, runtime_parameters, width, height)
        
        cam.disable_positional_tracking()
        cam.close()
        
        print("Challenge 4 Complete")
    
    general_functions.LandDrone(vehicle)
    vehicle.close() #stop copter from running
    print("Drone Landed")

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(main())
    finally:
        loop.close()
