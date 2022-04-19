from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
from pymavlink import mavutil
import argparse  # Allows to input vals from command line to use them in python
import numpy as np
import threading
import sys
import asyncio
from ctypes import *

# Bench Testing
dummyDrone = False # Set to True to bench test and not connect to real drone, False for actual flights
if dummyDrone == True:
    import DummyGeneralFunctions as general_functions    
else:
    import GeneralDroneFunctions as general_functions

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
  transform_[0][3] = tx
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
    home_location = vehicle.location.global_relative_frame
    half_field = (field_width / 2) - home_location.lat

    distance_traveled = 0
    currentLocation = home_location

    general_functions.ArmDrone(vehicle)
    take_off = asyncio.create_task(general_functions.TakeOffDrone(vehicle, target_altitude))
    while not (take_off.done()):
        print("Await liftoff task")
        await asyncio.sleep(1)
        
    vehicle.airspeed = 1.5 # Test Speed
    general_functions.GoToTargetBody(vehicle, target_meters, 0, 0)

    while vehicle.mode.name == "GUIDED":
        
        currentLocation = vehicle.location.global_relative_frame
        distance_traveled = general_functions.get_distance_metres(currentLocation, home_location)
        print("Distance traveled: ", distance_traveled)

        if ((currentLocation.alt > target_altitude + 0.5) | (currentLocation.alt < target_altitude - 0.5)):
            diff_in_altitude = currentLocation.alt - target_altitude
            general_functions.GoToTargetBody(vehicle, 0, 0, diff_in_altitude)
            
            reachedElevation = False
            while reachedElevation == False:  # While the target elevation has not been reached
                currentAltLocation = vehicle.location.global_relative_frame
                currDroneHeight = currentAltLocation.alt
                print("Current drone elevation: ", currDroneHeight)

                if currDroneHeight >= (.95 * target_altitude):  # If the drone is at the target elevation (account for timing)
                    reachedElevation = True
                time.sleep(1)
                
            general_functions.GoToTargetBody(vehicle, target_meters - distance_traveled, 0, 0)

        if ((currentLocation.lat > half_field -2) | (currentLocation.lat < half_field + 2)):
            diff_left_and_right = home_location.lat - currentLocation.lat #calculates distance from the center of start position
            general_functions.GoToTargetBody(vehicle, target_meters - distance_traveled, diff_left_and_right, 0) 

        if distance_traveled >= target_meters * 0.99:
            print("Target Reached")
            break

        time.sleep(1)

async def challenge_4(cam, vehicle, target_meters, target_altitude, field_width):
    res_params = stereolabs.Resolution()
    width = round(cam.get_camera_information().camera_resolution.width / 2)
    height = round(cam.get_camera_information().camera_resolution.height / 2)
    res_params.width = width
    res_params.height = height
    
    # Enable positional tracking with default parameters
    tracking_parameters = stereolabs.PositionalTrackingParameters()
    err = cam.enable_positional_tracking(tracking_parameters)

    # Set sensing mode in FILL
    runtime_parameters = stereolabs.RuntimeParameters()
    runtime_parameters.sensing_mode = stereolabs.SENSING_MODE.STANDARD

    translation_left_to_center = cam.get_camera_information().calibration_parameters.T[0]

    cam_pose = stereolabs.Pose()

    # Retrieve and transform the pose data into a new frame located at the center of the camera
    tracking_state = cam.get_position(cam_pose, stereolabs.REFERENCE_FRAME.WORLD)
    transform_pose(cam_pose.pose_data(stereolabs.Transform()), translation_left_to_center)

    py_translation = stereolabs.Translation()
    tx = round(cam_pose.get_translation(py_translation).get()[0], 3)
    ty = round(cam_pose.get_translation(py_translation).get()[1], 3)
    tz = round(cam_pose.get_translation(py_translation).get()[2], 3)

    home_location = Location(tx, ty, tz)
    half_field = (field_width/2) - home_location.lat
    
    sector_mat = stereolabs.Mat()
    point_cloud_mat = stereolabs.Mat()

    distance_traveled = 0
    currentLocation = home_location
    
    # Initialize ROS & MAVLink
    scan, node1, node2 = depth_analysis.intialize_ros()

    general_functions.ArmDrone(vehicle)
    take_off = asyncio.create_task(general_functions.TakeOffDrone(vehicle, target_altitude))
    while not (take_off.done()):
        print("Await liftoff task")
        await asyncio.sleep(1)
        
    vehicle.airspeed = 0.5
    general_functions.GoToTargetBody(vehicle, target_meters, 0, 0)

    while vehicle.mode.name == "GUIDED_NOGPS":
        
        tracking_state = cam.get_position(cam_pose, stereolabs.REFERENCE_FRAME.WORLD)
        transform_pose(tracking_state.pose_data(stereolabs.Transform()), translation_left_to_center)
        tx = round(cam_pose.get_translation(py_translation).get()[0], 3)
        ty = round(cam_pose.get_translation(py_translation).get()[1], 3)
        tz = round(cam_pose.get_translation(py_translation).get()[2], 3)
        
        # Start depth analysis here?
        depth_analysis.depth_sector(cam, sector_mat, point_cloud_mat, scan, node1, node2)

        currentLocation.set(tx, ty, tz)
        distance_traveled = general_functions.get_distance_metres(currentLocation, home_location)
        print("Distance traveled: ", distance_traveled)

        if ((currentLocation.alt > target_altitude + 0.5) | (currentLocation.alt < target_altitude - 0.5)):
            diff_in_altitude = currentLocation.alt - target_altitude
            general_functions.GoToTargetBody(vehicle, 0, 0, diff_in_altitude)
            
            reachedElevation = False
            while reachedElevation == False:  # While the target elevation has not been reached
                tracking_state = cam.get_position(cam_pose, stereolabs.REFERENCE_FRAME.WORLD)
                transform_pose(cam_pose.pose_data(stereolabs.Transform()), translation_left_to_center)
                currDroneHeight = round(cam_pose.get_translation(py_translation).get()[2], 3)
                print("Current drone elevation: ", currDroneHeight)

                if currDroneHeight >= (.95 * target_altitude):  # If the drone is at the target elevation (account for timing)
                    reachedElevation = True
                time.sleep(1)
                
            general_functions.GoToTargetBody(vehicle, target_meters - distance_traveled, 0, 0)

        if ((currentLocation.lat > half_field -2) | (currentLocation.lat < half_field + 2)):
            diff_left_and_right = home_location.lat - currentLocation.lat #calculates distance from the center of start position
            general_functions.GoToTargetBody(vehicle, target_meters - distance_traveled, diff_left_and_right, 0)

        if distance_traveled >= target_meters*0.99:
            print("Target Reached")
            break

        time.sleep(1)

def main():
    target_meters = general_functions.YardsToMeters(10)
    target_altitude = general_functions.FeetToMeters(3)
    field_width = general_functions.YardsToMeters(9)

    print("Connecting To Drone...")
    #vehicle = connect('127.0.0.1:14550', wait_ready=True)
    vehicle = connect('/dev/ttyTHS2', wait_ready=True, baud=1500000)

    #OA PArameters
    vehicle.parameters["OA_DB_EXPIRE"] = 15
    vehicle.parameters["OA_DB_QUEUE_SIZE"] = 40
    vehicle.parameters["OA_BR_LOOKAHEAD"] = 10 
    vehicle.parameters["OA_MARGIN_MAX"] = 2.7
    vehicle.parameters["OA_TYPE"] = 1
    vehicle.parameters["OA_DB_DIST_MAX"] = 6
    vehicle.parameters["OA_DB_SIZE"] = 100
    vehicle.parameters["PRX_TYPE"] = 8
    vehicle.parameters["PRX_ORIENT"] = 0
    vehicle.parameters["AVOID_ENABLE"] = 7
    print("Connected to Drone")

    general_functions.ServoMovement(vehicle, 90)
    
    # Start the correct challenge
    if CURRENT_CHALLENGE == 3:
        challenge_3(vehicle, target_meters, target_altitude, field_width)
    elif CURRENT_CHALLENGE == 4:
        print("Intializing & Opening ZED Camera...")
        #ZED SDK Parameters
        cam = stereolabs.Camera()
        init_parameters = stereolabs.InitParameters()
        init_parameters.camera_resolution = stereolabs.RESOLUTION.HD720
        init_parameters.camera_fps = 60
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
                exit()
        
        # Create and set RuntimeParameters after opening the camera
        runtime_parameters = stereolabs.RuntimeParameters()
        runtime_parameters.sensing_mode = stereolabs.SENSING_MODE.FILL
        
        # Setting the depth confidence parameters
        runtime_parameters.confidence_threshold = 100
        runtime_parameters.textureness_confidence_threshold = 100
        print("ZED Camera Opened & Intialized")
        challenge_4(cam, vehicle, target_meters, target_altitude, field_width)
    
    general_functions.LandDrone(vehicle)
    vehicle.close() #stop copter from running
    cam.disable_positional_tracking()
    cam.close()

if __name__ == "__main__":
    main()