# Dependencies

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
from pymavlink import mavutil
import argparse  # Allows to input vals from command line to use them in python
import numpy as np
import threading
import sys
import pyzed.sl as sl
from ctypes import *
import cv2
dummyDrone = True # Set to True to bench test and not connect to real drone, False for actual flights
if dummyDrone == True:
    import src.challenge_2.DummyGeneralFunctions as gd    
else:
    import src.challenge_2.GeneralDroneFunctions as gd 


current_challenge = 4

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect  # Gives value after --connect; the IP address

    if not connection_string:  # If the connection string is empty; none provided
        # Create a SITL drone instance instead of launching one beforehand
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    vehicle = connect(connection_string, wait_ready=True)
    return vehicle

##############################################
def armDrone():
    while vehicle.is_armable == False:  # While the drone hasn't been armed
        print("Waiting for drone to become armable")
        time.sleep(1)  # Wait one second before checking if drone is armable
    print("The drone is now armable")

    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    while vehicle.mode != 'GUIDED_NOGPS':  # While drone is not in guided mode
        print("The drone is not in guided nogps mode yet")
        time.sleep(1)  # Wait one second before checking if drone is in guided mode
    print("The drone is now in guided nogps mode")

    vehicle.armed = True
    while vehicle.armed == False:  # While the vehicle has not been armed
        print("Waiting for drone to arm")
        time.sleep(1)  # Wait one second before checking if drone has been armed
    print("The drone has now been armed")

def arm_and_takeoff(elevation):
    armDrone()  # Arm the drone

    print("Flying up to ", elevation, "m")
    vehicle.simple_takeoff(elevation)  # Begin takeoff procedure to reach elevation

    reachedElevation = False
    while reachedElevation == False:  # While the target elevation has not been reached
        currDroneHeight = vehicle.location.global_relative_frame.alt
        print("Current drone elevation: ", currDroneHeight)

        if currDroneHeight >= (.95 * elevation):  # If the drone is at the target elevation (account for timing)
            reachedElevation = True
        time.sleep(1)
    print("Drone has reached target elevation")

def stayInAir(seconds):
    print("Staying in the air for ", seconds, " seconds")
    counter = 0
    while counter < seconds:
        print("Relaxing up high")
        time.sleep(1)
        counter += 1


def landDrone():
    vehicle.mode = VehicleMode('LAND')
    while vehicle.mode != 'LAND':  # While drone is not in land-mode
        time.sleep(1)  # Wait one second before checking if drone is in land-mode
    print("Okay, initiating landing now...")

    landed = False
    while landed == False:  # While the drone has not landed
        currElevation = vehicle.location.global_relative_frame.alt  # Get current elevation
        if currElevation <= 0.01:  # If the drone has reached the ground
            landed = True  # It has landed
    print("The drone has landed!")

def body_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    X will move the drone forward and backwards
    y will move the drone left and right
    z is up and down
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def goto_target_body_ned(north, east, down):
    """
    Send command to request the vehicle fly to a specified
    location in the North, East, Down frame of the drone's body. So north is direction that
    drone is facing.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def yards_to_meters(yards):
    meters = yards * 0.9144
    return meters

def feet_to_meters(feet):
    meters = feet * 0.3048
    return meters

# Function that transorms the position coordinate of the left eye of the camera to the
# position at the center of the camera
def transform_pose(pose, tx) :
  transform_ = sl.Transform()
  transform_.set_identity()
  # Translate the tracking frame by tx along the X axis
  transform_[0][3] = tx
  # Pose(new reference frame) = M.inverse() * pose (camera frame) * M, where M is the transform between the two frames
  transform_inv = sl.Transform()
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


##################  MAIN    ###################
target_meters = yards_to_meters(10)
target_altitude = feet_to_meters(3)
field_width = yards_to_meters(9)

#connect to vehicle and get the current
#vehicle = connect('127.0.0.1:14550', wait_ready=True)
vehicle = connect('/dev/ttyTHS2', wait_ready=True, baud=1500000)
print("Connected to Drone")

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

 #ZED SDK Parameters 
cam = sl.Camera()
init_parameters = sl.InitParameters()
init_parameters.camera_resolution = sl.RESOLUTION.HD720
init_parameters.camera_fps = 60
init_parameters.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_parameters.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
init_parameters.coordinate_units = sl.UNIT.METER

#Opening camera
if not cam.is_opened():
  print("Opening ZED Camera...")
  status = cam.open(init_parameters)
  if status != sl.ERROR_CODE.SUCCESS:
      print(repr(status))
      cam.close()
      exit()
  
print("Camera Opened")

# Declare your sl.Mat matrices
image_zed = sl.Mat()
depth_image_zed = sl.Mat()
point_cloud = sl.Mat()

#gd.ServoMovement(vehicle, 90)

if (current_challenge == 3):
    home_location = vehicle.location.global_relative_frame
    half_field = (field_width/2) - home_location.lat

    distance_traveled = 0
    currentLocation = home_location

    arm_and_takeoff(target_altitude)
    vehicle.airspeed = 0.5
    goto_target_body_ned(target_meters, 0, 0)

    while vehicle.mode.name == "GUIDED_NOGPS":
        
        currentLocation = vehicle.location.global_relative_frame
        distance_traveled = get_distance_metres(currentLocation, home_location)
        print("Distance traveled: ", distance_traveled)

        if ((currentLocation.alt > target_altitude + 0.5) | (currentLocation.alt < target_altitude - 0.5)):
            diff_in_altitude = currentLocation.alt - target_altitude
            goto_target_body_ned(0, 0, diff_in_altitude)
            
            reachedElevation = False
            while reachedElevation == False:  # While the target elevation has not been reached
                currentAltLocation = vehicle.location.global_relative_frame
                currDroneHeight = currentAltLocation.alt
                print("Current drone elevation: ", currDroneHeight)

                if currDroneHeight >= (.95 * elevation):  # If the drone is at the target elevation (account for timing)
                    reachedElevation = True
                time.sleep(1)
                
            goto_target_body_ned(target_meters - distance_traveled, 0, 0)

        if ((currentLocation.lat > half_field -2) | (currentLocation.lat < half_field + 2)):
            diff_left_and_right = home_location.lat - currentLocation.lat #calculates distance from the center of start position
            goto_target_body_ned(target_meters - distance_traveled, diff_left_and_right, 0) 

        if distance_traveled >= target_meters*0.99:
            print("Target Reached")
            break

        time.sleep(1)



elif (current_challenge == 4):

    # Enable positional tracking with default parameters
    tracking_parameters = sl.PositionalTrackingParameters()
    err = cam.enable_positional_tracking(tracking_parameters)

    # Set sensing mode in FILL
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD

    translation_left_to_center = cam.get_camera_information().calibration_parameters.T[0]

    cam_pose = sl.Pose()

    # Retrieve and transform the pose data into a new frame located at the center of the camera
    tracking_state = cam.get_position(cam_pose, sl.REFERENCE_FRAME.WORLD)
    transform_pose(cam_pose.pose_data(sl.Transform()), translation_left_to_center)

    py_translation = sl.Translation()
    tx = round(cam_pose.get_translation(py_translation).get()[0], 3)
    ty = round(cam_pose.get_translation(py_translation).get()[1], 3)
    tz = round(cam_pose.get_translation(py_translation).get()[2], 3)

    home_location = Location(tx, ty, tz)
    half_field = (field_width/2) - home_location.lat

    distance_traveled = 0
    currentLocation = home_location

    arm_and_takeoff(target_altitude)
    vehicle.airspeed = 0.5
    goto_target_body_ned(target_meters, 0, 0)

    while vehicle.mode.name == "GUIDED_NOGPS":
        
        tracking_state = cam.get_position(cam_pose, sl.REFERENCE_FRAME.WORLD)
        transform_pose(tracking_state.pose_data(sl.Transform()), translation_left_to_center)
        tx = round(cam_pose.get_translation(py_translation).get()[0], 3)
        ty = round(cam_pose.get_translation(py_translation).get()[1], 3)
        tz = round(cam_pose.get_translation(py_translation).get()[2], 3)

        currentLocation.set(tx, ty, tz)
        distance_traveled = get_distance_metres(currentLocation, home_location)
        print("Distance traveled: ", distance_traveled)

        if ((currentLocation.alt > target_altitude + 0.5) | (currentLocation.alt < target_altitude - 0.5)):
            diff_in_altitude = currentLocation.alt - target_altitude
            goto_target_body_ned(0, 0, diff_in_altitude)
            
            reachedElevation = Falsefse
            while reachedElevation == False:  # While the target elevation has not been reached
                tracking_state = cam.get_position(cam_pose, sl.REFERENCE_FRAME.WORLD)
                transform_pose(cam_pose.pose_data(sl.Transform()), translation_left_to_center)
                currDroneHeight = round(cam_pose.get_translation(py_translation).get()[2], 3)
                print("Current drone elevation: ", currDroneHeight)

                if currDroneHeight >= (.95 * elevation):  # If the drone is at the target elevation (account for timing)
                    reachedElevation = True
                time.sleep(1)
                
            goto_target_body_ned(target_meters - distance_traveled, 0, 0)

        if ((currentLocation.lat > half_field -2) | (currentLocation.lat < half_field + 2)):
            diff_left_and_right = home_location.lat - currentLocation.lat #calculates distance from the center of start position
            goto_target_body_ned(target_meters - distance_traveled, diff_left_and_right, 0)

        if distance_traveled >= target_meters*0.99:
            print("Target Reached")
            break

        time.sleep(1)



landDrone()
vehicle.close() #stop copter from running
zed.disable_positional_tracking()
zed.close()
