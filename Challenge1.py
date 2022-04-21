# Dependencies

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
from pymavlink import mavutil
import argparse  # Allows to input vals from command line to use them in python

# Functions

"""
Captures the IP address that we want to connect to
"""


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


def armDrone():
    while vehicle.is_armable == False:  # While the drone hasn't been armed
        print("Waiting for drone to become armable")
        time.sleep(1)  # Wait one second before checking if drone is armable
    print("The drone is now armable")

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':  # While drone is not in guided mode
        print("The drone is not in guided mode yet")
        time.sleep(1)  # Wait one second before checking if drone is in guided mode
    print("The drone is now in guided mode")

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

##################  MAIN    ###################


target_meters = yards_to_meters(30)
target_altitude = feet_to_meters(25)

#connect to vehicle and get the current
#vehicle = connect('127.0.0.1:14550', wait_ready=True)
print("Connecting to Drone")
vehicle = connect('/dev/ttyTHS2', wait_ready=True, baud=1500000)
print("Connected")



vehicle.parameters["WPNAV_SPEED"] = 5000 #cm/S
vehicle.parameters["OA_TYPE"] = 0 #cm/S
#arm and takeoff the drone
#head in direction that drone is facing
#move 30 yards and land

arm_and_takeoff(target_altitude)
home_location = vehicle.location.global_relative_frame
vehicle.airspeed = 4
time.sleep(1)
goto_target_body_ned(target_meters, 0, 0)

while vehicle.mode.name == "GUIDED":
    distance_traveled = get_distance_metres(vehicle.location.global_relative_frame, home_location)
    print("Distance traveled: ", distance_traveled)
    if distance_traveled >= target_meters*0.99:
        print("Target Reached")
        break
    time.sleep(1)

landDrone()
