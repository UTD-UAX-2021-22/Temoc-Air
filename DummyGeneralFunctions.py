"""
	Please stick to the syntax conventions when adding new functions
	Uncomment the main at the bottom to test anything
"""
from asyncio.log import logger
from dronekit import connect, mavutil, VehicleMode, LocationGlobalRelative, APIException, Vehicle
import time
import socket
import math
import argparse  # Allows to input vals from command line to use them in python
import asyncio
import logging
import utm

import numpy as np

from Utils import DummyVehicle
logger = logging.getLogger(__name__)

def ConnectToCopter(connection_string):
    logger.debug(f"Attempted connect to vehicle at {connection_string}")
    return DummyVehicle()

def ArmDrone(vehicle):
    logger.debug(f"Attempted to arm vehicle. Sleeping 5s to simulate")
    time.sleep(1)
    
async def TakeOffDrone(vehicle, elevation):
    logger.debug(f"Attempted takeoff to height of {elevation}. Sleeping 4s to simulate.")
    await asyncio.sleep(1)
    

def FrameVelocityControl(vehicle, velX, velY, velZ):
    logger.debug(f"Attempted velocity command of {velX} {velY} {velZ}")

def GimbalMovement(vehicle, pitch, roll, yaw, locationRoi):
    logger.debug(f"Attempted gimbal movement to {pitch} {roll} {yaw} {locationRoi} -- Sleeping 4s")
    time.sleep(1)

def DownloadChallenge(vehicle):
    logger.debug("Attempted command download. Sleeping 5s.")
    time.sleep(1)

def ClearCurrentChallenge(vehicle):
    logger.debug("Attempted t clear challenge. Sleeping 3s.")
    time.sleep(1)


"""(Unsure about how getting challange file works still) Get challenge file from copter"""
def GetCurrentChallenge(vehicle, challenge):
    logger.debug("Attempted to get current challenge. sleeping 3s.")
    time.sleep(1)


def PrintTelemetry(vehicle):
    """Read various info from the copter"""
    
    vehicle.wait_ready('autopilot_version')
    print(f"Autopilot version: {vehicle.version}")

    # Does the firmware support the companion pc to set the attitude?
    print(f"Supports set attitude from companion: {vehicle.capabilities.set_attitude_target_local_ned}")

    # Get the actual position
    print(f"Position: {vehicle.location.global_relative_frame}")

    # Get the actual attitude roll, pitch, yaw
    print(f"Attitude: {vehicle.attitude}");

    # Get the actual velocity (m/s)
    print(f"Velocity: {vehicle.velocity} (m/s)")  # North, east, down

    # When did we receive the last heartbeat
    print(f"Last Heartbeat: {vehicle.last_heartbeat}")

    # Is the vehicle good to Arm?
    print(f"Is the vehicle armable: {vehicle.is_armable}")

    # Which is the total ground speed?
    print(f"Groundspeed: {vehicle.groundspeed}")

    # What is the actual flight mode?
    print(f"Mode: {vehicle.mode.name}")

    # Is thestate estimation filter ok?
    print(f"EKF Ok: {vehicle.ekf_ok}")

def LandDrone(vehicle):
    logger.debug("Attempted landing. Sleeping 10s")
    time.sleep(1)

async def GoToTargetBody(vehicle, north, east, down, stop_speed=0.1, timeout=10):
    logger.debug(f"Attempted go to target body {north} {east} {down} {stop_speed} {timeout} -- Sleeping 4s")
    await asyncio.sleep(1)

async def GoToGlobal(vehicle: Vehicle, coords, alt=7.62, stop_speed=0.1, stop_distance=1, time_out=20):
    logger.debug(f"Attempted go to global {list(coords)} {stop_speed} {stop_distance} {time_out} -- Sleeping 4s")
    await asyncio.sleep(4)

def MoveRelative(vehicle, pos):
    logger.debug(f"Attempted move relative {list(pos)} -- Sleeping 4s")
    time.sleep(1)


def GetDistanceInMeters(vehicle, aLoc1, aLoc2):
    """ Returns the ground distance in metres between two LocationGlobal objects.
        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLoc2.lat - aLoc1.lat
    dlong = aLoc2.lon - aLoc1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def distance(lat1, lon1, lat2, lon2):
    p = math.pi/180
    a = 0.5 - math.cos((lat2-lat1)*p)/2 + math.cos(lat1*p) * math.cos(lat2*p) * (1-math.cos((lon2-lon1)*p))/2
    return 12742 * math.asin(math.sqrt(a)) #2*R*asin...

def YardsToMeters(yards):
	return yards * 0.9144

def FeetToMeters(feet):
	return feet * 0.3048

def SetConditionYaw(vehicle, heading, relative = False, speed = 60):
    logger.debug(f"Attempted set condition yaw {heading} {relative} {speed}")

def UpdateLandingTargetPosition(vehicle: Vehicle, x, y, z):
    logger.debug(f"Attempted update landing target position {x} {y} {z}")

def StartPrecisionLanding(vehicle):
    logger.debug("Attempted start precision land")

def Stop(vehicle):
    logger.debug("Attempted to stop vehicle")

def SetROI(loc):
   logger.debug(f"Attempted set ROI {loc}")
	
def ServoMovement(vehicle, position):
    logger.debug(f"Attempted set servo position {position}")

def SetGuided(vehicle):
    logger.debug(f"Attempted set guided")

async def FollowGlobalPath(vehicle: Vehicle, coords, **kwargs):
    logger.debug(f"(sleeping 10s) Attempted follow path {coords} with kwargs {kwargs}")
    for c in coords:
        await GoToGlobal(vehicle, c)
        
def IsMoving(vehicle):
    return False
    
def IsCloseEnough(vehicle, target, distance=1):
    return True


"""
----------------------
-------- Main --------
----------------------

# Connect to vehicle UDP endpoint for simulator
# vehicle = connect('127.0.0.1:14550', wait_ready=True)
# Connect to vehicle over com port serial
vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)

# TODO: add function or code here that asks for user input on what challenge to run 
# EX: print "1. Test" , "2. Challenge 1" , "3.." etc. for all challenge functions
# Ask user which one of these to run, get their input and set it as curMode with a switch statement or whatever

curMode = "CHALLENGE1_TEST" # Set to "GROUND" when not testing
wayPointCount = 0

#while True:
if curMode == "GROUND":
	time.sleep(2)
	if wayPointCount > 0: # exampl if else for determing challenge not functional right now
		print("Valid Challenge Uploaded -> Procees")
		curMode = "CHALLENGE1_TEST"
elif curMode == "CHALLENGE1_TEST":
	targetMeters = YardsToMeters(30)
	targetAltitude = FeetToMeters(15)
    
	ArmDrone()
	TakeOffDrone(targetAltitude)
	homeLocation = vehicle.location.global_relative_frame
	vehicle.airspeed = 4

	#Fly North for 10 seconds
	FrameVelocityControl(targetMeters, 0, 0)

	GoToTargetBody(targetMeters, 0, 0)
    
	while vehicle.mode.name == "GUIDED":
		distanceTraveled = GetDistanceInMeters(vehicle.location.global_relative_frame, homeLocation)
		print(f"Distance traveled: {distanceTraveled}")
		if distanceTraveled >= YardsToMeters(30) * 0.99:
			print("Target Reached")
			print(f"Final distance traveled: {distanceTraveled}")
			break
		time.sleep(1)

	LandDrone()
	vehicle.close() #stop copter from running
elif curMode == "BASIC_TEST":
	# Prep drone for flight and rise to a altidude of 15
	ArmDrone()
	TakeOffDrone(15)

	# Rn this just switches the vehicle mode to AUTO
	#GetCurrentChallenge(vehicle)

	# Fly North and up
	FrameVelocityControl(2, 0, -0.5)

	# Print various telemetry data
	PrintTelemetry()

	# Land Drone wherever it currently is at
	LandDrone()

	# Stop copter from running
	vehicle.close()
"""
