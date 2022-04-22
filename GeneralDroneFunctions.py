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

def ConnectToCopter(connection_string):
    #parser = argparse.ArgumentParser(description='commands')
    #parser.add_argument('--connect')
    #args = parser.parse_args()

    # Gives value after --connect; the IP address
    #connection_string = args.connect

    if not connection_string:  # If the connection string is empty; none provided
        # Create a SITL drone instance instead of launching one beforehand
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    vehicle = connect(connection_string, wait_ready=True)
    return vehicle

def ArmDrone(vehicle):
    print("Copter Pre-Arm Checks")
    while not vehicle.is_armable:  # Ensure autopilot is ready
        print(" Waiting for Copter to initialise...")
        time.sleep(1)
    print("Copter is Armed")

    while vehicle.gps_0.fix_type < 2:  # Ensure GPS is ready
        print(" Waiting for GPS to initialise...", vehicle.gps_0.fix_type)
        time.sleep(1)
    print("Copter GPS Ready")

    print("Enabling GUIDED Mode for Copter")
    vehicle.mode = VehicleMode("GUIDED")  # Copter is armed in GUIDED mode
    while vehicle.mode != "GUIDED":
        print("Drone is not in GUIDED Mode...")
        time.sleep(1)
    print("Drone is in GUIDED Mode")

    print("Arming Copter")
    vehicle.armed = True
    while not vehicle.armed:  # Ensure vehicle is armed before take off
        print(" Waiting for Copter arming...")
        time.sleep(1)
    print("Drone is Armed")
    
async def TakeOffDrone(vehicle, elevation):
    print("Flying up to ", elevation, "m")
    vehicle.simple_takeoff(elevation)  # Begin takeoff procedure to reach elevation

    reachedElevation = False
    while reachedElevation == False:  # While the target elevation has not been reached
        currDroneHeight = vehicle.location.global_relative_frame.alt
        print("Current drone elevation: ", currDroneHeight)

        if currDroneHeight >= (.95 * elevation):  # If the drone is at the target elevation (account for timing)
            reachedElevation = True
        await asyncio.sleep(1)
    print("Drone has reached target elevation")
    

def FrameVelocityControl(vehicle, velX, velY, velZ):
    """ Move copter in direction based on specified velocity vectors, velX and VelY are parallel to the North and East direction (not to the front and sie of the vehicle). velZ is perpendicular to the plane of velX and velY, with a positive value towards the ground (so up is negative) following right-hand convention

	velX > 0 -> fly North
	velX < 0 -> fly South
	velY > 0 -> fly East
	velY < 0 -> fly West
	velZ < 0 -> ascend
	velZ > 0 -> descend

	# Send command to copter on a 1Hz cycle
    # for x in range(0, duration):
    #    vehicle.send_mavlink(instructions)
    #    time.sleep(1)

	Local Tangent Plane Coorindaties Wiki -> https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
    """
    instructions = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velX, velY, velZ,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlin
    vehicle.send_mavlink(instructions)

def GimbalMovement(vehicle, pitch, roll, yaw, locationRoi):
    """
        Rotates the camera to a specific vector in space and tracks a location of interest.

        @param pitch [byte], Gimbal pitch in degrees relative to the vehicle (see diagram for attitude). A value of 0 represents a camera pointed straight ahead relative to the front of the vehicle, while -90 points the camera straight down.

        @param roll [byte] -  Gimbal roll in degrees relative to the vehicle (see diagram for attitude).

        @param yaw [byte] - Gimbal yaw in degrees relative to global frame (0 is North, 90 is West, 180 is South etc.)
        target_location(roi)
    """
    # Point gimbal in desired direction
    vehicle.gimbal.rotate(pitch, roll, yaw)
    time.sleep(10)

    # Set gimbal/camera to track specified location in global relative frame
    vehicle.gimbal.target_location(locationRoi)

def DownloadChallenge(vehicle):
    """Download current challenge from the copter"""
    
    commands = vehicle.commands
    commands.download()
    commands.wait_ready()  # Wait until download is finished

def ClearCurrentChallenge(vehicle):
    """Clears the Current mission (challenge)"""
    
    # commands = vehicle.commands
    print("Clearing current challenge/mission from vehicle")
    vehicle.commands.clear()  # Clear current mission
    vehicle.flush()


"""(Unsure about how getting challange file works still) Get challenge file from copter"""
def GetCurrentChallenge(vehicle, challenge):
    vehicle.mode = VehicleMode("AUTO")
    while vehicle.mode != "AUTO":
        print("Setting copter into AUTO mode...")
        time.sleep(1)
    print("Vehicle is in AUTO mode")


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
    print("Setting copter into LAND mode")
    vehicle.mode = VehicleMode('LAND')
    while vehicle.mode != 'LAND':
        time.sleep(1)
    print("Initiating landing now...")

    while vehicle.armed:  # While the drone has not landed
        currDroneHeight = vehicle.location.global_relative_frame.alt
        print("Current drone elevation: ", currDroneHeight)
        time.sleep(1)
	
    print("The copter has landed!")

async def GoToTargetBody(vehicle, north, east, down, stop_speed=0.1, timeout=20): #north is Y east is Xs
    """
        Send command to request the vehicle fly to a specified
        location in the North, East, Down frame of the drone's body. So north is direction that
        drone is facing.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	# send command to vehicle
    vehicle.send_mavlink(msg)
    await asyncio.sleep(0.5) # Wait to try and avoid reporting move completion prematurely
    start_time = time.time()
    while vehicle.groundspeed <= stop_speed and (time.time() - start_time) < timeout:
        # print(f"Vehicle knows it is at {vehicle.location.global_frame}")
        print(f"Vehicle Ground Speed: {vehicle.groundspeed} Vehicle Stop Speed: {stop_speed}")
        # logging.getLogger(__name__).debug(f"Vehicle knows it is at {vehicle.location.global_frame}")
        await asyncio.sleep(0.25)

    while vehicle.groundspeed > stop_speed and (time.time() - start_time) < timeout:
        # print(f"Vehicle knows it is at {vehicle.location.global_frame}")
        print(f"Vehicle Ground Speed: {vehicle.groundspeed} Vehicle Stop Speed: {stop_speed}")
        # logging.getLogger(__name__).debug(f"Vehicle knows it is at {vehicle.location.global_frame}")
        await asyncio.sleep(0.1)

    if (time.time() - start_time) > timeout:
        print(f"Vehicle move timeout at {time.time()}")
        logger.critical(f"Vehicle move timeout at {time.time()}")

async def GoToGlobal(vehicle: Vehicle, coords, alt=7.62, stop_speed=0.1, stop_distance=1, time_out=20):
    coords = np.asarray(coords).flatten()
    logger.debug(f"Going to {coords}")
    print("Going to global coords {coords}")
    # msg = vehicle.message_factory.set_position_target_global_int_encode(
    #     0,  # time_boot_ms (not used)
    #     0, 0,  # target system, target component
    #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
    #     0b0000111111111000 ,  # type_mask (only positions enabled)
    #     int(coords[0]*10_000_000), int(coords[0]*10_000_000), alt,
    #     0, 0, 0,  # x, y, z velocity in m/s  (not used)
    #     0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
    #     0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # # msg = vehicle.message_factory.set_position_target_local_ned_encode(

	# # send command to vehicle
    # vehicle.send_mavlink(msg)
    vehicle.simple_goto(LocationGlobalRelative(lat=coords[0], lon=coords[1], alt=alt))
    # while vehicle.groundspeed <= stop_speed:
    #     # print(f"Vehicle knows it is at {vehicle.location.global_frame}")
    #     # logging.getLogger(__name__).debug(f"Vehicle knows it is at {vehicle.location.global_frame}")
    #     await asyncio.sleep(0.25)

    # while vehicle.groundspeed > stop_speed:
    #     # print(f"Vehicle knows it is at {vehicle.location.global_frame}")
    #     # logging.getLogger(__name__).debug(f"Vehicle knows it is at {vehicle.location.global_frame}")
    #     await asyncio.sleep(0.1)

    tx, ty, *_ = utm.from_latlon(coords[0], coords[1])
    time_start = time.time()
    while True:
        cx, cy, *_ =  utm.from_latlon(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
        if np.linalg.norm(np.array([tx-cx , ty-cy, alt - vehicle.location.global_relative_frame.alt])) > stop_distance or vehicle.groundspeed > stop_speed:
            logging.getLogger(__name__).debug("Waiting for global move to complete")
            await asyncio.sleep(0.25)
        else:
            break

        if time.time() - time_start > time_out:
            logging.getLogger(__name__).critical(f"Global Move Exceeded Time Out of {time_out}s ")

async def FollowGlobalPath(vehicle: Vehicle, coords, **kwargs):
    logger.debug("Starting path follow")
    for c in coords:
        await GoToGlobal(vehicle, c, **kwargs)

def MoveRelative(vehicle, pos):
    """
        Send command to request the vehicle fly to a specified
        location in the North, East, Down frame of the drone's body. So north is direction that
        drone is facing.
    """
    pos = np.asarray(pos).flatten()
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b110111111000,  # type_mask (only positions enabled)
        pos[0], pos[1], pos[2],
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	# send command to vehicle
    vehicle.send_mavlink(msg)

def IsMoving(vehicle, speed=0.1):
    return vehicle.groundspeed < speed
    
def IsCloseEnough(vehicle, target, distance=1):
    cx, cy, *_ =  utm.from_latlon(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    tx, ty, *_ = utm.from_latlon(target[0], target[1])
    return math.sqrt((tx-cx)**2 + (ty-cy)**2) <= distance

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
    """ The vehicle “yaw” is the direction that the vehicle is facing in the horizontal plane. On Copter this yaw need not be the direction of travel (though it is by default).
    """
    if relative:
        relativeToDirOfTravel = 1 # yaw relative to direction of travel
    else:
        relativeToDirOfTravel = 0 # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        speed,          # param 2, yaw speed deg/s
        1 if heading >= 0 else -1,          # param 3, direction -1 ccw, 1 cw
        relativeToDirOfTravel, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def UpdateLandingTargetPosition(vehicle: Vehicle, x, y, z):
    # msg = vehicle.message_factory.landing_target_encode(
    #     0,          # time target data was processed, as close to sensor capture as possible
    #     1,          # target num, not used
    #     mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
    #     -x * math.pi / 180.0,          # X-axis angular offset, in radians
    #     -y * math.pi / 180.0,          # Y-axis angular offset, in radians
    #     z,          # distance, in meters
    #     0,          # Target x-axis size, in radians
    #     0,          # Target y-axis size, in radians
    #     0,          # x	float	X Position of the landing target on MAV_FRAME
    #     0,          # y	float	Y Position of the landing target on MAV_FRAME
    #     0,          # z	float	Z Position of the landing target on MAV_FRAME
    #     (1,0,0,0),  # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    #     2,          # type of landing target: 2 = Fiducial marker
    #     1,          # position_valid boolean
    # )
    msg = vehicle.message_factory.landing_target_encode(
        0,          # time since system boot, not used
        1,          # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
        # (x-horizontal_resolution/2)*horizontal_fov/horizontal_resolution,
        # (y-vertical_resolution/2)*vertical_fov/vertical_resolution,
        -x * math.pi / 180.0,          # X-axis angular offset, in radians
        -y * math.pi / 180.0,          # Y-axis angular offset, in radians
        z,          # distance, in meters
        0,          # Target x-axis size, in radians
        0           # Target y-axis size, in radians
    )
    vehicle.send_mavlink(msg)
    vehicle.message_factory

def StartPrecisionLanding(vehicle):
    vehicle.parameters['PLND_ENABLED'] = 1 # Enable precision landing
    vehicle.parameters['PLND_TYPE'] = 1 # Optical fiducial tracking
    #vehicle.parameters['ANGLE_MAX'] = 2.5*1000 # Angle in centidegress
    vehicle.mode = VehicleMode("LAND")

def Stop(vehicle):
    vehicle.mode = VehicleMode("BRAKE")
    
def SetGuided(vehicle):
    vehicle.mode = VehicleMode("GUIDED")

def SetROI(loc):
    """ Set ROI command to point camer gimbal at a specified region of interest, drone must also turn to face ROI
    """
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        loc.lat,
        loc.lon,
        loc.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
	
def ServoMovement(vehicle, position):
    servo_min = 775
    servo_max = 1775
    if(position <= 0):
        pwm = servo_min
    elif (position >= 90):
        pwm = servo_max
    # elif(22 <= position  <= 25):
    #     pwm = 1300
    # elif(position == 35):
    #     pwm = 1397
    # else:
    #     pwm = 1512
    else:   
        #range / 90
        calcPwm = float(((servo_max-servo_min)/90) * position)
        newPwm = int(calcPwm) + servo_min
        pwm = newPwm

    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
        0, #confirmation
        9, pwm , 0, 0, #params 1-4
        0,
        0,
        0
        )
    vehicle.send_mavlink(msg)



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
