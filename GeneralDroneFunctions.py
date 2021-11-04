from dronekit import connect, mavutil, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import argparse # Allows to input vals from command line to use them in python

def ConnectToCopter():
	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()
	
	# Gives value after --connect; the IP address
	connection_string = args.connect
	
	if not connection_string: #If the connection string is empty; none provided
		# Create a SITL drone instance instead of launching one beforehand
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string()		
	
	vehicle = connect(connection_string, wait_ready=True)
	return vehicle

def ArmAndTakeoff(targetAltitude):
	print("Copter Pre-Arm Checks")
	while not vehicle.is_armable: #Ensure autopilot is ready
		print(" Waiting for Copter to initialise...")
		time.sleep(1)
	print("Copter is Armed")

	while vehicle.gps_0.fix_type < 2: #Ensure GPS is ready
		print(" Waiting for GPS to initialise...", vehicle.gps_0.fix_type)
		time.sleep(1)
	print("Copter GPS Ready")

	print("Enabling GUIDED Mode for Copter")
	vehicle.mode = VehicleMode("GUIDED") #Copter is armed in GUIDED mode
	while vehicle.mode != "GUIDED":
		print("Drone is not in GUIDED Mode...")
		time.sleep(1)
	print("Drone is in GUIDED Mode")

	print("Arming Copter")
	vehicle.armed = True
	while not vehicle.armed: #Ensure vehicle is armed before take off
		print(" Waiting for Copter arming...")
		time.sleep(1)
	print("Drone is Armed")

"""Move copter in direction based on specified velocity vectors, velX and VelY are parallel to the North and East direction (not to the front and sie of the vehicle). velZ is perpendicular to the plane of velX and velY, with a positive value towards the ground (so up is negative) following right-hand convention

	velX > 0 -> fly North
	velX < 0 -> fly South
	velY > 0 -> fly East
	velY < 0 -> fly West
	velZ < 0 -> ascend
	velZ > 0 -> descend

	Local Tangent Plane Coorindaties Wiki -> https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
"""
def FrameVelocityControl(velX, velY, velZ, duration):
	instructions = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velX, velY, velZ, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlin

	# Send command to copter on a 1Hz cycle
	for x in range(0, duration):
		vehicle.send_mavlink(instructions)
		time.sleep(1)

"""Rotates the camera to a specific vector in space and tracks a location of interest.

	@param pitch [byte], Gimbal pitch in degrees relative to the vehicle (see diagram for attitude). A value of 0 represents a camera pointed straight ahead relative to the front of the vehicle, while -90 points the camera straight down.

	@param roll [byte] -  Gimbal roll in degrees relative to the vehicle (see diagram for attitude).

	@param yaw [byte] - Gimbal yaw in degrees relative to global frame (0 is North, 90 is West, 180 is South etc.)
	target_location(roi)
"""
def GimbalMovement(pitch, roll, yaw, locationRoi):
	#Point gimbal in desired direction
	vehicle.gimbal.rotate(pitch, roll, yaw)
	time.sleep(10)

	#Set gimbal/camera to track specified location in global relative frame
	vehicle.gimbal.target_location(locationRoi)

"""Download current challenge from the copter"""
def DownloadChallenge():
	commands = vehicle.commands
	commands.download()
	commands.wait_ready() # Wait until download is finished


"""Clears the Current mission (challenge)"""
def ClearCurrentChallenge():
	# commands = vehicle.commands
	print("Clearing current challenge/mission from vehicle")
	vehicle.commands.clear() # Clear current mission
	vehicle.flush()

"""(Unsure about how getting challange file works still) Get challenge file from copter"""
def GetCurrentChallenge(challenge):
	vehicle.mode = VehicleMode("AUTO")
	while vehicle.mode != "AUTO":
		print("Setting copter into AUTO mode...")
		time.sleep(1)
	print("Vehicle is in AUTO mode")

"""Read various info from the copter"""
def PrintTelemetry():
	vehicle.wait_ready('autopilot_version')
	print('Autopilot version: %s'%vehicle.version)

	# Does the firmware support the companion pc to set the attitude?
	print('Supports set attitude from companion: %s'%vehicle.capabilities.set_attitude_target_local_ned)

	# Get the actual position
	print('Position: %s'% vehicle.location.global_relative_frame)

	# Get the actual attitude roll, pitch, yaw
	print('Attitude: %s'% vehicle.attitude)

	# Get the actual velocity (m/s)
	print('Velocity: %s (m/s)'%vehicle.velocity) # North, east, down

	# When did we receive the last heartbeat
	print('Last Heartbeat: %s'%vehicle.last_heartbeat)

	# Is the vehicle good to Arm?
	print('Is the vehicle armable: %s'%vehicle.is_armable)

	# Which is the total ground speed?
	print('Groundspeed: %s'% vehicle.groundspeed) #(%)

	# What is the actual flight mode? 
	print('Mode: %s'% vehicle.mode.name)

	# Is thestate estimation filter ok?
	print('EKF Ok: %s'%vehicle.ekf_ok)

"""Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
    specified region of interest (LocationGlobal).
    The vehicle may also turn to face the ROI.
""" 
def SetLocationRoi(location):
    # create the MAV_CMD_DO_SET_ROI command
    locMsg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )

    # Send command to copter
    vehicle.send_mavlink(locMsg)

def LandDrone():
	print("Setting copter into LAND mode")	
	vehicle.mode = VehicleMode('LAND')
	while vehicle.mode != 'LAND':	
		time.sleep(1)
	print("Initiating landing now...")
	
	landed = False
	while landed == False: # While the drone has not landed
		currElevation = vehicle.location.global_relative_frame.alt # Get current elevation
		if currElevation <= 0.01: # If the drone has reached the ground
			landed = True # It has landed
	print("The copter has landed!")

""" Send command to request the vehicle fly to a specified
    location in the North, East, Down frame of the drone's body. So north is direction that
    drone is facing.
"""
def GoToTargetBody(north, east, down):
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

""" Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
"""
def GetDistanceInMeters(aLoc1, aLoc2):
    dlat = aLoc2.lat - aLoc1.lat
    dlong = aLoc2.lon - aLoc1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

def YardsToMeters(yards):
	return yards / 1.094

def MetersToYards(meters):
	return meters * 1.0936132983

"""
----------------------
-------- Main --------
----------------------
"""

#Connect to vehicle UDP endpoint or simulator
vehicle = connect('127.0.0.1:port', wait_Ready = True)
curMode = "ENABLED" # Set to "GROUND" when not testing
wayPointCount = 0

while True:
	if curMode == "GROUND":
		time.sleep(2)
		if wayPointCount > 0: # exampl if else for determing challenge not functional right now
			print("Valid Challange Uploaded -> Procees")
			curMode = "CHALLANGE1_TEST"
	elif curMode = "CHALLENGE1_TEST":
		ArmAndTakeoff(YardsToMeters(30))
		homeLocation = vehicle.location.global_relative_frame
		vehicle.airspeed = 4
		GoToTargetBody(FeetToMeters(25), 0, 0)

		while vehicle.mode.name == "GUIDED":
			distanceTraeled = GetDistanceMeters(vehicle.location.global_relative_frame, home_location)
			print(f"Distance traveled: {distanceTraveled}")
			if distanceTraeled >= target_meters * 0.99:
				print("Target Reached")
				break
			time.sleep(1)
		
		LandDrone()
	elif curMode == "BASIC_TEST":
		# Prep drone for flight and rise to a altidude of 15
		ArmAndTakeoff(15)

		# Rn this just switches the vehicle mode to AUTO
		GetCurrentChallenge(vehicle)

		# Fly North and up
		VelocityControl(2, 0, -0.5, 5)

		# Print various telemetry data
		PrintTelemetry()

		# Land Drone wherever it currently is at
		LandDrone()

		# Stop copter from running
		vehicle.close()