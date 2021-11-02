from dronekit import connect, mavutil, VehicleMode, LocationGlobalRelative, APIException
from geographiclib.geodesic import Geodesic
import time
import socket
import math
import argparse # Allows to input vals from command line to use them in python

def ChangeMode(mode):
    print(f"Enabling {mode} Mode for Copter")
    while vehicle.mode != VehicleMode(mode):
            vehicle.mode = VehicleMode(mode)
            time.sleep(1)
    print(f"Drone is in {mode} Mode")
    return True

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

def ArmCopter():
	print("Copter Pre-Arm Checks")
	while not vehicle.is_armable: #Ensure autopilot is ready
		print(" Waiting for Copter to initialise...")
		time.sleep(1)
	print("Copter is Armed")

	while vehicle.gps_0.fix_type < 2: #Ensure GPS is ready
		print(" Waiting for GPS to initialise...", vehicle.gps_0.fix_type)
		time.sleep(1)
	print("Copter GPS Ready")

	ChangeMode("GUIDED")

	print("Arming Copter")
	vehicle.armed = True
	while not vehicle.armed: #Ensure vehicle is armed before take off
		print(" Waiting for Copter arming...")
		time.sleep(1)
	print("Drone is Armed")
 
def ElevateCopter(targetAltitude):
	print("Flying up to %s meters" % targetAltitude)
	vehicle.simple_takeoff(targetAltitude) # Begin takeoff procedure to reach elevation
 
	# Wait to reach the target altitude
	while True:
		altitude = vehicle.location.global_relative_frame.alt
		if altitude >= targetAltitude -1:
			print("Altitude of %sm reached", targetAltitude)
 			break
        time.sleep(1)
        print("Drone has reached target elevation")

""" Move copter in direction based on specified velocity vectors, velX and VelY are parallel to the North and East direction 	(not to the front and sie of the vehicle). velZ is perpendicular to the plane of velX and velY, with a positive value towards the ground (so up is negative) following right-hand convention

	velX > 0 -> fly North
	velX < 0 -> fly South
	velY > 0 -> fly East
	velY < 0 -> fly West
	velZ < 0 -> ascend
	velZ > 0 -> descend

	Local Tangent Plane Coorindaties Wiki -> https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
"""
def VelocityControl(velX, velY, velZ, duration):
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
	vehicle.flush()

""" Rotates the camera to a specific vector in space and tracks a location of interest.

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

"""Download current challenge from the copter (Not Working unsure about how challnege files are grabbed as a mission)
"""
def DownloadChallenge():
	commands = vehicle.commands
	commands.download()
	commands.wait_ready() # Wait until download is finished

"""Clears the Current mission (challenge) (Not Working unsure about how challnege files are grabbed as a mission)
"""
def ClearCurrentChallenge():
	# commands = vehicle.commands
	print("Clearing current challenge/mission from vehicle")
	vehicle.commands.clear() # Clear current mission
	vehicle.flush()
 
	# After clearing you must re-download the mission from vehicle to enable vehicle.commands again (might be a depreciated bug)
	DownloadChallenge()

"""(Unsure about how getting challange file works still) It's supposed to Get challenge file from copter
	but right now it just sets the copter to AUTO mode
"""
def GetCurrentChallenge(challenge):
	ChangeMode("AUTO")

"""Read various info from the copter
"""
def PrintTelemetry():
	vehicle.wait_ready('autopilot_version')
	print("Autopilot version: %s" %vehicle.version)

	# Does the firmware support the companion pc to set the attitude?
	print("Supports set attitude from companion: %s" %vehicle.capabilities.set_attitude_target_local_ned)

	# Get the actual position
	print("Position: %s" % vehicle.location.global_relative_frame)

	# Get the actual attitude roll, pitch, yaw
	print("Attitude: %s" % vehicle.attitude)

	# Get the actual velocity (m/s)
	print("Velocity: %s (m/s)" % vehicle.velocity) # North, east, down

	# When did we receive the last heartbeat
	print("Last Heartbeat: %s" % vehicle.last_heartbeat)

	# Is the vehicle good to Arm?
	print("Is the vehicle armable: %s" % vehicle.is_armable)

	# What is the total ground speed?
	print("Groundspeed: %s" % vehicle.groundspeed) #(%)

	# What is the actual flight mode? 
	print("Mode: %s" % vehicle.mode.name)

	# Is the state estimation filter ok?
	print("EKF Ok: %s" %vehicle.ekf_ok)
 
	# What is the maximum throttle
 	print("Maximum Throttle: %d" % vehicle.parameters['THR_MIN']) 

""" Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
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

""" Point vehicle at a specified heading (in degrees). Sets an absolute heading by default, but you can 	set the relative param 		to "True" to set the yaw relative to the current yaw heading.

	More Info:
 	(https://ardupilot.org/copter/docs/mission-command-list.html#condition-yaw)
    
    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
"""
def ConditionYaw(heading, relative = False):
    if relative:
        isRelative = 1 # yaw is relative to direction of travel
    else:
        isRelative = 0 # yaw is an absolute angle
        
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,        # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0,           # confirmation
        heading,     # param 1, yaw in degrees
        0,           # param 2, yaw speed deg/s
        1,           # param 3, direction -1 ccw, 1 cw
        isRelative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)     # param 5 ~ 7 not used
    
    # send command to vehicle
    vehicle.send_mavlink(msg)

"""Aproximation of the bearing for medium latitudes and short distances
"""
def GetBearing(lat1, lat2, long1, long2):
    brng = Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['azi1']
    return brng

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
 
#Connect to vehicle UDP endpoint or simulator
vehicle = connect('127.0.0.1:14550', wait_Ready = True) # IP Address is random?

"""
----------------------
-------- Main --------
----------------------
"""

""" Main Function for Testing
curMode = "ENABLED" # Set to "GROUND" when not testing

while True:
	if curMode == "GROUND": # Do some prep and switch to "ENABLED", this is mainly used to get the next "mission" or challenge file to run and prep the copter before actually running the script
		time.sleep(2)
		curMode = "ENABLED"
	elif curMode == "ENABLED":
		#Connec to copter
		ConnectToCopter()
  
		# Prep drone for flight
		ArmCopter()
  
		# Rise drone to set altitude
		ElevateCopter(15)
  
		# Set the default speed
		vehicle.airspeed = 5;

		# Right now this just switches the vehicle mode to AUTO
		GetCurrentChallenge(vehicle)

		# Fly North and up for a duration of 5s
		VelocityControl(2, 0, -0.5, 5)

		# Print various telemetry data
		PrintTelemetry()

		# Land Drone wherever it currently is at
		LandDrone()

		# Stop copter from running
		vehicle.close() 
"""