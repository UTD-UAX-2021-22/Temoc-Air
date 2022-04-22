#!/usr/bin/env python3.9
'''
    Cmd Line Order (Can most likely put implement this as a function that auto runs commands)
    Terminal #1:
        source devel/setup.bash
        rosrun mavros mavsyst mode -c 5 (might not need to do this if the flight mode change works as intended)
        roscore

    Terminal #2:
	roslaunch zed_wrapper zed2.launch
        
    Terminal #3:
        source devel/setup.bash (only need to do this once after can run bottom cmd multiple times)
        rosrun challenge4 c4_mavlink.py
        
    Terminal #4:
        source devel/setup.bash
        roslaunch mavros apm.launch fcu_url:="/dev/ttyTHS2:1500000"
        
    Terminal #:
        source devel/setup.bash (only need to do this once after can run bottom cmd multiple times)
        rosrun challenge4 c4_distance.py
'''

import sys
import os
os.environ["MAVLINK20"] = "2"

import numpy as np
import math as m
import sys
import time
import argparse
import threading
import rospy

from dronekit import connect
from time import sleep
from apscheduler.schedulers.background import BackgroundScheduler
from pymavlink import mavutil
from sensor_msgs.msg import LaserScan, PointCloud
from transformations import transformations
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Connection string for the FCU
CONNECTION_DEFAULT_STRING = "/dev/ttyTHS2" #'127.0.0.1:14855'

# Ideal baudrate for Mission Planner
CONNECTION_DEFAULT_BAUD = 115200

# Enable/disable each message/function individually
enable_3D_msg_obstacle_distance = False
enable_vision_position_estimate = False
obstacle_distance_msg_hz_default = 15.0 # This needs to be tuned
curr_avoid_strategy = "bendyruler"
prev_avoid_strategy = ""

# Enable Arducopter 4.1 or higher
AC_VERSION_41 = True

# lock for thread synchronization
lock = threading.Lock()

# Default exit code is failure
exit_code = 1

# Data variables
mavlink_thread_should_exit = False
vehicle_pitch_rad = None
cur_time_in_us = 0
start_time =  int(round(time.time() * 1000))
current_milli_time = lambda: int(round(time.time() * 1000) - start_time)
cur_time_in_ms = current_milli_time()
print(cur_time_in_ms)
cur_time_in_ms = current_milli_time()
last_obstacle_distance_sent_ms = 0  # value of cur_time_in_us when obstacle_distance last sent

# Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
# Link: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
distances_array_length = 72
angle_offset = None
increment_f = None 
distances = np.ones((distances_array_length,), dtype=np.uint16) * (2000 + 1)

# Obstacle distances in nine segments for the new OBSTACLE_DISTANCE_3D message
mavlink_obstacle_coordinates = np.ones((9,3), dtype = float) * (9999)
dist_debug = np.ones((9), dtype = float)
debug_enable = 1

parser = argparse.ArgumentParser(description='Reboots vehicle')

# Parses user input for the connection
parser.add_argument('--connect',
                        help="Vehicle connection target string. If not specified, a default string will be used.")
parser.add_argument('--baudrate', type=float,
                    help="Vehicle connection baudrate. If not specified, a default value will be used.")
parser.add_argument('--obstacle_distance_msg_hz', type=float,
                    help="Update frequency for OBSTACLE_DISTANCE message. If not specified, a default value will be used.")
parser.add_argument('--debug_enable',type=float,
                    help="Enable debugging information")
parser.add_argument('--camera_name', type=str,
                    help="Camera name to be connected to. If not specified, any valid camera will be connected to randomly. For eg: type 'D435I' to look for Intel RealSense D435I.")  
args = parser.parse_args()

connection_string = args.connect
connection_baudrate = args.baudrate
obstacle_distance_msg_hz = args.obstacle_distance_msg_hz
debug_enable = args.debug_enable

def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()

# Using default values if no specified inputs
if not connection_string:
    connection_string = CONNECTION_DEFAULT_STRING
    progress("INFO: Using default connection_string %s" % connection_string)
else:
    progress("INFO: Using connection_string %s" % connection_string)

if not connection_baudrate:
    connection_baudrate = CONNECTION_DEFAULT_BAUD
    progress("INFO: Using default connection_baudrate %s" % connection_baudrate)
else:
    progress("INFO: Using connection_baudrate %s" % connection_baudrate)
    
if not obstacle_distance_msg_hz:
    obstacle_distance_msg_hz = obstacle_distance_msg_hz_default
    progress("INFO: Using default obstacle_distance_msg_hz %s" % obstacle_distance_msg_hz)
else:
    progress("INFO: Using obstacle_distance_msg_hz %s" % obstacle_distance_msg_hz)

###  MAVLink Functions ###

def mavlink_loop(conn, callbacks):
    '''
        A main routine for a thread; reads data from a mavlink connection,
        calling callbacks based on message type received.
    '''
    interesting_messages = list(callbacks.keys())
    while not mavlink_thread_should_exit:
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                0,
                                0,
                                0)
        m = conn.recv_match(type=interesting_messages, timeout=1, blocking=True)
        if m is None:
            continue
        callbacks[m.get_type()](m)

# Prepare for Arducopter 4.1 3D Obstacle Avoidance
def send_obstacle_distance_3D_message():
    global mavlink_obstacle_coordinates, min_depth_cm, max_depth_cm
    global last_obstacle_distance_sent_ms
    global cur_time_in_ms
    if (enable_3D_msg_obstacle_distance == True):
        # if cur_time_in_ms == last_obstacle_distance_sent_ms:
        #     # no new frame
        #     progress("no new frame")
        #     return
        # last_obstacle_distance_sent_ms = cur_time_in_ms

        for q in range(1, 7, 3):
            # send 9 sector array but only the 3 middle sectors
            conn.mav.obstacle_distance_3d_send(
                cur_time_in_ms,    # ms Timestamp (UNIX time or time since system boot)
                0,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                65535,
                float(mavlink_obstacle_coordinates[q][0]),
                float(mavlink_obstacle_coordinates[q][1]),
                float(mavlink_obstacle_coordinates[q][2]),
                float(min_depth_cm / 100),
                float(max_depth_cm / 100) #needs to be in meters
            )
        cur_time_in_ms = current_milli_time()

def send_msg_to_gcs(text_to_be_sent):
    # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
    text_msg = 'ROS2Mav: ' + text_to_be_sent
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    progress("INFO: %s" % text_to_be_sent)

# Listen to ATTITUDE data: https://mavlink.io/en/messages/common.html#ATTITUDE
def att_msg_callback(value):
    global vehicle_pitch_rad
    vehicle_pitch_rad = value.pitch
    if debug_enable == 1:
        progress("INFO: Received ATTITUDE msg, current pitch is %.2f degrees" % (m.degrees(vehicle_pitch_rad),))

# Listen to flightmode data: https://mavlink.io/en/messages/common.html#HEARTBEAT
def fltmode_msg_callback(value):
    global curr_avoid_strategy
    global prev_avoid_strategy
    global enable_3D_msg_obstacle_distance, enable_vision_position_estimate
    global distances, AC_VERSION_41
    curr_flight_mode = (value.base_mode, value.custom_mode)
    #print(f"Value : {value}")
    #print(f"Flight Mode: {curr_flight_mode}")
    if curr_flight_mode[0] == 89:
        # print(f"Avoid Strat: {curr_avoid_strategy}")
        if ((curr_flight_mode[1] == 5) or (curr_flight_mode[1] == 2)): # Loiter and AltHold only
            curr_avoid_strategy="simple_avoid"
            if (curr_avoid_strategy != prev_avoid_strategy):
                if ((AC_VERSION_41 == True) and (curr_flight_mode[1] == 5)): # only AC 4.1 or higher and LOITER
                    enable_3D_msg_obstacle_distance = True
                    enable_vision_position_estimate = True
                    print(enable_3D_msg_obstacle_distance)
                    send_msg_to_gcs('Sending 3D obstacle distance messages to FCU')
                prev_avoid_strategy = curr_avoid_strategy
        elif ((curr_flight_mode[1] == 3) or (curr_flight_mode[1] == 4) or (curr_flight_mode[1] == 6)): # for Auto Guided and RTL modes
            curr_avoid_strategy = "bendyruler"
            if (curr_avoid_strategy != prev_avoid_strategy):
                if (AC_VERSION_41 == True): # only AC 4.1 or higher
                    enable_3D_msg_obstacle_distance = True
                    enable_vision_position_estimate = True
                    print(enable_3D_msg_obstacle_distance)
                    send_msg_to_gcs('Sending 3D obstacle distance messages to FCU')
                prev_avoid_strategy = curr_avoid_strategy
        elif (curr_flight_mode[0] != 0):
            curr_avoid_strategy = "none"
            if (curr_avoid_strategy != prev_avoid_strategy):
                enable_3D_msg_obstacle_distance = False
                enable_vision_position_estimate = False
                send_msg_to_gcs('No valid flt mode for obstacle avoidance')
                prev_avoid_strategy = curr_avoid_strategy

# Listen to ZED ROS node for SLAM data
def data_callback_from_zed(msg):
    global distances, angle_offset, increment_f, min_depth_cm, max_depth_cm
    distances = np.array([i * 100 for i in msg.ranges]).astype(int)
    min_depth_cm = int(msg.range_min * 100)
    max_depth_cm = int(msg.range_max * 100)
    increment_f = msg.angle_increment
    angle_offset = msg.angle_min

def sector_data_callback(msg):
    global mavlink_obstacle_coordinates, min_depth_cm, max_depth_cm
    min_depth_cm = int(msg.channels[0].values[0] * 100)
    max_depth_cm = int(msg.channels[0].values[1] * 100)
    for j in range(9):
        mavlink_obstacle_coordinates[j][0] = msg.points[j].z
        mavlink_obstacle_coordinates[j][1] = (msg.points[j].x)
        mavlink_obstacle_coordinates[j][2] = (-1 * msg.points[j].y)
        # dist_debug[j] = m.sqrt(mavlink_obstacle_coordinates[j][0] * mavlink_obstacle_coordinates[j][0] + mavlink_obstacle_coordinates[j][1] * mavlink_obstacle_coordinates[j][1] + mavlink_obstacle_coordinates[j][2] * mavlink_obstacle_coordinates[j][2]) - 0.45
    # print("\033c")
    # print(min_depth_cm, max_depth_cm)
    # print (mavlink_obstacle_coordinates)
    # print (dist_debug)
    
def visual_odemetry_callback(msg):
    global odomoetry_x, odomoetry_y, odomoetry_z
    global roll, pitch, yaw
    
    # Camera position in map frame
    odomoetry_x = msg.pose.pose.position.x
    odomoetry_y = msg.pose.pose.position.y
    odomoetry_z = msg.pose.pose.position.z

    # Returns rotation matrix from quaternion
    rotation_mat = transformations.quaternion_matrix(np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]))
    
    # Get roll, pitch, and yaw from rotation matrix
    roll, pitch, yaw = transformations.euler_from_matrix(rotation_mat)
    
def pose_callback(msg):
    global pose_x, pose_y, pose_z
    global roll, pitch, yaw
    
    # Camera position in map frame
    pose_x = msg.pose.position.x
    pose_y = msg.pose.position.y
    pose_z = msg.pose.position.z

    rotation_mat = transformations.quaternion_matrix(np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))
    
    # Get roll, pitch, and yaw from rotation matrix from quaternion
    roll, pitch, yaw = transformations.euler_from_matrix(rotation_mat)

    rospy.loginfo(f"Received Pose in %s : X: {pose_x} Y: {pose_y} Z: {pose_z} - R: {roll} P: {pitch} Y: {yaw}")

    mavros_node.publish(msg)
    

######################################################
##  Main code starts here                           ##
######################################################

progress("INFO: Starting Vehicle communications")
conn = mavutil.mavlink_connection(
    connection_string,
    autoreconnect = True,
    source_system = 1,
    source_component = 93,
    baud = connection_baudrate,
    force_connected = True,
)

send_msg_to_gcs('Connecting to ROS node...')
rospy.init_node('listener')
rospy.loginfo('listener node started')
rospy.Subscriber('/Tobor/distance_array', LaserScan, data_callback_from_zed)
rospy.Subscriber('/Tobor/9sectorarray', PointCloud, sector_data_callback)
rospy.Subscriber('/zed2/zed_node/pose', PoseStamped, pose_callback)
mavros_node = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size = 10)
rospy.Subscriber('/zed2/zed_node/odom', Odometry, visual_odemetry_callback)
send_msg_to_gcs('ROS node connected')
sleep(1) # wait until the ROS node has booted
# register the callbacks
mavlink_callbacks = {
    #'ATTITUDE': att_msg_callback,
    'HEARTBEAT': fltmode_msg_callback,
}
mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
mavlink_thread.start()

# Send MAVlink messages in the background at pre-determined frequencies, delete?
msg_scheduler = BackgroundScheduler()
msg_scheduler.add_job(send_obstacle_distance_3D_message, 'interval', seconds = 1 / obstacle_distance_msg_hz, id='3d_obj_dist')
#msg_scheduler.add_job(send_vision_position_estimate, 'interval', seconds = 1 / obstacle_distance_msg_hz, id='vis_pos_est')
msg_scheduler.start()

# Begin of the main loop
last_time = time.time()
# Store the timestamp for MAVLink messages
cur_time_in_us = int(round(time.time() * 1000000))
cur_time_in_ms = current_milli_time()
rospy.spin()

mavlink_thread_should_exit = True
mavlink_thread.join()
conn.close()
progress("INFO: ZED pipe and vehicle object closed.")
sys.exit(exit_code)
