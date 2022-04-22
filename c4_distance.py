 #!/usr/bin/env python3.9

import pyzed.sl as stereolabs
import math
import numpy as np
import logging
import rospy
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry

# Global constants
FRONT_AVOID_DISTANCE = 2.5 # In meters, if any object is within this distance in front of the copter from the center of the copter then the copter needs to start making adjustments
PERPENDICULAR_LENGTH = 0.5 # In meters, the length of the copter from the bottom to the top with leeway to determine the height dimensions needed to be checked in the depth map
WIDTH_AVOID_DISTANCE = 1.2 + 0.3 # The distance from the center of the copter to the blade + leeway (0.3 meters?) if it's not 
FORWARD_VELECOTY = 5 # This value is just random right now but it is the velocity in the flat north direction
ERROR_MARGIN = 0 # Error margin from copter in meters

# Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
# See here: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
DISTANCES_ARRAY_LENGTH = 72 # Obstacle distances in front of the stereo camera from left to right

# Data Variables
cv2_grid_line_thickness = 1
angle_offset = 0
increment_l = 0

# Sensor Params
DEPTH_RANGE_METERS = [0.75, 6] # Range for the cam camera meters, currently testing a min distance of 0.75 meters to 6 meterse
DEPTH_HFOV_DEG = 86

sector_obstacle_coordinates = np.ones((9,3), dtype = float) * 9999
distances = np.ones((DISTANCES_ARRAY_LENGTH), dtype = float)

# Setup logger file
log_file = logging.FileHandler("challenge4_depth_analysis.log")
log_file.setLevel(logging.INFO)
log_format = logging.Formatter("%(asctime)s [%(levelname)s in %(filename)s] - %(message)s", datefmt="%H:%M:%S")
log_file.setFormatter(log_format)

log = logging.getLogger(__name__)
log.setLevel(logging.DEBUG)
log.addHandler(log_file)
 
"""
    Last Edit: 4/12/2022
    By: Sean Njenga
    
    Calculates the x, y, z position for a sector in the point_cloud image that has the min distance

    depth: Point cloud data of whole frame.
    bounds: Bounding box for object in pixels.
        bounds[0]: x-center
        bounds[1]: y-center
        bounds[2]: width of bounding box.
        bounds[3]: height of bounding box.
    
    TODO: Most likley the bounds pased to the function which are given by the current sector it is checking are wrong so it is only checking for distance in the middle of the sector
"""
def get_object_depth(depth, bounds, sector_num):
    area_div = 2 # How many pixels in the y direction to analyze

    if sector_num <= 2:
        sector_num = 1
    elif sector_num >= 3 and sector_num <= 5:
        sector_num = 3
    else:
        sector_num = 6

    x_vect = []
    y_vect = []
    z_vect = []

    for x in range(int(int(sector_num / 3) * bounds[2]), int(bounds[0] + bounds[2] / 2)):
        for y in range(int(bounds[1] - area_div), int(bounds[1] + area_div)):
            z = depth[y, x, 2]
            if not np.isnan(z) and not np.isinf(z): # Checks to see if the measured depth (z) is valid
                x_vect.append(depth[y, x, 0])
                y_vect.append(depth[y, x, 1])
                z_vect.append(z)
    try:
        # Add all distance values to an array to then find closests distance and return the (x,y,z) coordinates for it
        min_dis = [0] * (len(z_vect) - 1)
        for val in range(0, len(z_vect) - 1):
            min_dis[val] = math.sqrt(x_vect[val] * x_vect[val] + y_vect[val] * y_vect[val] + z_vect[val] * z_vect[val])

        # print(f"Array of Distances for Sector {sector_num} (Min Distance = {min_dis})\n{min_dis}")
        index = min_dis.index(min(min_dis))
        x_min = x_vect[index]
        z_min = y_vect[index]
        y_min = z_vect[index]

        min_dis = None # Clear the array from memory incase of duplicate allocation
  
    except Exception:
        x_min = 0
        y_min = 0
        z_min = 0
        pass
    return x_min, y_min, z_min

"""
    Calculate the distances array by dividing the FOV (horizontal) into $DISTANCES_ARRAY_LENGTH rays,
    then pick out the depth value at the pixel corresponding to each ray. Based on the definition of
    the MAVLink messages, the invalid distance value (below MIN/above MAX) will be replaced with MAX+1.
"""
def distances_from_depth_image(point_cloud_mat, distances, min_depth_m, max_depth_m, width, height):
    depth_img_width = width * 2  # Parameters for depth image
    step = depth_img_width / DISTANCES_ARRAY_LENGTH # Parameters for obstacle distance message
    
    # Take the mean value over the obstacle line thickness
    for i in range(DISTANCES_ARRAY_LENGTH):
        err, point_cloud_value = point_cloud_mat.get_value((int(i * step)), height)
        if err != stereolabs.ERROR_CODE.SUCCESS:
            print(repr(err))

        dist_m = math.sqrt(point_cloud_value[0] * point_cloud_value[0] + point_cloud_value[1] * point_cloud_value[1] + point_cloud_value[2] * point_cloud_value[2]) - ERROR_MARGIN # ERROR_MARGIN is 0 rn

        # A value of max_distance + 1 (cm) means no obstacle is present. 
        # A value of UINT16_MAX (65535) for unknown/not used.
        if dist_m >= min_depth_m and dist_m <= max_depth_m:
            distances[i] = dist_m
        elif dist_m < min_depth_m:
            distances[i] = 0
        elif dist_m >= max_depth_m or np.isnan(dist_m):
            distances[i] = max_depth_m + 0.01
        elif np.isinf(dist_m):
            distances[i] = 655.36

    # log.info(f"final distances array in m:\n{distances}")
    # print("{}".format(distances), end='\r')
    
"""
    Last Edit: 4/19/2022
    By: Sean Njenga
    
    Initializes the ROS nodes that are being used to send SLAM data across to MAVLink
"""
def initialize_ros():
    angle_offset = float(0 - (DEPTH_HFOV_DEG / 2))
    increment_l = float(DEPTH_HFOV_DEG / DISTANCES_ARRAY_LENGTH)
    
    # Start ROS nodes
    rospy.init_node('distance')
    rospy.loginfo('distance node started')
    laser_scan_node = rospy.Publisher('/Tobor/distance_array', LaserScan, queue_size = 10)
    point_cloud_node = rospy.Publisher('/Tobor/9sectorarray', PointCloud, queue_size = 12) # queue_size was 10 before
    #odometry_node = rospy.Publisher('/zed2/zed_node/odom', Odometry, queue_size = 10)
    #pose_node = rospy.Publisher('/zed2/zed_node/pose', PoseStamped, queue_size = 10)
    
    #Initialize laserscan node
    scan = LaserScan()
    scan.header.frame_id = 'zed_horizontal_scan'
    scan.angle_min = angle_offset
    scan.angle_max = DEPTH_HFOV_DEG / 2
    scan.angle_increment = float(increment_l)
    scan.range_min = DEPTH_RANGE_METERS[0]
    scan.range_max = DEPTH_RANGE_METERS[1]
    scan.intensities = []
    scan.time_increment = 0
    fps = 1

    #Initialize PointCloud node
    channel = ChannelFloat32()
    channel.name = "depth_range"
    channel.values = [DEPTH_RANGE_METERS[0], DEPTH_RANGE_METERS[1]]
    ros_point_cloud = PointCloud()
    ros_point_cloud.header.frame_id ='zed_9_sector_scan'
    ros_point_cloud.channels = [channel]
    
    return scan, laser_scan_node, point_cloud_node, ros_point_cloud#, odometry_node, pose_node

"""
    Last Edit: 4/19/2022
    By: Sean Njenga
    
    Initializes the cam stereo camera with the specified parameters and captures multiple images to create a depth map. Then calls other methods to perform analysis to determine the closets object and the distance in each sector of a 3x3 grid of the image.
"""
def depth_sector(cam, sector_mat, point_cloud_mat, scan, laser_scan_node, point_cloud_node, ros_point_cloud, runtime_parameters, width, height):
    # A new image is available to process if grab() returns SUCCESS
    if cam.grab(runtime_parameters) == stereolabs.ERROR_CODE.SUCCESS:
        cam.retrieve_image(sector_mat, stereolabs.VIEW.LEFT)
        image_sector = sector_mat.get_data()
        
        cam.retrieve_measure(point_cloud_mat, stereolabs.MEASURE.XYZ)
        depth = point_cloud_mat.get_data()
        distances_from_depth_image(point_cloud_mat, distances, DEPTH_RANGE_METERS[0], DEPTH_RANGE_METERS[1], width, height)
        scan.ranges = distances

        #publish distance information for mavros node
        laser_scan_node.publish(scan)
        
        # create 9 sector image with distance information
        # divide view into 3x3 matrix
        sector = np.empty(4, dtype = int)
        x_step = int(width * 2 / 3)
        y_step = int(height * 2 / 3)
            
        # measure sector depth and printout in sectors
        gx = 0
        gy = 0
        i = 0
        sector[1] = y_step / 2 + gy
        sector[2] = x_step
        sector[3] = y_step
        while gx < (width * 2 - 2):
            gy = 0
            sector[1] = y_step / 2 + gy
            sector[0] = x_step / 2 + gx

            if i == 1 or i == 4 or i == 7:
                # Get coordinates of closest object in a sector
                x, y, z = get_object_depth(depth, sector, i)
                sector_obstacle_coordinates[i][0] = x
                sector_obstacle_coordinates[i][1] = y
                sector_obstacle_coordinates[i][2] = z

            gy += y_step
            i += 1

            while gy < (height * 2):
                sector[1] = y_step / 2 + gy
        
                if i == 1 or i == 4 or i == 7:
                    # Get coordinates of closest object in a sector
                    x, y, z = get_object_depth(depth, sector, i)
                    sector_obstacle_coordinates[i][0] = x
                    sector_obstacle_coordinates[i][1] = y
                    sector_obstacle_coordinates[i][2] = z 

                gy += y_step
                i += 1

            gx += x_step
            
        # Send point_cloud data to ROS node
        ros_point_cloud.points = [
            Point32(x = sector_obstacle_coordinates[j][0],y = sector_obstacle_coordinates[j][1],z = sector_obstacle_coordinates[j][2])
            for j in range(1, 9, 3)]
        point_cloud_node.publish(ros_point_cloud)

def depth_sector_test():
    import cv2

    # Open/start the camera and check for initialization errors
    cam = stereolabs.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = stereolabs.InitParameters()
    init_params.depth_mode = stereolabs.DEPTH_MODE.PERFORMANCE # Use PERFORMANCE or QUALITY depth mode
    init_params.coordinate_units = stereolabs.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = stereolabs.RESOLUTION.HD720 # Resolution set to 720p could go up to 1080p
    #err = cam.open(init_params)

    #print(f"{cam.is_opened()}")
    #if err != stereolabs.ERROR_CODE.SUCCESS:
        #print(repr(err))
        #cam.close()
        #exit(1)

    print("Enable ZED Pos Tracking")
    # Enable positional tracking with default parameters
    tracking_parameters = stereolabs.PositionalTrackingParameters()
    tracking_parameters.enable_area_memory = True
    tracking_parameters.enable_pose_smoothing = True
    #err = cam.enable_positional_tracking(tracking_parameters)
    if err != stereolabs.ERROR_CODE.SUCCESS:
        print("Pos Tracking Error")
        cam.close()
        exit()

    cam.set_camera_settings(stereolabs.VIDEO_SETTINGS.EXPOSURE, -1) # very bright day .1-.5 # (0, 100) % of camera frame rate. -1 sets it to auto
    cam.set_camera_settings(stereolabs.VIDEO_SETTINGS.CONTRAST, -1) #-1 is auto (0,8) possible values
    cam.set_camera_settings(stereolabs.VIDEO_SETTINGS.SATURATION, -1)
    cam.set_camera_settings(stereolabs.VIDEO_SETTINGS.SHARPNESS, -1)
    cam.set_camera_settings(stereolabs.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, -1)
    cam.set_camera_settings(stereolabs.VIDEO_SETTINGS.BRIGHTNESS, -1)
    #cam.set_camera_settings(stereolabs.VIDEO_SETTINGS.HUE, -1)

    print("Setup runtime")
    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = stereolabs.RuntimeParameters()
    #Ian changed sensing mode from FILL to STANDARD on 4/20 (haha)
    runtime_parameters.sensing_mode = stereolabs.SENSING_MODE.STANDARD
    
    # Setting the depth confidence parameters
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100



    print("Starting ROS")
    scan, laser_scan_node, point_cloud_node, ros_point_cloud = initialize_ros()
    
    # Initialze the matrix's for analysis
    sector_mat = stereolabs.Mat()
    point_cloud_mat = stereolabs.Mat()

    while True:
        # A new image is available if grab() returns SUCCESS
        if cam.grab(runtime_parameters) == stereolabs.ERROR_CODE.SUCCESS:
            cam.retrieve_image(sector_mat, stereolabs.VIEW.DEPTH)
            image_sector = sector_mat.get_data()
            
            cam.retrieve_measure(point_cloud_mat, stereolabs.MEASURE.XYZ)
            depth = point_cloud_mat.get_data()
            distances_from_depth_image(point_cloud_mat, distances, DEPTH_RANGE_METERS[0], DEPTH_RANGE_METERS[1], width, height)
            scan.ranges = distances

            # Publish distance information for mavros node
            # log.info(f"Print Scan: {scan}")
            # print(f"Print Scan: {scan}")
            laser_scan_node.publish(scan)
            
            # Create 9 sector image with distance information and divide view into 3x3 matrix
            sector = np.empty(4, dtype = int)
            x_step = int(width * 2 / 3)
            y_step = int(height * 2 / 3)
            gx = 0
            gy = 0
            
            # Draw sector lines on OpenCV image, comment this out after testing is done on drone to see if it's working
            while gx < (width * 2):
                cv2.line(image_sector, (gx, 0), (gx, (height * 2)), color=(0, 0, 255), thickness = 1)
                gx += x_step
                
            while gy <= (height * 2):
                cv2.line(image_sector, (0, gy), ((width * 2), gy), color=(0, 0, 255),thickness = 1)
                gy += y_step
  
            # measure sector depth and printout in sectors
            gx = 0
            gy = 0
            i = 0
            sector[1] = y_step / 2 + gy
            sector[2] = x_step
            sector[3] = y_step
            distance = 0
            while gx < (width * 2 - 2):
                gy = 0
                sector[1] = y_step / 2 + gy
                sector[0] = x_step / 2 + gx

                if i == 1 or i == 4 or i == 7:
                    # calculate depth of closest object in sector
                    x, y, z = get_object_depth(depth, sector, i)
                    sector_obstacle_coordinates[i][0] = x
                    sector_obstacle_coordinates[i][1] = y
                    sector_obstacle_coordinates[i][2] = z
                    distance = math.sqrt(x * x + y * y + z * z)
                    distance = "{:.2f}".format(distance)
                    cv2.putText(image_sector, " " +  (str(distance) + " m"),
                            ((gx + 10), (gy + y_step - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                gy += y_step
                i += 1

                while gy < (height * 2):
                    sector[1] = y_step / 2 + gy

                    if i == 1 or i == 4 or i == 7:
                        # calculate depth of closest object in sector
                        x, y, z = get_object_depth(depth, sector, i)
                        sector_obstacle_coordinates[i][0] = x
                        sector_obstacle_coordinates[i][1] = y
                        sector_obstacle_coordinates[i][2] = z
                        distance = math.sqrt(x * x + y * y + z * z)
                        distance = "{:.2f}".format(distance)
                        cv2.putText(image_sector, " " +  (str(distance) + " m"),
                                ((gx + 10), (gy + y_step - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    
                    gy += y_step
                    i += 1

                gx += x_step
            cv2.imshow("Distance Grid", image_sector)
            cv2.waitKey(1)

            for j in range(1,9,3):
                print(f"Sector {j} = {math.sqrt(sector_obstacle_coordinates[j][0] * sector_obstacle_coordinates[j][0] + sector_obstacle_coordinates[j][1] * sector_obstacle_coordinates[j][1] + sector_obstacle_coordinates[j][2] + sector_obstacle_coordinates[j][2])}m")
                
            # Send point_cloud data to ROS node
            ros_point_cloud.points = [
                Point32(x=sector_obstacle_coordinates[j][0],y=sector_obstacle_coordinates[j][1],z=sector_obstacle_coordinates[j][2])
                for j in range(1, 9, 3)]
            point_cloud_node.publish(ros_point_cloud)

    cv2.destroyAllWindows()
    cam.close()     
    
"""
    Last Edit: 3/31/2022
    By: Sean Njenga
    
    Starts the depth sense script (needs to be in a loop)
"""   
def main():
    depth_sector_test()

if __name__ == "__main__":
    main()
