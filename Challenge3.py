import pyzed.sl as stereolabs
import math
import cv2
import sys
import statistics
import numpy as np
import logging
from multiprocessing import Process

# from dronekit import connect, mavutil, VehicleMode
# from GeneralDroneFunctions import ConnectToCopter, FrameVelocityControl

# Global constants
DEPTH_RANGE_M = [1, 20.0] # Range for the ZED camera
OBSTANCLE_LINKE_PIXEL_THICKNESS = 1 
FRONT_AVOID_DISTANCE = 2.5 # In meters, if any object is within this distance in front of the copter from the center of the copter then the copter needs to start making adjustments
PERPENDICULAR_LENGTH = 0.5 # In meters, the length of the copter from the bottom to the top with leeway to determine the height dimensions needed to be checked in the depth map
WIDTH_AVOID_DISTANCE = 1.2 + 0.3 # The distance from the center of the copter to the blade + leeway (0.3 meters?) if it's not 
FORWARD_VELECOTY = 5 # This value is just random right now but it is the velocity in the flat north direction
DISTANCES_ARRAY_LENGTH = 72 # Obstacle distances in front of the stereo camera from left to right

# 
zed = stereolabs.Camera() # Create a Camera object for interacting with the ZED
last_direction = "r"
sector_obstacle_coordinates = np.ones((9,3), dtype = np.float) * (9999)
distances = np.ones((DISTANCES_ARRAY_LENGTH), dtype=np.float)
closests_distances = []

# Setup logger file
log_file = logging.FileHandler("challenge4.log")
log_file.setLevel(logging.INFO)
log_format = logging.Formatter("%(asctime)s [%(levelname)s in %(filename)s] - %(message)s", datefmt="%H:%M:%S")
log_file.setFormatter(log_format)

log = logging.getLogger(__name__)
log.setLevel(logging.DEBUG)
log.addHandler(log_file)
 
"""
    Last Edit: 4/2/2022
    By: Sean Njenga
    
    Calculates the smallest x, y, z position for a sector in the point_cloud image
"""
def get_object_depth(depth, bounds):
    area_div = 2

    x_vect = []
    y_vect = []
    z_vect = []

    for j in range(int(bounds[0] - area_div), int(bounds[0] + area_div)):
        print(f"j-Start {int(bounds[0] - area_div)}, j-Stop {int(bounds[0] + area_div)}")
        for i in range(int(bounds[1] - area_div), int(bounds[1] + area_div)):
            print(f"i-Start {int(bounds[0] - area_div),} i-Stop {int(bounds[0] + area_div)}")
            z = depth[i, j, 2]
            if not np.isnan(z) and not np.isinf(z):
                x_vect.append(depth[i, j, 0])
                y_vect.append(depth[i, j, 1])
                z_vect.append(z)
    try:
        x_min = min(x_vect)
        y_min = min(y_vect)
        z_min = min(z_vect)
    except Exception:
        x_min = -1
        y_min = -1
        z_min = -1
        pass
    return x_min, y_min, z_min

"""
    Calculate the distances array by dividing the FOV (horizontal) into $DISTANCES_ARRAY_LENGTH rays,
    then pick out the depth value at the pixel corresponding to each ray. Based on the definition of
    the MAVLink messages, the invalid distance value (below MIN/above MAX) will be replaced with MAX+1.
"""
def distances_from_depth_image(point_cloud_mat, distances, min_depth_m, max_depth_m, width, height, OBSTANCLE_LINKE_PIXEL_THICKNESS):
    depth_img_width = width * 2  # Parameters for depth image
    step = depth_img_width / DISTANCES_ARRAY_LENGTH # Parameters for obstacle distance message
    
    # Do the depth sensing at each individual segment in the 3x3 and take the mean value over the obstacle line thickness
    for i in range(DISTANCES_ARRAY_LENGTH):
        err, point_cloud_value = point_cloud_mat.get_value((int(i * step)), height)
        dist_m = math.sqrt(point_cloud_value[0] * point_cloud_value[0] + point_cloud_value[1] * point_cloud_value[1] + point_cloud_value[2] * point_cloud_value[2])

        # A value of max_distance + 1 (cm) means no obstacle is present. 
        # A value of UINT16_MAX (65535) for unknown/not used.
        if dist_m >= min_depth_m and dist_m <= max_depth_m:
            distances[i] = dist_m
        elif dist_m < min_depth_m:
            distances[i] = 0
        elif dist_m >= max_depth_m or np.isnan(dist_m):
            distances[i] = 20.01
        elif np.isinf(dist_m):
            distances[i] = 655.35

    # log.info(f"final distances array in m: \n{distances}")
    print("final distances array in m:\n{}".format(distances))

"""
    Last Edit: 4/4/2022
    By: Sean Njenga
    
    Initializes the ZED stereo camera with the specified parameters and captures multiple images to create a depth map. Then calls other methods to perform analysis to determine the closets object and the distance in each sector of a 3x3 grid of the image.

    TODO: Some slight issues with distnce measurements as seen in the screenshot (prob just the median calculation is off) need to look into that and then how to send distance measurements to ArduPilot through MAVLink & ROS Nodes to use BendyRuler
"""
def depth_sector():
    # Create a InitParameters object and set configuration parameters
    init_params = stereolabs.InitParameters()
    init_params.depth_mode = stereolabs.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = stereolabs.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = stereolabs.RESOLUTION.HD720 # Resolution set to 720p could go up to 1080p

    # Open/start the camera and check for initialization errors
    err = zed.open(init_params) 
    if err != stereolabs.ERROR_CODE.SUCCESS:
        log.error(repr(err))
        zed.close()
        exit(1)

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = stereolabs.RuntimeParameters()
    runtime_parameters.sensing_mode = stereolabs.SENSING_MODE.STANDARD
    
    # Setting the depth confidence parameters
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100
    
    res_params = stereolabs.Resolution()
    width = round(zed.get_camera_information().camera_resolution.width / 2)
    height = round(zed.get_camera_information().camera_resolution.height / 2)
    res_params.width = width
    res_params.height = height

    # Initialze the matrix's for analysis
    image_mat = stereolabs.Mat()
    sector_mat = stereolabs.Mat()
    point_cloud_mat = stereolabs.Mat()
    
    # Start ROS nodes
    # node1 = rospy.Publisher('/darknet_ros/detection_image', Image, queue_size = #)
    # node2 = rospy.Publisher('/darknet_ros/distance_array', LaserScan, queue_size = #)
    # node3 = rospy.Publisher('/darknet_ros/color_image', Image, queue_size = #)
    # node4 = rospy.Publisher('/darknet_ros/nine_sector_image', Image, queue_size = #)
    # node5 = rospy.Publisher('/darknet_ros/9sectorarray', PointCloud, queue_size = #)
    
    #Initialize laserscan node
    # scan = LaserScan()
    # scan.header.frame_id = 'zed_horizontal_scan'
    # scan.angle_min = angle_offset
    # scan.angle_max = depth_hfov_deg/2
    # scan.angle_increment = increment_f
    # scan.range_min = DEPTH_RANGE_M[0]
    # scan.range_max = DEPTH_RANGE_M[1]
    # scan.intensities = []
    # scan.time_increment = 0
    # fps = 1

    #Initialize pointcloud node
    # channel = ChannelFloat32()
    # channel.name = "depth_range"
    # channel.values = [DEPTH_RANGE_M[0],DEPTH_RANGE_M[1]]
    # pointcloud = PointCloud()
    # pointcloud.header.frame_id ='zed_9_sector_scan'
    # pointcloud.channels = [channel]

    cur = 0
    while cur < 1: # Capturing 1 images
        # A new image is available if grab() returns SUCCESS
        if zed.grab(runtime_parameters) == stereolabs.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image_mat, stereolabs.VIEW.LEFT)
            image = image_mat.get_data()
            
            zed.retrieve_image(sector_mat, stereolabs.VIEW.LEFT)
            image_sector = sector_mat.get_data()
            
            # Retrieve depth map. Depth is aligned on the left image
            # zed.retrieve_measure(depth, stereolabs.MEASURE.DEPTH)
            
            # Retrieve point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(point_cloud_mat, stereolabs.MEASURE.XYZ)
            depth = point_cloud_mat.get_data()
            distances_from_depth_image(point_cloud_mat, distances, DEPTH_RANGE_M[0], DEPTH_RANGE_M[1], width, height, OBSTANCLE_LINKE_PIXEL_THICKNESS)
            center = np.mean(distances[34:38])
            
            x1, y1 = int(0), int(height)
            x2, y2 = int(width * 2), int(height)
            line_thickness = OBSTANCLE_LINKE_PIXEL_THICKNESS
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), thickness = line_thickness)
            
            # Format the output image for OpenCV for testing
            if center > DEPTH_RANGE_M[1]:
                cv2.circle(image, (width, height), 20, (0, 255, 0), 2)
            elif center <= DEPTH_RANGE_M[1] and center > 10:
                cv2.putText(image, ("{:1.3} m".format(center)), ((width - 50), (height + 50)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.circle(image, (width, height), 20, (0, 255, 0), 2)
            elif center > 5 and center <= 10:
                cv2.putText(image, ("{:1.3} m".format(center)), ((width - 70), (height + 65)), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 2)
                cv2.circle(image, (width, height), 20, (0, 255, 255), 4)
            else:
                cv2.putText(image, ("{:1.3} m".format(center)), ((width - 100), (height + 70)), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 0, 255), 2)
                cv2.circle(image, (width, height), 20, (0, 0, 255), -1)
                cv2.rectangle(image, (0, 100), ((width * 2 - 5),(height * 2)), (30,30,255), 3)
            
            # create 9 sector image with distance information
            # divide view into 3x3 matrix
            sector = np.empty(4, dtype = int)
            x_step = int(width * 2 / 3)
            y_step = int(height * 2 / 3)
            gx = 0
            gy = 0
            
            # Draw sector lines on OpenCV image
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
            while gx < (width * 2 - 2):
                gy = 0
                sector[1] = y_step / 2 + gy
                sector[0] = x_step / 2 + gx
                
                # calculate depth of closest object in sector
                x, y, z = get_object_depth(depth, sector)
                sector_obstacle_coordinates[i][0] = x
                sector_obstacle_coordinates[i][1] = y
                sector_obstacle_coordinates[i][2] = z
                distance = math.sqrt(x * x + y * y + z * z)
                distance = "{:.2f}".format(distance)
                closests_distances.append(distance)
                cv2.putText(image_sector, " " +  (str(distance) + " m"),
                            ((gx + 10), (gy + y_step - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                gy += y_step
                i += 1

                while gy < (height * 2):
                    sector[1] = y_step / 2 + gy
                   
                    # calculate depth of closest object in sector
                    x, y, z = get_object_depth(depth, sector)
                    sector_obstacle_coordinates[i][0] = x
                    sector_obstacle_coordinates[i][1] = y
                    sector_obstacle_coordinates[i][2] = z
                    distance = math.sqrt(x * x + y * y + z * z)
                    distance = "{:.2f}".format(distance)
                    closests_distances.append(distance)
                    cv2.putText(image_sector, " " +  (str(distance) + " m"),
                                ((gx + 10), (gy + y_step - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    gy += y_step
                    i += 1
                gx += x_step
                
            # Send point_cloud data to ROS node
            # pointcloud.points = [
            #     Point32(x=sector_obstacle_coordinates[j][0],y=sector_obstacle_coordinates[j][1],z=sector_obstacle_coordinates[j][2])
            #     for j in range(9)]
            # node5.publish(pointcloud)
            
            cur = cur + 1


    # Testing custom velocity vector sending incase the communication link to Mavlink through ROS nodes does not work
    # if  float(closests_distances[1]) > 1.5 and float(closests_distances[7]) < 1.5 and float(closests_distances[4]) > FRONT_AVOID_DISTANCE:
    #     print("Continue Going Straight")
    # elif  float(closests_distances[7]) < FRONT_AVOID_DISTANCE and  float(closests_distances[1]) > FRONT_AVOID_DISTANCE and  float(closests_distances[4]) > FRONT_AVOID_DISTANCE:
    #     print("Go Left")

    log.info(f"Middle 3 sectors of the image {closests_distances[1]} {closests_distances[4]} {closests_distances[7]}")
    cv2.imshow("Distance Grid", image_sector)
    cv2.waitKey()
            
    # Close all cv2 images
    cv2.destroyAllWindows()
    
    # Close the camera
    zed.close()

    log.info("\nFINISH")
    
"""
    Last Edit: 3/31/2022
    By: Sean Njenga
    
    Starts the depth sense script (needs to be in a loop)
"""   
def main():
    depth_sector()

if __name__ == "__main__":
    main()
