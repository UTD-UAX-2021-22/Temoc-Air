import pyzed.sl as stereolabs
import math
import cv2
#import ogl_viewer.viewer as gl
import numpy as np
import sys

# from dronekit import connect, mavutil, VehicleMode
# from GeneralFunctions import ConnectToCopter

# Create a Camera object
zed = stereolabs.Camera()

def start_camera(init_params):
    # Open the camera
    err = zed.open(init_params)
    if err != stereolabs.ERROR_CODE.SUCCESS:
        print(repr(err))
        zed.close()
        exit(1)

"""
    Last Edit: 2/25/2022
    By: Sean Njenga
    
    Captures multiple images to create a depth map and uses a point cloud to calculate distance from a specific point.
    Currently has no way to find the object itself from the map.
"""
def depth_sense():
    # Create a InitParameters object and set configuration parameters
    init_params = stereolabs.InitParameters()
    init_params.depth_mode = stereolabs.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = stereolabs.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = stereolabs.RESOLUTION.HD720

    start_camera(init_params) # Enable ZED Camera

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = stereolabs.RuntimeParameters()
    runtime_parameters.sensing_mode = stereolabs.SENSING_MODE.STANDARD
    
    # Setting the depth confidence parameters
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100

    # Capture 50 images and depth, then stop
    i = 0
    image = stereolabs.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, stereolabs.MAT_TYPE.U8_C4)
    depth = stereolabs.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, stereolabs.MAT_TYPE.F32_C1)
    point_cloud = stereolabs.Mat()

    mirror_ref = stereolabs.Transform()
    mirror_ref.set_translation(stereolabs.Translation(2.75,4.0,0))
    tr_np = mirror_ref.m

    while i < 25: # Capturing 50 images rn for testing
        # A new image is available if grab() returns SUCCESS
        if zed.grab(runtime_parameters) == stereolabs.ERROR_CODE.SUCCESS:
            # Retrieve left image
            #zed.retrieve_image(image, stereolabs.VIEW.LEFT)
            # zed.retrieve_image(image, stereolabs.VIEW.DEPTH) # Retrieve normalized depth image
            
            # Retrieve depth map. Depth is aligned on the left image
            zed.retrieve_measure(depth, stereolabs.MEASURE.DEPTH)
            
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(point_cloud, stereolabs.MEASURE.XYZRGBA)

            # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            x = round(image.get_width() / 2)
            y = round(image.get_height() / 2)
            err, point_cloud_value = point_cloud.get_value(x, y)

            distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                 point_cloud_value[1] * point_cloud_value[1] +
                                 point_cloud_value[2] * point_cloud_value[2])

            point_cloud_np = point_cloud.get_data()
            point_cloud_np.dot(tr_np)
            
            if not np.isnan(distance) and not np.isinf(distance):
                print("Distance to Camera at ({}, {}) (image center): {:1.3} m".format(x, y, distance), end = "\r")
                #print("\n")
                # Increment the loop
                i = i + 1 
            else:
                print("Can't estimate distance at this position.")
            sys.stdout.flush()

            zed.retrieve_image(image, stereolabs.VIEW.LEFT)
            image_depth_ocv = image.get_data()
            #cv2.putText(, Z,(10,100), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)
            cv2.imshow("Image", image_depth_ocv)
            cv2.waitKey()

    # Close all cv2 images
    cv2.destroyAllWindows()
    
    # Close the camera
    zed.close()

"""
    Last Edit: 2/25/2022
    By: Sean Njenga
    
    HAS NOT BEEN TESTED YET, will test on Sunday
    Very basic script to detect a object and print it's position vector, unsure if noodles will be detected or if I need to make and train a custom model yet will see after testing.
"""
def noodle_detection():
    # Create a InitParameters object and set configuration parameters
    init_params = stereolabs.InitParameters()
    init_params.camera_resolution = stereolabs.RESOLUTION.HD1080  # Use HD1080 video mode    
    init_params.coordinate_units = stereolabs.UNIT.METER
    init_params.camera_fps = 30                          # Set fps at 30
    init_params.coordinate_system = stereolabs.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    
    runtime_parameters = stereolabs.RuntimeParameters() # Set runtime parameters
    start_camera(init_params) # Enable ZED Camera
    obj_param = stereolabs.ObjectDetectionParameters() # Enable object detection module
    
    # Defines if the object detection will track objects across images flow.
    obj_param.enable_tracking = True       

    # if True, enable positional tracking
    if obj_param.enable_tracking:
        zed.enable_positional_tracking()
        
    zed.enable_object_detection(obj_param)
    camera_info = zed.get_camera_information()
    
    # Create OpenGL viewer
    viewer = gl.GLViewer()
    viewer.init(camera_info.calibration_parameters.left_cam, obj_param.enable_tracking)

    # Configure object detection runtime parameters
    obj_runtime_param = stereolabs.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 60
    obj_runtime_param.object_class_filter = [stereolabs.OBJECT_CLASS.PERSON]    # Only detect Persons

    # Create ZED objects filled in the main loop
    objects = stereolabs.Objects()
    image = stereolabs.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, stereolabs.MAT_TYPE.U8_C4)

    while viewer.is_available():
        # Grab an image, a RuntimeParameters object must be given to grab()
        if zed.grab(runtime_parameters) == stereolabs.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, stereolabs.VIEW.LEFT) # Retrieve left image
            zed.retrieve_objects(objects, obj_runtime_param) # Retrieve objects
            
            if objects.is_new:
                # Print num objects detected in the scene
                print("{} Object(s) detected".format(len(objects.object_list)))
                
                # Display the 3D location of an object if any has been detected
                if len(objects.object_list):
                    position = objects.object_list[0].position
                    print(" Position Vector : [{0},{1},{2}]".format(position[0], position[1], position[2]))
                    
            # Update GL view
            viewer.update_view(image, objects)

    # Close GL Viewer
    viewer.exit()

    # Disable modules and close camera
    image.free(memory_type = stereolabs.MEM.CPU)
    zed.disable_object_detection()
    zed.disable_positional_tracking()

    # Close Camera
    zed.close()
    
def comm_link():
    print()
    
def main():
    # ConnectToCopter() # Start a SITL instance 
    
    depth_sense()
    # noodle_detection()

if __name__ == "__main__":
    main()
