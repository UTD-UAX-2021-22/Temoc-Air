import argparse
import logging
import time
import sys
import pyzed.sl as sl

from tqdm import tqdm
#import chall2_test
#print(vars(chall2_test))
# import GeneralDroneFunctions
import GeneralDroneFunctions as gd
from GeneralDroneFunctions import ServoMovement
from LogoDetection import detectLogo
from POI import POI_Tracker
print("Local imports ~50%")
import cv2
from Utils import MissionContext, pixCoordToAngle, setupLoggers, VehicleInfo
import Utils
from LogoDetection import LogoDetector
from GeoTracker import GeoTracker
from Utils import pixCoordToRelativePosition, pixCoordToWorldPosition
import numpy as np
#from RosAsync import AsyncSubscriber
import utm
from scipy.spatial.transform import Rotation
from PIL import Image as PilImage
logo_markers = list(range(5))

#from cv_bridge import CvBridge
#from sensor_msgs.msg import Image, CompressedImage
#import rospy


#TODO: Make actually work
#TODO: Fallback logic 
async def mainFunc():
    setupLoggers("challenge_2")
    logger = logging.getLogger(__name__)

    parser = argparse.ArgumentParser(description="Test logo detection")
    # parser.add_argument('video_input_path', default="rtsp://192.168.137.234:8080/video/h264", nargs='?')
    # parser.add_argument('--video', default="mavic_test_11_12_closeup.mp4", nargs='?')
    parser.add_argument('--template', default="logo_template_2.png",  nargs='?')
    args = parser.parse_args()

    # Load vehicle information. Currently contains only camera data
    vehicle_info = VehicleInfo.parse_file("vehicle_info.json")
    #bridge = CvBridge()
    fcam_info = [c for c in vehicle_info.cameras if c.name == "Forward Camera"][0]
    down_cam_info = [c for c in vehicle_info.cameras if c.name == "Downward Camera"][0]
    max_frame_fails = 30
    logger.debug(f"Loading template image {args.template}")
    logo_template_image = cv2.imread(args.template)
    mission_start_time = time.time()
    
    landing_tolerance = 0.1524 # 6 inches

    # Initialize Trackers/detectors
    poiTracker = POI_Tracker()
    logoDetector = LogoDetector(logo_template_image)
    #geoTracker = GeoTracker((30, 5), resolution=3, dims=(60, 60))

    # Begin mission
    with MissionContext("Startup"):
        # poiTracker = POI_Tracker()
        #cam_front = cv2.VideoCapture(fcam_info.id)
        #cam_down = cv2.VideoCapture(down_cam_info.id)
        #cam_front = AsyncSubscriber("/iris_demo/camera_front/image_raw/compressed", CompressedImage, queue_size=1)
        #cam_down = AsyncSubscriber("/iris_demo/camera/image_raw/compressed", CompressedImage, queue_size=1)
        cam = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps=60
        init.depth_mode = sl.DEPTH_MODE.NONE
        status = cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit(1)
        cam_front = cam
        cam_down = cam

      
        vehicle = gd.ConnectToCopter("127.0.0.1:14550")

    # Yards to meters
    def y2m(v):
        return v * 0.9144

    # Meters to yards
    def m2y(v):
        return v * 1.09361

    # Positions of the corners of the field (well actually only half) relative to the center of the zero yard line
    # Order: Bottom left -> Top Left -> Top Right -> Bottom Right   ---  This is VERY important to have right
    # Units in yards
    field_corners_y = np.array([
        [-53.333/2, 0, 0],
        [-53.333/2, 50, 0],
        [53.333/2, 50, 0],
        [53.333/2, 0, 0]
    ])

    # Allocate space in a numpy array for home position in UTM coords
    base_pos = np.zeros(2)
    # Takeoff
    with MissionContext("Ascent"):
        gd.ArmDrone(vehicle) # Arm Vehicle
        
        # Print and log telemetry
        print(f"Vehicle coords {vehicle.location.global_frame}")
        logger.debug(f"Vehicle coords {vehicle.location.global_frame}")

        # Store home position in UTM coords
        base_pos[0], base_pos[1], zl, zn = utm.from_latlon(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
        
        # print(base_pos + field_corners_y)
        # Calculate real world coords of field corners in UTM system
        new_pos = base_pos + (Rotation.from_euler('Z', [vehicle.heading], degrees=True).apply(y2m(field_corners_y))[:,0:2])
        logger.debug(f"Calculated base UTM {base_pos} with {zl} {zn} ")
        logger.debug(f"Relative field coordinates {Rotation.from_euler('Z', [-vehicle.heading], degrees=True).apply(y2m(field_corners_y))[:,0:2]}")
        
        coords_lat = np.zeros((4,2))
        geoTracker = GeoTracker(corners=new_pos)
        coords_lat[:,0], coords_lat[:,1] = utm.to_latlon(new_pos[:,0], new_pos[:,1], zl, zn)
        print(f"Field Corners: {coords_lat}")
        vehicle.parameters['ANGLE_MAX'] = 10*100 # Angle in centidegress
        await asyncio.sleep(3)
        async def liftOffAndMoveToCenter():
            await gd.TakeOffDrone(vehicle, 7.62*1.5)
            await gd.GoToTargetBody(vehicle, gd.FeetToMeters(75), 0, 0)
        
        lft_off_task = asyncio.create_task(liftOffAndMoveToCenter())

    # print(f"Coords lat: {coords_lat}")
    visitCorners = False
    skipToLand = False



    # print("Attempting goto current position")
    # await gd.GoToGlobal(vehicle, np.asarray([vehicle.location.global_frame.lat, vehicle.location.global_frame.lon]))
    # print("Done going to current position")
    if visitCorners:
        with MissionContext("Visiting Corners"):
            for row in coords_lat:
                await gd.GoToGlobal(vehicle, row)
                await asyncio.sleep(4)

    logo_found = False
    ServoMovement(90)
    cf =  cam_front.open(init)
    if cf != sl.ERROR_CODE.SUCCESS:
        print(repr(cf))
        zed.close()
        exit(1)
    runtime = sl.RuntimeParameters()
    imageSize = zed.get_camera_information().camera_resolution
    zedImage = sl.Mat(imageSize.width, imageSize.height, sl.MAT_TYPE.U8_C4)
    with MissionContext("POI Search"):
        # await gd.GoToTargetBody(vehicle, gd.FeetToMeters(75), 0, 0) # Move forward to the middle of the field
        fail_count = 0
        rotate_time = 15
        spin_started = False
         # Command the vehicle to rotate 360 degrees over 12 seconds
        rot_start_time = 0
        sim_multiplier = 2
        while True:
            err = cam_front.grab(runtime)
            if err == sl.ERROR_CODE.SUCCESS:
        # async for ftemp in cf:
                ftemp = cam_front.retrieve_image(zedImage, sl.VIEW.LEFT, sl.MEM.CPU, imageSize) # Get frame from front camera
                #frame_status, img = (True, bridge.imgmsg_to_cv2(rospy.wait_for_message('/iris_demo/camera/image_raw',Image), desired_encoding="bgr8")) # Acquire image from front camera
                #frame_status, img = (True, bridge.compressed_imgmsg_to_cv2(ftemp, desired_encoding="bgr8")) # Convert front camera frame to OpenCV image
                frame_status = True
                img = zedImage.get_data()
                mtime = time.time() - mission_start_time
                print(img.shape)
                # If vehicle has finished moving to the center of the field, begin survey spin
                if lft_off_task.done() and not spin_started:
                    gd.SetConditionYaw(vehicle, 360, relative = True, speed = 360//rotate_time)
                    rot_start_time = time.time()
                    spin_started = True
                elif lft_off_task.done() and spin_started and (time.time() - rot_start_time) > (sim_multiplier*rotate_time+1):
                    break
                #cv2.imshow('Downward Camera', img)
                if not frame_status:
                    fail_count += 1
                    logger.critical(f"Failed to acquire frame from forward camera. Failed {fail_count} times")
                    continue
                else:
                    fail_count = 0

                pos = vehicle.location.global_relative_frame
                attttt = vehicle.attitude
                Utils.dumpDebugData("v_att", yaw=attttt.yaw, pitch=attttt.pitch , roll=attttt.roll, heading=vehicle.heading, mission_time=mtime)
                pois_seen, centroids, bboxes = poiTracker.processFrame(vehicle, img)
                for row, ridx in zip(centroids, range(centroids.shape[0])):
                     Utils.dumpDebugData("centroids", x=row[0], y=row[1], index=ridx, mission_time=mtime)
                if pois_seen:
                    world_coords = pixCoordToWorldPosition(vehicle, fcam_info, centroids, mission_time=mtime)
                    geoTracker.reportPoi(world_coords, mission_time=mtime)
                
                # if (time.time() - rot_start_time) > (2*rotate_time+1):
                #     break
            else:
                print("Camera Failed line 202")
        cam_front.close()
    PilImage.fromarray(geoTracker.getGrayscaleMap(), 'L').save("poi_heatmap.png")
    

    with MissionContext("POI Visit"):
        start_x, start_y, *_ = utm.from_latlon(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
        path = Utils.calculateVisitPath(geoTracker.getPOIs(), np.array([start_x, start_y]))
        logger.debug("Cam start")
        #NEED SERVO MOVEMENT HERE
        ServoMovement(0)
        cd ==  cam_down.open(init)
        if cd != sl.ERROR_CODE.SUCCESS:
            print(repr(cd))
            zed.close()
            exit(1)
        runtime = sl.RuntimeParameters()
        imageSize = zed.get_camera_information().camera_resolution
        zedImage = sl.Mat(imageSize.width, imageSize.height, sl.MAT_TYPE.U8_C4) 
        #cam_down.subscribe()
        logger.debug("Visit start")
        for target_to_investigate in tqdm(path, desc="Investigating POIs"):
            lat, lon = utm.to_latlon(target_to_investigate[0], target_to_investigate[1], zl, zn)
            logger.debug(f"Visting xy {target_to_investigate[0]}, {target_to_investigate[1]} at lat lon {lat}, {lon}")
            await gd.GoToGlobal(vehicle, [lat, lon] )
            logger.debug(f"Arrived")
            #await cd.__anext__()
            logger.debug("Starting downward scan")
            for i in tqdm(range(30), desc="Looking for logo", leave=False):
                err = cam_down.grab(runtime)
                if err == sl.ERROR_CODE.SUCCESS:
                    ftemp = cam_down.retrieve_image(zedImage, sl.VIEW.LEFT, sl.MEM.CPU, imageSize)
                    #frame_status, img = (True, bridge.compressed_imgmsg_to_cv2(ftemp, desired_encoding="bgr8"))
                    frame_status = True
                    img = zedImage.get_data()
                    logo_found, _ = logoDetector.processFrame(vehicle, img)
                    if logo_found:
                        break
            
            if logo_found:
                break

    
    # At this point, the logo has been found and is within the FOV of the downward camera
    with MissionContext("Landing"):
        # await gd.GoToTargetBody(vehicle, gd.FeetToMeters(75), 0, 0) # TODO: Remove. only for testing
        # gd.GoToTargetBody(vehicle, 0, 0.5, 0) # TODO: Remove. only for testing
        # gd.SetConditionYaw(vehicle, vehicle.heading)

        fail_count = 0
        # Switch vehicle to landing mode and enable "precision" landing
        gd.StartPrecisionLanding(vehicle)
        while True:
        # async for ftemp in cam_down.subscribe():
            frame_acq_start = time.time()
            # Position at start of the frame
            frame_acq_pos = utm.from_latlon(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
            #frame_status, img = (True, bridge.imgmsg_to_cv2(rospy.wait_for_message('/iris_demo/camera/image_raw',Image), desired_encoding="bgr8")) # Acquire image from front camera
            err = cam_down.grab(runtime)              
            if err == sl.ERROR_CODE.SUCCESS:
                ftemp = cam_down.retrieve_image(zedImage, sl.VIEW.LEFT, sl.MEM.CPU, imageSize)
                frame_status = True
                img = zedImage.get_data()

                logging.getLogger("timing").debug(f"Frame took {(time.time()-frame_acq_start)*1000} ms") # Log frame acquisition time
                frame_acq_pos_end = utm.from_latlon(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
                logger.debug(f"Vehicle moved {frame_acq_pos_end[0]-frame_acq_pos[0]} {frame_acq_pos_end[1]-frame_acq_pos[1]} since frame capture")

                mtime = time.time() - mission_start_time
                # print("Down frame acquired")
                # frame_status, img = cam_down.read()
                # Dump data for visualization later in pandas
                Utils.dumpDebugData("main_dump", pitch=vehicle.attitude.pitch, roll=vehicle.attitude.roll, yaw=vehicle.attitude.yaw,
                    lat=vehicle.location.global_relative_frame.lat, lon=vehicle.location.global_relative_frame.lon, alt=vehicle.location.global_relative_frame.alt,
                    heading=vehicle.heading, mission_time=mtime)
                if not frame_status:
                    fail_count += 1
                    logger.critical(f"Failed to acquire frame from downward camera. Failed {fail_count} times")
                    continue
                else:
                    fail_count = 0

                logo_found, logo_center = logoDetector.processFrame(vehicle, img)
                if logo_found:
                    # from dronekit import VehicleMode
                    # vehicle.mode = VehicleMode('LAND')
                    logo_position_relative = pixCoordToRelativePosition(vehicle, down_cam_info, logo_center)
                    logo_x_angle, logo_y_angle = pixCoordToAngle(logo_center, down_cam_info.hfov, down_cam_info.vfov, down_cam_info.resolution[0], down_cam_info.resolution[1])
                    Utils.dumpDebugData("logo_seek", logo_found=logo_found, x_l=logo_center.item(0), y_l=logo_center.item(1),
                        x_lr=logo_position_relative.flatten().item(0), y_lr=logo_position_relative.flatten().item(1),
                        dist=float(np.linalg.norm(logo_position_relative)), mission_time=mtime)
                
                    if np.linalg.norm(logo_position_relative*np.array([1,1,0])) < landing_tolerance:
                        logger.info(f"Horizontal distance of {np.linalg.norm(logo_position_relative)} within {landing_tolerance}m tolerance. Begin Descent")
                        # gd.LandDrone(vehicle)
                
                    gd.UpdateLandingTargetPosition(vehicle, logo_x_angle, logo_y_angle, np.linalg.norm(logo_position_relative))
                    # gd.MoveRelative(vehicle, logo_position_relative[0, [1,0,2]] * np.array([1, 1, 0])*0.5)
                    # gd.SetConditionYaw(vehicle, 0, relative=False)
            cam_down.close()
        # with MissionContext("Final Descent"):
        #     gd.LandDrone(vehicle)


    while True:
        if not logo_found:
            img_front = cam_front.read()
            img_hsv_front = cv2.cvtColor(img_front, cv2.COLOR_BGR2HSV_FULL)

            
            if poiTracker.processFrame(img_hsv_front, vehicle):
                newPath = calculateVisitPath(poiTracker.getUnvisitedPOIs())
                #TODO: Set vehicle waypoints to visit path
        else:
            img_down = cam_down.read()
            logo_found, stat, bbox = detectLogo(img_down, logo_markers)

            if logo_found:
                gd.LandDrone()
                #TODO: Steer drone based of tracking pattern position



if __name__ == "__main__":
    #rospy.init_node('challenge2_main')
    import asyncio
    loop = asyncio.get_event_loop()
    asyncio.ensure_future(mainFunc())
    loop.run_forever()
    #rospy.spin()
    #loop.close()



