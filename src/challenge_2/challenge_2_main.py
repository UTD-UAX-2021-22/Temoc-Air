import argparse
import logging
import time
import sys
from Utils import AdvancedLogger, DummyVehicle, calculateVisitPath
import pyzed.sl as sl

from tqdm import tqdm
#import chall2_test
#print(vars(chall2_test))
#import GeneralDroneFunctions
dummyDrone = True # Set to True to bench test and not connect to real drone, False for actual flights
if dummyDrone == True:
    import DummyGeneralFunctions as gd    
else:
    import GeneralDroneFunctions as gd #TODO REANABLE FOR FLIGHT
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
from dronekit import connect
logo_markers = list(range(5))
print("Imports 100%")

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
    parser.add_argument('--template', default="logo_template_final_medium.png",  nargs='?')
    args = parser.parse_args()

    # Load vehicle information. Currently contains only camera data
    vehicle_info = VehicleInfo.parse_file("vehicle_info.json")
    #bridge = CvBridge()
    fcam_info = [c for c in vehicle_info.cameras if c.name == "Forward Camera"][0]
    down_cam_info = [c for c in vehicle_info.cameras if c.name == "Downward Camera"][0]
    fcam_angle = fcam_info.rotation[1] 
    dcam_angle = down_cam_info.rotation[1]
    print("Camera angles are F: " + str(fcam_angle) + " and D: " + str(dcam_angle))
    max_frame_fails = 30
    logger.debug(f"Loading template image {args.template}")
    logo_template_image = cv2.imread(args.template)
    mission_start_time = time.time()
    
    landing_tolerance = 0.1524 # 6 inches

    telem_logger = Utils.AdvancedLogger()
    frame_count = 0

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
        # cam = sl.Camera()
        # init = sl.InitParameters()
        # init.camera_resolution = sl.RESOLUTION.HD720
        # init.camera_fps=60
        # init.depth_mode = sl.DEPTH_MODE.NONE
        # status = cam.open(init)
        # if status != sl.ERROR_CODE.SUCCESS:
        #     print(repr(status))
        #     exit(1)
        # cam_front = cam
        # cam_down = cam

        cam = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD1080
        init.camera_fps=30
        init.depth_mode = sl.DEPTH_MODE.NONE
        status = cam.open(init)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, .01) # very bright day .1-.5 # (0, 100) % of camera frame rate. -1 sets it to auto
        cam.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, -1) #-1 is auto (0,8) possible values 
        cam.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, -1) #(2800, 6500), -1 is auto
        recording_param = sl.RecordingParameters(f'{time.strftime("%Y-%m-%d %H-%M-%S", time.localtime())}.svo', sl.SVO_COMPRESSION_MODE.H265)
        cam.enable_recording(recording_param)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit(1)
        else:
            print("Camera Open")
        cam_front = cam
        cam_down = cam
        zedImage = sl.Mat(cam.get_camera_information().camera_resolution.width, cam.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
        imageSize = cam.get_camera_information().camera_resolution
        print("Connecting to Drone")
        if dummyDrone == True:
            vehicle = DummyVehicle()  
        else:
            vehicle = connect('/dev/ttyTHS2', wait_ready=True, baud=1500000) # TODO REANABLE FOR FLIGHT
        #vehicle = gd.ConnectToCopter('dev/ttyTHS2')
        print("Connected")
        Utils.setUpTelemetryLog(vehicle, telem_logger)


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
        [-53.333/4, 0, 0],
        [-53.333/4, 50, 0],
        [53.333/4, 50, 0],
        [53.333/4, 0, 0]
    ])

    # Allocate space in a numpy array for home position in UTM coords
    base_pos = np.zeros(2)
    # Takeoff
    with MissionContext("Ascent"):
              
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
        if dummyDrone == False:
            vehicle.parameters['ANGLE_MAX'] = 30*100 # Angle in centidegress TODO REANABLE FOR FLIGHT
        await asyncio.sleep(5)
        #print("Sleep Done")
        gd.ArmDrone(vehicle) # Arm Vehicle
        gd.ServoMovement(vehicle, 90+fcam_angle)
        async def liftOffAndMoveToCenter():
            print("Takeoff")
            await gd.TakeOffDrone(vehicle, 7.62)
            print("Goto body")
            await gd.GoToTargetBody(vehicle, gd.FeetToMeters(75), 0, 0)
            print("Finished Liftoff and Move to Center")
            
        lft_off_task = asyncio.create_task(liftOffAndMoveToCenter())
        await asyncio.sleep(5)

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
    gd.ServoMovement(vehicle, 90+fcam_angle)
    # err = cam.grab(status)
    # if err != sl.ERROR_CODE.SUCCESS:
    #     print(repr(err))
    #     exit(1)
    # runtime = cam.RuntimeParameters()
    # #imageSize = cam.get_camera_information().camera_resolution
    # zedImage = cam.Mat(round(cam.get_camera_information().camera_resolution.width, 2), cam.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
    with MissionContext("POI Search"):
        # await gd.GoToTargetBody(vehicle, gd.FeetToMeters(75), 0, 0) # Move forward to the middle of the field
        fail_count = 0
        rotate_time = 26
        spin_started = False
         # Command the vehicle to rotate 360 degrees over 12 seconds
        rot_start_time = 0
        sim_multiplier = 1
        print("Await liftoff task")
        while not (lft_off_task.done()):
            print("Await liftoff task")
            await asyncio.sleep(1)
        print("Liftoff task Done")
        while True:
            err = cam.grab(status)
            telem_logger.writeValues(frame_count=frame_count)
            frame_count += 1
            if err == sl.ERROR_CODE.SUCCESS:
        # async for ftemp in cf:
                ftemp = cam.retrieve_image(zedImage, sl.VIEW.LEFT, sl.MEM.CPU, imageSize) # Get frame from front camera
                #frame_status, img = (True, bridge.imgmsg_to_cv2(rospy.wait_for_message('/iris_demo/camera/image_raw',Image), desired_encoding="bgr8")) # Acquire image from front camera
                #frame_status, img = (True, bridge.compressed_imgmsg_to_cv2(ftemp, desired_encoding="bgr8")) # Convert front camera frame to OpenCV image
                frame_status = True
                img = zedImage.get_data()
                mtime = time.time() - mission_start_time
                #print(img.shape)
                # If vehicle has finished moving to the center of the field, begin survey spin
                #print("If statement to spin")
                if lft_off_task.done() and not spin_started:
                    print("Spin")
                    gd.SetConditionYaw(vehicle, 360, relative = True, speed = 15)#speed = 360//rotate_time) Commented part makes speed 24 deg/sec
                    rot_start_time = time.time()
                    spin_started = True
                elif lft_off_task.done() and spin_started and (time.time() - rot_start_time) > (sim_multiplier*rotate_time+1):
                    break
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
                telem_logger.writeValues(centroid_np=centroids, centroid_bboxes=bboxes)
                if dummyDrone == True:
                    cv2.imshow('Downward Camera', img)
                    cv2.waitKey(1)
                for row, ridx in zip(centroids, range(centroids.shape[0])):
                     #print("commented utils")
                     Utils.dumpDebugData("centroids", x=row[0], y=row[1], index=ridx, mission_time=mtime)
                     telem_logger.writeValues(centroid=dict(x=row[0], y=row[1], index=ridx))
                if pois_seen:
                    world_coords = pixCoordToWorldPosition(vehicle, fcam_info, centroids, mission_time=mtime)
                    geoTracker.reportPoi(world_coords, mission_time=mtime)
                # else: 
                #     print("No POI's Seen")
                
                # if (time.time() - rot_start_time) > (2*rotate_time+1):
                #     break
            else:
                print("Camera Failed line 202")
        #cam_front.close()
    #PilImage.fromarray(geoTracker.getGrayscaleMap(), 'L').save("poi_heatmap.png")
    

    with MissionContext("POI Visit"):
        start_x, start_y, *_ = utm.from_latlon(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
        path = Utils.calculateVisitPath(geoTracker.getPOIs(), np.array([start_x, start_y]))
        logger.debug("Cam start")
        #NEED SERVO MOVEMENT HERE
        gd.ServoMovement(vehicle, 90+dcam_angle)
        err = cam.grab(status)
        telem_logger.writeValues(frame_count=frame_count)
        frame_count += 1
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            exit(1)
        # runtime = sl.RuntimeParameters()
        # imageSize = sl.get_camera_information().camera_resolution
        # zedImage = sl.Mat(imageSize.width, imageSize.height, sl.MAT_TYPE.U8_C4) 
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
                err = cam.grab(status)
                telem_logger.writeValues(frame_count=frame_count)
                frame_count += 1
                if err == sl.ERROR_CODE.SUCCESS:
                    ftemp = cam.retrieve_image(zedImage, sl.VIEW.LEFT_GRAY, sl.MEM.CPU, imageSize)
                    #frame_status, img = (True, bridge.compressed_imgmsg_to_cv2(ftemp, desired_encoding="bgr8"))
                    frame_status = True
                    img = zedImage.get_data()
                    logo_found, _ = logoDetector.processFrame(vehicle, img)
                    if dummyDrone == True:
                        cv2.imshow('Downward Camera', img)
                        cv2.waitKey(1)
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
            err = cam.grab(status)
            telem_logger.writeValues(frame_count=frame_count)
            frame_count += 1         
            if err == sl.ERROR_CODE.SUCCESS:
                ftemp = cam.retrieve_image(zedImage, sl.VIEW.LEFT_GRAY, sl.MEM.CPU, imageSize)
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
                    cv2.circle(img, logo_center.astype(np.int32).flatten(), 15, (255, 255, 0), 4)
                    logo_position_relative = pixCoordToRelativePosition(vehicle, down_cam_info, logo_center)
                    logo_x_angle, logo_y_angle = pixCoordToAngle(logo_center, down_cam_info.hfov, down_cam_info.vfov, down_cam_info.resolution[0], down_cam_info.resolution[1])
                    Utils.dumpDebugData("logo_seek", logo_found=logo_found, x_l=logo_center.item(0), y_l=logo_center.item(1),
                        x_lr=logo_position_relative.flatten().item(0), y_lr=logo_position_relative.flatten().item(1),
                        dist=float(np.linalg.norm(logo_position_relative)), mission_time=mtime)
                    telem_logger.writeValues(logo_info=dict(logo_found=logo_found, x_l=logo_center.item(0), y_l=logo_center.item(1),
                        x_lr=logo_position_relative.flatten().item(0), y_lr=logo_position_relative.flatten().item(1),
                        dist=float(np.linalg.norm(logo_position_relative))))
                
                    if np.linalg.norm(logo_position_relative*np.array([1,1,0])) < landing_tolerance:
                        logger.info(f"Horizontal distance of {np.linalg.norm(logo_position_relative)} within {landing_tolerance}m tolerance. Begin Descent")
                        # gd.LandDrone(vehicle)
                
                    gd.UpdateLandingTargetPosition(vehicle, logo_x_angle, logo_y_angle, np.linalg.norm(logo_position_relative))
                    # gd.MoveRelative(vehicle, logo_position_relative[0, [1,0,2]] * np.array([1, 1, 0])*0.5)
                    # gd.SetConditionYaw(vehicle, 0, relative=False)
                if dummyDrone == True:
                    cv2.imshow('Downward Camera', img)
                    cv2.waitKey(1)
        cam.disable_recording()
        cam.close()
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
