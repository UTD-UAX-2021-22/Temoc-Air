import argparse
import logging
import GeneralDroneFunctions as gd
from LogoDetection import detectLogo
from POI import POI_Tracker
import cv2
from Utils import MissionContext, setupLoggers, VehicleInfo
import Utils
from LogoDetection import LogoDetector
from GeoTracker import GeoTracker
from Utils import pixCoordToRelativePosition, pixCoordToWorldPosition
import numpy as np

logo_markers = list(range(5))




#TODO: Make actually work
#TODO: Fallback logic 
if __name__ == "__main__":
    setupLoggers("challenge_2")
    logger = logging.getLogger(__name__)

    parser = argparse.ArgumentParser(description="Test logo detection")
    # parser.add_argument('video_input_path', default="rtsp://192.168.137.234:8080/video/h264", nargs='?')
    # parser.add_argument('--video', default="mavic_test_11_12_closeup.mp4", nargs='?')
    parser.add_argument('--template', default="logo_template_2.png",  nargs='?')
    args = parser.parse_args()

    vehicle_info = VehicleInfo.parse_file("vehicle_info.json")

    fcam_info = [c for c in vehicle_info.cameras if c.name == "Forward Camera"][0]
    down_cam_info = [c for c in vehicle_info.cameras if c.name == "Downward Camera"][0]
    max_frame_fails = 30
    logger.debug(f"Loading template image {args.template}")
    logo_template_image = cv2.imread(args.template)

    landing_tolerance = 0.1524 # 6 inches

    # Initialize Trackers/detectors
    poiTracker = POI_Tracker()
    logoDetector = LogoDetector(logo_template_image)
    geoTracker = GeoTracker((30, 5), resolution=3, dims=(60, 60))

    # Begin mission
    with MissionContext("Startup"):
        # poiTracker = POI_Tracker()
        cam_front = cv2.VideoCapture(fcam_info.id)
        cam_down = cv2.VideoCapture(down_cam_info.id)
        vehicle = gd.ConnectToCopter()

    # Takeoff
    with MissionContext("Ascent"):
        gd.ArmDrone(vehicle)
        gd.TakeOffDrone(vehicle, 7.62)

    logo_found = False
    
    with MissionContext("POI Search"):
        gd.GoToTargetBody(vehicle, gd.FeetToMeters(75), 0, 0) # Move forward to the middle of the field
        fail_count = 0
        gd.SetConditionYaw(vehicle, 360, relative = True, speed = 360//12) # Command the vehicle to rotate 360 degrees over 12 seconds
        while True:
            frame_status, img = cam_front.read() # Acquire image from front camera

            if not frame_status:
                fail_count += 1
                logger.critical(f"Failed to acquire frame from forward camera. Failed {fail_count} times")
                continue
            else:
                fail_count = 0

            pos = vehicle.location.global_relative_frame
            pois_seen, centroids, bboxes = poiTracker.processFrame(vehicle, img)
            if pois_seen:
                world_coords = pixCoordToWorldPosition(vehicle, fcam_info, centroids)
                geoTracker.reportPoi(world_coords)
                break
    
    with MissionContext("POI Visit"):
        path = Utils.calculateVisitPath(geoTracker.getPOIs())
        gd.GoToGlobal(vehicle, coords)
    
    with MissionContext("Landing"):
        fail_count = 0
        while True:
            frame_status, img = cam_down.read()
            if not frame_status:
                fail_count += 1
                logger.critical(f"Failed to acquire frame from downward camera. Failed {fail_count} times")
                continue
            else:
                fail_count = 0

            logo_found, logo_center = logoDetector.processframe(vehicle, img)
            if logo_found:
                logo_position_relative = pixCoordToRelativePosition(vehicle, down_cam_info, logo_center)
                if np.linalg.norm(logo_position_relative) < landing_tolerance:
                    logging.info(f"Horizontal distance of {np.linalg.norm(logo_position_relative)} within {landing_tolerance}m tolerance. Begin Descent")
                    break
                gd.MoveRelative(vehicle, logo_position_relative)

        with MissionContext("Final Descent"):
            gd.LandDrone(vehicle)



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







