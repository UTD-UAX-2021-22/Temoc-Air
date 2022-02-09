import argparse
import logging
import GeneralDroneFunctions as gd
from LogoDetection import detectLogo
from POI import POI_Tracker
import cv2
from Utils import MissionContext, setupLoggers
from LogoDetection import LogoDetector
from GeoTracker import GeoTracker

logo_markers = list(range(5))

def calculateVisitPath(pois):
    #TODO: Create optimized visit path
    #TODO: Perfrom any necessary lat-long conversions
    return []


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


    logger.debug(f"Loading template image {args.template}")
    logo_template_image = cv2.imread(args.template)

    poiTracker = POI_Tracker()
    logoDetector = LogoDetector(logo_template_image)
    geoTracker = GeoTracker((30, 5), resolution=3, dims=(60, 60))

    with MissionContext("Startup"):
        poiTracker = POI_Tracker()
        cam_front = cv2.VideoCapture(0)
        cam_down = cv2.VideoCapture(1)
        vehicle = gd.ConnectToCopter()

    with MissionContext("Ascent"):
        gd.ArmDrone(vehicle)
        gd.TakeOffDrone(vehicle, 7.62)

    logo_found = False
    
    with MissionContext("POI Search"):
        gd.GoToTargetBody(vehicle, gd.FeetToMeters(75), 0, 0)
        pos = vehicle.location.global_relative_frame


    
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







