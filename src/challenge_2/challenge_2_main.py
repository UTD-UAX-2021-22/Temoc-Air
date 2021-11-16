import GeneralDroneFunctions as gd
from challenge_2.LogoDetection import detectLogo
from challenge_2.POI import POI_Tracker
import cv2

logo_markers = list(range(5))

def calculateVisitPath(pois):
    #TODO: Create optimized visit path
    #TODO: Perfrom any necessary lat-long conversions
    return []


#TODO: Make actually work
#TODO: Fallback logic 
if __name__ == "__main__":
    poiTracker = POI_Tracker()
    cam_front = cv2.VideoCapture(0)
    cam_down = cv2.VideoCapture(1)
    vehicle = gd.ConnectToCopter()
    gd.ArmDrone()
    gd.TakeOffDrone(7.62)

    logo_found = False
    
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







