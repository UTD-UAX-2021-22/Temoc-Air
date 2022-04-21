import argparse
from itertools import count
import logging
from typing import Any, Dict, Tuple
import cv2
from dataclasses import dataclass, field
import functools

import numpy as np
from Utils import setupLoggers
# from challenge_2.VisionTests import maskHue
from cv2.legacy import TrackerMOSSE_create #type: ignore


def maskHue(hue, tol, img):
    _, res = cv2.threshold(img, hue*(1+tol), 0, cv2.THRESH_TOZERO_INV)
    _, res = cv2.threshold(res, hue*(1-tol), 255, cv2.THRESH_BINARY)
    return res

class DefaultVars:
    optical_flow_feature_params = dict( maxCorners = 100, qualityLevel = 0.3, minDistance = 10, blockSize = 7 )
    optical_flow_params = dict( winSize  = (15,15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# @dataclass
# class HueDetector:
#     target_hue: float
#     tolerance: float
#     kernel_size: int
    
#     def processFrame(self, frame):
#     _, res = cv2.threshold(img, hue*(1+tol), 0, cv2.THRESH_TOZERO_INV)
#     _, res = cv2.threshold(res, hue*(1-tol), 255, cv2.THRESH_BINARY)
#     return res

def bbox_union(a,b):
  x = min(a[0], b[0])
  y = min(a[1], b[1])
  w = max(a[0]+a[2], b[0]+b[2]) - x
  h = max(a[1]+a[3], b[1]+b[3]) - y
  return (x, y, w, h)

def bbox_intersection(a,b):
  x = max(a[0], b[0])
  y = max(a[1], b[1])
  w = min(a[0]+a[2], b[0]+b[2]) - x
  h = min(a[1]+a[3], b[1]+b[3]) - y
  if w<0 or h<0: return (0,0,0,0) # or (0,0,0,0) ?
  return (x, y, w, h)

def bbox_area(a):
    _, _, w, h = a
    return w*h

@dataclass
class Hue_Detector_Opts:
    target_hue: float = 300#300 #353 #220#349#330#302
    hue_range: Tuple[float, float] = (270,320) #2680,320#240,320#(330,360)#(290, 315) (0,360)->usingForBlackmarkers
    tolerance: float = .065#.1#.009#.006#.1 #.005
    opening_size: int = 10
    value_range: Tuple[float, float] =  (.5*255,255)#(1,255)#(0.5*255, 255)
    # saturation_range: Tuple[float, float] = (0.1*255, 255)
    saturation_range: Tuple[float, float] = (40,255)#(40,200)#(100,255)#(50,200)#(0, 10)

    def hueInHSV(self):
        return (self.target_hue / 360)*255

    def hueRange_HSVFULL(self):
        return tuple([(x/360)*255 for x in self.value_range])

    def asUpperLower(self):
        hl, hu = self.hueRange_HSVFULL()
        return ((self.hue_range[0]/360)*255, self.saturation_range[0] , self.value_range[0]), ((self.hue_range[1]/360)*255, self.saturation_range[1], self.value_range[1])


@functools.cache
def getOpeningKernel(kern_size):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kern_size, kern_size))


def geoRegister(pix_coords, vehicle_info):
    #TODO: Geo registration (plane interception, lat-long conversions, etc)
    return [1.0, 1.0]


def drawCountorAABBox(cnt, img, color=(0,255,0)):
    x,y,w,h = cv2.boundingRect(cnt)
    cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0), 2)

@dataclass
class POI_Tracker:
    new_poi_found: bool = field(default=False, init=False)
    detected_pois: list[Any] = field(default_factory=list, init=False)
    poi_trackers: Dict[int, Any] = field(default_factory=dict, init=False)
    highest_assigned_id: int = field(default=1, init=False)
    minimum_area: int = 4
    hue_detect_params: Hue_Detector_Opts = Hue_Detector_Opts()
    flow_feature_params: Dict = field(default_factory=lambda: DefaultVars.optical_flow_feature_params.copy())
    flow_params: Dict = field(default_factory=lambda: DefaultVars.optical_flow_params.copy())
    
    def processFrame(self, vehicle, frame_bgr, frame_hsv=None):# -> Tuple[bool, Any]:
        """Process a frame of HSV video and update POI information/detect new POIs
        :param frame: Captured frame in BGR format
        :param vehicle: Dronekit vehicle object
        """
        logger = logging.getLogger(__name__)
        if frame_hsv is None:
            frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV_FULL)


        mask = maskHue(self.hue_detect_params.hueInHSV(), self.hue_detect_params.tolerance, frame_hsv[:,:,0])

        # _, mask = cv2.threshold(frame_hsv[:,:,0], self.hue_detect_params.hueRange_HSVFULL()[1], 0, cv2.THRESH_TOZERO_INV)
        # _, mask = cv2.threshold(mask, self.hue_detect_params.hueRange_HSVFULL()[0], 255, cv2.THRESH_BINARY)

        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, getOpeningKernel(self.hue_detect_params.opening_size))


        
        enable_sat = True
        enable_value = True
        draw_contours = True
        in_range = False
        poi_optical_tracking = False
        occupied_ratio_test = False
        minOccupancyRatio = 0.7

        if in_range:
            lower, upper = self.hue_detect_params.asUpperLower()
            mask = cv2.inRange(frame_hsv, np.array(list(lower)), np.array(list(upper)))

        if enable_sat:
            _, sat_mask = cv2.threshold(frame_hsv[:,:,1], self.hue_detect_params.saturation_range[1], 0, cv2.THRESH_TOZERO_INV)
            _, sat_mask = cv2.threshold(sat_mask, self.hue_detect_params.saturation_range[0], 255, cv2.THRESH_BINARY)
            mask = cv2.bitwise_and(mask, sat_mask)

        if enable_value:
            _, value_mask = cv2.threshold(frame_hsv[:,:,2], self.hue_detect_params.value_range[1], 0, cv2.THRESH_TOZERO_INV)
            _, value_mask = cv2.threshold(value_mask, self.hue_detect_params.value_range[0], 255, cv2.THRESH_BINARY)
            mask = cv2.bitwise_and(mask, value_mask)

        # num_regions, labels = cv2.connectedComponents(mask)

        def rotatedRectArea(rrdat):
            _, (w,h), _ = rrdat
            return w*h
        #im = frame_bgr
        #imgray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        #ret, thresh = cv2.threshold(imgray, 0, 255, 0)
        #contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if occupied_ratio_test:
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug(f"Rejected by area: {len([c for c in contours if cv2.contourArea(c) <= self.minimum_area])}")
                logger.debug(f"Rejected by occupancy {len([c for c in contours if cv2.contourArea(c) > 0 and cv2.contourArea(c)/rotatedRectArea(cv2.minAreaRect(c)) < minOccupancyRatio])}")
                

            contours = [c for c in contours if cv2.contourArea(c) > self.minimum_area and cv2.contourArea(c)/rotatedRectArea(cv2.minAreaRect(c)) >= minOccupancyRatio]
        else:
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug(f"Rejected by area: {len([c for c in contours if cv2.contourArea(c) <= self.minimum_area])}")
            contours = [cont for cont in contours if cv2.contourArea(cont) > self.minimum_area]
        
        detected_poi_bboxes = [cv2.boundingRect(x) for x in contours]
        
        centroids = np.zeros( (len(contours), 2))
        for i, cnt in enumerate(contours):
            cm = cv2.moments(cnt)
            centroids[i, 0] = int(cm['m10']/cm['m00'])
            centroids[i, 1] = int(cm['m01']/cm['m00'])

        if draw_contours:
            cv2.drawContours(frame_bgr, contours, -1, (255, 255, 0), 2)

            #cv2.drawContours(frame_bgr, contours, -1, (0,255,0), 3)
			
            for c in contours:
                drawCountorAABBox(c, frame_bgr)

        if poi_optical_tracking:
            
            new_bboxes = {}
            failed_tracker_ids = [];
            for id, tracker in self.poi_trackers.items():
                res, bbox = tracker.update(frame_bgr)
                if res:
                    new_bboxes[id] = bbox
                else:
                    failed_tracker_ids.append(id)

            for id in failed_tracker_ids:
                del self.poi_trackers[id]

            for det_bbox in detected_poi_bboxes:
                if all([ (bbox_area(bbox_intersection(x, det_bbox)) / bbox_area(det_bbox)) < 0.80 for x in new_bboxes.values()]):
                    new_id = self.highest_assigned_id
                    self.highest_assigned_id += 1
                    new_tracker = TrackerMOSSE_create()
                    new_tracker.init(frame_bgr, det_bbox)
                    self.poi_trackers[new_id] = new_tracker
                    
            # for track_bbox in new_bboxes.values:

        #TODO: Image Segmentation? - In progress
        #TODO: POI tracking - In progress
                
        #TODO: Georegistration
            
        return len(detected_poi_bboxes)>0, centroids, detected_poi_bboxes

    def getUnvisitedPOIs(self):
        return detected_pois;
        





if __name__ == "__main__":
    setupLoggers(filename="poi_detect_test")
    parser = argparse.ArgumentParser(description="Test Far Field Logo detection")
    parser.add_argument('--video', default="2021_11_12_10_58_15.mp4", nargs='?')
    # parser.add_argument('template', default="logo_template_2.png",  nargs='?')
    args = parser.parse_args()
    draw_centroids = True

    cap = cv2.VideoCapture(args.video)
    poi_track = POI_Tracker()

    while True:
        success, img = cap.read()
        new_pois_found, centroids, bboxes = poi_track.processFrame(None, img)
        if draw_centroids:
            # print(f"Centroids: {centroids}")
            for i in range(centroids.shape[0]):
                cv2.circle(	img, centroids[i,:].astype(np.uintc), 10, (0,0,255) ,2 )

        cv2.imshow('img', img)

        k = cv2.waitKey(1) & 0xff

        if k == 27:
            break
