from typing import Any, Dict
import cv2
from dataclasses import dataclass, field
import functools
from challenge_2.VisionTests import maskHue
from cv2.legacy import TrackerMOSSE_create #type: ignore


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
    target_hue: float = 342
    tolerance: float = 0.05
    opening_size: int = 10

    def hueInHSV(self):
        return (self.target_hue / 360)*255


@functools.cache
def getOpeningKernel(kern_size):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kern_size, kern_size))


def geoRegister(pix_coords, vehicle_info):
    #TODO: Geo registration (plane interception, lat-long conversions, etc)
    return [1.0, 1.0]


@dataclass
class POI_Tracker:
    new_poi_found: bool = field(default=False, init=False)
    detected_pois: list[Any] = field(default=[], init=False)
    poi_trackers: Dict[int, Any] = field(default={}, init=False)
    highest_assigned_id = field(default=1, init=False)
    minimum_area: int = 10
    hue_detect_params: Hue_Detector_Opts = Hue_Detector_Opts()
    flow_feature_params: Dict = DefaultVars.optical_flow_feature_params
    flow_params: Dict = DefaultVars.optical_flow_params
    
    def processFrame(self, vehicle, frame_bgr, frame_hsv=None) -> bool:
        """Process a frame of HSV video and update POI information/detect new POIs
        :param frame: Captured frame in BGR format
        :param vehicle: Dronekit vehicle object
        """
        if frame_hsv is None:
            frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV_FULL)


        mask = maskHue(self.hue_detect_params.hueInHSV(), self.hue_detect_params.tolerance, frame_hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, getOpeningKernel(self.hue_detect_params.opening_size))

        num_regions, labels = cv2.connectedComponents(mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cont for cont in contours if cv2.contourArea(cont) > self.minimum_area]
        
        detected_poi_bboxes = [cv2.boundingRect(x) for x in contours]
        
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
            
        return True

    def getUnvisitedPOIs(self):
        return [];
        