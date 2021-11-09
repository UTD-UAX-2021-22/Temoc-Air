from typing import Any, Dict
import cv2
from dataclasses import dataclass, field
import functools
from challenge_2.VisionTests import maskHue


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


@dataclass
class Hue_Detector_Opts:
    target_hue: float = 342
    tolerance: float = 0.05
    opening_size: int = 10

    def hueInHSV(self):
        return (self.target_hue /360)*255


@functools.cache
def getOpeningKernel(kern_size):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kern_size, kern_size))


def geoRegister(pix_coords, vehicle_info):
    #TODO: Geo registration (plane interception, lat-long conversions, etc)
    return [1.0, 1.0]


@dataclass
class POI_Tracker:
    new_poi_found: bool = field(default=False, init=False)
    detected_pois: list[Any] = field(default=[], init=False);
    hue_detect_params: Hue_Detector_Opts = Hue_Detector_Opts()
    flow_feature_params: Dict = DefaultVars.optical_flow_feature_params
    flow_params: Dict = DefaultVars.optical_flow_params
    
    def processFrame(self, frame, vehicle) -> bool:
        """
        Process a frame of HSV video and update POI information/detect new POIs

        :return: true if new POI was found
        """
        mask = maskHue(self.hue_detect_params.target_hue, self.hue_detect_params.tolerance, frame)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, getOpeningKernel(self.hue_detect_params.opening_size))

        num_regions, labels = cv2.connectedComponents(mask)

        #TODO: Image Segmentation?
        #TODO: Georegistration
        #TODO: POI tracking
        return True

    def getUnvisitedPOIs(self):
        return [];
        