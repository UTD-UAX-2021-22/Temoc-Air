from typing import Deque, Tuple
import cv2
import cv2.aruco as aruco # type: ignore
import numpy as np
import os
import enum
import POI
from enum import auto

from collections import deque

optical_flow_feature_params = dict( maxCorners = 100, qualityLevel = 0.3, minDistance = 10, blockSize = 7 )
optical_flow_params = dict( winSize  = (15,15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

class VisModes(enum.Enum):
    NORMAL = auto()
    HUE_DISTANCE = auto()
    HUE_MASK = auto()
    HSV_CHANNELS = auto()
    REGIONS = auto()


def findArucoMarkers(img, markerSize = 4, totalMarkers=250, draw=True, enablePrint=False):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    if enablePrint and (ids is not None):
        print(ids)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs)

def maskHue(hue, tol, img):
    _, res = cv2.threshold(img, hue*(1+tol), 0, cv2.THRESH_TOZERO_INV)
    _, res = cv2.threshold(res, hue*(1-tol), 255, cv2.THRESH_BINARY)
    return res

def segment(img) -> Tuple[int, np.ndarray, np.ndarray]:
    res, labels = cv2.connectedComponents(img)
    value_multi = labels * (255 // res)
    return res, labels, value_multi


if __name__ == "__main__":
    mode: VisModes = VisModes.NORMAL

    hsvChannel = 2
    optical_flow_enabled = True
    enableOverlay = False
    optical_flow_features = []
    enableOpening = True
    targetHue = (5/360)*255
    hueTolerance = 0.08
    cap = cv2.VideoCapture("output1.avi")
    contours_enabled = False

    open_kernel_size = 10
    opening_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_kernel_size, open_kernel_size))

    frame_memory: Deque[np.ndarray] = deque(maxlen=2)
    track = 0
    while True:
        success, img = cap.read()
        #frame_memory.appendleft(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL);
        # findArucoMarkers(img) 
        mskh = maskHue(targetHue, hueTolerance, hsv[:,:,0])
        # frame_memory.appendleft(cv2.cvtColor(mskh, cv2.COLOR_BGR2GRAY))  # Moved to try to print the mask hue image to screen
        if enableOpening:
            mskh = cv2.morphologyEx(mskh, cv2.MORPH_OPEN, opening_kernel)

        imageShow = img
        
        if mode == VisModes.HUE_MASK:
            imageShow = mskh
        elif mode == VisModes.HUE_DISTANCE:
            imageShow = np.abs(hsv[:,:,0]-targetHue)
        elif mode == VisModes.NORMAL:
            imageShow = img
        elif mode == VisModes.HSV_CHANNELS:
            imageShow = hsv[:,:,hsvChannel]
        elif mode == VisModes.REGIONS:
            # mskh = maskHue(targetHue, hueTolerance, hsv[:,:,0])
            num_regions, labels, scaled_labels = segment(mskh)
            if enableOverlay:
                mask = labels != 0
                mapped_regions = cv2.applyColorMap(scaled_labels.astype(np.uint8), cv2.COLORMAP_JET)
                print((img * (1-mask)[..., None]))
                imageShow =  (( img * (1-mask)[..., None] ) + (mapped_regions * mask[..., None])).astype(np.uint8)
            else:
                imageShow = cv2.applyColorMap(scaled_labels.astype(np.uint8), cv2.COLORMAP_JET)
        if optical_flow_enabled:
            imageShow
        
        if contours_enabled:
            contours, _ = cv2.findContours(mskh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [cont for cont in contours if cv2.contourArea(cont) > 10]
            cv2.drawContours(imageShow, contours, -1, (255, 255, 0), 4)


        poi_track = POI.POI_Tracker()


        new_pois_found, centroids, bboxes = poi_track.processFrame(None, imageShow)
        if(new_pois_found):
            # print(new_pois_found)          Debugging poi tracker
            # print(centroids)
            # print(bboxes)
            track+=1
            print("found" + str(track))    
        cv2.imshow('img', imageShow)
        k = cv2.waitKey(1) & 0xff
        
        if k == 27:
            break
        if k == ord('p'):
            x, y, c = hsv.shape
            targetHue = hsv[x//2, y//2, 0]
        if k == ord('s'):
            hsvChannel = (hsvChannel+1) % 3
            mode = VisModes.HSV_CHANNELS
            print(f"Using HSV {hsvChannel}")
        if k == ord('d'):
            mode = VisModes.HUE_DISTANCE
        if k == ord('m'):
            mode = VisModes.HUE_MASK
            print("Masking Hues")
        if k == ord('r'):
            mode = VisModes.REGIONS
        if k == ord('n'):
            mode = VisModes.NORMAL
        if k == ord('o'):
            enableOpening = not enableOpening
        if k == ord('t'):
            optical_flow_enabled = not optical_flow_enabled
            optical_flow_features = cv2.goodFeaturesToTrack(frame_memory[0], mask = mskh, **optical_flow_feature_params)
        if k == ord('c'):
            contours_enabled = not contours_enabled



    cap.release()
    cv2.destroyAllWindows()
