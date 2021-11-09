import cv2
import cv2.aruco as aruco # type: ignore
from enum import Enum, auto


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


class DetectionInfo(Enum):
    DUPLICATE_MARKERS = auto()
    NOT_FOUND = auto()
    NORMAL = auto()


def detectLogo(img, logo_markers, markerSize = 4, totalMarkers=250, draw=True, enablePrint=False):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    wasFound = False
    detType = DetectionInfo.NOT_FOUND
    
    id_set = set(ids)
    if set(logo_markers) <= id_set:
        wasFound = True
        detType = DetectionInfo.NORMAL
        if len(id_set) < len(ids):
            detType = DetectionInfo.DUPLICATE_MARKERS

    return wasFound, detType, bboxs


