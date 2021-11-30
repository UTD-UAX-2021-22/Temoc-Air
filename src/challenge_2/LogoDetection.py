from dataclasses import dataclass
from typing import Any, Dict, List, Tuple
import cv2
import cv2.aruco as aruco # type: ignore
from enum import Enum, auto
import collections
from dataclasses import dataclass, field
import numpy as np
import argparse


# @dataclass
# class LogoTemplate:

#     def from_file(file_path, **marker_params):
#         img = cv2.imread(file_path)
#         findArucoMarkers(img, **marker_params)
    
    # https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
def template_from_image(image, markersToFind=None, markerSize = 4, totalMarkers=250) -> Tuple[Dict[int, np.ndarray], Tuple[float, float]]:
    corners, ids, _ = findArucoMarkers(image, markerSize=markerSize, totalMarkers=totalMarkers, draw=False)
    print(ids)
    print(ids.shape)
    y, x, _ = image.shape
    return {id:corn.reshape(4,2) for (corn, id) in zip(corners, ids.flatten()) if (markersToFind is None) or (id in markersToFind) }, (x/2, y/2)



def comp_homograph(detected: Dict[int, np.ndarray], template: Dict[int, np.ndarray]):
    common_marker_ids = set(detected.keys()).intersection(set(template.keys()))

    template_points = np.concatenate([template[id] for id in common_marker_ids], axis=0)
    detected_points = np.concatenate([detected[id] for id in common_marker_ids], axis=0)

    homo_matrix, _ = cv2.findHomography(template_points, detected_points, method=cv2.RANSAC)
    return homo_matrix
    


def findArucoMarkers(img, markerSize = 4, totalMarkers=250, detectorParams=aruco.DetectorParameters_create(), draw=False, enablePrint=False):
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = detectorParams
    bboxs, ids, rejected = aruco.detectMarkers(img, arucoDict, parameters = arucoParam)
    if enablePrint and (ids is not None):
        print(ids)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs)

    return bboxs, ids, rejected


class DetectionInfo(Enum):
    DUPLICATE_MARKERS = auto()
    NOT_FOUND = auto()
    NORMAL = auto()

def createMaskFromBoxes(rects, shape, enlargment_size):
    enlarged_rects = [(center, (w + enlargment_size, h + enlargment_size), angle) for center, (w,h), angle in rects]

    color = (255, 255, 255)
    mask = np.zeros(shape, dtype="uint8")

    for rect in enlarged_rects:
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(mask,[box],0,color,-1)

    return mask

@dataclass
class LogoDetector:
    projected_markers: List[Any] = field(default_factory=list, init=False)
    def processFrame(self, frame_buffer: collections.deque, frame=None, frame_gray=None, debug=False) -> Tuple[bool, Any]:
        template_ids = range(5)
        frame = frame_buffer[0]["BGR"]

        original_bboxs, ids_found, rejected = findArucoMarkers(frame)
        if original_bboxs is None or ids_found is None:
            return False, None

        lk_params = dict( winSize  = (15,15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


        for id, original_points, prior_frame_pts in self.projected_markers:
            new_points, stat, _ = cv2.calcOpticalFlowPyrLK(frame_buffer[1]["GRAY"], frame_buffer[0]["GRAY"], prior_frame_pts, None, **lk_params)
            new_orig = original_points[stat==1]
            new_points = new_points[stat==1]
            

        ids_found = ids_found.flatten()

        bboxs: List[Tuple[int, np.ndarray]] = [(id, box) for box, id in zip(original_bboxs, ids_found) if id in template_ids]

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame_gray is None else frame_gray

        aruco_rects = [cv2.minAreaRect(bbox.reshape(4,2)) for _, bbox in bboxs]

        mask = createMaskFromBoxes(aruco_rects, frame.shape[:2], 20)

        masks = [createMaskFromBoxes(rec,  frame.shape[:2], 20) for rec in aruco_rects]



        
        optflow_corners = cv2.goodFeaturesToTrack(frame_gray, 20, 0.3, 5, mask=mask)

        

        return False, None




        




def detectLogo(img, logo_markers, markerSize = 4, totalMarkers=50, draw=True, enablePrint=False):
    # Convert input color image into grayscale
    #TODO: Investigate adaptive thresholding
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # gray = img
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    arucoParam.adaptiveThreshWinSizeMax = 13
    bboxs, ids_found, rejected = aruco.detectMarkers(img, arucoDict, parameters = arucoParam)
    wasFound = False
    detType = DetectionInfo.NOT_FOUND
    
    # detector = LogoDetector()

    # aaaaa = detector.processFrame(img, bboxs, ids_found, rejected)
    # cv2.bitwise_and(img, aaaaa, dst= img)
    if ids_found is None:
        return False, {}, detType
    color = np.random.randint(0,255,(100,3))
    optflow_corners = cv2.goodFeaturesToTrack(gray, 100, 0.3, 5)

    for i, corner in enumerate(optflow_corners):
        a,b = corner.ravel()
        cv2.circle(img,(int(a),int(b)),5,color[i].tolist(), -1)

    ids_found = ids_found.flatten()

    id_set = set(ids_found)
    if set(logo_markers) <= id_set:
        wasFound = True
        detType = DetectionInfo.NORMAL
        if len(id_set) < len(ids_found):
            detType = DetectionInfo.DUPLICATE_MARKERS
            
            
    
    detection_counts = collections.Counter(ids_found) # Count number of times each marker id was detected

    # Create set of ids from the logo that were detected only once
    good_ids = set([x for x in logo_markers if detection_counts[x] == 1] if len(id_set) < len(ids_found) else logo_markers)

    # Create dictionary that maps marker id to bounding box corner coords (for use with comp_homograph)
    detection_dict = { id:bbox.reshape(4,2) for id, bbox in zip(ids_found, bboxs) if id in good_ids}

    if len(good_ids) == 0:
        wasFound = False

    return wasFound, detection_dict, detType


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test logo detection")
    # parser.add_argument('video_input_path', default="rtsp://192.168.137.234:8080/video/h264", nargs='?')
    parser.add_argument('video_input_path', default="mavic_test_11_12_closeup.mp4", nargs='?')
    parser.add_argument('template', default="logo_template_2.png",  nargs='?')
    args = parser.parse_args()

    template_image = cv2.imread(args.template)
    temp_dict, temp_center = template_from_image(template_image)
    print(f"Looking for {list(temp_dict.keys())}")
    marker_ids = range(5)
    cap = cv2.VideoCapture(args.video_input_path)
    text_org = (50, 100)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # detector = LogoDetector()
    # detector.processFrame(template_image)

    while True:
        success, img = cap.read()
        logo_detected, det_dict, detType = detectLogo(img, marker_ids)
        if logo_detected:
            size = img.shape[0:1]
            h_mat = comp_homograph(det_dict, temp_dict)
            # print(np.atleast_2d(np.array(temp_center)))
            new_center: np.ndarray = cv2.perspectiveTransform(np.atleast_3d(np.array(temp_center, dtype='float32')).reshape(-1,1,2), h_mat)
            # print(f"New center {new_center}")
            cv2.circle(img, new_center.astype(np.int32).flatten(), 15, (255, 255, 0), 4)

            cv2.putText(img, f"Logo Found: {logo_detected} // IDs Detected: {det_dict.keys()}", text_org,font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(img, f"Logo Found: {logo_detected}", text_org,font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow('img', img)

        # mask = detector.processFrame(img)
        # if mask is None:
        cv2.imshow('img', img)
        # else:
            # cv2.imshow('img', mask)
        # print(img)
        k = cv2.waitKey(1) & 0xff

        if k == 27:
            break