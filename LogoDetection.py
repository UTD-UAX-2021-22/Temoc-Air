from dataclasses import dataclass
from typing import Any, Dict, List, Tuple
import cv2
from Utils import setupLoggers
import cv2.aruco as aruco # type: ignore
from enum import Enum, auto
import collections
from dataclasses import dataclass, field
import numpy as np
import argparse
import logging
import time
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


def detectLogo(img, logo_markers, board, markerSize = 4, totalMarkers=50, draw=True, enablePrint=False):
    logger = logging.getLogger(f"{__name__}")
    # logger.debug("Testing")
    # Convert input color image into grayscale
    #TODO: Investigate adaptive thresholding
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # img = gray
    # gray = img
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    # arucoParam.adaptiveThreshWinSizeMax = 13
    bboxs, ids_found, rejected = aruco.detectMarkers(img, arucoDict, parameters = arucoParam)
    bboxs, ids_found, rejected, recovered_idx = aruco.refineDetectedMarkers(img, board, bboxs, ids_found, rejected)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs)
    wasFound = False
    detType = DetectionInfo.NOT_FOUND
    
    # detector = LogoDetector()

    # aaaaa = detector.processFrame(img, bboxs, ids_found, rejected)
    # cv2.bitwise_and(img, aaaaa, dst= img)
    if ids_found is None:
        return False, {}, detType, 0
    color = np.random.randint(0,255,(100,3))
    # optflow_corners = cv2.goodFeaturesToTrack(gray, 100, 0.3, 5)

    # for i, corner in enumerate(optflow_corners):
    #     a,b = corner.ravel()
    #     cv2.circle(img,(int(a),int(b)),5,color[i].tolist(), -1)

    ids_found = ids_found.flatten()

    id_set = set(ids_found)
    logo_markers = logo_markers.flatten().tolist() 
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

    return wasFound, detection_dict, detType, len(ids_found)


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

# @dataclass
class LogoDetector:
    projected_markers: List[Any] = field(default_factory=list, init=False)

    def __init__(self, template) -> None:
        """
        template: Image of logo 
        """
        self.logger = logging.getLogger(__name__)
        self.template = template
        self.template_bbox, self.template_ids, _ = findArucoMarkers(template)
        self.homography_dict, self.homography_center = template_from_image(template) 
        self.logger.debug(f"Template processed. Ids: ({self.template_ids}) BBOXS: ({self.template_bbox})")
        self.cv_board = createBoardFromTemplateImage(template)
        self.logger.debug(f"Created OpenCV board from template")
        pass

    def processFrame(self, vehicle, frame_bgr) -> Tuple[bool, Any, Any]:
        logo_detected, det_dict, detType, num_found = detectLogo(frame_bgr, self.template_ids, self.cv_board, draw=False, totalMarkers=250)
        if logo_detected:
            h_mat = comp_homograph(det_dict, self.homography_dict)
            # print(np.atleast_2d(np.array(temp_center)))
            original_bbox = np.atleast_3d(np.array([
                [self.homography_center[0]*2, self.homography_center[1]*2],
                [0, self.homography_center[1]*2],
                [self.homography_center[0]*2, 0],
                [0,0]
            ], dtype="float32")).reshape(-1,1,2)
            new_center: np.ndarray = cv2.perspectiveTransform(np.atleast_3d(np.array(self.homography_center, dtype='float32')).reshape(-1,1,2), h_mat)
            new_bbox = cv2.perspectiveTransform(original_bbox, h_mat)
            new_bbox = new_bbox[:,0,:]
            bbox_max = np.max(new_bbox, axis=0)
            bbox_min = np.min(new_bbox, axis=0)
            print(f"New bbox {new_bbox}")
            print(f"Max: {bbox_max} -- Min: {bbox_min}")
            return True, new_center.flatten(), [bbox_min[0], bbox_min[1], bbox_max[0] - bbox_min[0], bbox_max[1] - bbox_min[1]]
        else:
            return False, np.array([0,0]), [0,0,0,0]
        


def createBoardFromTemplateImage(img, markerSize = 4, totalMarkers=250):
    bboxs, ids, rejected = findArucoMarkers(img)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    markerDict = aruco.Dictionary_get(key)

    bn = np.array(bboxs)
    bn = bn.squeeze()
    bn = np.array([bn])
    logging.getLogger().debug(f"Detected 2 bboxs from template. IDS: ({ids})   ({bn.shape}): {bn}")
    bn = np.pad(bn, ((0,0), (0,0), (0, 1)), 'constant', constant_values=(123, 0))
    logging.getLogger().debug(f"Detected bboxs from template. IDS: ({ids})   ({bn.shape}): {bn}")
    return cv2.aruco.Board_create(bn, markerDict, ids)



if __name__ == "__main__":
    setupLoggers(filename='logo_detect_test')

    # create logger
    logger = logging.getLogger(__name__)
    # logger.setLevel(logging.DEBUG)
    # import logging.handlers
    # handler = logging.handlers.RotatingFileHandler('logs/logo_detect_test.log', backupCount=10)
    # # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    # formatter = logging.Formatter('%(asctime)s - %(levelname)s::%(name)s - %(filename)s::%(funcName)s::%(lineno)d - %(message)s')
    # handler.setFormatter(formatter)
    # logger.addHandler(handler)
    # handler.doRollover()
    
    parser = argparse.ArgumentParser(description="Test logo detection")
    # parser.add_argument('video_input_path', default="rtsp://192.168.137.234:8080/video/h264", nargs='?')
    parser.add_argument('--video', default="mavic_test_11_12_closeup.mp4", nargs='?')
    parser.add_argument('--template', default="logo_template_2.png",  nargs='?')
    args = parser.parse_args()


    logger.debug(f"Loading template image {args.template}")
    template_image = cv2.imread(args.template)
    board = createBoardFromTemplateImage(template_image)
    temp_dict, temp_center = template_from_image(template_image)
    print(f"Looking for {list(temp_dict.keys())}")
    marker_ids = range(4)
    cap = cv2.VideoCapture(args.video)
    text_org = (50, 100)
    font = cv2.FONT_HERSHEY_SIMPLEX
    detector = LogoDetector(template_image)
    

    # detector = LogoDetector()
    # detector.processFrame(template_image)
    frame_number = 0
    logo_tracker = None
    while True:
        success, img = cap.read()
        logo_detected, center, bbox = detector.processFrame(None, img)

        if logo_detected:
            cv2.putText(img, f"Logo Found: {logo_detected}", text_org,font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.circle(img, np.asarray(center).astype(np.int32).flatten(), 15, (255, 255, 0), 4)
            tracked_bbox = None
            # if logo_tracker is None:
            #     logo_tracker = cv2.legacy.TrackerMedianFlow_create()
            #     logo_tracker.init(img, bbox)
            # else:
            #     track_ok, tracked_bbox = logo_tracker.update(img)

            # logo_tracker = cv2.legacy.TrackerMedianFlow_create()
            logo_tracker = cv2.legacy.TrackerMedianFlow_create()
            logo_tracker.init(img, bbox)
            tracked_bbox = None
            # track_ok, tracked_bbox = logo_tracker.update(img)

            if tracked_bbox is not None and track_ok:
                # print(tracked_bbox)
                cv2.rectangle(img, (int(tracked_bbox[0]), int(tracked_bbox[1])), (int(tracked_bbox[0]+ tracked_bbox[2]), int(tracked_bbox[1] + tracked_bbox[3])), (255, 255, 0), 4)
                cv2.circle(img, (int(tracked_bbox[0]+ tracked_bbox[2]/2), int(tracked_bbox[1] + tracked_bbox[3]/2)), 15, (0, 0, 255), 4)
            
            cv2.rectangle(img, (int(bbox[0]), int(bbox[1])), (int(bbox[0]+ bbox[2]), int(bbox[1] + bbox[3])), (255, 0, 0), 4)
        elif logo_tracker is not None:
            track_ok, tracked_bbox = logo_tracker.update(img)

            if tracked_bbox is not None and track_ok:
                # print(tracked_bbox)
                cv2.rectangle(img, (int(tracked_bbox[0]), int(tracked_bbox[1])), (int(tracked_bbox[0]+ tracked_bbox[2]), int(tracked_bbox[1] + tracked_bbox[3])), (255, 255, 0), 4)
                cv2.circle(img, (int(tracked_bbox[0]+ tracked_bbox[2]/2), int(tracked_bbox[1] + tracked_bbox[3]/2)), 15, (0, 0, 255), 4)

        # logo_detected, det_dict, detType, num_found = detectLogo(img, marker_ids, board)
        # if logo_detected:
        #     size = img.shape[0:1]
        #     h_mat = comp_homograph(det_dict, temp_dict)
        #     # print(np.atleast_2d(np.array(temp_center)))
        #     new_center: np.ndarray = cv2.perspectiveTransform(np.atleast_3d(np.array(temp_center, dtype='float32')).reshape(-1,1,2), h_mat)
        #     # print(f"New center {new_center}")
        #     cv2.circle(img, new_center.astype(np.int32).flatten(), 15, (255, 255, 0), 4)

        #     cv2.putText(img, f"Logo Found: {logo_detected} // IDs Detected: {det_dict.keys()}", text_org,font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        # else:
        #     cv2.putText(img, f"Logo Found: {logo_detected} // IDs Detected ({num_found}): {det_dict.keys()}", text_org,font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow('img', img)

        # mask = detector.processFrame(img)
        # if mask is None:
        # cv2.imshow('img', img)
        # else:
            # cv2.imshow('img', mask)
        # print(img)
        k = cv2.waitKey(1) & 0xff
        # logger.debug(f"Frame {frame_number} took {(time.perf_counter() -  frame_start)*1000}")
        frame_number += 1
        if k == 'r':
            logo_tracker = None
        if k == 27:
            break
