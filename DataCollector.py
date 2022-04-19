import json
import logging
import time
import cv2
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException

from Utils import setupLoggers

def tryMakeInt(s):
    try:
        return int(s)
    except ValueError:
        return s

if __name__ == "__main__":
    setupLoggers(filename="data_collector")
    logger = logging.getLogger(__name__)
    parser = argparse.ArgumentParser(description="Data Collector")
    parser.add_argument('--cam', default=1, nargs='?')
    parser.add_argument('--output', default="flight_video.avi")
    parser.add_argument('--drone', dest='connect_drone', action='store_true')
    parser.add_argument('--codec', default="MJPG")
    parser.add_argument('--size', default=None, nargs=2)
    parser.set_defaults(connect_drone=False)
    # parser.add_argument('template', default="logo_template_2.png",  nargs='?')
    args = parser.parse_args()
    cap = cv2.VideoCapture(tryMakeInt(args.cam))
    
    #connect to vehicle and get the current
    if args.size is None:
        _, img = cap.read()
        size = (img.shape[1], img.shape[0])
    else:
        size = tuple(args.size)

    logger.info(f"Output Video Size: {size}")
    print(f"Output Video Size: {size}")

    logger.info("Created video writer")
    video_writer = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*args.codec), 30, size)
    

    if args.connect_drone:
        logger.info("Connecting to drone")
    vehicle = connect('127.0.0.1:14550', wait_ready=True) if args.connect_drone else None
    
    
    frame_number = 0

    with open('flight_data_log.jsonl', 'w') as file:
        while True:
            frame_start = time.perf_counter()
            
            success, img = cap.read()
            if not success:
                break

            frame_log = {}
            frame_log['frame_number'] = frame_number
            if vehicle is not None:
                frame_log['position_global_relative'] = vehicle.location.global_relative_frame
                frame_log['position_global'] = vehicle.location.global_frame
                frame_log['position_local'] = vehicle.location.local_frame
                frame_log['attitude'] = vehicle.attitude
            else:
                frame_log['position_global_relative'] = {'lat':0, 'lon':0 , 'altitude':1}
                frame_log['position_global'] = {'lat':0, 'lon':0 , 'altitude':1}
                frame_log['position_local'] = {'lat':0, 'lon':0 , 'altitude':1}
                frame_log['attitude'] = {'pitch': 0, 'roll': 0, 'yaw': 0}

            # cap2 = cv2.VideoCapture(args.video2)
            # size = (426, 240)
            video_writer.write(cv2.resize(img, size, interpolation=cv2.INTER_AREA))
            file.write(f"{json.dumps(frame_log)}\n")
            logger.debug(f"Frame {frame_number} took {(time.perf_counter() -  frame_start)*1000}")
            frame_number += 1

        #arm and takeoff the drone
        #head in direction that drone is facing
        #move 30 yards and land

