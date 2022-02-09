from dataclasses import dataclass
import logging
import math
import cv2
import utm
import numpy as np
from scipy.spatial.transform import Rotation as R

def setupLoggers(filename='test_log'):
    import logging
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    import logging.handlers
    handler = logging.handlers.RotatingFileHandler(f"logs/{filename}.log", backupCount=10)
    # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    formatter = logging.Formatter('%(asctime)s - %(levelname)s::%(name)s - %(filename)s::%(funcName)s::%(lineno)d - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    handler.doRollover()

class MissionContext():
    def __init__(self, name, logger="mission_control") -> None:
        self.name = name
        self.logger = logging.getLogger(logger)
        pass

    def __enter__(self):
        self.logger.info(f"Begin {self.name}")
        return None

    def __exit__(self, type, value, traceback):
        self.logger.info(f"End {self.name}")



def pixCoordToAngle(pix_coords, hfov, vfov, hres, vres):
    return (pix_coords - np.asarray([hres/2, vres/2])) * np.asarray([hfov/hres, vfov/vres])

def contourCentroid(cnt):
    cm = cv2.moments(cnt)
    return int(cm['m10']/cm['m00']), int(cm['m01']/cm['m00'])

def get_line_plane_intercepts(plane_origin, plane_normal, line_origins, line_directions):
    intercept_d_values = np.inner((plane_origin - line_origins), plane_normal) / np.inner(line_directions, plane_normal)
    return line_origins + line_directions * np.atleast_2d(intercept_d_values).T

# def geoRegisterPoint

def attitudeToRotator(attitude, att_init=None):
    yaw = attitude.yaw if att_init is None else attitude.yaw - att_init.yaw
    return R.from_euler('zyx', [-yaw, attitude.pitch, attitude.roll])


@dataclass
class DummyAtt:
    pitch: float
    roll: float
    yaw: float

if __name__ == "__main__":
    vec = np.array([1, 0, 0])
    att = DummyAtt(0,0,math.pi/2)
    res = attitudeToRotator(att).apply(vec)
    print(res)
    print(utm.from_latlon(32.72881384085485, -97.12668390777232))
    print(utm.from_latlon(32.728810921372826, -97.12616281223721))
    # pixc = np.asarray([[100, 50], [0, 0], [200, 100]])
    # print(f"Result: {pixCoordToAngle(pixc, 45, 30, 200, 100)}")