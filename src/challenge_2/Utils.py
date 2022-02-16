from dataclasses import dataclass
import json
import logging
import math
from pathlib import Path
from typing import List, Tuple
import cv2
import utm
import numpy as np
from scipy.spatial.transform import Rotation
from pydantic import BaseModel


def setupLoggers(filename='test_log'):
    import logging
    logger = logging.getLogger()
    Path("logs").mkdir(parents=True, exist_ok=True)
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
    return Rotation.from_euler('ZYX', [-yaw, -attitude.pitch, -attitude.roll])

def pixCoordToRot(pix_coords, hfov, vfov, hres, vres) -> Rotation:
    px_angle = pixCoordToAngle(pix_coords, hfov, vfov, hres, vres)
    return Rotation.from_euler('zyx', [px_angle[0], px_angle[1], 0], degrees=True)

def pixCoordToWorldPosition(vehicle, camera, pix_coords, att_init=None):
    x, y, *_ = utm.from_latlon(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    veh_pos = np.array([x, y, vehicle.location.global_relative_frame.alt])
    pix_rot = pixCoordToRot(pix_coords,  camera.hfov, camera.vfov, camera.resolution[0], camera.resolution[1])
    cam_rot = Rotation.from_euler('ZYX', camera.rotation, degrees=True)
    vehicle_rot = attitudeToRotator(vehicle.attitude, att_init=att_init)
    cam_pos = vehicle_rot.apply(np.asarray(camera.position)) + veh_pos
    logging.getLogger(__name__).debug(f"Rotators: {vehicle_rot.as_euler('ZYX', degrees=True)} -- {cam_rot.as_euler('ZYX', degrees=True)} -- {pix_rot.as_euler('ZYX', degrees=True)}")
    total_rot = vehicle_rot * cam_rot * pix_rot
    ray = total_rot.apply(np.array([1, 0, 0]))
    logging.getLogger(__name__).debug(f"Ray: {ray}")
    return get_line_plane_intercepts( np.array([0,0,0]), np.array([0,0, 1]), cam_pos, ray)






class CameraInfo(BaseModel):
    name: str = "Dummy Camera"
    id: int = -1
    position: List[float] = [0.0, 0.0, 0.0]
    rotation: List[float] = [0.0, 0.0, 0.0]
    hfov: float = 60
    vfov: float = 60
    resolution: Tuple[float, float] = (200, 200)

class VehicleInfo(BaseModel):
    cameras: List[CameraInfo] = [CameraInfo(name="Dummy Camera 1"), CameraInfo(name="Dummy Camera 2")]

class DummyAttitude(BaseModel):
    pitch: float = 0
    roll: float = 0
    yaw: float = 0

class DummyGPSCoords(BaseModel):
    lat: float = 32.729018289461976
    lon: float = -97.12642125790629
    alt: float = 10

class DummyLocation(BaseModel):
    global_relative_frame: DummyGPSCoords = DummyGPSCoords()

class DummyVehicle(BaseModel):
    location: DummyLocation = DummyLocation()
    attitude: DummyAttitude = DummyAttitude()

@dataclass
class DummyAtt:
    pitch: float
    roll: float
    yaw: float

if __name__ == "__main__":
    setupLoggers(filename="util_log")
    vec = np.array([1, 0, 0])
    att = DummyAtt(0,0,math.pi/2)
    res = attitudeToRotator(att).apply(vec)
    print(res)
    print(utm.from_latlon(32.72881384085485, -97.12668390777232))
    print(utm.from_latlon(32.728810921372826, -97.12616281223721))

    t = VehicleInfo()
    print(t.json())

    # with open('vehicle_info.json') as vf:
    #     t = VehicleInfo(**json.load(vf))
    print(t)

    test_pos = DummyVehicle(attitude=DummyAttitude(yaw=math.pi/2, pitch=-math.pi/4))
    test_cam = CameraInfo(rotation=[0,-90,0], hfov=90, vfov=90)
    print("Raycast Tests")
    print(utm.from_latlon(32.729018289461976, -97.12642125790629))
    print(pixCoordToWorldPosition(test_pos, test_cam, [100,0]))
    # pixc = np.asarray([[100, 50], [0, 0], [200, 100]])
    # print(f"Result: {pixCoordToAngle(pixc, 45, 30, 200, 100)}")