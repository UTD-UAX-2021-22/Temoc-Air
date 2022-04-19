import Utils as util
from Utils import attitudeToRotator, VehicleInfo, DummyAttitude, DummyVehicle, pixCoordToWorldPosition, CameraInfo
import utm
import numpy as np
import math
from scipy.spatial.transform import Rotation

if __name__ == "__main__":
    util.setupLoggers(filename="util_log")
    vec = np.array([1, 0, 0])
    att = util.DummyAtt(0,0,math.pi/2)
    res = attitudeToRotator(att).apply(vec)
    print(res)
    print(utm.from_latlon(32.72881384085485, -97.12668390777232))
    print(utm.from_latlon(32.728810921372826, -97.12616281223721))

    t = VehicleInfo()
    # print(t.json())

    # with open('vehicle_info.json') as vf:
    #     t = VehicleInfo(**json.load(vf))
    print(t)

    test_pos = DummyVehicle(attitude=DummyAttitude(yaw=math.pi/2, pitch=-math.pi/4))
    # test_cam = CameraInfo(rotation=[0,-90,0], hfov=90, vfov=90)
    test_cam = CameraInfo(id=-1, position=[0.0, 0.0, 0.0], rotation=[0.0, -90.0, 0.0], hfov=60.0, vfov=60.0, resolution=(400.0, 400.0))
    test_pc = np.array([[326, 232], [614, 223]])
    print("Raycast Tests")
    # print(utm.from_latlon(32.729018289461976, -97.12642125790629))
    print(pixCoordToWorldPosition(test_pos, test_cam, test_pc))

    def y2m(v):
        return v * 0.9144

    def m2y(v):
        return v * 1.09361

    field_corners_y = np.array([
        [-53.333/2, 0],
        [-53.333/2, 50],
        [53.333/2, 50],
        [53.333/2, 0]
    ])
    yaw = test_pos.attitude.yaw
    Rotation.from_euler('ZYX', [-yaw, 0, 0 ])
    print(y2m(field_corners_y))

    # print("Relative Raycast Tests")
    # test_pos = DummyVehicle()
    # test_cam = CameraInfo(rotation=[0,-90,0], hfov=90, vfov=90)
    # print(pixCoordToRelativePosition(test_pos, test_cam, [100, 0]))

    # print("Visit Test")

    # test_pois = np.array([[0, 1], [0, 2], [0, 3]])
    # start = np.array([0,4])
    # print(calculateVisitPath(test_pois, start))
    
    # pixc = np.asarray([[100, 50], [0, 0], [200, 100]])
    # print(f"Result: {pixCoordToAngle(pixc, 45, 30, 200, 100)}")
