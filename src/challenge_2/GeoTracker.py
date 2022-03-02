import json
import time
import numpy as np
import logging
from Utils import setupLoggers, contourCentroid, calculateVisitPath
import cv2
import utm

class GeoTracker:

    def __init__(self, corners, resolution=(50,50), threshold = 6) -> None:
        """
        corners: x,y coords of corners of the search area. Bottom left -> Top Left -> Top Right -> Bottom Right
        resolution
        """
        logging.getLogger(__name__).debug("TEST")
        target_maps = np.array([[0,resolution[1],1], [0, 0, 1], [resolution[0], 0, 1], [resolution[0], resolution[1], 1]])
        logging.getLogger(__name__).debug(f"Creating Geotracker with Corners: {corners}")
        logging.getLogger(__name__).debug(f"Target map: {target_maps}")
        augmented_corners = np.pad(corners, ((0,0), (0,1)), 'constant', constant_values=(0, 1))
        logging.getLogger(__name__).debug(f"Augmented corners: {augmented_corners}")
        self.tf_mat = np.linalg.lstsq(augmented_corners, target_maps, rcond=None)[0]
        self.inv_tf_mat = np.linalg.lstsq(target_maps, augmented_corners, rcond=None)[0]
        self.resolution = resolution
        self.poi_map = np.zeros(resolution)
        self.poi_reports = []
        self.threshold = threshold
        self.poi_counter = 0
        self.prior_poi = {}
        x_vales = np.arange(resolution[0])
        y_vals = np.arange(resolution[1])
        self.coord_grid = np.zeros((resolution[0], resolution[1], 2))
        for xi in range(resolution[0]):
            for yi in range(resolution[1]):
                self.coord_grid[xi, yi, 0] = xi
                self.coord_grid[xi, yi, 1] = yi
        logging.getLogger(__name__).debug(f"Coord Grid:{self.coord_grid}")
        

    def reportPoi(self, position, mission_time = time.time()):
        logger = logging.getLogger(__name__)
        self.poi_reports.append(position)
        position = np.asarray(position)

        if position.shape[1] == 3:
            position[:,2] = 1
            pos_aug=position
        else:
            pos_aug = np.pad(np.atleast_2d(np.asarray(position)), ((0,0), (0,1)), 'constant', constant_values=(0,1))

        if logger.isEnabledFor(logging.DEBUG):
            with open('pd_data/geo_tracker_log.jsonl', 'a') as file:
                for r in pos_aug:
                    frame_log = {'lat':r[0], 'lon':r[1], 'index': len(self.poi_reports), 'time': mission_time }
                    file.write(f"{json.dumps(frame_log)}\n")

        logging.getLogger(__name__).debug(f"Adding report at {pos_aug}")
        pp = pos_aug @ self.tf_mat
        logging.getLogger(__name__).debug(f"Adding report at translated {pp}")
        
        # print(f"PP: {pp}")
        pp = pp.astype(int)
        for row in pp:
            # print(f"Row: {row}")
            if 0 <= row[0] < self.resolution[0] and 0 <= row[1] < self.resolution[1]:
                self.poi_map[row[1], row[0]] += 1


    def getGrayscaleMap(self):
        return (self.poi_map/( self.poi_map.max() ) * 255).astype(np.uint8)

    def POIs(self):
        cnts, _ = cv2.findContours(self.poi_map >= self.threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centroids = [contourCentroid(c) for c in cnts]
        assigned = [False] * len(cnts)
        for ci, c in enumerate(cnts):
            assigned_ids = []
            for id, loc in self.prior_poi.items():
                if cv2.pointPolygonTest(c, loc, False) == 1:
                    assigned_ids.append(id)
                    assigned[ci] = True
            
            if len(assigned_ids) == 0:
                self.poi_counter += 1
                self.prior_poi[self.poi_counter] = centroids[ci]
            elif len(assigned_ids) >= 2:
                id_to_keep = min(assigned_ids)
                for i in assigned_ids:
                    if i != id_to_keep:
                        del self.poi_map[i]

    def getPOIs(self):
        mask = self.poi_map > self.threshold
        indexs = self.coord_grid[mask]
        indexs = np.pad(indexs, ((0,0), (0,1)), 'constant', constant_values=(0, 1))
        result_coords =  (indexs @ self.inv_tf_mat)[:,0:2]
        logging.getLogger(__name__).debug(f"POI's detected at {result_coords}")
        return result_coords


if __name__ == "__main__":
    setupLoggers(filename='GeoTracker_test')
    from numpy.random import default_rng
    from PIL import Image
    rng = default_rng()
    actual_points = [ [675568.1741, 3622798.785], [675578.1741, 3622808.785], [675588.1741, 3622818.785], [675558.1741, 3622788.785], [675548.1741, 3622778.785], [675558.1741, 3622808.785], [675548.1741, 3622818.785], [675578.1741, 3622788.785], [675588.1741, 3622778.785]  ]
    a = np.array([[0,0,1],[0,1,1],[1,1,1],[1,0,1]])
    b = np.array([[2,2,1],[2,3,1],[3,3,1],[3,2,1]])
    x_lsq = np.linalg.lstsq(a,b, rcond=None)[0]
    print(f"Estimated Affine:\n{x_lsq.T}")
    print(f"Test{ np.array([0,0,1]) @ x_lsq }")
    stadium_coords = np.array([
        [32.728812333507065, -97.12668348228334],
        [32.7292257238372, -97.1266797964984],
        [32.72922196356897, -97.12615773301923],
        [32.728809482130494, -97.12616011845971]
    ])
    coords_utm = np.zeros_like(stadium_coords)
    coords_utm[:,0], coords_utm[:,1], znum, zlet = utm.from_latlon(stadium_coords[:,0], stadium_coords[:,1])
    tracker = GeoTracker(coords_utm, resolution=(20,20), threshold=30)
    print("Field Corner Coordinates")
    print(coords_utm)


    max_rdist = 1
    num_reports = 120
    for p in actual_points:
        pn = np.atleast_2d(np.asarray(p))
        for i in range(num_reports):
            # report_point = pn + 2 * max_rdist * rng.random((1, 2)) - max_rdist
            report_point = pn + rng.normal(0, max_rdist, (1,2))
            # print(report_point)
            tracker.reportPoi(report_point)
    tracker.reportPoi(np.array(actual_points))
    Image.fromarray(tracker.getGrayscaleMap(), 'L').show()
    print(f"Reported POIs with length ({tracker.getPOIs().shape})")
    print(tracker.getPOIs())
    print(calculateVisitPath(tracker.getPOIs(), [0,0]))