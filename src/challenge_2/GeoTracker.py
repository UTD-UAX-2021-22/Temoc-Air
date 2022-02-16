import numpy as np
import logging
from Utils import setupLoggers, contourCentroid
import cv2
import utm

class GeoTracker:

    def __init__(self, corners, resolution=(50,50), threshold = 6) -> None:
        """
        corners: x,y coords of corners of the search area. Bottom left -> Top Left -> Top Right -> Bottom Right
        resolution
        """
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
        pass

    def reportPoi(self, position):
        self.poi_reports.append(position)
        pos_aug = np.pad(np.atleast_2d(np.asarray(position)), ((0,0), (0,1)), 'constant', constant_values=(0,1))
        logging.getLogger(__name__).debug(f"Adding report at {pos_aug}")
        pp = pos_aug @ self.tf_mat
        logging.getLogger(__name__).debug(f"Adding report at translated {pp}")

        # print(pp)
        pp = pp.astype(int)
        if 0 <= pp[0,0] < self.resolution[0] and 0 <= pp[0,1] < self.resolution[1]:
            self.poi_map[pp[0, 1], pp[0, 0]] += 1


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
    tracker = GeoTracker(coords_utm, resolution=(60,60))
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

    Image.fromarray(tracker.getGrayscaleMap(), 'L').show()
