import numpy as np
import logging
from Utils import setupLoggers, contourCentroid
import cv2


class GeoTracker:

    def __init__(self, center, resolution, dims = (60, 60), threshold = 6) -> None:
        self.dims = np.asarray(dims)
        self.poi_map = np.zeros((dims[0]*resolution, dims[1]*resolution))
        self.poi_reports = []
        self.resolution = resolution
        self.center = np.asarray(center)
        self.threshold = threshold
        self.poi_counter = 0
        self.prior_poi = {}
        pass

    def reportPoi(self, position):
        self.poi_reports.append(position)
        pp = ( (np.asarray(position) + self.center) * self.resolution).astype(np.int32).flatten()
        logging.getLogger().debug(f"Adding report at {pp}")
        print(pp)
        self.poi_map[pp[1], pp[0]] += 1;

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
    tracker = GeoTracker((30, 5), resolution=1, dims=(60, 60))
    actual_points = [    [0, 20],        [-20, 20],       [20, 20],        [0, 22]    ]
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
