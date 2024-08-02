import shapely
import sympy

from utils.enviorment import Room, Furniture
from utils.visualizer import visualize
import numpy as np
# from sympy import Polygon, Point, Segment2D, Line
import pyclipper
from shapely.geometry import Polygon, Point, LineString

import time
start_time = time.time()

room = Room(50, 50)
divan = Furniture.create(room,
                         20,
                         10,
                         10,
                         Point(np.array([20, 10])),
                         np.array([4, 0, 0, 0]))
tumba = Furniture.create(room,
                         20,
                         10,
                         10,
                         Point(np.array([10, 30])),
                         np.array([10, 0, 0, 0]))
# tumba.rotate(np.radians(-25))
# divan.rotate(np.radians(45))
# tumba.move(np.array([-5, -13]))
# tumba.snap_to_closest_furniture()
# divan.snap_to_closest_furniture()

# print(room.compute_clearance_metric())







print("--- Computation %s seconds ---" % (time.time() - start_time))
start_time = time.time()
visualize(room)
print("--- Render %s seconds ---" % (time.time() - start_time))


