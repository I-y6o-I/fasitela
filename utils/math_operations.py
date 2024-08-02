import numpy as np
import typing as tp
from .settings import MathConstants
import sympy

Vector = np.ndarray[float]


def rotate_vector(vector: Vector, rotation_center: Vector, angle: float) -> Vector:
    rot_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                           [np.sin(angle), np.cos(angle)]])

    return rot_matrix.dot(vector - rotation_center) + rotation_center


def scale_vector(vector: Vector) -> Vector:
    return vector * (1 - MathConstants.snap_margin.value / np.linalg.norm(vector))


def polygon_intersection(p1: sympy.Polygon, p2: sympy.Polygon) -> sympy.Polygon:
    intersection_result = []

    assert isinstance(p1, sympy.Polygon)
    assert isinstance(p2, sympy.Polygon)

    k = p2.sides
    for side in p1.sides:
        if p2.encloses_point(side.p1):
            intersection_result.append(side.p1)
        if p2.encloses_point(side.p2):
            intersection_result.append(side.p2)
        for side1 in k:
            if p1.encloses_point(side1.p1):
                intersection_result.append(side1.p1)
            if p1.encloses_point(side1.p2):
                intersection_result.append(side1.p2)
            for res in side.intersection(side1):
                if isinstance(res, sympy.Segment):
                    intersection_result.extend(res.points)
                else:
                    intersection_result.append(res)

    intersection_result = list(sympy.utilities.iterables.uniq(intersection_result))

    if intersection_result and len(intersection_result) != 1:
        return sympy.geometry.util.convex_hull(*intersection_result)
    else:
        return None

