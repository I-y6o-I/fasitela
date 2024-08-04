import typing as tp
import numpy as np
from abc import ABC, abstractmethod
from sympy import Point, Polygon, Segment2D, Line, Segment, Point2D
from sympy.sets import EmptySet
from . import math_operations
from .settings import MathConstants

Vector = np.ndarray[float]
WALL_CLEARANCE: int = MathConstants.wall_clearance.value


# TODO: factory??

class Room:
    def __init__(self, width: float, length: float):
        self._width: float = width
        self._length: float = length
        self._center: Point = Point(np.array([12, 15]))

        self._polygon: Polygon = Polygon(
            Point(np.array([self._center.x - self._width / 2, self._center.y - self._length / 2])),
            Point(np.array([self._center.x - self._width / 2, self._center.y + self._length / 2])),
            Point(np.array([self._center.x + self._width / 2, self._center.y + self._length / 2])),
            Point(np.array([self._center.x + self._width / 2, self._center.y - self._length / 2]))
        )
        self._bounded_box: list[Point] = [
            point for point in self._polygon.vertices
        ]

        # TODO: make not square
        self._walls: list[Wall] = [
            Wall.create(self, self._bounded_box[0], self._bounded_box[1], np.array([1, 0])),
            Wall.create(self, self._bounded_box[1], self._bounded_box[2], np.array([0, -1])),
            Wall.create(self, self._bounded_box[2], self._bounded_box[3], np.array([-1, 0])),
            Wall.create(self, self._bounded_box[3], self._bounded_box[0], np.array([0, 1]))
        ]
        # self._walls: list[Wall] = [Wall(self._bounded_box[0],
        #                                 self._bounded_box[1],
        #                                 np.array([1, 0])),
        #                            Wall(self._bounded_box[1],
        #                                 self._bounded_box[2],
        #                                 np.array([0, -1])),
        #                            Wall(self._bounded_box[2],
        #                                 self._bounded_box[3],
        #                                 np.array([-1, 0])),
        #                            Wall(self._bounded_box[3],
        #                                 self._bounded_box[0],
        #                                 np.array([0, 1])),
        #                            ]

        self._objects: dict[int, RoomObject] = {}
        self._objects_len: int = 0

    def get_polygon(self) -> Polygon:
        return self._polygon

    def add_furniture(self, furniture):
        self._objects[self._objects_len] = furniture
        self._objects_len += 1

    def get_objects(self):
        return self._objects.values()

    def get_walls(self):
        return self._walls

    def compute_clearance_metric(self) -> float:
        intersection_area: float = 0.0

        for i in range(self._objects_len):
            for j in range(i + 1, self._objects_len):
                intersection: Polygon = math_operations.polygon_intersection(
                    self._objects[i].get_clearance_bounds(),
                    self._objects[j].get_clearance_bounds()
                )

                if intersection is None:
                    continue
                intersection_area += intersection.area

        return intersection_area / (self._objects_len * (self._objects_len - 1))


class RoomObject(ABC):
    def __init__(self, room: Room):
        self._room = room
        self._clearance_bounds: Polygon = None

    def get_clearance_bounds(self):
        return self._clearance_bounds

    # def __init__(self,
    #              room: Room,
    #              width: float,
    #              length: float,
    #              height: float,
    #              center: Point,
    #              clearance_values: Vector):

    # @classmethod
    # @abstractmethod
    # def create_room_object(cls,
    #                        room: Room,
    #                        width: float,
    #                        length: float,
    #                        height: float,
    #                        center: Point,
    #                        clearance_values: Vector):
    #     raise NotImplemented

    # @classmethod
    # @abstractmethod
    # def create(cls,
    #            room: Room,
    #            *args):
    #     raise NotImplemented


class Wall(RoomObject):
    __create_key = object()

    def __init__(self,
                 create_key: object,
                 room: Room,
                 p1: Point,
                 p2: Point,
                 front_vector: Vector):
        assert (create_key == Wall.__create_key), \
            "Room objects should be created via create method"

        super().__init__(room)

        self._segment: Segment2D = Segment2D(p1, p2)
        self._front_vector: Vector = front_vector
        self._clearance_bounds: Polygon = Polygon(
            p1,
            p1.translate(*(front_vector * -WALL_CLEARANCE)),
            p2.translate(*(front_vector * -WALL_CLEARANCE)),
            p2
        )

    @classmethod
    def create(cls, room: Room, p1: Point, p2: Point, front_vector: Vector):
        obj: Wall = Wall(cls.__create_key, room, p1, p2, front_vector)
        return obj

    def get_segment(self) -> Segment2D:
        return self._segment

    def get_front_vector(self) -> Vector:
        return self._front_vector

    def get_center(self) -> Point:
        return self._segment.midpoint

    def get_clearance_bounds(self) -> Polygon:
        return self._clearance_bounds


class Furniture(RoomObject):
    __create_key = object()

    def __init__(self,
                 create_key: object,
                 room: Room,
                 width: float,
                 length: float,
                 height: float,
                 center: Point,
                 clearance_values: Vector):
        assert (create_key == Furniture.__create_key), \
            "Room objects should be created via create method"

        super().__init__(room)


        self._width: float = width
        self._length: float = length
        self._height = height
        self._front_vector: Vector = np.array([0, 1])  # TODO: define front vector
        self._room: Room = room

        # Clearance constraints: front, back, left, right
        self._clearance_values: Vector = clearance_values

        self._polygon = Polygon(
            Point([center.x - self._width / 2, center.y - self._length / 2]),
            Point([center.x - self._width / 2, center.y + self._length / 2]),
            Point([center.x + self._width / 2, center.y + self._length / 2]),
            Point([center.x + self._width / 2, center.y - self._length / 2])
        )

        self._clearance_bounds: Polygon = Polygon(
            Point(
                [center.x - self._width / 2 - self._clearance_values[2],
                 center.y - self._length / 2 - self._clearance_values[1]]
            ),
            Point(
                [center.x - self._width / 2 - self._clearance_values[2],
                 center.y + self._length / 2 + self._clearance_values[0]]
            ),
            Point(
                [center.x + self._width / 2 + self._clearance_values[3],
                 center.y + self._length / 2 + self._clearance_values[0]]
            ),
            Point(
                [center.x + self._width / 2 + self._clearance_values[3],
                 center.y - self._length / 2 - self._clearance_values[1]]
            )
        )
    def get_polygon(self) -> Polygon:
        return self._polygon

    def get_front_vector(self) -> Vector:
        return self._front_vector

    def get_center(self) -> Point:
        return self._polygon.centroid

    def get_clearance_bounds(self) -> Polygon:
        return self._clearance_bounds

    @classmethod
    def create(cls,
               room: Room,
               width: float,
               length: float,
               height: float,
               center: Point,
               clearance_values: Vector):
        obj = Furniture(cls.__create_key, room, width, length, height, center, clearance_values)
        room.add_furniture(obj)
        return obj

    def move(self, transport_vector: Vector) -> tp.NoReturn:
        self._polygon = Polygon(*[
            point.translate(*transport_vector)
            for point in self._polygon.vertices
        ])

        self._clearance_bounds = Polygon(*[
            point.translate(*transport_vector)
            for point in self._clearance_bounds.vertices
        ])

    def rotate(self, angle_rad: float) -> tp.NoReturn:
        self._front_vector: Vector = (math_operations
                                      .rotate_vector(self._front_vector,
                                                     np.array([0, 0]),
                                                     angle_rad))

        self._polygon = self._polygon.rotate(angle_rad, self._polygon.centroid)
        self._clearance_bounds = self._clearance_bounds.rotate(angle_rad, self._polygon.centroid)

    def align(self, obj) -> tp.NoReturn:
        a = obj.get_front_vector()
        b = self._front_vector
        rotation_angle = -np.arctan2(b[1], b[0]) + np.arctan2(a[1], a[0])

        if rotation_angle > np.pi:
            rotation_angle -= 2 * np.pi
        elif rotation_angle <= -np.pi:
            rotation_angle += 2 * np.pi

        self.rotate(rotation_angle)

    def closest_furniture(self):
        closest_furniture = None
        min_distance = 1e9
        for obj in self._room.get_objects():
            if obj == self:
                continue

            cur_distance = obj.get_polygon().distance(self._polygon)
            if cur_distance < min_distance:
                min_distance = cur_distance
                closest_furniture = obj

        return closest_furniture

    def closest_wall(self) -> Wall:
        closest_wall = None
        min_distance = 1e9
        for obj in self._room.get_walls():

            cur_distance = (min([
                obj.get_segment().distance(point)
                for point in self._polygon.vertices
            ]))
            if cur_distance < min_distance:
                min_distance = cur_distance
                closest_wall = obj

        return closest_wall

    def align_with_closest_furniture(self) -> tp.NoReturn:
        closest_furniture = self.closest_furniture()
        self.align(closest_furniture)

    def align_with_closest_wall(self) -> tp.NoReturn:
        closest_wall = self.closest_wall()
        self.align(closest_wall)

    def snap_to_closest_furniture(self) -> tp.NoReturn:
        furniture_a: Furniture = self
        furniture_b: Furniture = self.closest_furniture()
        reverse_vector = False

        diagonals: list[Line] = [Line(furniture_a.get_polygon().centroid, vertex)
                                 for vertex in furniture_a.get_polygon().vertices]
        if all(diag.intersect(furniture_b.get_polygon()) == EmptySet for diag in diagonals):
            furniture_a, furniture_b = furniture_b, furniture_a
            reverse_vector = True

        closest_vertex: tp.Optional[Point] = None
        min_distance: float = 1e9

        for vertex in furniture_a._polygon.vertices:
            cur_distance = furniture_b._polygon.distance(vertex)
            if cur_distance < min_distance:
                min_distance = cur_distance
                closest_vertex = vertex

        closest_side: tp.Optional[Segment] = None
        min_distance: float = 1e9
        for side in furniture_b._polygon.sides:
            cur_distance = closest_vertex.distance(side)
            if cur_distance < min_distance:
                min_distance = cur_distance
                closest_side = side

        distance_segment: Segment = closest_side.perpendicular_segment(closest_vertex)
        if isinstance(distance_segment, Point2D):
            return

        transport_vector: Vector = np.array([distance_segment.p2.x - distance_segment.p1.x,
                                             distance_segment.p2.y - distance_segment.p1.y
                                             ], dtype=float)

        # transport_vector = math_operations.scale_vector(transport_vector)
        transport_vector *= -1 if reverse_vector else 1
        self.move(transport_vector)
