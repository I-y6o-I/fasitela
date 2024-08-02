import sympy
from sympy import Point2D, Line

from .enviorment import Room, Furniture
import typing as tp
from spb import plot_geometry
from spb.graphics.vectors import arrow_2d
from spb import graphics
from .settings import MathConstants


OBJECT_VECTOR_SCALE: int = MathConstants.object_vector_scale.value
WALL_VECTOR_SCALE: int = MathConstants.wall_vector_scale.value


def visualize(room: Room, additional_objects: list = ()) -> tp.NoReturn:
    objects = [obj.get_polygon() for obj in room.get_objects()]
    clearance_bounds = [obj.get_clearance_bounds() for obj in room.get_objects()]
    clearance_bounds.extend([obj.get_clearance_bounds() for obj in room.get_walls()])
    object_front_vectors = [arrow_2d(obj.get_center(), obj.get_front_vector() * OBJECT_VECTOR_SCALE)
                            for obj in room.get_objects()]
    walls_front_vectors = [arrow_2d(wall.get_center(), wall.get_front_vector() * WALL_VECTOR_SCALE)
                           for wall in room.get_walls()]

    plt = plot_geometry(room.get_polygon(),
                        *objects,
                        *additional_objects,
                        is_filled=False, show=False)
    clearance_plt = plot_geometry(*clearance_bounds,
                                  is_filled=False,
                                  show=False,
                                  rendering_kw=[{"color": "tab:purple", "linestyle": "--"} for _ in clearance_bounds]
                                  )
    vec = graphics(*object_front_vectors, *walls_front_vectors, show=False)

    plt.extend(vec)
    plt.extend(clearance_plt)
    plt.legend = False
    plt.show()
