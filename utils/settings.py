from betterconf import Config, field


class MathConstants(Config):
    object_vector_scale = field("OBJECT_VECTOR_SCALE", default=10)
    wall_vector_scale = field("WALL_VECTOR_SCALE", default=5)
    snap_margin = field("SNAP_MARGIN", default=1)
    wall_clearance = field("WALL_CLEARANCE", default=10)
