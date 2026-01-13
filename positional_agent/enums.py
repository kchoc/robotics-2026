from math import pi

# TODO: Insert actual position values
ZONE_POSITIONS = [
    (-2500, -2500),  # Zone 0
    ( 2500, -2500),   # Zone 1
    ( 2500,  2500),    # Zone 2
    (-2500,  2500)    # Zone 3
]

WALL_MARKERS = [
    (1525,     -2287),
    (762.5,    -2287),
    (0,        -2287),
    (-762.5,   -2287),
    (-1525,    -2287),
    (-2287,    -1525),
    (-2287,    -762.5),
    (-2287,     0),
    (-2287,     762.5),
    (-2287,     1525),
    (-1525,     2287),
    (-762.5,    2287),
    (0,         2287),
    (762.5,     2287),
    (1525,      2287),
    (2287,      1525),
    (2287,      762.5),
    (2287,      0),
    (2287,     -762.5),
    (2287,     -1525),
]

WALL_ROTATION = [
    pi, pi*3/2, 0, pi/2,
]

CAMERAS = {
  "camera": {
    "position": (0.19, 0.249 - 0.125),
    "orientation": 0.0
  },
  "back camera": {
    "position": (-0.3, 0.249 - 0.125),
    "orientation": pi
  }
}

ROTATION_TOLERANCE = 0.01  # Radians
