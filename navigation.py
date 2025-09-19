from position import Position

import numpy as np

class Navigation(Position):
    _target_position: np.ndarray = np.zeros(2)
    _target_rotation: float = 0.0
    is_rotating: bool = False
    is_moving: bool = False

    def __init__(self, zone, cardinal_offset=0, cameras = ...):
        super().__init__(zone, cardinal_offset, cameras)
        
    def move_distance(self, distance: float, speed: float = 0.5):
        pass

    def rotate_to(self, angle: float, speed: float = 0.5):
        pass

    def rotate_by(self, angle: float, speed: float = 0.5):
        pass

    def move_to(self, position: np.ndarray, speed: float = 0.5):
        pass

    def move_to_quick(self, position: np.ndarray, speed: float = 1.0):
        pass

    def move_to_zone(self, zone: int, speed: float = 0.5):
        pass

    def stop(self):
        pass
