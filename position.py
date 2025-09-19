import numpy as np
from sr.robot3.camera import AprilCamera as Camera

from utils import mod_angle
from enums import ZONE_POSITIONS

class Position:
    _position: np.ndarray = np.zeros(2)
    _last_update: float = 0.0
    _cardinal_offset: float = 0.0
    _last_front_frame: float = 0.0
    _last_back_frame: float = 0.0
    _cameras: list[Camera] = []

    zone_position: np.ndarray

    @property
    def position(self) -> np.ndarray:
        return self.estimate_position()
        
    @property
    def rotation(self) -> float:
        return self.get_rotation()
    
    @property
    def velocity(self) -> np.ndarray:
        pass

    @property
    def acceleration(self) -> np.ndarray:
        pass

    @property
    def angular_velocity(self) -> float:
        pass

    @property
    def angular_acceleration(self) -> float:
        pass

    def __init__(self, zone, cardinal_offset=0.0, cameras: list[Camera] = []):
        self._position = np.array([0.0, 0.0])
        self._cardinal_offset = cardinal_offset
        self.zone_position = np.array(ZONE_POSITIONS[zone])
        self._cameras = cameras

    def estimate_position(self) -> np.ndarray:
        pass

    def get_rotation(self) -> float:
        pass

    def relative_angle_to_bearing(self, angle: float) -> float:
        return mod_angle(angle + self._cardinal_offset)
    
    def bearing_to_relative_angle(self, bearing: float) -> float:
        return mod_angle(bearing - self._cardinal_offset)
