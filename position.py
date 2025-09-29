from sr.robot3.camera import AprilCamera as Camera

import numpy as np
from typing import TYPE_CHECKING

from utils import mod_angle
from enums import ZONE_POSITIONS, WALL_MARKERS

if TYPE_CHECKING:
    from sr.robot3.marker import Marker
    from robot import Robot

class Position:
    _update_position: np.ndarray = np.zeros(2)
    _update_rotation: float = 0.0
    _last_update: float = 0.0
    _cardinal_offset: float = 0.0
    _robot: "Robot"

    zone_position: np.ndarray

    @property
    def position(self) -> np.ndarray:
        return self._update_position
        
    @property
    def rotation(self) -> float:
        return mod_angle(self._robot._robot.compass.heading + self._cardinal_offset)
    
    @property
    def velocity(self) -> np.ndarray:
        pass

    @property
    def acceleration(self) -> np.ndarray:
        return np.array(self._robot._robot.accelerometer.acceleration[:2])

    @property
    def angular_velocity(self) -> float:
        pass

    @property
    def angular_acceleration(self) -> float:
        pass

    def __init__(self, robot:"Robot", zone, cardinal_offset=0.0):
        self._robot = robot
        self._update_position = np.array([0.0, 0.0])
        self._update_rotation = 0.0
        self._cardinal_offset = cardinal_offset
        self.zone_position = np.array(ZONE_POSITIONS[zone])

    def estimate_position(self) -> np.ndarray:
        pass

    def relative_angle_to_bearing(self, angle: float) -> float:
        return mod_angle(angle + self._cardinal_offset)
    
    def bearing_to_relative_angle(self, bearing: float) -> float:
        return mod_angle(bearing - self._cardinal_offset)

    def update_transform(self, cam_markers: dict[str, list["Marker"]], update_rotation) -> bool:
        # TODO: The robot API for time is too slow, replace with a mock time for now
        # update_time = self._robot._robot.time()
        update_time = 0.0

        front_markers = cam_markers.get("Camera", [])
        back_markers = cam_markers.get("Back Camera", [])

        camera_offset = 50  # in mm
        best_marker = None
        best_marker_data = None

        for markers, offset, rotation_adjust in [
            (front_markers, 50, 0),
            (back_markers, -50, np.pi)
        ]:
            for marker in markers:
                if marker.id > 27:
                    continue
                marker_score = marker.position.distance
                if best_marker is None or marker_score < best_marker_data[0]:
                    best_marker = marker
                    best_marker_data = (
                        marker_score,
                        offset,
                        update_rotation + rotation_adjust + marker.position.horizontal_angle
                    )

        if not best_marker:
            return False

        marker_distance, camera_offset, marker_angle = best_marker_data
        marker_pos = np.array(WALL_MARKERS[best_marker.id])
        marker_rel_pos = np.array([
            marker_distance * np.sin(marker_angle) + camera_offset * np.sin(update_rotation),
            marker_distance * np.cos(marker_angle) + camera_offset * np.cos(update_rotation)
        ])
        self._update_position = marker_pos - marker_rel_pos
        self._update_rotation = update_rotation
        self._last_update = update_time

        return True
