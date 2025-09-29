import numpy as np
from typing import TYPE_CHECKING

from position import Position
from utils import mod_angle
from enums import ROTATION_TOLERANCE

if TYPE_CHECKING:
    from robot import Robot

class Navigation(Position):
    def __init__(self, robot: "Robot", zone: int = 0, cardinal_offset: float = 0.0):
        super().__init__(robot, zone, cardinal_offset)

    def _move(self, distance: float, speed: float = 0.5):
        start_position = self.position.copy()
        while True:
            current_position = self.position
            traveled_distance = np.linalg.norm(current_position - start_position)
            if traveled_distance >= abs(distance):
                break

            self.robot.move.servo_powers = (speed, speed)

    def move(self, distance: float, speed: float = 0.5):
        self._move(distance, speed)

    def _rotate_to(self, angle: float, speed: float = 0.5):
        while True:
            angle_dif = mod_angle(self._robot.nav.rotation - angle)
            if abs(angle_dif) <= ROTATION_TOLERANCE:
                break

            spin = speed * np.tanh(angle_dif)
            self._robot.move.servo_powers = (-spin, spin)

    def rotate_to(self, angle: float, speed: float = 0.5):
        self._rotate_to(angle, speed)

    def rotate_by(self, angle: float, speed: float = 0.5):
        self.rotate_to(self.rotation + angle, speed)

    def look_at(self, position: np.ndarray, speed: float = 0.5):
        direction_vector = self.position - position
        target_angle = np.arctan2(direction_vector[1], direction_vector[0])
        self.rotate_to(target_angle, speed)

    def _move_to(self, position: np.ndarray, speed: float = 0.5):
        self.look_at(position, speed)
        move_time = self._estimate_move_time(position, speed)
        self.robot.move.power_for_time(speed, speed, move_time)

    def _move_to_quick(self, position: np.ndarray, speed: float = 1.0):
        pass

    def _move_to_zone(self, zone: int, speed: float = 0.5):
        pass

    def _estimate_move_time(self, position: np.ndarray, speed: float = 0.5) -> float:
        direction_vector = position - self.position
        distance = np.linalg.norm(direction_vector)
        if distance == 0:
            return 0.0
        return distance / speed / 1000  # Convert mm/s to seconds

    def _stop(self):
        self._robot.move.stop()
