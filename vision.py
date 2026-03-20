from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from robot import Robot
  from sr.robot3.marker import Marker as AprilTagMarker
import numpy as np
import math
from pydantic import BaseModel

from utils import true_floor_yaw
from enums import WALL_MARKERS, CAMERA_TILT

marker_cache = {}

class Marker(BaseModel):
  """Represents a detected marker with its position and orientation relative to the robot."""
  id: int
  horizontal_distance: float
  height: float
  horizontal_angle: float
  yaw: float
  movement_measurement: int  # 1 for edge marker, 2 for center marker, -1 for no marker
  time: float | None = None

  def __init__(self, marker_id, horizontal_distance, height, horizontal_angle, yaw, movement_measurement, time=None, **kwargs):
    """Initialize a marker with positional arguments."""
    super().__init__(
      id=marker_id,
      horizontal_distance=horizontal_distance,
      height=height,
      horizontal_angle=horizontal_angle,
      yaw=yaw,
      movement_measurement=movement_measurement,
      time=time,
      **kwargs
    )

  @staticmethod
  def invalid() -> "Marker":
    """Create an invalid/missing marker."""
    return Marker(-1, 0, 0, 0, 0, -1, time=None)

  def isinvalid(self):
    return self.movement_measurement == -1

  @classmethod
  def from_april(cls, marker: "AprilTagMarker", time=None) -> "Marker":
    """Convert an AprilTag marker from the camera into our Marker format."""
    if marker is None: return cls.invalid()
    horizontal_distance = marker.position.distance * math.cos(marker.position.vertical_angle - CAMERA_TILT) / 1000
    height = marker.position.distance * math.sin(marker.position.vertical_angle - CAMERA_TILT) / 1000
    horizontal_angle = marker.position.horizontal_angle

    yaw, is_upright = true_floor_yaw(marker.orientation.roll, marker.orientation.pitch, marker.orientation.yaw, CAMERA_TILT)
    best_yaw = yaw
    for i in range(0, 4):
      test_yaw = (yaw + i * np.pi / 2 + np.pi / 4) % (2 * np.pi) - np.pi / 4
      if abs(test_yaw + horizontal_angle) < abs(best_yaw + horizontal_angle):
        best_yaw = test_yaw
    
    movement_measurement = 1 if is_upright else 2  # Default to center of marker being the center of the box if upright, otherwise edge
    return cls(marker.id, horizontal_distance, height, horizontal_angle, best_yaw, movement_measurement, time=time)

  @staticmethod
  def marker_displacement(marker1, marker2) -> tuple[float, float]:
    """Calculate the displacement of marker 2 relative to marker 1 in distance and angle."""
    if marker1.movement_measurement == -1 or marker2.movement_measurement == -1:
      return (float('inf'), float('inf'))  # If either marker is invalid, return infinite displacement
    distance_diff = np.sqrt(marker1.horizontal_distance**2 + marker2.horizontal_distance**2 - 2 * marker1.horizontal_distance * marker2.horizontal_distance * math.cos(marker1.horizontal_angle - marker2.horizontal_angle))
    angle_diff = math.atan2(marker2.horizontal_distance * math.sin(marker2.horizontal_angle) - marker1.horizontal_distance * math.sin(marker1.horizontal_angle),
                            marker2.horizontal_distance * math.cos(marker2.horizontal_angle) - marker1.horizontal_distance * math.cos(marker1.horizontal_angle))
    return (distance_diff, angle_diff)

  def add_offset(self, distance, direction=None, wall_marker: "WallMarker"=None) -> "Marker":
    """Calculate the offset from a marker of a box to a point in front of the box (for grabbing) or in the center of the box (for navigation)."""
    if direction is not None: return
    
    if self.movement_measurement == 1: distance -= BOX_SIZE / 2  # If the marker is on the edge of the box, add half the box size to get to the center
    angle = self.horizontal_angle - self.yaw  # horizontal angle minus yaw gives the angle to the front of the box
    dist = np.sqrt(self.horizontal_distance**2 + distance**2 - 2 * self.horizontal_distance * distance * math.cos(angle))
    alpha = math.atan2(self.horizontal_distance * math.sin(self.horizontal_angle) - distance * math.sin(self.yaw),
                        self.horizontal_distance * math.cos(self.horizontal_angle) - distance * math.cos(self.yaw))
    if distance < self.horizontal_distance:
      movement_measurement = self.movement_measurement  # If the offset point is further away than the original marker, keep the same movement measurement (edge or center)
    else:
      movement_measurement = 3  # New movement measurement for the offset point, meaning it's in front of the box
    # print(f"Adding offset: original marker {self.__repr__()}, distance={distance:.2f}m, new marker has dist={dist:.2f}m, h_angle={math.degrees(alpha):.1f}°")
    return Marker(self.id, dist, self.height, alpha, self.yaw, movement_measurement, time=self.time)


  def get_center(self) -> "Marker":
    """Calculate the center of the box from a marker on the edge of the box."""
    if self.movement_measurement == 2: return self  # Already at the center
    return self.add_offset(-BOX_SIZE / 2)

  def __repr__(self):
    """String representation of the marker for debugging purposes."""
    return f"Marker(id={self.id}, dist={self.horizontal_distance:.2f}m, height={self.height:.2f}m, h_angle={math.degrees(self.horizontal_angle):.1f}°, yaw={math.degrees(self.yaw):.1f}°, measurement={self.movement_measurement}, time={self.time})"

class WallMarker(Marker):
  rel_x: float # The relative x position of the wall marker from the robot's perspective, calculated from the marker ID and the known wall layout
  rel_y: float # The relative y position of the wall marker from the robot's perspective, calculated from the marker ID and the known wall layout
  abs_x: float # The absolute x position of the wall marker in world coordinates
  abs_y: float # The absolute y position of the wall marker in world coordinates
  robot_x: float = 0.0 # The absolute x position of the robot in world coordinates, calculated from the wall marker's position and the robot's position relative to the marker
  robot_y: float = 0.0 # The absolute y position of the robot in world coordinates, calculated from the wall marker's position and the robot's position relative to the marker
  robot_yaw: float = 0.0 # The absolute yaw of the robot in world coordinates, calculated from the wall marker's orientation and the robot's orientation relative to the marker

  def __init__(self, marker_id, horizontal_distance, height, horizontal_angle, yaw, movement_measurement, time=None):
    rel_x = horizontal_distance * math.cos(horizontal_angle)
    rel_y = horizontal_distance * math.sin(horizontal_angle)
    abs_x = WALL_MARKERS[marker_id][0]
    abs_y = WALL_MARKERS[marker_id][1]
    robot_x = abs_x - rel_x
    robot_y = abs_y - rel_y
    robot_yaw = np.pi/2 * (marker_id // 5) - horizontal_angle  # The robot's yaw is the angle to the wall marker plus the angle of the wall (which is a multiple of 90 degrees)
    super().__init__(marker_id, horizontal_distance, height, horizontal_angle, yaw, movement_measurement, time=time, rel_x=rel_x, rel_y=rel_y, abs_x=abs_x, abs_y=abs_y, robot_x=robot_x, robot_y=robot_y, robot_yaw=robot_yaw)
  
  def absolute_yaw_to_relative(self, yaw):
    """Convert an absolute yaw in world coordinates to a relative yaw from the robot's perspective."""
    return (yaw - self.robot_yaw + np.pi) % (2 * np.pi) - np.pi

  def marker_absolute_position(self):
    """Calculate the absolute position of the marker in world coordinates."""
    abs_x = self.robot_x + self.rel_x * math.cos(self.robot_yaw) - self.rel_y * math.sin(self.robot_yaw)
    abs_y = self.robot_y + self.rel_x * math.sin(self.robot_yaw) + self.rel_y * math.cos(self.robot_yaw)
    return abs_x, abs_y

"""
Maker Catalog:
0-19: Wall markers
20-99: Reserved for future use
100-139: Acid markers
140-179: Basic markers
"""

BOX_SIZE = 0.13  # meters, used for calculating offsets when grabbing boxes

class Vision:
  closest_box = Marker.invalid()
  home = 0
  home_marker_id = 0

  def __init__(self, robot: "Robot"):
    self.robot = robot

  def init_cache(self):
    marker_cache.update({f"LOW:{box}": None for box in range(-3, 5)})
    marker_cache.update({f"HIGH:{box}": None for box in range(-3, 5)})
    marker_cache.update({f"HOME:{home}": [] for home in range(-1, 3)})


  def get_home_marker(self) -> int:
    """Returns the ID of the home marker (the closest wall marker)."""
    home_marker = (self.home * 5) % 20
    return home_marker

  def see(self):
    seen_markers = self.robot.sr_robot.camera.see()
    closest_wall_marker = min((m for m in seen_markers if 0 <= m.id < 20), key=lambda m: m.position.distance * np.cos(m.position.vertical_angle - CAMERA_TILT), default=None)
    markers = np.array([[marker.position.distance / 1000,
                      marker.position.vertical_angle,
                      marker.position.horizontal_angle,
                      marker.id] 
                      for marker in seen_markers if marker.id >= 100])
    
    if closest_wall_marker is None:
      return markers, None
  
    wall_marker = np.array([closest_wall_marker.position.distance * math.cos(closest_wall_marker.position.vertical_angle - CAMERA_TILT) / 1000,
                            closest_wall_marker.position.horizontal_angle,
                            WALL_MARKERS[closest_wall_marker.id][0],
                            WALL_MARKERS[closest_wall_marker.id][1]])
    
    return markers, wall_marker
  
  def get_marker(self, marker_id: int) -> Marker:
    seen_markers = self.robot.sr_robot.camera.see()
    self.wall_marker = WallMarker.from_april(min((m for m in seen_markers if 0 <= m.id < 20), key=lambda m: m.position.distance, default=None))
    self.closest_box = Marker.from_april(min(seen_markers, key=lambda m: m.position.distance if 100 <= m.id < 180 else float('inf'))) if seen_markers else Marker.invalid()
    target_markers = [marker for marker in seen_markers if marker.id == marker_id]
    if not target_markers:
      return Marker.invalid()
    
    marker = min(target_markers, key=lambda m: m.position.distance if abs(m.orientation.roll) < 0.1 else float('inf'))  # Prefer markers that are more face-on and closer
    return Marker.from_april(marker)

  def get_marker_with_offset(self, marker_id, distance=-BOX_SIZE/2, direction=None) -> Marker:
    marker = self.get_marker(marker_id)
    if marker.movement_measurement == -1: return marker
    return marker.add_offset(distance)
