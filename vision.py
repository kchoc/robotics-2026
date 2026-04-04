from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from robot import Robot
  from sr.robot3.marker import Marker as AprilTagMarker
  from sr.robot3.camera import AprilCamera
import numpy as np
import cv2
import math
from pydantic import BaseModel

from utils import true_floor_yaw
from enums import WALL_MARKERS, CAMERA_TILT, ALL_MARKERS

class Marker(BaseModel):
  """Represents a detected marker with its position and orientation relative to the robot."""
  id: int
  horizontal_distance: float
  height: float
  horizontal_angle: float
  yaw: float
  true_yaw: float
  movement_measurement: int  # 1 for edge marker, 2 for center marker, -1 for no marker

  def __init__(self, marker_id, horizontal_distance, height, horizontal_angle, yaw, true_yaw, movement_measurement, **kwargs):
    """Initialize a marker with positional arguments."""
    super().__init__(
      id=marker_id,
      horizontal_distance=horizontal_distance,
      height=height,
      horizontal_angle=horizontal_angle,
      yaw=yaw,
      true_yaw=yaw,
      movement_measurement=movement_measurement,
      **kwargs
    )

  @staticmethod
  def invalid() -> "Marker":
    """Create an invalid/missing marker."""
    return Marker(-1, 0, 0, 0, 0, 0, -1)

  def isinvalid(self):
    return self.movement_measurement == -1
  
  def isvalid(self):
    return self.movement_measurement != -1

  def isestimate(self):
    return self.movement_measurement == -2

  @classmethod
  def from_april(cls, marker: "AprilTagMarker"):
    """Convert an AprilTag marker from the camera into our Marker format."""
    if marker is None: return cls.invalid()
    horizontal_distance = marker.position.distance * math.cos(marker.position.vertical_angle - CAMERA_TILT) / 1000
    height = marker.position.distance * math.sin(marker.position.vertical_angle - CAMERA_TILT) / 1000
    horizontal_angle = marker.position.horizontal_angle

    yaw, is_upright = true_floor_yaw(marker.rvec, CAMERA_TILT)
    best_yaw = yaw
    if marker.id >= 100:
      for i in range(0, 4):
        test_yaw = (yaw + i * np.pi / 2 + np.pi / 4) % (2 * np.pi) - np.pi / 4
        if abs(test_yaw + horizontal_angle) < abs(best_yaw + horizontal_angle):
          best_yaw = test_yaw
    
    if marker.id == 157 and Vision.robot.autonomous == False:
      print("Is upright:", is_upright)
      print("Yaw:", yaw)
    
    movement_measurement = 1 if is_upright else 2  # Default to center of marker being the center of the box if upright, otherwise edge
    return cls(marker.id, horizontal_distance, height, horizontal_angle, best_yaw, yaw, movement_measurement)

  @staticmethod
  def marker_displacement(marker1, marker2) -> tuple[float, float]:
    """Calculate the displacement of marker 2 relative to marker 1 in distance and angle."""
    if marker1.movement_measurement == -1 or marker2.movement_measurement == -1:
      return (float('inf'), float('inf'))  # If either marker is invalid, return infinite displacement
    distance_diff = np.sqrt(marker1.horizontal_distance**2 + marker2.horizontal_distance**2 - 2 * marker1.horizontal_distance * marker2.horizontal_distance * math.cos(marker1.horizontal_angle - marker2.horizontal_angle))
    angle_diff = math.atan2(marker2.horizontal_distance * math.sin(marker2.horizontal_angle) - marker1.horizontal_distance * math.sin(marker1.horizontal_angle),
                            marker2.horizontal_distance * math.cos(marker2.horizontal_angle) - marker1.horizontal_distance * math.cos(marker1.horizontal_angle))
    return (distance_diff, angle_diff)

  def add_offset(self, distance, direction=None) -> "Marker":
    """Calculate the offset from a marker of a box to a point in front of the box (for grabbing) or in the center of the box (for navigation)."""
    if direction is not None: return
    
    if self.movement_measurement == 1: distance -= BOX_SIZE / 2  # If the marker is on the edge of the box, add half the box size to get to the center
    angle = self.horizontal_angle - self.true_yaw  # horizontal angle minus yaw gives the angle to the front of the box
    dist = np.sqrt(self.horizontal_distance**2 + distance**2 - 2 * self.horizontal_distance * distance * math.cos(angle))
    alpha = math.atan2(self.horizontal_distance * math.sin(self.horizontal_angle) - distance * math.sin(self.true_yaw),
                        self.horizontal_distance * math.cos(self.horizontal_angle) - distance * math.cos(self.true_yaw))
    if distance < self.horizontal_distance:
      movement_measurement = self.movement_measurement  # If the offset point is further away than the original marker, keep the same movement measurement (edge or center)
    else:
      movement_measurement = 3  # New movement measurement for the offset point, meaning it's in front of the box
    return Marker(self.id, dist, self.height, alpha, self.yaw, self.true_yaw, movement_measurement)

  def get_center(self) -> "Marker":
    """Calculate the center of the box from a marker on the edge of the box."""
    if self.movement_measurement == 2: return self  # Already at the center
    return self.add_offset(-BOX_SIZE / 2)

  def __repr__(self):
    """String representation of the marker for debugging purposes."""
    return f"Marker(id={self.id}, dist={self.horizontal_distance:.2f}m, height={self.height:.2f}m, h_angle={math.degrees(self.horizontal_angle):.1f}°, yaw={math.degrees(self.yaw):.1f}°, measurement={self.movement_measurement})"

class WallMarker(Marker):
  rel_x: float # The relative x position of the wall marker from the robot's perspective
  rel_y: float # The relative y position of the wall marker from the robot's perspective
  abs_x: float # The absolute x position of the wall marker in world coordinates
  abs_y: float # The absolute y position of the wall marker in world coordinates
  abs_yaw: float # The absolute yaw of the wall marker in world coordinates
  robot_x: float = 0.0 # The absolute x position of the robot in world coordinates
  robot_y: float = 0.0 # The absolute y position of the robot in world coordinates
  robot_yaw: float = 0.0 # The absolute yaw of the robot in world coordinates

  def __init__(self, marker_id, horizontal_distance, height, horizontal_angle, yaw, true_yaw, movement_measurement):
    abs_x = WALL_MARKERS[marker_id][0]
    abs_y = WALL_MARKERS[marker_id][1]
    abs_yaw = (marker_id // 5) * np.pi / 2
    robot_yaw = (abs_yaw - yaw + np.pi) % (2 * np.pi) - np.pi

    # Calculate relative position of marker from robot's perspective
    rel_x = horizontal_distance * math.sin(robot_yaw + horizontal_angle)
    rel_y = horizontal_distance * math.cos(robot_yaw + horizontal_angle)

    # Calculate robot's absolute position from marker's absolute position and relative offset
    robot_x = abs_x - rel_x
    robot_y = abs_y - rel_y

    super().__init__(marker_id, horizontal_distance, height, horizontal_angle, yaw, true_yaw, movement_measurement, rel_x=rel_x, rel_y=rel_y, abs_x=abs_x, abs_y=abs_y, abs_yaw=abs_yaw, robot_x=robot_x, robot_y=robot_y, robot_yaw=robot_yaw)
  
  def absolute_yaw_to_relative(self, yaw):
    """Convert an absolute yaw in world coordinates to a relative yaw from the robot's perspective."""
    return (yaw - self.robot_yaw + np.pi) % (2 * np.pi) - np.pi

  def marker_absolute_position(self, marker: Marker):
    """Calculate absolute position of a marker in world coordinates."""
    abs_x = self.robot_x + marker.horizontal_distance * math.sin(self.robot_yaw + marker.horizontal_angle)
    abs_y = self.robot_y + marker.horizontal_distance * math.cos(self.robot_yaw + marker.horizontal_angle)
    return abs_x, abs_y
  
  def marker_estimated_position(self, marker_tag: str) -> Marker:
    """Creates a marker relative location to the robot based on the known absolute position of the wall marker and the known absolute position of the target marker."""
    abs_x, abs_y = ALL_MARKERS[marker_tag]

    horizontal_distance = math.sqrt((abs_x - self.robot_x)**2 + (abs_y - self.robot_y)**2)
    horizontal_angle = math.atan2(abs_x - self.robot_x, abs_y - self.robot_y) - self.robot_yaw
    return Marker(-1, horizontal_distance, 0, horizontal_angle, 0, 0, movement_measurement=-2)  # Use a special movement measurement to indicate this is an estimated position based on the wall marker

"""
Maker Catalog:
0-19: Wall markers
20-99: Reserved for future use
100-139: Acid markers
140-179: Basic markers
"""

BOX_SIZE = 0.13  # meters, used for calculating offsets when grabbing boxes

class Vision:
  robot: "Robot" = None
  marker_cache = {}
  camera: "AprilCamera" = None
  seen_markers: list = []
  zone_rot_mat: np.ndarray = None

  K: np.ndarray = None
  D: np.ndarray = None

  def __init__(self, robot: "Robot"):
    Vision.robot = robot
    Vision.camera = robot.sr_robot.camera

    Vision.zone_rot_mat = np.array([[np.cos(-robot.sr_robot.zone * np.pi / 2), -np.sin(-robot.sr_robot.zone * np.pi / 2)],
                                   [np.sin(-robot.sr_robot.zone * np.pi / 2), np.cos(-robot.sr_robot.zone * np.pi / 2)]])

    fs = cv2.FileStorage("calibration.xml", cv2.FILE_STORAGE_READ)

    Vision.K = fs.getNode("cameraMatrix").mat()
    Vision.D = fs.getNode("dist_coeffs").mat().flatten()  # [k1, k2, p1, p2, k3]
    fs.release()

    Vision.init_cache()

  @classmethod
  def init_cache(cls):
    cls.marker_cache.update({f"LOW:{box}": None for box in range(-3, 5)})
    cls.marker_cache.update({f"HIGH:{box}": None for box in range(-3, 5)})
    cls.marker_cache.update({f"HOME:{home}": [] for home in range(-1, 3)})

  @classmethod
  def translate_marker_id(cls, key: str):
    if key.split(":")[0] == "HOME":
      offset = int(key.split(":")[1])
      return (Vision.get_home_marker() - offset*5) % 20 + (1 if offset < 0 else 0)
    try:
      if key.split(":")[0] == "WALL":
        return int(key.split(":")[1])
    except (KeyError, ValueError):
      return -1
    
    return cls.marker_cache[key]

  @classmethod
  def get_home_marker(cls) -> int:
    """Returns the ID of the home marker (the closest wall marker)."""
    home_marker = (cls.robot.sr_robot.zone * 5 - 1) % 20
    return home_marker

  @classmethod
  def get_marker_abs_position(cls, marker_tag: str) -> tuple[float, float]:
    """Looks up the absolute position of a marker in world coordinates from the enums and transforms it based on the current home marker."""
    relative_position = np.array(ALL_MARKERS[marker_tag])
    absolute_position = cls.zone_rot_mat @ relative_position
    return absolute_position

  @classmethod
  def see(cls):
    """Returns a list of currently visible markers in our Marker format."""
    frame = cls.camera.capture()
    # Undistort using standard pinhole model
    undistorted = cv2.undistort(frame, Vision.K, Vision.D)
    seen_markers = cls.camera.see(frame=undistorted)
    cls.seen_markers = seen_markers
    return seen_markers

  @classmethod
  def scan(cls):
    """Scans for markers in opponents zone to update cache"""
    seen_markers = [Marker.from_april(m) for m in cls.see()]

    # Identify home wall markers 4, 9, 14, 19 excluding the current home marker
    opponent_wall_ids = { "HOME:-1": (cls.get_home_marker() - 5) % 20, 
                          "HOME:1": (cls.get_home_marker() + 5) % 20,
                          "HOME:2": (cls.get_home_marker() + 10) % 20}
    
    # Add to home cache
    for key, wall_id in opponent_wall_ids.items():
      wall_marker = next((m for m in seen_markers if m.id == wall_id), None)
      if wall_marker is not None:
        markers_to_remove = []
        for marker in seen_markers:
          print(f"Checking marker {marker.id} against {wall_marker} with distance {Marker.marker_displacement(wall_marker, marker)[0]}")
          if marker.id >= 100 and Marker.marker_displacement(wall_marker, marker)[0] < 1.0:  # If the marker is within 1 meter of the wall marker, consider it as a potential box marker
            cls.marker_cache[key].append(marker.id)
            markers_to_remove.append(marker)
        for marker in markers_to_remove:
          seen_markers.remove(marker)
  
  @classmethod
  def get_marker(cls, marker_tag: str, marker_id: int = -1, preference=None) -> Marker:
    seen_markers = cls.see()

    if marker_id == -1 and cls.marker_cache.get(marker_tag) is None:
      # Attempt to find a marker that closely matches the expected position of the marker based on the wall marker
      target_marker = cls.estimate_marker_position(marker_tag)
      if target_marker.isinvalid(): return Marker.invalid()

      closest_marker = min((Marker.from_april(m) for m in seen_markers if m.id >= 100), key=lambda m: Marker.marker_displacement(target_marker, m)[0], default=None)

      if closest_marker is not None and Marker.marker_displacement(target_marker, closest_marker)[0] < 0.3:  # If there's a marker within 0.5 meters of the expected position, consider it as the target marker
        cls.marker_cache[marker_tag] = closest_marker.id
        return closest_marker
      else:
        return target_marker

    target_marker_id = cls.translate_marker_id(marker_tag) if marker_id == -1 else marker_id
    target_markers = [marker for marker in seen_markers if marker.id == target_marker_id]
    if len(target_markers) == 0:
      return cls.estimate_marker_position(marker_tag) if marker_tag is not None else Marker.invalid()
    
    if preference is not None and preference == "TOP":
      marker = next((marker for marker in target_markers if true_floor_yaw(marker.rvec, CAMERA_TILT)[1]), target_markers[0])
    else:
      marker = min(target_markers, key=lambda m: m.position.distance)  # Prefer markers that are closer

    return Marker.from_april(marker)

  @classmethod
  def get_marker_with_offset(cls, marker_id, distance=-BOX_SIZE/2) -> Marker:
    marker = cls.get_marker(marker_id)
    if marker.movement_measurement == -1: return marker
    return marker.add_offset(distance)

  @classmethod
  def estimate_marker_position(cls, marker_tag: str) -> Marker:
    """Estimate the absolute position of a marker in world coordinates using the wall marker as a reference."""
    if marker_tag is None: return Marker.invalid()
    wall_marker = WallMarker.from_april(min((m for m in cls.seen_markers if m.id < 20), key=lambda m: m.position.distance, default=None))
    if wall_marker.isinvalid(): return Marker.invalid()

    marker = wall_marker.marker_estimated_position(str(marker_tag))
    return marker