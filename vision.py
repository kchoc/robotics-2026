from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from robot import Robot
import numpy as np

"""
Maker Catalog:
0-19: Wall markers
20-99: Reserved for future use
100-139: Acid markers
140-179: Basic markers
"""

#TODO: Update to use the camera rotation and position for angle and distance calculations
CAMERA_ROTATION = np.array([0, 0, 0])  # Roll, Pitch, Yaw in radians
CAMERA_POSITION = np.array([0, 0.05, 0])  # X, Y, Z in meters from front center of robot

class Vision:

  # [horizontal_distance, height, horizontal_angle, yaw, timestamp]
  wall_markers = np.zeros((20, 5), dtype=np.float64)
  boxes = np.zeros((80, 5), dtype=np.float64)
  robots = np.zeros((3, 5), dtype=np.float64)
  last_update = 0.0

  def __init__(self, robot: "Robot"):
    self.robot = robot

  def see(self):
    seen_markers = self.robot.sr_robot.camera.see()
    time = self.robot.sr_robot.time()
    self.last_update = time
    for marker in seen_markers:
      if 0 <= marker.id < 20:
        self.wall_markers[marker.id] = np.array([
          marker.position.distance,
          0,
          marker.position.horizontal_angle - CAMERA_ROTATION[2],
          (marker.orientation.yaw - CAMERA_ROTATION[0]) % (np.pi / 2),
          time
        ])
      elif 100 <= marker.id < 180:
        # Calculate the yaw of the box. The box may be rotated / flipped or otherwise different
        # The box however can be assumed to be flat on the ground. Therefore we will take the
        # measurement most different from a factor of 90 degrees (pi/2 radians)
        yaw = max(
          marker.orientation.roll,
          marker.orientation.pitch,
          marker.orientation.yaw,
          key=lambda angle: abs((angle % (np.pi / 2)) - (np.pi / 4))
        )


        self.boxes[marker.id - 100] = np.array([
          marker.position.distance,
          0,
          marker.position.horizontal_angle,
          yaw,
          time
        ])
      
      # print(f"Marker {marker.id}: Distance={horizontal_distance:.2f}m, Height={height:.2f}m, H_Angle={marker.position.horizontal_angle:.2f}rad, Yaw={yaw:.2f}rad")
  
    # Later, we can add robot detection here

  def get_box(self, marker_id: int) -> np.ndarray:
    return self.boxes[marker_id - 100]


