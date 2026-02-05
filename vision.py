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

#TODO: I think true yaw calculations are incorrect as when looking at clearly turned boxes, it still states the true_yaw is 0 rad
# even though the yaw is wrong it still manages to go to the box??
#TODO Not repeat calculations if multiple faces of same box are visible?
#TODO tbh I'm not sure if I kept units consistent across everything, should probably check that 
# Distance is given in mm 

CAMERA_ROTATION = np.array([0, 0, 0])  # Roll, Pitch, Yaw in radians
CAMERA_POSITION = np.array([0, 0.005, 0])  # X, Y, Z in METRES from front center of robot

def unwrap_angle(new, prev):
    """Keep angle continuous across +-2pi boundaries (Had problems with box yaw jumping)"""
    if prev is None:
        return new
    delta = new - prev
    delta = np.arctan2(np.sin(delta), np.cos(delta))
    return prev + delta

def rot_z(yaw):
  """Rotation matrix around Z axis"""
  c, s = np.cos(yaw), np.sin(yaw)
  return np.array([[c, -s], [s, c]])

def polar_to_cartesian(distance, angle):
  """Convert polar coordinates to cartesian"""
  x = distance * np.cos(angle)
  y = distance * np.sin(angle)
  return np.array([x, y])

def camera_to_robot_frame(cam_xy):
  """Apply camera yaw rotation and translation into robot frame"""
  R = rot_z(CAMERA_ROTATION[2])
  return R @ cam_xy + CAMERA_POSITION[:2]

def compute_box_center_polar(distance, horizontal_angle, yaw, box_width, vertical_angle=0):
    """Find the center position of a box given one marker position and yaw"""

     # Marker position in camera frame
    cam_marker_xy = polar_to_cartesian(distance, horizontal_angle)

    # Marker position in robot frame
    marker_pos = camera_to_robot_frame(cam_marker_xy)

    # Box front face normal (unit vector in robot frame)
    face_normal = np.array([np.cos(yaw), np.sin(yaw)])

    # Offset marker along face normal by half the box width to get center
    center_pos = marker_pos + face_normal * (box_width / 2.0)

    # Convert back to polar coordinates
    center_distance = np.linalg.norm(center_pos)
    center_angle = np.arctan2(center_pos[1], center_pos[0])

    height = np.sin(vertical_angle) * center_distance  # Find height based on angle and distance
    center_distance = np.cos(vertical_angle) * center_distance  # Adjust distance for height
        
    

    return center_distance, center_angle, height

class Vision:

  # [horizontal_distance, height, horizontal_angle, yaw, timestamp]
  wall_markers = np.zeros((20, 5), dtype=np.float64)
  boxes = np.zeros((80, 5), dtype=np.float64)
  robots = np.zeros((3, 5), dtype=np.float64)
  last_update = 0.0
  
  def __init__(self, robot: "Robot"):
    self.robot = robot
    self.prev_box_yaw = [None] * 80

  def see(self):
    seen_markers = self.robot.sr_robot.camera.see()
    time = self.robot.sr_robot.time()
    self.last_update = time
    for marker in seen_markers:
      if 0 <= marker.id < 20:
        # Convert camera polar to robot polar
        cam_xy = polar_to_cartesian(marker.position.distance, marker.position.horizontal_angle)
        robot_xy = camera_to_robot_frame(cam_xy)

        distance = np.linalg.norm(robot_xy)
        angle = np.arctan2(robot_xy[1], robot_xy[0])

        # Add camera yaw offset and wrap to [0, 2*pi]
        yaw = (marker.orientation.yaw + CAMERA_ROTATION[2]) % (2*np.pi)

        # Store wall marker data
        self.wall_markers[marker.id] = np.array([
          distance,
          0,
          angle,
          yaw,
          time
        ])

      elif 100 <= marker.id < 180:

        #Finding actual yaw of box depending on fiducial marker orientation assuming box is flat on the ground
        if (marker.orientation.roll + 0.1) % (np.pi/2) > 0.2: # Roll is roughly pi/2 or 3pi/2
          true_yaw = marker.orientation.pitch
        
        elif (marker.orientation.roll + 0.1) % (np.pi) > 0.2: # Roll is roughly 0 or pi
          true_yaw = marker.orientation.yaw

        else:
          true_yaw = marker.orientation.roll
        print(f"True yaw for marker {marker.id} based on roll and pitch: {true_yaw:.2f}rad")

        # Add camera yaw and ensure angle continuity across frames
        raw_yaw = true_yaw + CAMERA_ROTATION[2]
        idx = marker.id - 100
        raw_yaw = unwrap_angle(raw_yaw, self.prev_box_yaw[idx])
        self.prev_box_yaw[idx] = raw_yaw
        true_yaw = raw_yaw

        # Compute robot-frame center of the box
        center_distance, center_angle, center_height = compute_box_center_polar(
          marker.position.distance,
          marker.position.horizontal_angle,
          true_yaw,
          box_width=80, #(mm)
          vertical_angle=marker.position.vertical_angle
        )

        # Store box data
        self.boxes[marker.id - 100] = np.array([
          center_distance,
          center_height,
          center_angle,
          true_yaw,
          time
        ])
        print(f"Box {marker.id}: Center Distance={center_distance:.2f}mm, Height={center_height:.2f}mm, Center Angle={center_angle:.2f}rad, Yaw={true_yaw:.2f}rad")      
      # print(f"Marker {marker.id}: Distance={horizontal_distance:.2f}m, Height={height:.2f}m, H_Angle={marker.position.horizontal_angle:.2f}rad, Yaw={yaw:.2f}rad")
  
    # Later, we can add robot detection here

  def get_box(self, marker_id: int) -> np.ndarray:
    return self.boxes[marker_id - 100]