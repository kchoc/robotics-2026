from typing import TYPE_CHECKING

if TYPE_CHECKING:
  from robot import Robot
import math
import random

from vision import Marker, Vision
from motion import Motion

K_RHO = 0.5    # Proportional gain for distance
K_ALPHA = 2.4  # Proportional gain for heading
K_BETA = 0.0 # Proportional gain for final orientation correction

LOST = "LOST"
FOUND = "FOUND"
LOOKING = "LOOKING"
ESTIMATING = "ESTIMATING"

MIN_SPEED = 0.15 # 0.125 on floor
MIN_MOVE_SPEED = 0.3

class Navigation:
  robot: "Robot"
  last_marker = Marker.invalid()
  lost_count = 0
  target_marker_id = -1
  target_marker_tag = None
  target_marker_type = "LOW"  # or "HIGH" or "HOME"
  seen_count = 0
  max_speed = 0.5
  turn_speed = 0.5
  target_distance = 0.3  # meters

  def __init__(self, robot: "Robot"):
    Navigation.robot = robot

  @classmethod
  def set_target(cls, marker_id, marker_tag, type, distance=0.3):
    cls.target_marker_id = marker_id
    cls.target_marker_tag = marker_tag
    cls.target_distance = distance
    cls.lost_count = 0
    cls.seen_count = 0
    cls.target_marker_type = type
    cls.last_marker = Marker.invalid()
    print(f"Moving to marker {marker_id} {marker_tag} {type}")

  @classmethod
  def marker_lost(cls):
    cls.lost_count += 1
    if cls.lost_count > 20:
      print(f"Marker {cls.target_marker_id} {cls.target_marker_tag} lost for too long, giving up.")
      return LOST
    
    if cls.last_marker.isvalid():
      if cls.last_marker.horizontal_distance < 0.1:
        return FOUND
      
      if cls.lost_count < 5:
        Motion.stop()
      else:
        Motion.rotate_angle(math.radians(60) * math.copysign(1, cls.last_marker.horizontal_angle), cls.turn_speed)
        cls.robot.sr_robot.sleep(0.3)
      return LOOKING
    
    Motion.rotate_angle(math.radians(60), cls.turn_speed)
    return LOOKING

  @classmethod
  def debug_marker(cls, marker_id):
    marker = Vision.get_marker(None, marker_id)
    if marker.isinvalid():
      print(f"Marker {marker_id} not found")
    else:
      print(marker.__repr__())
    cls.robot.sr_robot.sleep(1)

  @classmethod
  def move_to_marker(cls):
    default_marker = Vision.get_marker(cls.target_marker_tag, cls.target_marker_id)
    if default_marker.isinvalid():
      return cls.marker_lost()
    
    if default_marker.isestimate() and (cls.seen_count > 2 or abs(default_marker.horizontal_angle) > 1):
      cls.last_marker = default_marker
      return cls.marker_lost()
    
    if not default_marker.isestimate():
      if cls.target_marker_id == -1:
        print("Found target marker ID:", default_marker.id)
      cls.target_marker_id = default_marker.id
      cls.seen_count += 1

    if abs(default_marker.horizontal_angle) > 1 and default_marker.horizontal_distance < 0.3:
      return FOUND

    marker = Marker.add_offset(default_marker, cls.target_distance) if not default_marker.isestimate() else default_marker

    cls.last_marker = marker
    
    cls.lost_count = 0
    dist = marker.horizontal_distance
    cls.last_marker.horizontal_angle = alpha =  marker.horizontal_angle if marker.horizontal_distance > 0.2 else default_marker.horizontal_angle
    beta = marker.yaw  # Desired final orientation correction

    if dist < 0.1 or marker.movement_measurement == 3:
      if cls.target_marker_id == -1:
        return LOST
      return FOUND
    
    proximity_factor = max(0, 1 - dist / 2.0)  # 0 when far, 1 at target
    v = K_RHO * dist
    omega = K_ALPHA * alpha * (1 - proximity_factor) + K_BETA * beta * (1 + proximity_factor * 2)
    wheel_base = 0.4 # Conversion from linear and angular velocity to wheel speeds
    left_speed = v + (omega * wheel_base / 2)
    right_speed = v - (omega * wheel_base / 2)

    scale = cls.max_speed / max(abs(left_speed), abs(right_speed), cls.max_speed)
    if max(left_speed, right_speed) < MIN_MOVE_SPEED:
      scale = MIN_MOVE_SPEED / max(left_speed, right_speed)
    
    Motion.set_motor_speeds(left_speed * scale, right_speed * scale)
    return LOOKING
  
  @classmethod
  def final_approach(cls, accuracy=0.05, dynamic_speed = False):
    # First align to face the box directly
    cls.lost_count = 0
    while True:
      marker = Marker.get_center(Vision.get_marker(None, cls.target_marker_id, preference="TOP"))
      if marker.isinvalid():
        if cls.lost_count > 5: return LOST
        cls.lost_count += 1
        Motion.move_distance(-0.1, -cls.max_speed)
        continue

      cls.lost_count = 0
      if abs(angle:= marker.horizontal_angle) > accuracy:
        speed = math.copysign(MIN_SPEED + random.uniform(0, 0.05), angle)
        if dynamic_speed:
          speed = max(speed, math.tanh(angle) * cls.turn_speed * 0.8, key=lambda x: abs(x))
        Motion.set_motor_speeds(speed, -speed)
        cls.robot.sr_robot.sleep(abs(angle))
        Motion.stop()
        continue
      
      return FOUND

  @classmethod
  def grab(cls, high_or_low="LOW"):
    Motion.stop()
    res = cls.final_approach()
    if res == LOST:
      cls.target_marker_id = -1
      cls.target_marker_tag = None
      return
    
    Vision.scan()

    if high_or_low == "LOW":
      Motion.release_low()
      marker = Marker.get_center(Vision.get_marker(None, cls.target_marker_id, preference="TOP"))
      Motion.move_distance(marker.horizontal_distance + 0.5, cls.max_speed)
      Motion.grab_low()
      cls.robot.low_box = cls.target_marker_id
    elif high_or_low == "HIGH":
      marker = Marker.get_center(Vision.get_marker(None, cls.target_marker_id, preference="TOP"))
      Motion.move_distance(marker.horizontal_distance - 0.1, cls.max_speed)
      Motion.grab_high()
      cls.robot.high_box = cls.target_marker_id
    Motion.move_distance(-0.4, -cls.max_speed)
    cls.target_marker_id = -1
    cls.target_marker_tag = None

  @classmethod
  def drop(cls):
    Motion.stop()
    Motion.release()
    Motion.move_distance(-0.5, -cls.max_speed)
    Motion.stop()
    Motion.grab_low()
    cls.robot.low_box = -1
    cls.robot.high_box = -1
    cls.target_marker_id = -1
    cls.target_marker_tag = None
  
  @classmethod
  def push(cls):
    Motion.stop()
    cls.final_approach(0.15, dynamic_speed=True)
    marker = Marker.get_center(Vision.get_marker(None, cls.target_marker_id))
    Motion.move_distance(1.8 + marker.horizontal_distance, cls.max_speed)
    Motion.stop()
    Motion.move_distance(-1, -cls.max_speed)
    cls.target_marker_id = -1
    cls.target_marker_tag = None
