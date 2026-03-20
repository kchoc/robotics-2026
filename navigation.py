from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from robot import Robot
import math
import random
from vision import Marker

K_RHO = 0.5    # Proportional gain for distance
K_ALPHA = 2.4  # Proportional gain for heading
K_BETA = 0.0 # Proportional gain for final orientation correction

LOST = "LOST"
FOUND = "FOUND"
LOOKING = "LOOKING"

MIN_SPEED = 0.15 # 0.125 on floor
MIN_MOVE_SPEED = 0.3

class Navigation:
  last_marker = Marker.invalid()
  lost_count = 0
  target_marker_id = -1
  target_marker_type = "LOW"  # or "HIGH" or "HOME"
  max_speed = 0.5
  turn_speed = 0.25
  target_distance = 0.3  # meters
  last_alpha = 0.0

  def __init__(self, robot: "Robot"):
    self.robot = robot
    
  def set_target(self, marker_id, type, distance=0.3, last_alpha=0.0):
    self.target_marker_id = marker_id
    self.target_distance = distance
    self.lost_count = 0
    self.last_alpha = last_alpha
    self.target_marker_type = type
    self.last_marker = Marker.invalid()
    print(f"Moving to marker {marker_id} {type}")

  def marker_lost(self):
    self.lost_count += 1
    if self.lost_count > 50:
      print(f"Marker {self.target_marker_id} lost for too long, giving up.")
      return LOST
    
    if self.last_marker.movement_measurement != -1:
      if self.last_marker.horizontal_distance < 0.1:
        return FOUND
      
      if self.lost_count < 5:
        self.robot.motion.stop()
      else:
        speed = self.turn_speed * math.copysign(1, self.last_alpha)
        self.robot.motion.set_motor_speeds(speed, -speed) # Turn towards last known direction
    else:
      self.robot.motion.set_motor_speeds(self.turn_speed, -self.turn_speed) # Turn in place to search
    return LOOKING

  def move_to_marker(self):
    default_marker = self.robot.vision.get_marker(self.target_marker_id)
    if default_marker.movement_measurement == -1:
      return self.marker_lost()
    if abs(default_marker.horizontal_angle) > 1 and default_marker.horizontal_distance < 0.3:
      return FOUND
    marker = Marker.add_offset(default_marker, self.target_distance)
    
    self.lost_count = 0
    dist = marker.horizontal_distance
    self.last_alpha = alpha =  marker.horizontal_angle if marker.horizontal_distance > 0.3 else default_marker.horizontal_angle
    beta = marker.yaw  # Desired final orientation correction

    if dist < 0.1 or marker.movement_measurement == 3: return FOUND # Target reached
      
    proximity_factor = max(0, 1 - dist / 2.0)  # 0 when far, 1 at target
    v = K_RHO * dist
    omega = K_ALPHA * alpha * (1 - proximity_factor) + K_BETA * beta * (1 + proximity_factor * 2)
    wheel_base = 0.4 # Conversion from linear and angular velocity to wheel speeds
    left_speed = v + (omega * wheel_base / 2)
    right_speed = v - (omega * wheel_base / 2)

    scale = self.max_speed / max(abs(left_speed), abs(right_speed), self.max_speed)
    if max(left_speed, right_speed) < MIN_MOVE_SPEED:
      scale = MIN_MOVE_SPEED / max(left_speed, right_speed)
    
    self.robot.motion.set_motor_speeds(left_speed * scale, right_speed * scale)
    print(f"Moving to marker {self.target_marker_id}: dist={dist:.2f}, alpha={math.degrees(alpha):.1f}°, beta={math.degrees(beta):.1f}°, left_speed={left_speed*scale:.2f}, right_speed={right_speed*scale:.2f}, measurement={marker.movement_measurement}")
    return LOOKING
  
  def final_approach(self, accuracy=0.05):
    # First align to face the box directly
    self.lost_count = 0
    while True:
      marker = Marker.get_center(self.robot.vision.get_marker(self.target_marker_id))
      if marker.isinvalid():
        if self.lost_count > 5: return LOST
        self.lost_count += 1
        self.robot.motion.move_distance(-0.1, -self.max_speed)
        continue

      self.lost_count = 0
      if abs(angle:= marker.horizontal_angle) > accuracy:
        speed = math.copysign(MIN_SPEED + random.uniform(0, 0.05), angle)
        self.robot.motion.set_motor_speeds(speed, -speed)
        self.robot.sr_robot.sleep(abs(angle))
        self.robot.motion.stop()
        continue
      
      return FOUND

  def grab(self, high_or_low="LOW"):
    res = self.final_approach()
    if res == LOST:
      self.target_marker_id = -1
      return

    if high_or_low == "LOW":
      marker = Marker.get_center(self.robot.vision.get_marker(self.target_marker_id))
      self.robot.motion.move_distance(marker.horizontal_distance + 0.4, self.max_speed)
      self.robot.motion.grab_low()
      self.turn_speed *= 1.2
      self.robot.low_box = self.target_marker_id
    elif high_or_low == "HIGH":
      marker = Marker.get_center(self.robot.vision.get_marker(self.target_marker_id))
      self.robot.motion.move_distance(marker.horizontal_distance - 0.1, self.max_speed)
      self.robot.motion.grab_high()
      self.robot.high_box = self.target_marker_id
    self.robot.motion.move_distance(-0.4, -self.max_speed)
    self.target_marker_id = -1  

  def drop(self):
    self.robot.motion.stop()
    self.robot.motion.release_low()
    self.robot.motion.set_motor_speeds(-self.max_speed, -self.max_speed, time=0.35)  # Move back a bit to clear the box
    self.robot.motion.stop()
    self.robot.motion.release_high()
    self.robot.motion.set_motor_speeds(-self.max_speed, -self.max_speed, time=0.3)  # Move back a bit to clear the box
    self.robot.motion.stop()
    if self.robot.low_box != -1:
      self.turn_speed /= 1.2
    self.robot.low_box = -1
    self.robot.high_box = -1
    self.target_marker_id = -1