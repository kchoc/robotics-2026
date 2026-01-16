from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from robot import Robot
import math
from threading import Thread

K_RHO = 0.5    # Proportional gain for distance
K_ALPHA = 0.8  # Proportional gain for heading
K_BETA = -0.8  # Proportional gain for final orientation correction
ULTRASONIC_START = 150  # mm

LOST = "LOST"
FOUND = "FOUND"
LOOKING = "LOOKING"

class Navigation:
  lost_count = 0
  target_marker_id = -1

  def __init__(self, robot: "Robot"):
    self.robot = robot

  def get_marker(self, marker_id):
    self.robot.vision.see()
    marker = self.robot.vision.get_box(marker_id)
    return marker
  
  def get_marker_yaw(self, marker):
    side = (int(marker[3] * 2.0 / math.pi + 4.5)) % 4
    return marker[3] if side == 0 else marker[3] if side == 1 else -marker[3] if side == 2 else -marker[3]
  
  def move_to_marker(self, marker_id, target_distance=0.05, last_alpha=-1, turn_speed=0.2, max_speed = 0.5):
    if self.target_marker_id != marker_id:
      self.target_marker_id = marker_id
      self.lost_count = -20
    
    marker = self.get_marker(marker_id)
    if marker[4] != self.robot.vision.last_update:
      if (dist := self.robot.sr_robot.arduino.ultrasound_measure(2,3)) < ULTRASONIC_START:
        if dist / 1000 <= target_distance:
          return FOUND
        self.robot.motion.set_motor_speeds(max_speed / 2, max_speed / 2)
      else:
        self.lost_count = self.lost_count + 1
        if self.lost_count > 20: return LOST
        self.robot.motion.set_motor_speeds(turn_speed if last_alpha > 0 else -turn_speed, -turn_speed if last_alpha > 0 else turn_speed, time=0.05)
      return LOOKING
      
    self.lost_count = 0
    dist = marker[0] / 1000
    last_alpha = alpha = marker[2]  # Angle to target
    beta = - alpha - self.get_marker_yaw(marker)  # Desired final orientation correction
      
    if dist < target_distance: found = True; return FOUND # Target reached
      
    proximity_factor = max(0, 1 - (dist - target_distance) / 2.0)  # 0 when far, 1 at target
    v = K_RHO * dist  # Linear velocity
    omega = K_ALPHA * alpha + K_BETA * beta * (1 + proximity_factor * 2)  # Boost beta as we get closer
    wheel_base = 0.24 # Conversion from linear and angular velocity to wheel speeds
    left_speed = v + (omega * wheel_base / 2)
    right_speed = v - (omega * wheel_base / 2)
    scale = max_speed / max(abs(left_speed), abs(right_speed), max_speed)
    self.robot.motion.set_motor_speeds(left_speed * scale, right_speed * scale)

    return LOOKING
