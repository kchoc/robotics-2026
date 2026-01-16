from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from robot import Robot
from inputs import get_gamepad
from threading import Thread

def normalize_axis(value):
    """Convert axis (0-255) to -1.0 -> 1.0"""
    return (value - 128) / 128

class Controller:
  def __init__(self, robot: "Robot"):
    self.robot = robot
    self.forward = 0
    self.turn = 0
    self.open_grabber = 0
    self.extend_grabber = 0

    if self.robot.sr_robot.is_simulated:
      return  # No manual control in simulation

    self.control_thread = Thread(target=self.run, daemon=True)
    self.control_thread.start()
  
  def run(self):
    while True:
      self.process_inputs()

  def process_inputs(self):
    events = get_gamepad()
    for e in events:
      # Joystick controls
      if e.ev_type == "Absolute" and self.robot.autonomous == False:
        if e.code == "ABS_Y":  # forward/back
          self.forward = normalize_axis(e.state)  # invert: up = positive
        elif e.code == "ABS_X":  # left/right
          self.turn = normalize_axis(e.state)
        elif e.code == "ABS_RZ":  # forward/back
          self.open_grabber = normalize_axis(e.state) / 4  # invert: up = positive
        elif e.code == "ABS_Z":  # left/right
          self.extend_grabber = normalize_axis(e.state) / 4

        # Compute motor speeds (-1.0 to 1.0)
        left_speed = self.forward + self.turn
        right_speed = self.forward - self.turn

        # Clamp speeds
        left_speed = max(min(left_speed, 1.0), -1.0)
        right_speed = max(min(right_speed, 1.0), -1.0)

        self.robot.motion.grab_analog(self.open_grabber, self.extend_grabber)

        self.robot.motion.set_motor_speeds(left_speed, right_speed)
      
      # Button controls
      elif e.ev_type == "Key" and e.state == 1:  # Button pressed
        if e.code == "BTN_SOUTH":  # X button
          self.robot.autonomous = not self.robot.autonomous
        elif e.code == "BTN_NORTH":  # Triangle button
          self.robot.motion.reset_grabber()
          self.robot.motion.grab()
        elif e.code == "BTN_WEST":  # Square button
          self.robot.motion.reset_grabber()
          self.robot.motion.release()
        elif e.code == "BTN_EAST":  # Circle button
          if not self.robot.motion.emergency_stopped:
            self.robot.motion.emergency_stop()
          else:
            self.robot.motion.reset_emergency_stop()
          
