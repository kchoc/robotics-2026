from typing import TYPE_CHECKING

if TYPE_CHECKING:
  from robot import Robot
from inputs import get_gamepad

from motion import Motion
from vision import Vision

def normalize_axis(value):
    """Convert axis (0-255) to -1.0 -> 1.0"""
    return (value - 128) / 128

class Controller:
  robot: "Robot" = None
  forward = 0
  turn = 0
  open_grabber = 0
  extend_grabber = 0
  max_speed = 0.4


  def __init__(self, robot: "Robot"):
    Controller.robot = robot

  @classmethod
  def process_inputs(cls):
    events = get_gamepad()
    for e in events:
      # Joystick controls
      if e.ev_type == "Absolute" and Controller.robot.autonomous == False:
        if e.code == "ABS_Y":  # forward/back
          Controller.forward = normalize_axis(e.state)  # invert: up = positive
        elif e.code == "ABS_X":  # left/right
          Controller.turn = normalize_axis(e.state)
        elif e.code == "ABS_RZ":  # forward/back
          Controller.open_grabber = normalize_axis(e.state) * 0.5  # invert: up = positive
        elif e.code == "ABS_Z":  # left/right
          Controller.extend_grabber = normalize_axis(e.state) * 0.5
        elif e.code == "ABS_BRAKE": # Speed Boost
          Controller.max_speed = 0.7 + normalize_axis(e.state) * 0.3

        # Compute motor speeds (-1.0 to 1.0)
        left_speed = Controller.forward + Controller.turn
        right_speed = Controller.forward - Controller.turn

        # Clamp speeds
        left_speed = max(min(left_speed, 1.0), -1.0)
        right_speed = max(min(right_speed, 1.0), -1.0)

        if not Motion.emergency_stopped:
          Motion.movement_board.motors[0].power = right_speed * Controller.max_speed
          Motion.movement_board.motors[1].power = -left_speed * Controller.max_speed
          Motion.grabber_board.motors[1].power = Controller.open_grabber
          Motion.grabber_board.motors[0].power = Controller.extend_grabber
      
      # Button controls
      elif e.ev_type == "Key" and e.state == 1:  # Button pressed
        if e.code == "BTN_START":
          Controller.robot.autonomous = not Controller.robot.autonomous
          Motion.stop()
        elif e.code == "BTN_SOUTH":
          Vision.scan()
        elif e.code == "BTN_NORTH":  # Triangle button
          Motion.grab_low()
        elif e.code == "BTN_WEST":  # Square button
          Motion.release_low()
        elif e.code == "BTN_EAST":  # Circle button
          if not Motion.emergency_stopped:
            print("Emergency Stopped!")
            Motion.emergency_stop()
          else:
            print("Emergency Stop Resuming!")
            Motion.reset_emergency_stop()
