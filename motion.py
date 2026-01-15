from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from robot import Robot

GRABBER_SPEED = 0.2
EXTEND_SPEED = 0.2

# Grabber wrapper for decorator
def grabber(func):
  def wrapper(self, *args, **kwargs):
    if self.grabber_moving: return
    self.grabber_moving = True
    result = func(self, *args, **kwargs)
    self.grabber_moving = False
    return result
  return wrapper

class Motion:
  is_open = False
  is_extended = False
  grabber_moving = False

  def __init__(self, robot: "Robot"):
    self.robot = robot
  
    # Initialize motor boards
    if self.robot.sr_robot.is_simulated:
      self.movement_board = self.robot.motor_board
      self.grabber_board = self.robot.motor_board
    else:
      self.movement_board = self.robot.motor_boards["SR0HDM"]
      self.grabber_board = self.robot.motor_boards["SR0PED"]
  
  def set_motor_speeds(self, left_speed, right_speed, time=0):
    self.movement_board.motors[0].power = max(-1, min(1, left_speed))
    self.movement_board.motors[1].power = max(-1, min(1, right_speed))
    if time > 0:
      self.robot.sr_robot.sleep(time)

  @grabber
  def grabber_open(self):
    if self.is_open: return
    self.is_open = True
    self.grabber_board.motors[0].power = GRABBER_SPEED
    self.robot.sleep(0.5)
  
  @grabber
  def grabber_close(self):
    if not self.is_open: return
    self.is_open = False
    self.grabber_board.motors[0].power = -GRABBER_SPEED
    self.robot.sleep(0.5)
  
  @grabber
  def grabber_extend(self):
    if self.is_extended: return
    self.is_extended = True
    self.grabber_board.motors[1].power = EXTEND_SPEED
    self.robot.sleep(0.5)
  
  @grabber
  def grabber_retract(self):
    if not self.is_extended: return
    self.is_extended = False
    self.grabber_board.motors[1].power = -EXTEND_SPEED
    self.robot.sleep(0.5)
  
  def grab(self):
    self.grabber_extend()
    self.grabber_close()
  
  def release(self):
    self.grabber_open()
    self.grabber_retract()
  
  @grabber
  def grab_analog(self, open_power: float, extend_power: float):
    self.grabber_board.motors[0].power = max(-1, min(1, open_power))
    self.grabber_board.motors[1].power = max(-1, min(1, extend_power))

  @grabber
  def reset_grabber(self):
    """Move grabber motors with low power to slowly reset their position until they hit their mechanical stops."""
    self.grabber_board.motors[0].power = -0.1  # Close grabber slowly
    self.grabber_board.motors[1].power = -0.1  # Retract grabber slowly
    self.robot.sleep(2)  # Adjust time as necessary

  def stop(self):
    self.set_motor_speeds(0, 0)
  
