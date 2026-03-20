from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from robot import Robot
from sr.robot3.motor_board import MotorBoard
from sr.robot3 import OUT_H0, OUT_H1

GRABBER_SPEED = 0.5
GRABBED_SPEED = 0.2
EXTEND_SPEED = 0.5
GRABBER_TIME = 0.5 / GRABBER_SPEED
EXTEND_TIME = 0.3 / EXTEND_SPEED

class Motion:
  emergency_stopped = False
  grabber_board: MotorBoard
  movement_board: MotorBoard

  def __init__(self, robot: "Robot"):
    self.robot = robot
  
    # Initialize motor boards
    if self.robot.sr_robot.is_simulated:
      self.movement_board = self.robot.sr_robot.motor_board
      self.grabber_board = self.robot.sr_robot.motor_board
    else:
      self.movement_board = self.robot.sr_robot.motor_boards["SR0PED"]
      self.grabber_board = self.robot.sr_robot.motor_boards["SR0HDM"]
  
  def set_motor_speeds(self, left_speed, right_speed, time=0):
    """Sets both of the motor speeds and sleeps x time if provided"""
    if self.emergency_stopped: return
    self.movement_board.motors[0].power = -max(-1, min(1, left_speed))
    self.movement_board.motors[1].power = max(-1, min(1, right_speed))
    if time > 0: self.robot.sr_robot.sleep(time)
  
  def move_distance(self, distance, speed):
    """Attempts to move an approximate distance with a set speed doesnt account for acceleration"""
    self.set_motor_speeds(speed, speed, time=distance/speed*0.5)
    self.stop()

  def emergency_stop(self):
    self.set_motor_speeds(0, 0)
    self.emergency_stopped = True
  
  def reset_emergency_stop(self):
    self.emergency_stopped = False
  
  def grab_high(self):
    if self.robot.sr_robot.is_simulated:
      self.robot.sr_robot.power_board.outputs[OUT_H1].is_enabled = True
      self.robot.sr_robot.sleep(0.5)
    elif not self.robot.sr_robot.is_simulated:
      self.grabber_board.motors[1].power = -EXTEND_SPEED
      self.robot.sr_robot.sleep(EXTEND_TIME)
      self.grabber_board.motors[1].power = 0
      self.grabber_board.motors[0].power = -GRABBER_SPEED
      self.robot.sr_robot.sleep(GRABBER_TIME)
      self.grabber_board.motors[0].power = -GRABBED_SPEED
      self.grabber_board.motors[1].power = EXTEND_SPEED
      self.robot.sr_robot.sleep(EXTEND_TIME / 2)
      self.grabber_board.motors[1].power = 0
  
  def release_high(self):
    if self.robot.sr_robot.is_simulated:
      self.robot.sr_robot.power_board.outputs[OUT_H1].is_enabled = False
      self.robot.sr_robot.sleep(0.5)
    elif not self.robot.sr_robot.is_simulated:
      self.grabber_board.motors[1].power = EXTEND_SPEED
      self.grabber_board.motors[0].power = GRABBER_SPEED
      self.robot.sr_robot.sleep(EXTEND_TIME/2)
      self.grabber_board.motors[1].power = 0
      self.robot.sr_robot.sleep(GRABBER_TIME - EXTEND_TIME/2)
      self.grabber_board.motors[0].power = 0

  def grab_low(self):
    if self.robot.sr_robot.is_simulated:
      self.robot.sr_robot.power_board.outputs[OUT_H0].is_enabled = True
      self.robot.sr_robot.sleep(0.5)
    else:
      self.robot.sr_robot.servo_board.servos[3].position = 0.3
      self.robot.sr_robot.sleep(1)
  
  def release_low(self):
    if self.robot.sr_robot.is_simulated:
      self.robot.sr_robot.power_board.outputs[OUT_H0].is_enabled = False
    else:
      self.robot.sr_robot.servo_board.servos[3].position = -1
      self.robot.sr_robot.sleep(1)
  
  def release(self):
    self.release_low()
    self.release_high()
  
  def grab_analog(self, open_power: float, extend_power: float):
    if self.emergency_stopped: return 
    self.grabber_board.motors[1].power = max(-1, min(1, open_power))
    self.grabber_board.motors[0].power = max(-1, min(1, extend_power))

  def reset_grabber(self):
    """Move grabber motors with low power to slowly reset their position until they hit their mechanical stops."""
    self.grabber_board.motors[0].power = -0.1  # Close grabber slowly
    self.grabber_board.motors[1].power = -0.1  # Retract grabber slowly
    self.robot.sr_robot.sleep(2)  # Adjust time as necessary

  def stop(self):
    self.set_motor_speeds(0, 0)
  
