import math
from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from robot import Robot
from sr.robot3.motor_board import MotorBoard
from sr.robot3.servo_board import ServoBoard

GRABBER_SPEED = 0.8
GRABBED_SPEED = 0.4
EXTEND_SPEED = 0.5
GRABBER_TIME = 0.5 / GRABBER_SPEED
EXTEND_TIME = 0.4 / EXTEND_SPEED

class Motion:
  robot: "Robot" = None
  emergency_stopped = False
  grabber_board: MotorBoard = None
  movement_board: MotorBoard = None
  servo_board: ServoBoard = None

  def __init__(self, robot: "Robot"):
    Motion.robot = robot
  
    # Initialize motor boards
    if robot.sr_robot.is_simulated:
      Motion.movement_board = robot.sr_robot.motor_board
      Motion.grabber_board = robot.sr_robot.motor_board
    else:
      Motion.movement_board = robot.sr_robot.motor_boards["SR0PED"]
      Motion.grabber_board = robot.sr_robot.motor_boards["SR0HDM"]
    
    Motion.servo_board = robot.sr_robot.servo_board
  
  @classmethod
  def set_motor_speeds(cls, left_speed, right_speed, time=0):
    """Sets both of the motor speeds and sleeps x time if provided"""
    if cls.emergency_stopped: return
    cls.movement_board.motors[0].power = -max(-1, min(1, left_speed))
    cls.movement_board.motors[1].power = max(-1, min(1, right_speed))
    if time > 0: cls.robot.sr_robot.sleep(time)
  
  @classmethod
  def move_distance(cls, distance, speed):
    """Attempts to move an approximate distance with a set speed doesnt account for acceleration"""
    cls.set_motor_speeds(speed, speed, time=distance/speed*0.5)
    cls.stop()
  
  @classmethod
  def rotate_angle(cls, angle, speed):
    """Attempts to rotate an approximate angle in radians with a set speed doesnt account for acceleration"""
    # Assuming robot rotates at 0.5 m/s and has a turning radius of 0.2 m, we can calculate the time to rotate the desired angle
    turning_radius = 0.2
    linear_speed = speed
    angular_speed = linear_speed / turning_radius
    time_to_rotate = abs(angle) / angular_speed
    speed = math.copysign(speed, angle)
    cls.set_motor_speeds(speed, -speed, time=time_to_rotate)
    cls.stop()

  @classmethod
  def emergency_stop(cls):
    cls.set_motor_speeds(0, 0)
    cls.emergency_stopped = True
  
  @classmethod
  def reset_emergency_stop(cls):
    cls.emergency_stopped = False
  
  @classmethod
  def grab_high(cls):
    cls.grabber_board.motors[1].power = -EXTEND_SPEED
    cls.robot.sr_robot.sleep(EXTEND_TIME)
    cls.grabber_board.motors[1].power = 0
    cls.grabber_board.motors[0].power = -GRABBER_SPEED 
    cls.robot.sr_robot.sleep(GRABBER_TIME)
    cls.grabber_board.motors[0].power = -GRABBED_SPEED
    cls.grabber_board.motors[1].power = EXTEND_SPEED
    cls.robot.sr_robot.sleep(EXTEND_TIME / 2)
    cls.grabber_board.motors[1].power = 0
  
  @classmethod
  def release_high(cls):
    cls.grabber_board.motors[1].power = EXTEND_SPEED
    cls.grabber_board.motors[0].power = GRABBER_SPEED
    cls.robot.sr_robot.sleep(EXTEND_TIME/2)
    cls.grabber_board.motors[1].power = 0
    cls.robot.sr_robot.sleep(GRABBER_TIME - EXTEND_TIME/2)
    cls.grabber_board.motors[0].power = 0

  @classmethod
  def grab_low(cls):
    cls.servo_board.servos[3].position = 0.3
    cls.robot.sr_robot.sleep(0.5)
  
  @classmethod
  def release_low(cls):
    cls.servo_board.servos[3].position = -1
    cls.robot.sr_robot.sleep(0.5)
  
  @classmethod
  def release(cls):
    cls.servo_board.servos[3].position = -1
    cls.release_high()

  @classmethod
  def stop(cls):
    cls.set_motor_speeds(0, 0)
  
