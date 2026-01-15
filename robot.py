from sr.robot3 import Robot as SRRobot

from vision import Vision
from tactics import Tactics
from motion import Motion
from manual_controller import Controller
from navigation import Navigation

class Robot:
  def __init__(self):
    self.sr_robot = SRRobot()
    self.vision = Vision(self)
    self.tactics = Tactics(self)
    self.motion = Motion(self)
    self.controller = Controller(self)
    self.navigation = Navigation(self)

    self.navigation.move_to_marker(171)

  def __getattr__(self, name):
    return getattr(self.sr_robot, name)

  def run(self):
    while True:
      ...

robot = Robot()    
