from sr.robot3 import Robot as SRRobot

from vision import Vision
from tactics import Tactics
from motion import Motion
from manual_controller import Controller
from navigation import Navigation

class Robot:
  autonomous = True
  target_box_id = -1

  def __init__(self):
    self.sr_robot = SRRobot()
    self.vision = Vision(self)
    self.tactics = Tactics(self)
    self.motion = Motion(self)
    # self.controller = Controller(self)
    self.navigation = Navigation(self)

    # while True:
      # self.vision.see()

    self.run()

  def __getattr__(self, name):
    """Delegate attribute access to sr_robot if not found in Robot."""
    return getattr(self.sr_robot, name)

  def run(self):
    while True:
      if not self.sr_robot.is_simulated:
        self.controller.process_inputs()

      if self.autonomous and self.target_box_id != -1:
        if self.navigation.move_to_marker(self.target_box_id) == "FOUND":
          print(f"Marker {self.target_box_id} found")
          self.motion.stop()
          self.motion.grab()
          self.target_box_id = -1
      

robot = Robot()    
