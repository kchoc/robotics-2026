from sr.robot3 import Robot as SRRobot

from vision import Vision
from tactics import Tactics
from motion import Motion
from manual_controller import Controller
from navigation import Navigation, LOOKING, FOUND, LOST

class Robot:
  autonomous = False
  low_box = -1
  high_box = -1
  action = "IDLE"  # or "GRAB HIGH" or "GRAB LOW" or "PUSHING" or "DROPPING"

  def __init__(self):
    self.sr_robot = SRRobot()
    self.vision = Vision(self)
    self.tactics = Tactics(self)
    self.motion = Motion(self)

    self.controller = Controller(self)
    self.navigation = Navigation(self)

    self.motion.release()

    if self.sr_robot.is_simulated:
      self.autonomous = True

    # while True:
      # self.vision.see()
    self.sr_robot.sleep(0.2)
    self.run()

  def run(self):
    while True:
      if not self.sr_robot.is_simulated and not self.autonomous:
        self.controller.process_inputs()

      if self.autonomous:
        if self.navigation.target_marker_id != -1:
          match self.navigation.move_to_marker():
            case "FOUND":
              if self.navigation.target_marker_type in ["HIGH", "LOW"]:
                self.navigation.grab(self.navigation.target_marker_type)
              elif self.navigation.target_marker_type == "HOME":
                self.navigation.drop()
            case "LOST":
              print(f"Marker {self.navigation.target_marker_id} lost")
              self.navigation.target_marker_id = -1
            case "LOOKING":
              pass
        else:
          self.tactics.process_script()
      
      if self.sr_robot.is_simulated: self.sr_robot.sleep(0.01)

robot = Robot()    
