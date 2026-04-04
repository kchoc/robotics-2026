from sr.robot3 import Robot as SRRobot

from vision import Vision
from tactics import Tactics
from motion import Motion
from manual_controller import Controller
from navigation import Navigation

class Robot:
  autonomous = False
  low_box = -1
  high_box = -1

  def __init__(self):
    self.sr_robot = SRRobot()
    Vision(self)
    Motion(self)
    Controller(self)
    Navigation(self)
    Tactics(self)

    Motion.release_high()
    Motion.grab_low()

    if self.sr_robot.is_simulated:
      self.autonomous = True

    self.run()

  def run(self):
    while True:
      if not self.sr_robot.is_simulated and not self.autonomous:
        Controller.process_inputs()

      if self.autonomous:
        if Navigation.target_marker_id != -1 or Navigation.target_marker_tag is not None:
          match Navigation.move_to_marker():
            case "FOUND":
              if Navigation.target_marker_type in ["HIGH", "LOW"]:
                Navigation.grab(Navigation.target_marker_type)
              elif Navigation.target_marker_type == "HOME":
                Navigation.drop()
              elif Navigation.target_marker_type == "PUSH":
                Navigation.push()
              elif Navigation.target_marker_type == "GOING":
                Navigation.target_marker_id = -1
            case "LOST":
              print(f"Marker {Navigation.target_marker_id} lost")
              Navigation.target_marker_id = -1
            case "LOOKING":
              pass
        else:
          Tactics.process_script()
      
      if self.sr_robot.is_simulated: self.sr_robot.sleep(0.01)

robot = Robot()    
