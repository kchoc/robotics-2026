import sys
from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from robot import Robot

class Tactics:
  target = "Acids"
  def __init__(self, robot: "Robot"):
    self.robot = robot
    self.load_tactics()
  
  def load_tactics(self):
    with open (self.target.lower()+".tactics", "r") as file:
      self.script = file.readlines()
    self.line_no = 0
    self.loop_start_line = -1
  
  def param_to_marker_id(self, param):
    target, offset = param.split(":")
    offset = int(offset)
    match target.upper():
      case "LOW":
        return 111 if offset == 0 else 157
      case "HIGH":
        return 127 if offset == 0 else 173
      case "HOME":
        return 0
  
  def process_script(self):
    line = self.script[self.line_no]
    match line.split()[0].upper():
      case "GO":
        param = line.split()[1]
        marker_id = self.param_to_marker_id(param)
        match param.split(":")[0].upper():
          case "HOME":
            self.robot.navigation.set_target(marker_id, "HOME", distance=0.5)
          case "LOW":
            self.robot.navigation.set_target(marker_id, "LOW", distance=0.2)
          case "HIGH":
            self.robot.navigation.set_target(marker_id, "HIGH", distance=0.3)
      case "IF":
        return
      case "ENDIF":
        return
      case "LOOP":
        self.loop_start_line = self.line_no
      case "ENDLOOP":
        if self.loop_start_line != -1:
          self.line_no = self.loop_start_line
          return
      case "END":
        sys.exit()
    self.line_no += 1
    if self.line_no >= len(self.script):
      self.line_no = 0
