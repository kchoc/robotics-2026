from sr.robot3 import Robot as RB

from arena import Arena
from navigation import Navigation
from movement import Movement

from typing import Literal
from types import MappingProxyType
from math import pi
import numpy as np

class Robot:
  nav: "Navigation"
  world: "Arena"

  _actions: MappingProxyType[str, list[tuple[callable, dict]]] = {
    "on_transform_update": [],
    "on_marker_update": []
  }

  def __init__(self, *args, **kwargs):
    self._robot = RB(*args, **kwargs)
    self._robot.sleep(1)  # Allow time for sensors to initialize

    self.world = Arena(self)
    self.nav = Navigation(self, cardinal_offset=pi/2)
    self.move = Movement(self._robot.motor_board)
  
  def add_action(self, action: callable, trigger: Literal["on_transform_update", "on_marker_update"] = "on_transform_update", **args):
    """
    Add an action to the robot's action queue.
    """
    if trigger not in self._actions:
      raise ValueError(f"Invalid trigger: {trigger}. Valid triggers are: {list(self._actions.keys())}")

    self._actions[trigger].append((action, args))
    
  async def trigger_actions(self, trigger: Literal["on_transform_update", "on_marker_update"]):
    """
    Trigger all actions for the given trigger.
    """
    if trigger not in self._actions:
      raise ValueError(f"Invalid trigger: {trigger}. Valid triggers are: {list(self._actions.keys())}")

    for action, args in self._actions[trigger]:
      action(**args)
  
r = Robot()
# r.add_action(r.world.display_grid, trigger="on_marker_update")

while True:
  r._robot.sleep(2)
  random_pos = [np.random.randint(1000, 2500), np.random.randint(-2500, -1000)]
  print("Looking at:", random_pos)
  print("Current pos:", r.nav.position, "Rotation:", r.nav.rotation)
  r.nav.look_at(random_pos, speed=0.5)
  # r.nav.move_to(random_pos, speed=0.5)
