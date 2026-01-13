from sr.robot3 import Robot as RB

from positional_agent.arena import Arena
from positional_agent.navigation import Navigation
from positional_agent.movement import Movement

from typing import Literal
from types import MappingProxyType
from math import pi

class Robot:
  nav: "Navigation"
  world: "Arena"

  _actions: MappingProxyType[str, list[tuple[callable, dict, bool]]] = {
    "on_transform_update": [],
    "on_marker_update": []
  }

  def __init__(self, *args, **kwargs):
    self._robot = RB(*args, **kwargs)
    self._robot.sleep(1)  # Allow time for sensors to initialize

    self.world = Arena(self)
    self.nav = Navigation(self, cardinal_offset=pi/2)
    self.move = Movement(self._robot.motor_board)
  
  def add_action(self, action: callable, reapeat:bool = False, trigger: Literal["on_transform_update", "on_marker_update"] = "on_transform_update", **args):
    """
    Add an action to the robot's action queue.
    """
    if trigger not in self._actions:
      raise ValueError(f"Invalid trigger: {trigger}. Valid triggers are: {list(self._actions.keys())}")

    self._actions[trigger].append((action, args, reapeat))
    
  async def trigger_actions(self, trigger: Literal["on_transform_update", "on_marker_update"]):
    """
    Trigger all actions for the given trigger.
    """
    if trigger not in self._actions:
      raise ValueError(f"Invalid trigger: {trigger}. Valid triggers are: {list(self._actions.keys())}")

    for action, args, _ in self._actions[trigger]:
      action(**args)

    # Cleanup non-repeating actions
    self._actions[trigger] = [(action, args, repeat) for action, args, repeat in self._actions[trigger] if repeat]
