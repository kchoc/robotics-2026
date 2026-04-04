import random
import sys
from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from robot import Robot

from vision import Vision
from navigation import Navigation
from motion import Motion

RETURNING = "RETURNING"
STEALING = "STEALING"
GOING = "GOING"
DISPOSING = "DISPOSING"
PATROLLING = "PATROLLING"

class Tactics:
  target = "Acids"

  is_steal = False
  steal_state = "RETURNING"  # or "STEALING" or "GOING" (moving to the arena) or "DISPOSING" (moving the stolen box outside the arena)
  steal_arena = "HOME:0"  # or "HOME:-1" or "HOME:1" or "HOME:2"
  steal_marker_id = None
  steal_direction = 1

  script = []
  line_no = 0
  loop_start_line = -1

  def __init__(self, robot: "Robot"):
    Tactics.robot = robot
    Tactics.load_tactics()
  
  @classmethod
  def load_tactics(cls):
    with open (cls.target.lower()+".tactics", "r") as file:
      cls.script = file.readlines()
  
  @classmethod
  def param_to_marker_id(cls, param):
    target, offset = param.split(":")
    offset = int(offset)
    match target.upper():
      case "LOW":
        return Vision.translate_marker_id(param)
      case "HIGH":
        return Vision.translate_marker_id(param)
      case "HOME":
        return (Vision.get_home_marker() - offset*5) % 20 + (1 if offset < 0 else 0)

  @classmethod
  def next_zone(cls, current_zone, direction=1):
    if current_zone is None:
      return "HOME:0"
    
    _, offset = current_zone.split(":")
    offset = int(offset)
    offset = (offset + direction + 1) % 3 - 1
    return f"HOME:{offset}"

  @classmethod
  def is_target_marker(cls, marker_id):
    if cls.target == "Acids":
      return 100 <= marker_id < 140
    elif cls.target == "Bases":
      return 140 <= marker_id < 180
  
  @classmethod
  def steal(cls):
    print("STEALING STATE:", cls.steal_state)
    state = RETURNING
    if cls.steal_state == RETURNING: # Returned
      if cls.steal_arena == "HOME:0":
        state = cls.select_zone()
      else:
        Vision.scan()
        state = cls.stolen_box()
        
    elif cls.steal_state == GOING: # Arrived at arena
      state = cls.arrived_at_zone()

    elif cls.steal_state == STEALING: # Stole the box
      state = cls.stolen_box()

    elif cls.steal_state == DISPOSING: # Disposed of the box
      state = cls.disposed_box()
    
    cls.steal_state = state
    if state == PATROLLING:
      cls.steal_arena = cls.next_zone(cls.steal_arena, direction=cls.steal_direction)
      Navigation.set_target(-1, cls.steal_arena, "GOING", distance=1.5)

  @classmethod
  def select_zone(cls):
    best_zone = None
    best_zone_weight = 0
    for zone in ["HOME:-1", "HOME:1", "HOME:2"]:
      if Vision.marker_cache[zone] is not None and len(Vision.marker_cache[zone]) > 0:
        new_weight = sum([1 if Vision.is_target_marker(marker_id) else 0.5 for marker_id in Vision.marker_cache[zone]])
        if new_weight > best_zone_weight:
          best_zone = zone
          best_zone_weight = new_weight

    if best_zone is None: return PATROLLING
    
    Navigation.set_target(cls.param_to_marker_id(best_zone), best_zone, "GOING", distance=1.5)
    cls.steal_arena = best_zone
    return GOING

  @classmethod
  def arrived_at_zone(cls):
    Vision.scan()

    best_marker_id = None
    best_marker_weight = -1
    for marker_id in Vision.marker_cache[cls.steal_arena]:
      new_weight = 1 if Vision.is_target_marker(marker_id) else 0.5
      if new_weight > best_marker_weight:
        best_marker_id = marker_id
        best_marker_weight = new_weight

    if best_marker_id is None:
      return PATROLLING
    
    Vision.marker_cache[cls.steal_arena].remove(best_marker_id)
    Navigation.set_target(best_marker_id, None, "LOW", distance=0.2)
    cls.steal_marker_id = best_marker_id
    
    return STEALING

  @classmethod
  def stolen_box(cls):
    if cls.is_target_marker(cls.steal_marker_id):
      if cls.steal_arena == "HOME:2":
        cls.steal_arena = "HOME:1"
        Navigation.set_target(-1, "HOME:1", RETURNING, distance=0.5)
      else:
        cls.steal_arena = "HOME:0"
        Navigation.set_target(-1, "HOME:0", RETURNING, distance=0.5)
      return RETURNING
    
    Navigation.set_target(-1, f"WALL:{(cls.steal_arena + 2) % 20}", DISPOSING, distance=0.5)
    return DISPOSING

  @classmethod
  def disposed_box(cls):
    if len(Vision.marker_cache[cls.steal_arena]) == 0:
      return PATROLLING

    Motion.rotate_angle(3.14, speed=0.5)

    return GOING # This shortcuts back to the select box state, since no target is being set.

  @classmethod
  def process_script(cls):
    if cls.is_steal:
      cls.steal()
      return

    line = cls.script[cls.line_no]
    print("Current Instruction:", line)
    match line.split()[0].upper():
      case "GO":
        param = line.split()[1]
        match param.split(":")[0].upper():
          case "HOME":
            Navigation.set_target(-1, param, "HOME", distance=0.6)
          case "LOW":
            Navigation.set_target(-1, param, "LOW", distance=0.2)
          case "HIGH":
            Navigation.set_target(-1, param, "HIGH", distance=0.3)
      case "PUSH":
        param = line.split()[1]
        Navigation.set_target(-1, param, "PUSH", distance=0.6)
      case "STEAL":
        cls.is_steal = True
        cls.steal_direction = int(line.split()[1]) if len(line.split()) > 1 else 1
        cls.steal()
      case "END":
        sys.exit()
    cls.line_no += 1
    if cls.line_no >= len(cls.script):
      cls.line_no = 0
