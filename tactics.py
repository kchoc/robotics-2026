import numpy as np

from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from robot import Robot

class Tactics:
  def __init__(self, robot: "Robot"):
    self.robot = robot
    self.target = "Acids"
  
  def calculate_risk_reward(self):
    # Iterate through seen boxes and calculate risk/reward
    # first sort boxes by angle to robot
    boxes = self.robot.vision.boxes.sort(axis=2)
    risk_reward = []
    for i, box in enumerate(boxes):
      prev_box = boxes[i-1] if i > 0 else boxes[-1]
      next_box = boxes[i+1] if i < len(boxes)-1 else boxes[0]

      # Calculate risk based on proximity to other boxes
      risk = 0
      dist_prev = box[0] - prev_box[0]
      dist_next = box[0] - next_box[0]
      if dist_prev < 0.5:
        risk += (0.5 - dist_prev) * 2  # Closer than 0.5m increases risk
      if dist_next < 0.5:
        risk += (0.5 - dist_next) * 2  # Closer than 0.5m increases risk
      
      # Calculate reward based on box properties
      reward = 1  # Base reward
      if self.target == "Acids" and 100 <= box[0] < 140:
        reward += 2  # Acid boxes are more rewarding
      elif self.target == "Basics" and 140 <= box[0] < 180:
        reward += 2  # Basic boxes are more rewarding
      # Add reward for being closer
      reward += max(0, 2 - box[0])  # Closer boxes are more rewarding
    
      box_risk_reward = reward - risk
      risk_reward.append((i, box_risk_reward))
    
    # Select box with highest risk-reward ratio
    risk_reward.sort(key=lambda x: x[1], reverse=True)
    if risk_reward:
      best_box_index = risk_reward[0][0]
      return boxes[best_box_index]

