# environments/marker_navigation_env.py

from deepbots.supervisor.controllers.robot_supervisor_env import RobotSupervisorEnv
import numpy as np
import json

class MarkerNavigationEnv(RobotSupervisorEnv):
    def __init__(self):
        super().__init__()
        self.observation_space_size = 2  # [distance, angle]
        self.action_space_size = 2       # [left_speed, right_speed]
        self.target_distance = 0.05      # meters

        self.observations = np.array([np.inf, 0.0])

    def get_observations(self):
        data = self.receive_from_robot()
        if data:
            self.observations = np.array(json.loads(data))
        return self.observations

    def get_reward(self, action):
        distance, angle = self.observations
        reward = -distance - abs(angle) * 0.2
        return reward

    def is_done(self):
        distance, _ = self.observations
        return distance < self.target_distance

    def reset(self):
        super().reset()
        self.observations = np.array([np.inf, 0.0])
        return self.observations
