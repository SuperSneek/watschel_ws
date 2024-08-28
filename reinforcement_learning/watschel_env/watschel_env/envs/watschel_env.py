from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from gymnasium import ActionWrapper, ObservationWrapper, RewardWrapper, Wrapper
from gymnasium.spaces import Box, Discrete
import time


class WatschelWorld(gym.Env):


    def __init__(self):
        #Init coppeliasim api
        self._reset_client()

        self.sim.setStepping(True)


        self.episode_index = 0
        self.step_index = 0

        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        self.observation_space = spaces.Dict(
            {
                "legs": spaces.Box(-2, 2, shape=(4,), dtype=float),
                "position": spaces.Box(-1000, 1000, shape=(2,), dtype=float),
            }
        )

        maxForce = 300

        # 4 bounded intervals to apply force to the 4 motors respectively
        self.action_space = spaces.Box(low=-maxForce, high=maxForce, shape=(4,))



    def _get_obs(self):
        return { "legs": self.sim.callScriptFunction("get_joint_angles", self.bridge_handle), "position" :  self.sim.callScriptFunction("get_position", self.bridge_handle)}
    
    def _get_info(self):
        return {}
    

    def reset(self, seed=None, options=None):
        self.episode_index = self.episode_index + 1
        self.step_index = 0

        # We need the following line to seed self.np_random
        super().reset(seed=seed)

        #Stop and restart simulation
        self.sim.stopSimulation()
        time.sleep(0.5)


        #Reset client every 50 episodes
        if self.episode_index % 50 == 0:
            self._reset_client()
            time.sleep(1)

        
        self.sim.startSimulation()

        return self._get_obs(), self._get_info()



    def _reset_client(self):
        #Init coppeliasim api
        client = RemoteAPIClient()
        self.sim = client.require('sim')
        self.bridge_handle = self.sim.getObject("/base_link_visual/zmq_bridge")

    def step(self, action):
        ## Send forces from action to coppeliasim
        self.sim.callScriptFunction("set_forces", self.bridge_handle, action.tolist())


        # An episode is done if the agent has reached the target
        terminated = self.sim.getSimulationTime() < 50 and self.sim.callScriptFunction("getIsFallen", self.bridge_handle)
        reward = self._get_reward()
        observation = self._get_obs()
        info = self._get_info()

        self.step_index = self.step_index + 1

        if self.step_index % 100 == 0:
            print("Taking action: " + str(action))
            print("Reward is " + str(reward))
            print("Observation was:" + str(observation))

        for i in range(5): #only do rewards every x steps
            self.sim.step()

        return observation, reward, terminated, False, info
    
    def _get_reward(self):
        #get reward from coppeliasim
        return self.sim.callScriptFunction("get_reward", self.bridge_handle)
    
    def close(self):
        self.sim.stopSimulation()

