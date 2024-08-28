from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import gym
from gym import spaces
import numpy as np
from gym import ActionWrapper, ObservationWrapper, RewardWrapper, Wrapper
from gym.spaces import Box, Discrete

class WatschelWorld(gym.Env):


    def __init__(self, render_mode=None, size=5):
        self.size = size  # The size of the square grid
        self.window_size = 512  # The size of the PyGame window

        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        self.observation_space = spaces.Dict(
            {
                "legs": spaces.Box(-2, 2, shape=(2,4), dtype=float),
                "position": spaces.Box(-1000, 1000, shape=(2,2), dtype=float),
            }
        )

        maxForce = 300

        # 4 bounded intervals to apply force to the 4 motors respectively
        self.action_space = spaces.Box(low=-maxForce, high=maxForce, shape=(2,4))



    def _get_obs(self):
        return None # todo: get observation from coppeliasim
    
    def _get_info(self):
        return {"distance": np.linalg.norm(self._agent_location - self._target_location, ord=1)}
    

    def reset():
        pass

    def step(self, action):
        ## Send forces from action to coppeliasim
        pass


        # An episode is done if the agent has reached the target
        terminated = False ##ask coppeliasim if terminated
        reward = self._get_reward()
        observation = self._get_obs()
        info = self._get_info()

        return observation, reward, terminated, False, info
    
    def _get_reward(self):
        #get reward from coppeliasim
        return 0
    
    def close(self):
        pass

class RelativePosition(ObservationWrapper):
    def __init__(self, env):
        super().__init__(env)
        self.observation_space = Box(shape=(2,), low=-np.inf, high=np.inf)

    def observation(self, obs):
        return obs["target"] - obs["agent"]