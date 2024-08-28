from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import gym
from gym import spaces
import numpy as np
from gym import ActionWrapper, ObservationWrapper, RewardWrapper, Wrapper
from gym.spaces import Box, Discrete
import time

client = RemoteAPIClient()
sim = client.require('sim')

sim.setStepping(True)

sim.startSimulation()
while (t := sim.getSimulationTime()) < 3:
    print(f'Simulation time: {t:.2f} [s]')
    sim.step()
sim.stopSimulation()

class WatschelWorld(gym.Env):


    def __init__(self, render_mode=None, size=5):
        #Init coppeliasim api
        self._reset_client()

        self.sim.setStepping(True)


        self.episode_index = 0

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
        return {}
    

    def reset(self, seed=None, options=None):
        self.episode_index = self.episode_index + 1


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



    def _reset_client(self):
        #Init coppeliasim api
        client = RemoteAPIClient()
        self.sim = client.require('sim')
        self.bridge_handle = self.sim.getObject("/base_link_visual/zmq_bridge")

    def step(self, action):
        ## Send forces from action to coppeliasim
        self.sim.callScriptFunction("set_forces", self.bridge_handle, action)


        # An episode is done if the agent has reached the target
        terminated = sim.getSimulationTime() < 5 and self.sim.callScriptFunction("is_fallen", self.bridge_handle)
        reward = self._get_reward()
        observation = self._get_obs()
        info = self._get_info()

        if self.episode_index % 10 == 0:
            print("Taking action: " + str(action))
            print("Reward is " + str(reward))

        for i in len(range(5)): #only do rewards every x steps
            self.sim.step()

        return observation, reward, terminated, False, info
    
    def _get_reward(self):
        #get reward from coppeliasim
        return self.sim.callScriptFunction("get_reward", self.bridge_handle)
    
    def close(self):
        self.sim.stopSimulation()

class RelativePosition(ObservationWrapper):
    def __init__(self, env):
        super().__init__(env)
        self.observation_space = Box(shape=(2,), low=-np.inf, high=np.inf)

    def observation(self, obs):
        return obs["target"] - obs["agent"]