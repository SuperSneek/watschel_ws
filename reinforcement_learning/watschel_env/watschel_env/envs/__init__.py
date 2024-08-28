from gymnasium.envs.registration import register
from watschel_env.envs.watschel_env import WatschelWorld

register(
     id="watschel_env/watschelworld-v0",
     entry_point="watschel_env.envs.watschel_env:WatschelWorld",
     max_episode_steps=3000,
)

print("YAYY")