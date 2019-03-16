import gym
import numpy as np
from gym_chipsat.envs.chipsat_env import ChipSatEnv
env = gym.make('ChipSat-v0')
observation = env.reset()
try:
    for t in range(100):
        ### Uncomment below to see a window appear with the
        ### environment running
        # env.render()
        action = env.action_space.sample()
        
        observation, reward, done, info = env.step(action)
        print(reward)
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
finally:
    env.close()