**NOTE I** This code has been updated based on OpenAI code [Gym Soccer](https://github.com/openai/gym-soccer/tree/master/gym_soccer).
**NOTE II** This code was part of a hackaton for Chip Satellites at the University of Auckland

**Status:** Archive (code is provided as-is, no updates expected)

# gym-chipsat

The ChipSat environment is simulation environment which tries to replicate an environment
where chip satellites are discharged from a rocket. Initially, it will rotate on various
directions given its initial momentum, an its job is to stabilize itself with respect to
earth.

# Installation

```bash
cd gym-chipsat
pip install -e .
```

# Run environment

An example code is provided in loop.py, where the important bits are:

```
import gym
from gym_chipsat.envs.chipsat_env import ChipSatEnv
env = gym.make('ChipSat-v0')
```
