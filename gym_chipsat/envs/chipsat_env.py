"""
Chip satellite environment implemented for the hackaton at the University of Auckland
Based on code from https://github.com/openai/gym-soccer/tree/master/gym_soccer
"""

import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np

import serial

class ChipSatEnv(gym.Env):
    """
    Description:
        A chip satellite has been released from a rocket in space, which is rolling
        uncontrollably since then. The job of this environment is to create an
        algorithm to stabilize the mechanism within a reasonable time.

    Observation: dr per second
        Type: Box(3)
        Num	Observation             Min             Max
        0	Gyroscope x axis         -1               1
        1	Gyroscope y axis         -1               1
        5   Power                     0               1
        
    Actions:
        Type: Box(2)
        Num	Action
        0	Magnetorquer X            0               1
        0	Magnetorquer Y            0               1
        
    NOTE: Activation of the magnetorquer to its full value (1) would consume 0.05
    units of power, whereas no activation at all would still consume 0.0005 unit,
    everything in between would be linear in betweek.

    Reward:
        Reward is calculated as the ability to reach gyroscope values of 0 while
        keeping power above 0. If any action changes the gyroscope value closer
        to 0, then the reward is incremented.

    Starting State:
        Gyro observations are assigned a uniform random value between its boundaries,
        Power is initialized to 1.

    Episode Termination:
        Power is 0
        
    Solved Requirements:
        Considered solved when the gyroscope values are 0 and power is greater
        than 0 for 10 consecutive trials.
    """
    
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50
    }

    def __init__(self):
        high = 1
        self.massCS = 1.0
        self.powerCS = 1.0
        self.gyroX = 0
        self.gyroY = 0
        self.posX = 0
        self.posY = 0
        self.tau = 0.02  # seconds between state updates
        self.minPower = 0.0005
        self.maxPower = 0.05
        self.action_space = spaces.Box(0, high, dtype=np.float32, shape=(2,))
        self.observation_space = spaces.Box(-high, high, dtype=np.float32, shape=(3,))
        self.csState = None
        self.counter = None
        
        self.enableSerial = True
        
        ##### This is the serial part, i
        if self.enableSerial:
            self.ser = serial.Serial('/dev/tty.usbmodem14501')
        
        
        self.gravity = 9.8
        self.masscart = 1
        self.masspole = 0.1
        self.total_mass = (self.masspole + self.masscart)
        self.length = 0.5 # actually half the pole's length
        self.polemass_length = (self.masspole * self.length)
        self.force_mag = 10.0
        self.tau = 0.02  # seconds between state updates
        self.kinematics_integrator = 'euler'

        # Angle at which to fail the episode
        self.theta_threshold_radians = 12 * 2 * math.pi / 360
        self.x_threshold = 2.4

        # Angle limit set to 2 * theta_threshold_radians so failing observation is still within bounds
        high = np.array([
            self.x_threshold * 2,
            np.finfo(np.float32).max,
            self.theta_threshold_radians * 2,
            np.finfo(np.float32).max])

        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self.seed()
        self.viewer = None
        self.state = None

        self.steps_beyond_done = None

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        ########### Simlate a reset
        self.csState = self.np_random.uniform(-1, 1, size=(3,))
        self.csState[2] = self.np_random.uniform(0, 1)
        magX = self.np_random.uniform(0,1)
        magY = self.np_random.uniform(0,1)
        action = np.array([magX, magY])
        
        # assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))
        csState = self.csState
        
        gyroX, gyroY, powerCS = csState
        forceX = action[0]
        forceY = action[1]
        
        powerFX = max(min(forceX * self.minPower / self.maxPower, self.maxPower), self.minPower)
        powerFY = max(min(forceX * self.minPower / self.maxPower, self.maxPower), self.minPower)
        
        self.powerCS -= powerFX + powerFY
        
        cosX = math.cos(forceX)
        sinY = math.sin(forceY)
        
        self.gyroX = forceX * cosX
        self.gyroY = forceY * sinY
        
        # x  = x + self.tau * x_dot
        # x_dot = x_dot + self.tau * xacc
        # theta = theta + self.tau * theta_dot
        # theta_dot = theta_dot + self.tau * thetaacc
        
        self.state = (self.gyroX, self.gyroY, self.powerCS)
        
        if self.enableSerial:
            done = False
        else:
            done = self.powerCS < 0
        
        
        if not done:
            reward = self.powerCS + math.sqrt(((-self.gyroX) ** 2) + ((-self.gyroY) ** 2))
        elif self.steps_beyond_done is None:
            # Pole just fell!
            self.steps_beyond_done = 0
            reward = 0
        else:
            if self.steps_beyond_done == 0:
                logger.warn("You are calling 'step()' even though this environment\
                has already returned done = True. You should always call 'reset()'\
                once you receive 'done = True' -- any further steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0
        
        return np.array(self.csState), reward, done, {}

    def reset(self):
        self.state = self.np_random.uniform(low=-0.05, high=0.05, size=(4,))
        self.steps_beyond_done = None
        return np.array(self.state)

    def render(self, mode='human'):
        screen_width = 1024
        screen_height = 800

        world_width = self.x_threshold*2
        scale = screen_width/world_width
        carty = 100 # TOP OF CART
        polewidth = 10.0
        polelen = scale * (2 * self.length)
        
        csWidth = screen_width/5
        csHeight = screen_width/7

        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(screen_width, screen_height)

            l,r,t,b = -csWidth/2, csWidth/2, csHeight/2, -csHeight/2
            chipSat = rendering.FilledPolygon([(l,b), (l,t), (r,t), (r,b)])
            self.csTrans = rendering.Transform()
            chipSat.add_attr(self.csTrans)
            self.viewer.add_geom(chipSat)
            
            axleoffset = csHeight/2.0 - csWidth/32
            
            l,r,t,b = -csWidth/32, csWidth/32, csHeight*2, -csHeight*2
            csAntenna1 = rendering.FilledPolygon([(l,b), (l,t), (r,t), (r,b)])
            csAntenna1.set_color(.3,.7,.2)
            self.csAntennaTrans1 = rendering.Transform(translation=(axleoffset, 0))
            csAntenna1.add_attr(self.csAntennaTrans1)
            csAntenna1.add_attr(self.csTrans)
            self.viewer.add_geom(csAntenna1)
            
            axleoffset = csHeight/2.0 - csWidth/32
            
            l,r,t,b = csHeight*2, -csHeight*2, -csWidth/32, csWidth/32,
            csAntenna2 = rendering.FilledPolygon([(l,b), (l,t), (r,t), (r,b)])
            csAntenna2.set_color(.8,.1,.3)
            self.csAntennaTrans2 = rendering.Transform(translation=(axleoffset, 0))
            csAntenna2.add_attr(self.csAntennaTrans2)
            csAntenna2.add_attr(self.csTrans)
            self.viewer.add_geom(csAntenna2)


        if self.state is None: return None

        if self.counter is None:
            self.counter = 0
        elif self.counter > 200 :
            self.counter = 200
        else: self.counter += 1
        
        # self.csTrans.set_translation(self.counter * 2, self.counter)
        csX = scale + screen_width / 2.0 # MIDDLE OF CART
        # self.csTrans.set_translation(csX, csX)
        
        ##### This is the serial part, i
        self.csTrans.set_translation(screen_width/2 - csWidth/2, screen_height/2 - csHeight/2)
        
        if self.enableSerial:
            line = self.ser.readline()
            self.csTrans.set_rotation(float(line) * math.pi / 180)
        else:
            self.csTrans.set_rotation(self.counter)

        return self.viewer.render(return_rgb_array = mode=='rgb_array')

    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None
