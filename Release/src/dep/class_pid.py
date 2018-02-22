#!/usr/bin/env python

from __future__ import division # get true floating point division in python 2
from math import copysign
import helpers
import numpy as np

class PID_controller(object):

    def __init__(self,freq, k_p = [0,0,0], k_i = [0,0,0], k_d = [0,0,0], absTol = 1e-6, minVel = 0, maxVel = 0.5):
        
        # Frequency of Updates
        self._freq = freq # Hz
        self._dt = 1/freq # s
        
        # Weights
        self.k_p = np.array(k_p)
        self.k_i = np.array(k_i)
        self.k_d = np.array(k_d)
        
        self.err_accul = np.array([0.,0.,0.])
        self.err_pose = np.array([0.,0.,0.])
        self.derivative = np.array([0.,0.,0.])
        
        # Previous State Value
        self.prev_pose = np.array([0.,0.,0.])
        
        # Absolute Tolerance
        self.absTol = absTol
        
        # Min/Max Output Target Velocity
        self.minVel = abs(minVel)
        self.maxVel = abs(maxVel)
        
        self.stepTime = None;
        
    def __call__(self):
        # Velocity
        set_V = np.zeros((3,))
        set_V = self.k_p*self.err_pose+self.k_i*self.err_accul-self.k_d*self.derivative
        
        
        #change x command depending on z error
        if abs(self.err_pose[2]) > 0.5:
            set_V[0]=self.minVel

        
        if np.any(np.abs(set_V) < self.minVel):
            for i,item in enumerate(set_V):
                if abs(item)< self.minVel:
                    set_V[i] = self.minVel*item/float(abs(item))
                return set_V
        elif np.any(np.abs(set_V) > self.maxVel):
            for i,item in enumerate(set_V):
                if abs(item)> self.maxVel:
                    set_V[i] = self.maxVel*item/float(abs(item))
                return set_V
        elif np.any(abs(self.err_pose) < self.absTol):
            return set_V  - (abs(self.err_pose) < self.absTol)*set_V
            
        else:
            return set_V
        
    def update(self,droneState,clockState,target):
        pos  = droneState['pos']
        pos[2] = droneState['height']
        #if self.stepTime == None:
        #    self._dt = (self.steptime - clockState['pose'])*1e-9#time units are [ns]
        #self.stepTime = clockState['pose']
        
        # Error
        self.err_pose = pos - target

        # Update Integral Error
        self.err_accul += self.err_pose * self._dt
        
        # Derivative
        self.derivative = (pos - self.prev_pose)/self._dt
        
        # Update Previous Position
        self.prev_pose = self.err_pose + target
        
    def reset(self,clockState):
        #reset controller state
        self.stepTime = clockState['pose']
        self.err_accul = np.array([0.,0.,0.])
        self.err_pose = np.array([0.,0.,0.])
        self.derivative = np.array([0.,0.,0.])
        
    def set_minVel(self, minVel):
    
        self.minVel = abs(minVel)
        
        
    def set_maxVel(self, maxVel):
    
        self.maxVel = abs(maxVel)
