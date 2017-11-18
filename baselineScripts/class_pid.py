#!/usr/bin/env python

from __future__ import division # get true floating point division in python 2
from math import copysign

class PID_controller(object):

    def __init__(self,freq, k_p = 1, k_i = 0, k_d = 0, absTol = 0.1, minVel = 0, maxVel = 0.5):
        
        # Frequency of Updates
        self._freq = freq # Hz
        self._dt = 1/freq # s
        
        # Weights
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        
        # Error Accumulator
        self.err_accul = 0
        
        # Previous State Value
        self.prev_pose = 0
        
        # Absolute Tolerance
        self.absTol = absTol
        
        # Min/Max Output Target Velocity
        self.minVel = abs(minVel)
        self.maxVel = abs(maxVel)
                
        
    def __call__(self, pose, target_pose):
    
        # Error
        err_pose = target_pose - pose      
        
        # Update Integral Error
        self.err_accul += err_pose * self._dt
        
        # Derivative
        derivative = (pose - self.prev_pose)/self._dt
        
        # Update Previous Position
        self.prev_pose = pose
        
        # Velocity
        set_V = self.k_p * err_pose + self.k_i * self.err_accul - self.k_d * derivative
        
        if abs(set_V) < self.minVel:
            set_V = copysign(self.minVel, set_V)
            
        elif abs(set_V) > self.maxVel:
            set_V = copysign(self.maxVel, set_V)
            
        elif abs(err_pose) < self.absTol:
            set_V = 0
        
        
        return set_V
        
        
    def set_minVel(self, minVel):
    
        self.minVel = abs(minVel)
        
        
    def set_maxVel(self, maxVel):
    
        self.maxVel = abs(maxVel)
