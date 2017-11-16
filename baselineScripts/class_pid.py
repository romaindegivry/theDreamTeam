#!/usr/bin/env python

from __future__ import division # get true floating point division in python 2

class PID_controller(object):

    def __init__(self,freq, k_p = 1, k_i = 0, k_d = 0):
        
        # Frequency of Updates
        self._freq = freq # Hz
        self._dt = 1/freq # s
        print(self._dt)
        
        # Weights
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        
        # Error Accumulator
        self.err_accul = 0
        
        # Previous State Value
        self.prev_pose = 0
        
        
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
        
        
        return set_V
