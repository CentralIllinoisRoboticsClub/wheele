#!/usr/bin/python

import numpy as np
from math import atan2, sin, cos, pi

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp= 0.5 #0.3
        self.ka= 2.0 #4
        self.kb= 0.001 #0.01
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
        self.target_rho = 1.0
        
    def update_target_rho(self, new_rho):
        self.target_rho = new_rho
        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        # YOUR CODE HERE
        #print "goal: ", goal
        #print "state: ", state
        dx = goal[0] - state[0]
        dy = goal[1] - state[1]
        theta = state[2]
        rho = np.sqrt(dx**2 + dy**2)
        pos_beta = atan2(dy,dx) #NOTE, I CHANGED THE DEFINITION BETA TO BE +ATAN2, SO NOW kb > 0
        alpha = pos_beta - theta
        if(alpha >= pi):
            alpha -= 2*pi
        elif(alpha < -pi):
            alpha += 2*pi
        
        v = self.kp * rho
        if(v < -self.MAX_SPEED):
          v = -self.MAX_SPEED
        elif(v > self.MAX_SPEED):
          v = self.MAX_SPEED
          
        w = self.ka*alpha + self.kb*pos_beta
        if(w < -self.MAX_OMEGA):
          w = -self.MAX_OMEGA
        elif(w > self.MAX_OMEGA):
          w = self.MAX_OMEGA
          
        #~ if(v < 0.15):
          #~ v = 0.15
          #~ if(abs(w) < 0.5):
            #~ v = 0.15
          #~ else:
            #~ v = 0.0
            #~ if(w < 0):
              #~ w = -1.0
            #~ else:
              #~ w = 1.0
        
        done = False
        if(rho < self.target_rho):
            v = 0.0
            w = 0.0
            done = True
        
        return v,w,done, alpha, pos_beta
