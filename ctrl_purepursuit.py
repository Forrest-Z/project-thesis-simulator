#!/usr/bin/env python
import numpy as np

from utils import Controller

class PurePursuit(Controller):
    def __init__(self, R2=50, mode='waypoint'):
        self.cGoal = None # Current Goal
        self.cWP   = 0    # Used if waypoint-navigation
        self.nWP   = 0
        self.is_initialized = False

        self.R2   = R2
        self.mode = mode

        self.wps  = None

    def update(self, vobj):
        if not self.is_initialized:
            # Reference to the vessel object's waypoints
            self.wps = vobj.waypoints
            if self.mode == 'waypoint':
                self.cGoal = self.wps[self.cWP]
                self.nWP   = len(self.wps)
                vobj.u_d   = 3.0

            elif self.mode == 'goal-switcher':
                self.cGoal = self.wps[self.cWP]
                vobj.current_goal = self.cGoal
                self.nWP = len(self.wps)

            self.is_initialized = True


        x = vobj.x[0]
        y = vobj.x[1]

        if not self.mode == 'pursuit':
            if (x - self.cGoal[0])**2 + (y - self.cGoal[1])**2 < self.R2:
                if self.cWP < self.nWP - 1:
                    self.cWP += 1
                    self.cGoal = self.wps[self.cWP]
                    vobj.current_goal = np.copy(self.cGoal)
                else:
                    vobj.u_d = 0.0

        if self.mode == 'waypoint' or self.mode == 'pursuit':
            vobj.psi_d = np.arctan2(self.cGoal[1] - y,
                                    self.cGoal[0] - x)

    def draw(self, axes, N, fcolor, ecolor):
        axes.plot(self.wps[:,0], self.wps[:,1], 'k--')

