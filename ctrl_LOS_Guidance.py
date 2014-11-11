#!/usr/bin/env python
import numpy as np

from matplotlib.patches import Circle

class LOSGuidance(object):
    """This class implements  """
    def __init__(self):
        self.R2 = 10.0**2 # Radii of acceptance (squared)
        self.de = 10 # TODO: determine Lookahead distance

        self.cWP = 0 # Current waypoint

        self.wp_initialized = False

        self.Xd = 0.0
        self.Xp = 0.0

    def update(self, vessel_object):
        if not self.wp_initialized:
            if vessel_object.waypoints.any():
                self.wp = vessel_object.waypoints
                self.nWP = len(self.wp[:,0])

                if self.nWP < 2:
                    print "Error! There must be more than 1 waypoint in the list!"
                    self.wp_initialized = False
                else:
                    self.wp_initialized = True
                    self.Xp = np.arctan2(self.wp[self.cWP + 1][1] - self.wp[self.cWP][1],
                                         self.wp[self.cWP + 1][0] - self.wp[self.cWP][0])
                    vessel_object.current_goal = self.wp[self.cWP+1]
                    print vessel_object.current_goal, self.Xp


        x = vessel_object.x[0]
        y = vessel_object.x[1]

        if (x - self.wp[self.cWP+1][0])**2 + (y - self.wp[self.cWP+1][1])**2 < self.R2:
            if self.cWP < self.nWP - 2:
                # There are still waypoints left
                print "Waypoint %d: (%.2f, %.2f) reached!" % (self.cWP,
                                                              self.wp[self.cWP][0],
                                                              self.wp[self.cWP][1])
                # print "Next waypoint: (%.2f, %.2f)" % (self.wp[self.cWP+1][0],
                #                                        self.wp[self.cWP+1][1])
                self.cWP += 1
                vessel_object.current_goal = self.wp[self.cWP+1]
                self.Xp = np.arctan2(self.wp[self.cWP + 1][1] - self.wp[self.cWP][1],
                                     self.wp[self.cWP + 1][0] - self.wp[self.cWP][0])
            else:
                # Last waypoint reached

                if self.R2 < 50000:
                    print "Waypoint %d: (%.2f, %.2f) reached!" % (self.cWP,
                                                                  self.wp[self.cWP][0],
                                                                  self.wp[self.cWP][1])
                    print "Last Waypoint reached!"
                    vessel_object.Ud = 0.0
                    self.R2 = np.Inf
                return


        xk = self.wp[self.cWP][0]
        yk = self.wp[self.cWP][1]

        # Eq. (10.10), [Fossen, 2011]
        e  = -(x - xk)*np.sin(self.Xp) + (y - yk)*np.cos(self.Xp)

        Xr = np.arctan2( -e, self.de)
        psi_d = self.Xp + Xr

        vessel_object.psi_d = psi_d
        vessel_object.u_d = 3.0

    def draw(self, axes, N, wpcolor='y', ecolor='k'):
        axes.plot(self.wp[:,0], self.wp[:,1], 'k--')
        ii = 1
        for wp in self.wp[1:]:
            circle = Circle((wp[0], wp[1]), 10, facecolor=wpcolor, alpha=0.3, edgecolor='k')
            axes.add_patch(circle)
            axes.annotate(str(ii), xy=(wp[0], wp[1]), xytext=(wp[0]+5, wp[1]-5))
            ii += 1
