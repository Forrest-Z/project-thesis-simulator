#!/usr/bin/env python
import numpy as np

class LOSGuidance(object):
    """This class implements  """
    def __init__(self, scenario):
        self.R2 = 5.0**2 # Radii of acceptance
        self.de = 10 # TODO: determine Lookahead distance

        self.cWP = 0 # Current waypoint

        if not scenario.waypoints.any():
            self.wp_initialized = False
        else:
            self.wp = scenario.waypoints
            self.nWP = len(self.wp[:,0])

            if self.nWP < 2:
                print "Error! There must be more than 2 waypoints in the list!"
                self.wp_initialized = False
            else:
                self.wp_initialized = True
                self.Xp = np.arctan2(self.wp[self.cWP + 1][1] - self.wp[self.cWP][1],
                                     self.wp[self.cWP + 1][0] - self.wp[self.cWP][0])



    def update(self, world_object):
        if not self.wp_initialized:
            if world_object.waypoints.any():
                self.wp = world_object.waypoints
                self.nWP = len(self.wp[:,0])

                if self.nWP < 2:
                    print "Error! There must be more than 2 waypoints in the list!"
                    self.wp_initialized = False
                else:
                    self.wp_initialized = True
                    self.Xp = np.arctan2(self.wp[self.cWP + 1][1] - self.wp[self.cWP][1],
                                         self.wp[self.cWP + 1][0] - self.wp[self.cWP][0])

        x = world_object.x[0]
        y = world_object.x[1]

        if (x - self.wp[self.cWP+1][0])**2 + (y - self.wp[self.cWP+1][1])**2 < self.R2:
            if self.cWP < self.nWP - 2:
                # There are still waypoints left
                print "Waypoint %d: (%.2f, %.2f) reached!" % (self.cWP,
                                                              self.wp[self.cWP][0],
                                                              self.wp[self.cWP][1])
                # print "Next waypoint: (%.2f, %.2f)" % (self.wp[self.cWP+1][0],
                #                                        self.wp[self.cWP+1][1])
                self.cWP += 1
                self.Xp = np.arctan2(self.wp[self.cWP + 1][1] - self.wp[self.cWP][1],
                                   self.wp[self.cWP + 1][0] - self.wp[self.cWP][0])
            else:
                # Last waypoint reached

                if self.R2 < 50000:
                    print "Waypoint %d: (%.2f, %.2f) reached!" % (self.cWP,
                                                                  self.wp[self.cWP][0],
                                                                  self.wp[self.cWP][1])
                    print "Last Waypoint reached!"
                    self.R2 = np.Inf
                return


        xk = self.wp[self.cWP][0]
        yk = self.wp[self.cWP][1]

        # Eq. (10.10), [Fossen, 2011]
        e  = -(x - xk)*np.sin(self.Xp) + (y - yk)*np.cos(self.Xp)

        Xr = np.arctan2( -e, self.de)
        Xd = self.Xp + Xr

        world_object.Xd = Xd
        world_object.Ud = 3.0
