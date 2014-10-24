#!/usr/bin/env python

"""
Dynamic Window controller

This module implements the Dynamic Window controller as proposed by Fox et. al., 1997.

"""

import numpy as np


class DynamicWindow(object):
    def __init__(self, vessel_object):


        self.alpha = 1
        self.beta  = 1

        self.window_res = [9 51]  # xy Dynamic Window resolution

        self.dT         = vessel_object.model.dT
        self.est_du_max = 1 # Estimated maximum rate of change in surge velocity


    def update(self, vessel_object):

        u = vessel_object.x[3]  # body frame forward velocity

        u_rad_max = min(u + self.est_du_max * self.dT,
                        min(self.u_max, vessel_object.))
