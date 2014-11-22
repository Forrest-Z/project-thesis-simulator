#!/usr/bin/env python
import numpy as np

from utils import Controller

class ConstantBearing(Controller):
    def __init__(self, target):
        self.target = target

        self.u_a_max = 2.0
        self.delta_p2 = 4.0

    def update(self, vobj):
        # According to (Fossen, 2011) page 244.
        p_err = vobj.x[0:2] - self.target[0:2]
        v_approach = - self.u_a_max * p_err / np.sqrt( np.dot(p_err.T, p_err) + self.delta_p2)

        v_d = self.target[3:5] + v_approach

        vobj.u_d = v_d[0]
        vobj.psi_d = np.arctan2(v_d[1], v_d[0])

    def visualize(self, fig, axarr, t, n):
        pass
