#!/usr/bin/env python

import numpy as np

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from ctrl_DWA import DynamicWindow

from matplotlib.patches import Circle

class Vessel(object):
    """General vessel class."""
    def __init__(self, x0, xg, h, dT, N, controllers, is_main_vessel=False, vesseltype='revolt'):

        self.model          = VesselModel(x0, h)   # Dynamical model for vessel
        self.controllers    = controllers          # List of controllers (A*, LOS, etc.)
        self.is_main_vessel = is_main_vessel       # Is this our main vessel?

        self.waypoints      = np.array([[xg]])
        self.current_goal   = self.waypoints[0]

        self.h              = h
        self.dT             = dT

        self.N              = N

        self.path           = np.zeros((self.N, 6))

        self.psi_d = 0
        self.u_d   = 0
        self.r_d   = 0

        self.x  = self.model.x # This is a numpy array -> will be a reference :D

        self.time = 0.0

        if vesseltype == 'revolt':
            self._scale   = 1.0/20.0
            self._length  = 60.0 * self._scale
            self._breadth = 14.5 * self._scale
        elif vesseltype == 'viknes':
            self._scale   = 1.0
            self._length  = 8.52 * self._scale
            self._breadth = 2.97 * self._scale
        elif vesseltype == 'danskebaaten':
            self._scale   = 1.0 / 4.0
            self._length  = 223.70 * self._scale
            self._breadth = 36.00  * self._scale
        # Vertices of a polygon.
        self._shape = np.asarray([(-self._length/2, -self._breadth/2),
                                  (-self._length/2,  self._breadth/2),
                                  ( self._length/3,  self._breadth/2),
                                  ( self._length/2,  0              ),
                                  ( self._length/3, -self._breadth/2)])

    def get_shape(self):
        return np.copy(self._shape)

    def update_model(self, n):
        self.path[n] = self.model.update(self.u_d, self.psi_d, self.r_d)

    def update_controllers(self):
        """Updates all the vessels controllers."""
        for ctrl in self.controllers:
            ctrl.update(self)

    def draw_collision(self, axes, n):
        """Draws a red X at the point of collision."""
        axes.plot(self.path[n,0], self.path[n,1], 'rx', ms=12, mew=4)

    def draw(self, axes, n, fcolor='y', ecolor='k'):
        """Draws the path of a Vessel to the given axes"""
        lcolor = 'r'
        wpcolor= 'y'
        if self.is_main_vessel:
            fcolor = 'b'
            lcolor = 'b'
            wpcolor= 'b'

        self.draw_path(axes, n, lcolor, fcolor)
        self.draw_waypoints(axes, n, wpcolor)

    def draw_path(self, axes, n, lcolor='r', fcolor='y', ecolor='k'):
        """Draws vessel path with patches."""
        axes.plot(self.path[:n,0], self.path[:n,1], linestyle='dashed', color=lcolor)

        N = int(n / (self.dT / self.h))

        for ctrl in self.controllers:
            if isinstance(ctrl, DynamicWindow):
                ctrl.draw(axes, N)

        for ii in range(0, n, int(self.dT/self.h) * 8):
            self.draw_patch(axes, self.path[ii], fcolor, ecolor)

    def draw_waypoints(self, axes, n, wpcolor):
        """Draws waypoints"""

        axes.plot(self.waypoints[:,0], self.waypoints[:,1], 'k--')

        ii = 1
        for wp in self.waypoints[:]:
            circle = Circle((wp[0], wp[1]), 10, facecolor=wpcolor, alpha=0.3, edgecolor='k')
            axes.add_patch(circle)
            axes.annotate(str(ii), xy=(wp[0], wp[1]), xytext=(wp[0]+5, wp[1]-5))
            ii += 1

    def draw_patch(self, axes, state, fcolor='y', ecolor='k'):
        """Draws the Vessel as a  matplotlib.patches.Polygon to the gives axes."""
        x   = state[0]
        y   = state[1]
        psi = state[2]

        Rz = np.array([[ np.cos(psi),-np.sin(psi)],
                       [ np.sin(psi), np.cos(psi)]])

        shape = np.dot(Rz, self._shape.transpose()).transpose()
        shape = np.add(shape, (x, y))

        poly = matplotlib.patches.Polygon(shape, facecolor=fcolor, edgecolor=ecolor)
        axes.add_patch(poly)

    def animate(self, fig, ax, n, has_collided=False):
        """Animate the path."""
        draw_explosion = has_collided and self.is_main_vessel

        if self.is_main_vessel:
            theface = 'b'
        else:
            theface = 'y'

        self.draw_waypoints(ax, n, theface)

        patch = plt.Polygon(self._shape,
                            fc=theface,
                            ec='k',
                            animated=False)

        explosion = plt.Circle(self.path[n-1][0:2], 5, fc='r', ec='none')

        def init():
            ax.add_patch(patch)
            if draw_explosion:
                ax.add_patch(explosion)
                explosion.set_visible(False)
                return patch, explosion

            return patch,


        data = self.path[0:n,0:3]
        bshape = patch.get_xy().transpose()

        def update_patch(num):
            if num == 0:
                explosion.set_visible(False)

            if num < n:
                xy = data[num,0:2]
                psi = data[num,2]

                Rz = np.array([[ np.cos(psi),-np.sin(psi)],
                               [ np.sin(psi), np.cos(psi)]])

                shape = np.dot(Rz, bshape).transpose()
                shape += xy

                patch.set_xy(shape)

            if draw_explosion and num >= n:
                explosion.set_visible(True)
                return patch, explosion

            return patch,

        ani = animation.FuncAnimation(fig, update_patch, range(0,n+50,10),
                                      init_func=init,
                                      interval=50,
                                      blit=False)
        return ani


class VesselModel(object):
    """3DOF nonlinear vessel model"""

    def __init__(self, x0, h, vessel_model='viknes'):
        self.x = np.copy(x0)
        self.h = h # Integrator time step

        if vessel_model == 'viknes':
            # Set model parameters
            self.d1u = 16.6
            self.d1v = 9900.0
            self.d1r = 330.0
            self.d2u = 8.25
            self.d2v = 330.0
            self.d2r = 0.0

            self.m   = 3300.0
            self.Iz  = 1320.0

            self.lr  = 4.0
            self.Fxmax = 2310.0
            self.Fymax = 28.8

            self.Kp_p = 0.1
            self.Kp_psi = 5.0
            self.Kd_psi = 1.0
            self.Kp_r   = 5.0

            self.rudder_max = 2*28.8
            self.rudder_K = 4.0
            self.propeller_max = 2310.0
        # Values other algorithms can use to get information about the model

        # Max yawrate:
        # inertia*r_dot = -(d1r + d2r*|r|)*r + fr_max*lx = 0
        if self.d2r > 0:
            self.est_r_max = 0.5*(-self.d1r + \
                                  np.sqrt(self.d1r**2 + 4*self.d1r*self.rudder_max*self.rudder_K)) / d2r
        else:
            self.est_r_max = self.rudder_max*self.rudder_K / self.d1r

        # Max yaw acceleration (at r = 0):
        self.est_dr_max = self.rudder_max*self.rudder_K / self.Iz

        # Max surge velocity
        # mass*u_dot = -(d1u + d2u*|u|)*u + force_max = 0
        if self.d2u > 0:
            self.est_u_max = 0.5*(-self.d1u + \
                                  np.sqrt(self.d1u**2 + 4*self.d1u*self.propeller_max)) / self.d2u
        else:
            self.est_u_max = self.propeller_max / self.d1u;

        # Min surge velocity (max reverse)
        self.est_u_min = -self.est_u_max;

        # Max surge acceleration
        self.est_du_max = self.propeller_max / self.m
        # Min surge acceleration (max reverse)
        self.est_du_min = -self.est_du_max


    def Cvv(self):
        return np.array([ self.x[4] * self.x[5],
                         -self.x[3] * self.x[5],
                          0                    ])
    def Dvv(self):
        return np.array([self.d2u*self.x[3]*np.abs(self.x[3]) + self.d1u*self.x[3],
                         self.d2v*self.x[4]*np.abs(self.x[4]) + self.d1v*self.x[4],
                         self.d1r*self.x[5]])

    def Tau(self, u_d, psi_d, r_d):
        Fx = (self.d1u + self.d2u*np.abs(self.x[3])) * self.x[3] + \
             (self.x[4]*self.x[5] + self.Kp_p*(u_d - self.x[3])) * self.m

        if psi_d == np.Inf:
            Fy = 1 / self.lr * ( (self.d1r + self.d2r*np.abs(self.x[5]))*self.x[5] + \
                                 self.Iz * self.Kp_r*(r_d - self.x[5]))
        else:
            Fy = self.Kp_psi * self.Iz / self.lr * ((psi_d - self.x[2]) - self.Kd_psi*self.x[5])

        if np.abs(Fx) > self.Fxmax:
            Fx = np.sign(Fx)*self.Fxmax # :todo: Not realistic to go full speed reverse?

        if np.abs(Fy) > self.Fymax:
            Fy = np.sign(Fy)*self.Fymax

        return np.array([Fx, Fy, self.lr*Fy])

    def update(self, u_d, psi_d, r_d):
        Rz = np.array([[ np.cos(self.x[2]),-np.sin(self.x[2]), 0],
                       [ np.sin(self.x[2]), np.cos(self.x[2]), 0],
                       [ 0                , 0                , 1]])

        self.x[0:3] += self.h * np.dot(Rz, self.x[3:6])
        self.x[3:6] += self.h * np.dot(np.diag([1/self.m, 1/self.m, 1/self.Iz]),
                                       self.Tau(u_d, psi_d, r_d) - self.Cvv() - self.Dvv())

        return self.x