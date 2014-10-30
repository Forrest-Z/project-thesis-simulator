#!/usr/bin/env python

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle

class World(object):
    def __init__(self, vessels, the_map):
        self._vessels = vessels
        self._map = the_map

        self._is_collided = False


    def get_num_vessels(self):
        return len(self._vessels)

    def update_world(self, t, n, dT=0.5):
        for v in self._vessels:
            v.time = t
            v.update_model(n)
            if np.fmod(t, dT) == 0:
                v.update_controllers()

    def is_occupied(self, x, y, t):
        """Is the point (x,y) occupied at time t?"""
        #if self._map.is_occupied((x,y)):
        #    return True

        # Check for collision with other vessels
        for ii in range(1, len(self._vessels)):
            xi  = self._vessels[ii].x[0:2]
            psi = self._vessels[ii].x[3]
            u   = self._vessels[ii].x[4]

            xnext = xi[0] #+ np.cos(psi)*(t - self._vessels[ii].time)*u
            ynext = xi[1] #+ np.sin(psi)*(t - self._vessels[ii].time)*u

            if (x - xnext)**2 + (y - ynext)**2 < 90:
                # Collision
                return True
        return False

    def collision_detection(self):
        p0 = self._vessels[0].model.x[0:2]

        # Have we crashed with land?
        if self._map.is_occupied(p0, safety_region=False):
            self._is_collided = True
            return True

        # Check for collision with other vessels
        for ii in range(1, len(self._vessels)):
            vi = self._vessels[ii].model.x[0:2]
            if (p0[0] - vi[0])**2 + (p0[1] - vi[1])**2 < 50:
                # Collision
                self._is_collided = True
                return True

        # No collision detected
        return False

    def draw(self, axes, n):
        self._map.draw(axes)
        for v in self._vessels:
            v.draw(axes, n)

        if self._is_collided:
            self._vessels[0].draw_collision(axes, n)

    def animate(self, fig, ax, n):

        self._map.draw(ax)

        patchlist = []
        shapelist = []

        def init():
            for v in self._vessels:

                if v.is_main_vessel:
                    p = plt.Polygon(v.get_shape(), fc='b', ec='k')
                    v.draw_waypoints(ax, n, 'b')
                else:
                    p = plt.Polygon(v.get_shape(), fc='y', ec='k')
                    v.draw_waypoints(ax, n, 'y')

                patchlist.append(p)
                shapelist.append(p.get_xy().transpose())
                ax.add_patch(p)

            return patchlist

        def update_patches(num):
            for ii in range(0, len(patchlist)):
                p  = patchlist[ii]
                x  = self._vessels[ii].path[num]

                Rz = np.array([[np.cos(x[2]), -np.sin(x[2])],
                               [np.sin(x[2]),  np.cos(x[2])]])
                newp = np.dot(Rz, shapelist[ii]).transpose()
                newp += x[0:2]

                p.set_xy(newp)
            return patchlist

        ani = animation.FuncAnimation(fig, update_patches, range(0, n, 10),
                                      init_func=init,
                                      interval=50,
                                      blit=False)
        return ani
