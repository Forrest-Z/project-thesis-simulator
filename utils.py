#!/usr/bin/env python
import time

import numpy as np
from numpy import linalg as LA


import matplotlib
import matplotlib.pyplot as plt


class Simulation(object):
    def __init__(self, scenario, algorithm, vessel):
        self.scenario = scenario
        self.algorithm = algorithm
        self.vessel = vessel
        self.path = []
        self.sim_time = 0

    def run_sim(self):
        self.sim_time = time.clock()
        self.path, succes = self.algorithm(self.scenario)
        self.sim_time = time.clock() - self.sim_time

        if succes:
            print "Found path!"
        else:
            print "Failed to find path..."

    def draw(self, axes):

        self.scenario.draw(axes)
        draw_path(axes, self.vessel, self.path, 5)

    def print_sim_time(self):
        print "Simulation time: %.3f seconds!" % (self.sim_time)


class Scenario(object):
    def __init__(self,
                 the_map,
                 initial_state,
                 goal_state):
        self.map = the_map
        self.initial_state = initial_state
        self.goal_state = goal_state

    def draw(self, axes, scolor='b', fcolor='r', ocolor='g', ecolor='k'):
        self.map.draw(axes, ocolor, ecolor)
        axes.plot(self.initial_state.x, self.initial_state.y, scolor.join('o'), markersize=12)
        axes.plot(self.goal_state.x, self.goal_state.y, fcolor.join('o'), markersize=12)

class Map(object):
    """This class provides a general map."""
    def __init__(self, maptype=None):
        """Initialize map. Default map is blank 160x160m.'"""
        self._dim = [160, 160]
        self._obstacles = []

        if maptype == 's1':
            self._dim = [160, 160]
            self._obstacles = [Polygon([(30.76, 23.75),
                                        (51.92, 20.79),
                                        (63.32, 35.44),
                                        (64.06, 47.28),
                                        (50.00, 50.00),
                                        (43.64, 35.89)]),
                               Polygon([(24.40, 55.13),
                                        (22.62, 69.04),
                                        (43.04, 76.59),
                                        (47.04, 67.71),
                                        (40.00, 60.00)]),
                               Polygon([(46.45, 94.35),
                                        (85.22, 69.04),
                                        (80.00, 90.00),
                                        (59.77, 94.64)])]

    def load_map(self, filename):
        with open(filename, 'r') as f:
            line = f.readline().split(" ")
            self._dim[0] = int(line[0])
            self._dim[1] = int(line[1])

            obstacles = f.readlines()
            self._obstacles = []

            for line in obstacles:
                o = line.split(" ")
                n = len(o)
                tmp = []
                for ii in range(0,n,2):
                    tmp.append((float(o[ii]), float(o[ii+1])))

                self._obstacles.append(Polygon(tmp))

    def add_obstacles(self, obstacles):
        """Adds obstacles to map."""
        for o in obstacles:
            self._obstacles.append(o)

    def get_dimension(self):
        return self._dim

    def get_obstacles(self):
        """Returns obstacles."""
        return self._obstacles

    def is_occupied(self, point):
        """Returns True if the given points is inside an obstacle."""
        for poly in self._obstacles:
            if poly.in_polygon(point):
                return True
        return False

    def draw(self, axes, pcolor, ecolor):
        """Draws the map in the given matplotlib axes."""
        for poly in self._obstacles:
            poly.draw(axes, pcolor, ecolor)
        axes.axis([0, self._dim[0], 0, self._dim[1]])



class State(object):
    def __init__(self, x=0, y=0, psi=0, gridsize=1):
        self.x   = x   # [m]   Position
        self.y   = y   # [m]   Position
        self.psi = psi # [rad] Orientation

        self.gridsize = gridsize

        self.grid_xy = (int(x / gridsize), int(y / gridsize))


    def __str__(self):
        return "(%.2f, %.2f, %.2f)"%(self.x, self.y, self.psi)

    def __eq__(self, other):
        return (isinstance(other, self.__class__) and self.dnorm(other) < 5*other.gridsize)

    def __ne__(self, other):
        return not self.__eq__(other)

    def dnorm(self, s2):
        return np.sqrt( (s2.x - self.x)**2 + (s2.y - self.y)**2 + (s2.psi - self.psi)**2)

    def dist(self, s2):
        return np.sqrt( (s2.x - self.x)**2 + (s2.y - self.y)**2 )

    def to_tuple(self):
        return (self.x, self.y)

    def neighbors(self):
       p = self.psi
       x0 = 2.5 * self.gridsize
       x = self.x
       y = self.y

       ret = []
       for dp in [-2, -1, 0, 1, 2]:
           d = dp * np.pi / 16
           n = State(x - x0*(np.sin(d)*np.sin(p) - np.cos(d)*np.cos(p)),
                     y - x0*(np.cos(d)*np.sin(p) + np.cos(p)*np.sin(d)),
                     p + d, self.gridsize)
           ret.append(n)

       return ret

class Vessel(object):
    """General vessel class."""
    def __init__(self, vesseltype='revolt'):
        if vesseltype == 'revolt':
            self._scale   = 1.0/20.0
            self._length  = 60.0 * self._scale
            self._breadth = 14.5 * self._scale
        elif vesseltype == 'other':
            self._scale   = 1.0
            self._length  = 7.5 * self._scale
            self._breadth = 3.0 * self._scale
        # Vertices of a polygon.
        self._shape = np.asarray([(-self._length/2, -self._breadth/2),
                                  (-self._length/2,  self._breadth/2),
                                  ( self._length/3,  self._breadth/2),
                                  ( self._length/2,  0              ),
                                  ( self._length/3, -self._breadth/2)])

    def draw(self, axes, state, fcolor, ecolor):
        Rz = np.array([[ np.cos(state.psi),-np.sin(state.psi)],
                       [ np.sin(state.psi), np.cos(state.psi)]])

        shape = np.dot(Rz.transpose(), self._shape.transpose()).transpose()
        shape = np.add(shape, (state.x, state.y))

        poly = matplotlib.patches.Polygon(shape, facecolor=fcolor, edgecolor=ecolor)
        axes.add_patch(poly)

class Polygon(object):
    """Generalized polygon class."""

    def __init__(self, vertices, safety_region=3.):
        """Initialize polygon with list of vertices."""
        self._V = np.asarray(vertices)

        if safety_region:
            self._V_safe = self.extrude(safety_region)
            self._safety_region = True

    def __str__(self):
        """Return printable string of polygon. For debugging."""
        return str(self._V)

    def get_vertices(self):
        return self._V

    def in_polygon(self, point):
        """Return True if point is in polygon."""

        if self._safety_region:
            vertices = self._V_safe
        else:
            vertices = self._V

        (x,y) = point

        n = len(vertices)
        inside = False

        p1x,p1y = self._V[0]
        for i in range(n+1):
            p2x,p2y = vertices[i % n]
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x,p1y = p2x,p2y

        return inside

    def extrude(self, length):
        self._safety_region = False
        n = len(self._V)
        vectors = np.empty([n,2])
        angles  = np.empty([n,2])

        a = self._V[n-1] - self._V[0]
        b = self._V[1]   - self._V[0]
        a = a / LA.norm(a, 2)
        b = b / LA.norm(b, 2)

        vectors[0] = a + b
        angles[0]  = np.dot(a.transpose(), b)
        for ii in range(1, n-1):
            a = self._V[ii-1] - self._V[ii]
            b = self._V[ii+1] - self._V[ii]
            a = a / LA.norm(a, 2)
            b = b / LA.norm(b, 2)

            vectors[ii] = a + b
            angles[ii]  = np.dot(a.transpose(), b)

        a = self._V[n-2] - self._V[n-1]
        b = self._V[0] - self._V[n-1]
        a = a / LA.norm(a, 2)
        b = b / LA.norm(b, 2)

        vectors[n-1] = a + b
        angles[n-1]  = np.dot(a.transpose(), b)
        new_polygon = np.empty([n,2])

        for ii in range(0,n):
            new_polygon[ii] = self._V[ii] - \
                              length / LA.norm(vectors[ii]) * vectors[ii] * 1.4142 / np.sqrt(1 - angles[ii])
            if self.in_polygon(new_polygon[ii]):
                new_polygon[ii] = self._V[ii] + \
                                  length / LA.norm(vectors[ii]) * vectors[ii] * 1.4142 / np.sqrt(1 - angles[ii])
        self._safety_region = True
        return new_polygon



    def draw(self, axes, fcolor, ecolor):
        """Plots the polygon with the given plotter."""
        poly = matplotlib.patches.Polygon(self._V, facecolor=fcolor, edgecolor=ecolor)
        axes.add_patch(poly)
        if self._safety_region:
            poly_safe = matplotlib.patches.Polygon(self._V_safe, facecolor='none', edgecolor=fcolor)
            axes.add_patch(poly_safe)


def draw_path(axes, vessel, path, spacing,
              pcolor='r', fcolor='b', ecolor='k'):
    n = len(path)

    x = []
    y = []
    psi = []

    for s in path:
        x.append(s.x)
        y.append(s.y)
        psi.append(s.psi)

    axes.plot(x,y, pcolor)

    for ii in range(0, n, spacing):
        vessel.draw(axes, path[ii], fcolor, ecolor)


if __name__ == "__main__":

    # myPoly = Polygon([(1., 1.), (2., 1), (2., 2)])

    # newPoly = myPoly.extrude(0.1)

    # fig, ax = plt.subplots()
    # myPoly.draw(ax, fcolor='b', ecolor='k')
    # newPoly.draw(ax, fcolor='none', ecolor='r')
    # ax.axis([0,3, 0,3])
    # plt.show()

    # mypath = []
    # for ii in range(0,20):
    #     mypath.append(State(ii, ii*3, ii/20))

    mymap = Map('s1')
    # myvessel = Vessel()
    fig, ax = plt.subplots()
    mymap.draw(ax, 'g', 'k')
    # draw_path(ax, myvessel, mypath, 3)
    plt.show()


