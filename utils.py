#!/usr/bin/env python
import time

import numpy as np
from numpy import linalg as LA

import matplotlib
import matplotlib.pyplot as plt

from matplotlib.patches import Circle


class Simulation(object):
    def __init__(self, scenario, vessel, controllers, model, tend=None):
        self.scenario = scenario
        self.controllers = controllers
        self.model = model
        self.vessel = vessel

        if tend:
            self.tend = tend
        else:
            self.tend = 100

        self.path = np.empty((int(self.tend / self.model.dT), 6))


    def run_sim(self):
        n = 0

        for t in np.linspace(0, self.tend, int(self.tend*10)):

            self.path[n] = self.model.update(self.scenario)
            for ctrl in self.controllers:
                ctrl.update(self.scenario)

            n += 1
        # # TODO: This is a general sketch
        # for it in range(0, num_iter):
        #     # Typical layout:
        #     # Global (A*) -> Waypoint tracker (LOS) -> DWA -> ..?
        #     for ctrl in controllers:
        #         ctrl.update()


    def draw(self, axes):

        axes.plot(self.path[:,0], self.path[:,1])
        self.scenario.draw(axes)

        axes.set_aspect('equal')

        n = len(self.path[:])
        for ii in range(0, n, 50):
            self.vessel.draw(axes, self.path[ii], 'b', 'k')

        #axes.autoscale_view()
        #draw_path(axes, self.vessel, self.path, 5)

    def print_sim_time(self):
        print "Simulation time: %.3f seconds!" % (self.sim_time)


## TODO: CLEAN THIS UP
d1u = 16.6
d1v = 9900.0
d1r = 330.0
d2u = 8.25
d2v = 330.0
d2r = 0.0

m   = 3300.0
Iz  = 1320.0

lr  = 4.0
Fxmax = 2310.0
Fymax = 28.8

Kp_p = 0.1
Kp_psi = 5.0
Kd_psi = 1.0

class VesselModel(object):
    """3DOF nonlinear vessel model"""

    def __init__(self, x0, dT):
        self.x = np.copy(x0)

        self.dT = dT

    def Cvv(self):
        return np.array([ self.x[4] * self.x[5],
                         -self.x[3] * self.x[5],
                          0                    ])
    def Dvv(self):
        return np.array([d2u*self.x[3]*np.abs(self.x[3]) + d1u*self.x[3],
                         d2v*self.x[4]*np.abs(self.x[4]) + d1v*self.x[4],
                                                         + d1r*self.x[5]])

    def update(self, world_object):
        u0 = world_object.Ud
        psi0 = world_object.Xd

        Rz = np.array([[ np.cos(self.x[2]),-np.sin(self.x[2]), 0],
                       [ np.sin(self.x[2]), np.cos(self.x[2]), 0],
                       [ 0                , 0                , 1]])

        Fx = (d1u + d2u*np.abs(self.x[3])) * self.x[3] + m*(self.x[4]*self.x[5] + Kp_p*(u0 - self.x[3]))
        Fy = Kp_psi*Iz / lr * ((psi0 - self.x[2]) - Kd_psi*self.x[5])

        if Fx > Fxmax:
            Fx = Fxmax
        if Fy > Fymax:
            Fy = Fymax

        tau = np.array([Fx, Fy, lr*Fy])

        self.x[0:3] += self.dT * np.dot(Rz, self.x[3:6])
        self.x[3:6] += self.dT * np.dot(np.diag([1/m, 1/m, 1/Iz]), tau - self.Cvv() - self.Dvv())

        # TODO: Is this necesarry? I don't think this copies the array, so it could be set in init
        # and world_object.x would always point to VesselModel.x
        world_object.x = self.x

        return self.x


class Scenario(object):
    def __init__(self, the_map, initial_state, goal_state, waypoints):
        if waypoints.any():
            self.waypoints = waypoints

        self.map = the_map
        self.initial_state = initial_state
        self.goal_state = goal_state
        self.Ud = 0.0
        self.Xd = 0.0
        self.x = None

    def draw(self, axes, scolor='b', fcolor='r', ocolor='g', ecolor='k'):
        self.map.draw(axes, ocolor, ecolor)
        axes.plot(self.initial_state[0], self.initial_state[1], scolor.join('o'), markersize=12)
        axes.plot(self.goal_state[0], self.goal_state[1], fcolor.join('o'), markersize=12)

        axes.plot(self.waypoints[:,0], self.waypoints[:,1], 'k--', linewidth=2)
        for wp in self.waypoints[:]:
            circle = Circle((wp[0], wp[1]), 10, facecolor='r', alpha=0.4)
            axes.add_patch(circle)

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
    def __init__(self, x=0, y=0, psi=0, xyres=1, psires=5.0/360.0):
        self.x   = x   # [m]   Position
        self.y   = y   # [m]   Position
        self.psi = psi # [rad] Orientation

        self.gridsize = gridsize

        self.grid_int = (int(x / gridsize), int(y / gridsize), int(psi / psires))


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
        elif vesseltype == 'viknes':
            self._scale   = 1.0
            self._length  = 8.52 * self._scale
            self._breadth = 2.97 * self._scale
        # Vertices of a polygon.
        self._shape = np.asarray([(-self._length/2, -self._breadth/2),
                                  (-self._length/2,  self._breadth/2),
                                  ( self._length/3,  self._breadth/2),
                                  ( self._length/2,  0              ),
                                  ( self._length/3, -self._breadth/2)])

    def draw(self, axes, state, fcolor, ecolor):
        x   = state[0]
        y   = state[1]
        psi = state[2]

        Rz = np.array([[ np.cos(psi),-np.sin(psi)],
                       [ np.sin(psi), np.cos(psi)]])

        shape = np.dot(Rz, self._shape.transpose()).transpose()
        shape = np.add(shape, (x, y))

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


# TODO: Remove. Not in use.
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

    myscenario = Scenario(Map(), np.empty(3), np.asarray([100, 100, np.pi/4]))
    mymodel    = VesselModel(np.empty(6), 0.05)


    mysim = Simulation(myscenario, [], mymodel, 20)
    mysim.run_sim()

    fig, ax = plt.subplots()
    mysim.draw(ax)
    plt.show()
    print "OK!"

