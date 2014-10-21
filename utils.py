#!/usr/bin/env python
import time

import numpy as np
from numpy import linalg as LA

import matplotlib
import matplotlib.pyplot as plt

from matplotlib.patches import Circle

from ctrl_LOS_Guidance import LOSGuidance
from ctrl_hybrid_astar import HybridAStar


class Simulation(object):
    """This class combines the different classes needed to simulate a scenario.

    Args:
        scenario   (class): a Scenario object (contains map, etc.)
        vessel     (class): the vessel to be used (e.g. the Viknes 830)
        controllers (list): list of controller types (A*, LOS, etc.)
        model      (class): the mathematical model for the vessel
                                :todo: make vessel the parent for this class

    Kwargs:
        tend       (float): The end time of the simulation (default 100)

    """
    def __init__(self, scenario):
        """Initializes the simulation."""
        self.scenario = scenario

        self.tend = self.scenario.tend
        self.N = int(self.tend/ self.scenario.dT)
        self.n = 0

        self._end_reason = "SimOK"

    def run_sim(self):
        """Runs the simulation."""

        print "Starting simulation"
        for t in np.linspace(0, self.tend, self.N):
            """The actual simulation"""
            tic = time.clock()

            self.scenario.world.update_world(self.n)
            if self.scenario.world.collision_detection():
                print "We have crashed!"
                self._end_reason = "Collision"
                break

            toc = time.clock()

            if self.n == 0:
                print self.n
                print "CPU time used: %f" % (toc - tic)
            self.n += 1

    def draw(self, axes):
        """Draw the simulation.

        Plots the result of the simulation.

        Args:
            axes (class): the matplotlib axes to draw to.
        """
        #axes.plot(self.path[:self.n,0], self.path[:self.n,1])
        self.scenario.draw(axes, self.n)



        #for ii in range(0, self.n, 50):
        #    self.vessel.draw(axes, self.path[ii], 'b', 'k')

        #if self._end_reason == "Collision":
        #    # Draw some nice explosion
        #    axes.plot(self.path[self.n,0], self.path[self.n,1], 'rx', markersize=12, mew=4)


    def print_sim_time(self):
        """Prints the time taken to simulate the scenario.

        :todo: implement
        """
        print "Simulation time: %.3f seconds!" % (self.sim_time)



class Scenario(object):
    def __init__(self, name='s1'):
        if name == 's1':
            the_map = Map('s1')

            self.tend = 100
            self.dT   = 0.1
            self.N = int(self.tend / self.dT)

            # Vessel 1 (Main vessel)
            x01 = np.array([0, 0, 0, 3.0, 0, 0])
            xg1 = np.array([100, 100, np.pi/4])

            myLOS1 = LOSGuidance()
            myAstar = HybridAStar(x01, xg1, the_map)

            v1 = Vessel(x01, xg1, self.dT, self.N, [myAstar, myLOS1], is_main_vessel=True, vesseltype='viknes')
            v1.waypoints = np.array([[0, 0], [140, 0], [120, 120], [100, 100]])


            # Vessel 2
            x02 = np.array([0, 120, 0, 3.0, 0, 0])
            xg2 = np.array([140, 0, 0])

            myLOS2 = LOSGuidance()

            v2 = Vessel(x02, xg2, self.dT, self.N, [myLOS2], is_main_vessel=False, vesseltype='viknes')
            v2.waypoints = np.array([[0, 120], [120, 120], [140, 0]])

            self.world = World([v1, v2], the_map)


    def is_occupied(self, point):
        return self.map.is_occupied(point)

    def draw(self, axes, n, scolor='b', fcolor='r', ocolor='g', ecolor='k'):
        # self.map.draw(axes, ocolor, ecolor)
        # axes.plot(self.initial_state[0], self.initial_state[1], scolor.join('o'), markersize=10)
        # axes.plot(self.goal_state[0], self.goal_state[1], fcolor.join('o'), markersize=10)

        # axes.plot(self.waypoints[:,0], self.waypoints[:,1], 'k--', linewidth=2)
        # for wp in self.waypoints[:]:
        #     circle = Circle((wp[0], wp[1]), 5, facecolor='r', alpha=0.3, edgecolor='k')
        #     axes.add_patch(circle)

        self.world.draw(axes, n)

class World(object):
    def __init__(self, vessels, the_map):
        self._vessels = vessels
        self._map = the_map

        self._is_collided = False

    def get_num_vessels(self):
        return len(self._vessels)

    def update_world(self, n):
        for v in self._vessels:
            v.update_model(n)
            v.update_controllers()

    def collision_detection(self):
        p0 = self._vessels[0].model.x[0:2]

        # Have we crashed with land?
        if self._map.is_occupied(p0):
            self._is_collided = True
            return True

        # Check for collision with other vessels
        for ii in range(1, len(self._vessels)):
            pi = self._vessels[ii].model.x[0:2]
            if (p0[0] - pi[0])**2 + (p0[1] - pi[1])**2 < 50:
                # Collision
                self._is_collided = True
                return True

        # No collision detected
        return False

    def draw(self, axes, n):
        self._map.draw(axes)
        for v in self._vessels:
            v.draw_path(axes, n)

        if self._is_collided:
            self._vessels[0].draw_collision(axes, n)

class Vessel(object):
    """General vessel class."""
    def __init__(self, x0, xg, dT, N, controllers, is_main_vessel=False, vesseltype='revolt'):

        self.model          = VesselModel(x0, dT)  # Dynamical model for vessel
        self.controllers    = controllers          # List of controllers (A*, LOS, etc.)
        self.is_main_vessel = is_main_vessel       # Is this our main vessel?

        self.waypoints      = np.array([x0, xg])
        self.N              = N

        self.path           = np.zeros((self.N, 6))

        self.Xd = 0
        self.Ud = 0
        self.x  = self.model.x # This is a numpy array -> will be a reference :D


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

    def update_model(self, n):
        self.path[n] = self.model.update(self.Ud, self.Xd)

    def update_controllers(self):
        for ctrl in self.controllers:
            ctrl.update(self)

    def draw_collision(self, axes, n):
        axes.plot(self.path[n,0], self.path[n,1], 'rx', ms=12, mew=4)

    def draw_path(self, axes, n, fcolor='y', ecolor='k'):

        lcolor = 'r'
        if self.is_main_vessel:
            fcolor = 'b'
            lcolor = 'b'

        axes.plot(self.path[:n,0], self.path[:n,1], linestyle='dashed', color=lcolor)
        axes.plot(self.waypoints[:,0], self.waypoints[:,1], 'k--')

        for ii in range(0, n, 50):
            self.draw(axes, self.path[ii], fcolor, ecolor)


    def draw(self, axes, state, fcolor='y', ecolor='k'):
        x   = state[0]
        y   = state[1]
        psi = state[2]

        Rz = np.array([[ np.cos(psi),-np.sin(psi)],
                       [ np.sin(psi), np.cos(psi)]])

        shape = np.dot(Rz, self._shape.transpose()).transpose()
        shape = np.add(shape, (x, y))

        poly = matplotlib.patches.Polygon(shape, facecolor=fcolor, edgecolor=ecolor)
        axes.add_patch(poly)


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

    def update(self, Ud, Xd):
        u0 = Ud
        psi0 = Xd

        Rz = np.array([[ np.cos(self.x[2]),-np.sin(self.x[2]), 0],
                       [ np.sin(self.x[2]), np.cos(self.x[2]), 0],
                       [ 0                , 0                , 1]])

        Fx = (d1u + d2u*np.abs(self.x[3])) * self.x[3] + m*(self.x[4]*self.x[5] + Kp_p*(u0 - self.x[3]))
        Fy = Kp_psi*Iz / lr * ((psi0 - self.x[2]) - Kd_psi*self.x[5])

        if np.abs(Fx) > Fxmax:
            Fx = np.sign(Fx)*Fxmax
        if np.abs(Fy) > Fymax:
            Fy = np.sign(Fy)*Fymax

        tau = np.array([Fx, Fy, lr*Fy])

        self.x[0:3] += self.dT * np.dot(Rz, self.x[3:6])
        self.x[3:6] += self.dT * np.dot(np.diag([1/m, 1/m, 1/Iz]), tau - self.Cvv() - self.Dvv())

        # TODO: Is this necesarry? I don't think this copies the array, so it could be set in init
        # and world_object.x would always point to VesselModel.x
        #world_object.x = self.x

        return self.x


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

    def disable_safety_region(self):
        for o in self._obstacles:
            o.set_safety_region(False)

    def get_obstacles(self):
        """Returns obstacles."""
        return self._obstacles

    def is_occupied(self, point):
        """Returns True if the given points is inside an obstacle."""
        for poly in self._obstacles:
            if poly.in_polygon(point):
                return True
        return False

    def get_obstacle_edge_samples(self, d):
        points = []
        for o in self._obstacles:
            points += o.get_edge_points(d)
        return points

    def draw(self, axes, pcolor='g', ecolor='k'):
        """Draws the map in the given matplotlib axes."""
        for poly in self._obstacles:
            poly.draw(axes, pcolor, ecolor)
        axes.axis([-10, self._dim[0], -10, self._dim[1]], 'equal')
        axes.set_xlabel('x [m]')
        axes.set_ylabel('y [m]')


class Polygon(object):
    """Generalized polygon class."""

    def __init__(self, vertices, safety_region_length=4.5):
        """Initialize polygon with list of vertices."""
        self._V = np.array(vertices)

        self._safety_region = False

        if safety_region_length > 1.0:
            self._V_safe = self.extrude(safety_region_length)
            self._safety_region = True

    def __str__(self):
        """Return printable string of polygon. For debugging."""
        return str(self._V)

    def get_vertices(self):
        return self._V

    def set_safety_region(self, val):
        self._safety_region = val

    def in_polygon(self, point):
        """Return True if point is in polygon."""

        if self._safety_region:
            vertices = self._V_safe
        else:
            vertices = self._V

        (x,y) = point

        n = len(vertices)
        inside = False

        p1x,p1y = vertices[0]
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

        #if inside:
            #print point, self._safety_region
            #fig, ax = plt.subplots()
            #self.draw(ax, 'r', 'k')
            #ax.plot(x,y, 'bx', ms=12, mew=4)
            #ax.autoscale_view()

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
        new_polygon = np.zeros([n,2])

        for ii in range(0,n):
            new_polygon[ii] = self._V[ii] - \
                              length / LA.norm(vectors[ii]) * vectors[ii] * 1.4142 / np.sqrt(1 - angles[ii])
            if self.in_polygon(new_polygon[ii]):
                new_polygon[ii] = self._V[ii] + \
                                  length / LA.norm(vectors[ii]) * vectors[ii] * 1.4142 / np.sqrt(1 - angles[ii])
        self._safety_region = True
        return new_polygon


    def get_edge_points(self, d):
        """Returns list of points along the edges of the polygon."""
        n = len(self._V)

        linesample = np.transpose(np.array([np.linspace(self._V[n-1][0], self._V[0][0], d),
                                            np.linspace(self._V[n-1][1], self._V[0][1], d)]))
        points = linesample.tolist()
        for ii in range(0,n-1):
            linesample = np.transpose(np.array([np.linspace(self._V[ii][0], self._V[ii+1][0], d),
                                                np.linspace(self._V[ii][1], self._V[ii+1][1], d)]))
            points += linesample.tolist()

        return points

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

    # myscenario = Scenario(Map(), np.empty(3), np.asarray([100, 100, np.pi/4]))
    # mymodel    = VesselModel(np.empty(6), 0.05)


    # mysim = Simulation(myscenario, [], mymodel, 20)
    # mysim.run_sim()

    # fig, ax = plt.subplots()
    # mysim.draw(ax)
    # plt.show()

    # x0 = np.array([0,0,0,3.0,0,0])
    # dT = 0.1
    # mymodel = VesselModel(x0, dT)
    # sc = Scenario(Map(), x0, np.zeros(3))
    # sc.Xd = -0*np.pi/180.0
    # tend = 2
    # for t in np.linspace(0, tend, int(tend/dT)):
    #     x = mymodel.update(sc)

    # print x[0:3]

    scen = Scenario('s1')
    sim  = Simulation(scen)

    sim.run_sim()

    fig, ax = plt.subplots()
    sim.draw(ax)

    plt.show()

