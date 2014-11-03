#!/usr/bin/env python

import numpy as np

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.cm as cm

import time

class Map(object):
    """This class provides a general map."""
    def __init__(self, maptype=None, gridsize=1):
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
        elif maptype == 'triangle':
            self._dim = [20, 20]

            self._obstacles = [Polygon([(1.5, 1.5),
                                        (18.5, 3.5),
                                        (10.7, 17.3)])]

        self._is_discretized = False
        self._gridsize = gridsize
        self._grid = None

    def discretize_map(self, gridsize=1):
        """
        Creates a discrete version of the map.

        This algorithm is based on the in_polygon-algorithm.
        See: http://alienryderflex.com/polygon_fill/
        """
        tic = time.clock()

        self._gridsize = gridsize

        self._discrete_dim = [int(self._dim[0]/self._gridsize),
                              int(self._dim[1]/self._gridsize)]

        self._grid = np.zeros(self._discrete_dim)

        for o in self._obstacles:
            V = o.get_vertices(safe=True)

            xymax = np.amax(V, axis=0)
            xmax  = int(np.ceil(xymax[0]))
            ymax  = int(np.ceil(xymax[1]))

            xymin = np.amin(V, axis=0)
            xmin  = int(np.floor(xymin[0]))
            ymin  = int(np.floor(xymin[1]))

            for gridY in range(ymin, ymax):
                # Build a list of nodes
                xnodes = []
                j     = len(V) - 1 # Index of last vertice
                for i in range(0, len(V)):
                    if (V[i][1] < gridY and V[j][1] >= gridY) or \
                       (V[j][1] < gridY and V[i][1] >= gridY):
                        x = int( V[i][0] + \
                                 (gridY - V[i][1])/(V[j][1] - V[i][1])*(V[j][0] - V[i][0]) )
                        xnodes.append(x)
                    j = i

                # Sort the nodes
                xnodes.sort()

                # Fill the pixels/cells between node pairs
                for i in range(0, len(xnodes), 2):
                    if xnodes[i] >= xmax:
                        # :todo: will this happen?
                        break
                    if xnodes[i+1] > xmin:
                        if xnodes[i] < xmin:
                            # :todo: will this happen?
                            xnodes[i] = xmin
                        if xnodes[i] > xmax:
                            # :todo: will this happen?
                            xnodes[i] = xmax
                        for j in range(xnodes[i], xnodes[i+1]):
                            self._grid[j, gridY] = 1

        self._is_discretized = True

        print "Discretization time: ", time.clock() - tic

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

    def is_occupied_discrete(self, point):
        if not self._is_discretized:
            self.discretize_map()
        p = int(point[0]/self._gridsize), int(point[1]/self._gridsize)

        if p[0] >= self._dim[0] or \
           p[1] >= self._dim[1]:
            return True
        else:
            return self._grid[p] > 0.0

    def is_occupied(self, point, safety_region=True):
        """Returns True if the given points is inside an obstacle."""
        for poly in self._obstacles:
            if poly.in_polygon(point, safety_region):
                return True
        return False

    def get_obstacle_edge_samples(self, d):
        points = []
        for o in self._obstacles:
            points += o.get_edge_points(d)
        return points

    def draw_discrete(self, axes, fill='Greens'):
        if not self._is_discretized:
            self.discretize_map()
        xvals = np.arange(0, self._dim[0], self._gridsize)
        yvals = np.arange(0, self._dim[1], self._gridsize)

        axes.pcolor(xvals, yvals, self._grid.T, cmap=plt.get_cmap(fill), alpha=0.2)

    def draw(self, axes, pcolor='g', ecolor='k', draw_discrete=False):
        """Draws the map in the given matplotlib axes."""

        if self._is_discretized and draw_discrete:
            for poly in self._obstacles:
                poly.draw(axes, pcolor, ecolor, alpha=0.5)

            self.draw_discrete(axes)

        else:
            for poly in self._obstacles:
                poly.draw(axes, pcolor, ecolor, alpha=1.0)

        axes.set_xlabel('x [m]')
        axes.set_ylabel('y [m]')

        axes.set_xlim([0, self._dim[0]])
        axes.set_ylim([0, self._dim[1]])
        axes.axis('equal')

class Polygon(object):
    """Generalized polygon class."""

    def __init__(self, vertices, safety_region_length=1.0):
        """Initialize polygon with list of vertices."""
        self._V = np.array(vertices)


        if safety_region_length >= 1.0:
            self._V_safe = self.extrude(safety_region_length)
            self._safety_region = True
        else:
            self._safety_region = False
            self._V_safe = self._V

    def __str__(self):
        """Return printable string of polygon. For debugging."""
        return str(self._V)

    def get_vertices(self, safe=False):
        if safe:
            return self._V_safe
        else:
            return self._V

    def set_safety_region(self, val):
        self._safety_region = val

    def in_polygon(self, point, safety_region=True):
        """Return True if point is in polygon."""

        if self._safety_region and safety_region:
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

        return inside

    def extrude(self, length):
        self._safety_region = False
        n = len(self._V)
        vectors = np.empty([n,2])
        angles  = np.empty([n,2])

        a = self._V[n-1] - self._V[0]
        b = self._V[1]   - self._V[0]
        a = a / np.linalg.norm(a, 2)
        b = b / np.linalg.norm(b, 2)

        vectors[0] = a + b
        angles[0]  = np.dot(a.transpose(), b)
        for ii in range(1, n-1):
            a = self._V[ii-1] - self._V[ii]
            b = self._V[ii+1] - self._V[ii]
            a = a / np.linalg.norm(a, 2)
            b = b / np.linalg.norm(b, 2)

            vectors[ii] = a + b
            angles[ii]  = np.dot(a.transpose(), b)

        a = self._V[n-2] - self._V[n-1]
        b = self._V[0] - self._V[n-1]
        a = a / np.linalg.norm(a, 2)
        b = b / np.linalg.norm(b, 2)

        vectors[n-1] = a + b
        angles[n-1]  = np.dot(a.transpose(), b)
        new_polygon = np.zeros([n,2])

        for ii in range(0,n):
            new_polygon[ii] = self._V[ii] - \
                              length / np.linalg.norm(vectors[ii]) * vectors[ii] * 1.4142 / np.sqrt(1 - angles[ii])
            if self.in_polygon(new_polygon[ii]):
                new_polygon[ii] = self._V[ii] + \
                                  length / np.linalg.norm(vectors[ii]) * vectors[ii] * 1.4142 / np.sqrt(1 - angles[ii])
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

    def draw(self, axes, fcolor, ecolor, alpha):
        """Plots the polygon with the given plotter."""
        poly = matplotlib.patches.Polygon(self._V, facecolor=fcolor, edgecolor=ecolor, alpha=alpha)
        axes.add_patch(poly)
        if self._safety_region:
            poly_safe = matplotlib.patches.Polygon(self._V_safe, facecolor='none', edgecolor=fcolor)
            axes.add_patch(poly_safe)


if __name__=="__main__":
    amap = Map('triangle')
    fig = plt.figure()
    ax  = fig.add_subplot(111, autoscale_on=False)

    #fig, ax = plt.subplots()

    amap.draw(ax)

    amap.draw_discrete(ax)

    ax.grid()
    plt.show()
