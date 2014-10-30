#!/usr/bin/env python

import numpy as np

import matplotlib

class Map(object):
    """This class provides a general map."""
    def __init__(self, maptype=None, gridsize=0.5):
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

        self._is_discretized = False
        self._gridsize = gridsize
        self._grid = None

    def discretize_map(self, gridsize=0.5):
        self._gridsize = gridsize

        self._discrete_dim = [int(self._dim[0]/self._gridsize),
                              int(self._dim[1]/self._gridsize)]

        self._grid = np.zeros(self._discrete_dim)

        for x in range(0, self._discrete_dim[0]):
            for y in range(0, self._discrete_dim[1]):
                if self.is_occupied((x,y), safety_region=True):
                    self._grid[x,y] = 1

        self._is_discretized = True

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
        return self._grid[point] == 1

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

    def draw(self, axes, pcolor='g', ecolor='k'):
        """Draws the map in the given matplotlib axes."""
        for poly in self._obstacles:
            poly.draw(axes, pcolor, ecolor)

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

    def draw(self, axes, fcolor, ecolor):
        """Plots the polygon with the given plotter."""
        poly = matplotlib.patches.Polygon(self._V, facecolor=fcolor, edgecolor=ecolor)
        axes.add_patch(poly)
        if self._safety_region:
            poly_safe = matplotlib.patches.Polygon(self._V_safe, facecolor='none', edgecolor=fcolor)
            axes.add_patch(poly_safe)
