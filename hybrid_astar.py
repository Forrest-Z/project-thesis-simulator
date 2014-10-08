#!/usr/bin/env python
import sys, time
import heapq
import cv2

import numpy as np
import matplotlib.pyplot as plt


NODEWIDTH = 1 # pixels
DELTA = 3.1415/5



class State(object):
    def __init__(self, s = (0,0,0)):
        self.x   = s[0]  # [m]   Position
        self.y   = s[1]  # [m]   Position
        self.psi = s[2]  # [rad] Orientation

        self.grid_id = "(%d,%d)"%(int(self.x) / NODEWIDTH, int(self.y) / NODEWIDTH )

    def __str__(self):
        return "(%.2f, %.2f, %.2f)"%(self.x, self.y, self.psi)

    def __eq__(self, other):
        return (isinstance(other, self.__class__) and self.dnorm(other) < 5*NODEWIDTH)

    def __ne__(self, other):
        return not self.__eq__(other)

    def dnorm(self, s2):
        return np.sqrt( (s2.x - self.x)**2 + (s2.y - self.y)**2 + (s2.psi - self.psi)**2)

    def dist(self, s2):
        return np.sqrt( (s2.x - self.x)**2 + (s2.y - self.y)**2 )

    def to_tuple(self):
        return (int(self.x)/NODEWIDTH, int(self.y)/NODEWIDTH)

    def neighbors(self):
       p = self.psi
       d = DELTA
       x0 = 2.5*NODEWIDTH
       y0 = 0.0
       x = self.x
       y = self.y

       ret = []
       for dp in [-2, -1, 0, 1, 2]:
           d = dp * np.pi / 16
           n = State((x - x0*(np.sin(d)*np.sin(p) - np.cos(d)*np.cos(p)),
                      y - x0*(np.cos(d)*np.sin(p) + np.cos(p)*np.sin(d)),
                      p + d))
           ret.append(n)


       # n1 = (x - x0*(np.sin(d)*np.sin(p) - np.cos(d)*np.cos(p)),
       #       y - x0*(np.cos(d)*np.sin(p) + np.cos(p)*np.sin(d)),
       #       p + d)
       # n2 = (x + x0*(np.sin(d)*np.sin(p) + np.cos(d)*np.cos(p)),
       #       y - x0*(np.cos(d)*np.sin(p) - np.cos(p)*np.sin(d)),
       #       p - d)
       # n3 = (x + np.cos(p)*x0,
       #       y - np.sin(p)*x0,
       #       p)

       return ret #[State(n1), State(n2), State(n3)]

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

class SearchGrid(object):
    """ General purpose N-dimentional search grid """
    def __init__(self, N, res):
        pass


class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
        self.weights = {}

    def in_bounds(self, s):
        (x, y) = (int(s.x) / NODEWIDTH, int(s.y) / NODEWIDTH)
        return 0 <= x < self.width and 0 <= y < self.height

    def cost(self, a, b):
        return self.weights.get(b, 1)

    def passable(self, s):
        point = s.to_tuple()

        return point not in self.walls

    def passable2(self, s0, points):
        ret = []
        s = s0.to_tuple()
        for state in points:
            p = state.to_tuple()
            if p not in self.walls:
                for w in self.walls:
                    crossproduct = (w[1] - s[1]) * (p[0] - s[0]) - (w[0] - s[0]) * (p[1] - s[1])
                    if abs(crossproduct) > 0.01:
                        ret.append(state)
        return ret


    def neighbors(self, s):
        results = s.neighbors()

        # simulate system elns -> return

        # results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1),
        #            (x+1, y+1), (x+1,y-1), (x-1,y+1), (x-1,y-1)]
#        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results




def heuristic(a, b):
    return a.dist(b) + 0*abs(a.psi-b.psi)

def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start.grid_id] = None
    cost_so_far[start.grid_id] = 0

    path_found = False

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            path_found = True
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current.grid_id] + graph.cost(current, next)

            if next.grid_id not in cost_so_far or new_cost < cost_so_far[next.grid_id]:
                cost_so_far[next.grid_id] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far, current, path_found

def check_pygame_events():
    for e in pygame.event.get():
        if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
            sys.exit("Leaving because you requested it.")


def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    return path


if __name__ == "__main__":
    mymap = cv2.imread('map.png', cv2.IMREAD_GRAYSCALE)

    height,width = mymap.shape[:2]

    costmap = SquareGrid(width, height)

    for x in range(0,width):
        for y in range(0,height):
            if mymap[x,y] < 10:
                costmap.walls.append( (x,y) )
    costmap.weights = {loc: 10 for loc in costmap.walls}

    start = State((0,0,0))
    finish = State((150,150,0))

    tic = time.clock()
    came_from, cost_so_far, last_state, path_found = a_star_search( costmap, start, finish )
    toc = time.clock()

    print toc-tic
    print path_found

    path = reconstruct_path(came_from, start, last_state)
    x = []
    y = []
    psi = []
    for s in path:
        #print "%.2f,%.2f"%(s.x, s.y)
        x.append(s.x)
        y.append(s.y)
        psi.append(s.psi)

    walls = map(list, zip(*costmap.walls))
    plt.plot(walls[0],walls[1], 'g.')
    plt.plot(x,y)
    plt.plot(finish.x, finish.y, 'ro')
    plt.show()

