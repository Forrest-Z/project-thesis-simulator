#!/usr/bin/env python
import sys, time
import heapq
import cv2

import numpy as np
import matplotlib.pyplot as plt


from utils import *

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
    """General purpose N-dimentional search grid."""
    def __init__(self, the_map, gridsize, N=2):
        self.N        = N
        self.grid     = []
        self.map      = the_map
        self.gridsize = gridsize

        dim = the_map.get_dimension()

        self.width  = dim[0]
        self.height = dim[1]

        if N==3:
            # yaw resolution
            dim.append( 5.0 / 360.0 )

        # TODO: Is this needed?
        # for ii in range(0,N):
        #     print self.grid, dim, gridsize, N, ii
        #     self.grid[ii].append( [0] * int(dim[ii] / gridsize[ii]) )



        #self.walls = []
        # for obstacle in the_map.get_obstacles():
        #     V = obstacle.get_vertices()
        #     n = len(V)
        #     for ii in range(1,n):
        #         self.walls.append( (int(V[ii][0]), int(V[ii][1])) )



        self.weights = {}

    def in_bounds(self, state):
        (x,y) = state.grid_xy
        return 0 <= x < self.width and 0 <= y < self.height

    def cost(self, a, b):
        return self.weights.get(b,1)

    def passable(self, state):
        # TODO Rename or change? Only returns true if object is _inside_ obstacle
        if not self.map.is_occupied( (state.x, state.y) ):
            return True
        else:
            return False
    def neighbors(self, state):
        results = state.neighbors()

        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

def heuristic(a, b):
    return a.dist(b) + 0*abs(a.psi-b.psi)

def hybrid_a_star(scenario):
    start = scenario.initial_state
    goal  = scenario.goal_state
    graph = SearchGrid(scenario.map, [start.gridsize, start.gridsize])

    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start.grid_xy] = None
    cost_so_far[start.grid_xy] = 0

    path_found = False

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            path_found = True
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current.grid_xy] + graph.cost(current, next)

            if next.grid_xy not in cost_so_far or new_cost < cost_so_far[next.grid_xy]:
                cost_so_far[next.grid_xy] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current


    path = reconstruct_path(came_from, start, current)

    return path, path_found


def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.append(start)
    return path


if __name__ == "__main__":
    mymap = Map('s1')
    #mymap.load_map('s1')

    myvessel = Vessel('other')
    start_state = State(0,0,0, gridsize=1)
    goal_state  = State(100,100,0)

    myscenario   = Scenario(mymap, start_state, goal_state)
    mysimulation = Simulation(myscenario, hybrid_a_star, myvessel)

    mysimulation.run_sim()

    fig, ax = plt.subplots()
    mysimulation.draw(ax)
    #mymap.draw(ax, 'g', 'k')

    plt.show()
