#!/usr/bin/env python
import sys, time
import heapq
import cv2

import numpy as np
import matplotlib.pyplot as plt


#from utils import State, Vessel, Map, Scenario, Simulation

def Rz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta),  np.cos(theta), 0],
                     [0            ,  0            , 1]])

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
    def get_grid_id(self, state):
        """Returns a tuple (x,y,psi) with grid positions."""
        return (int(state[0]/self.gridsize[0]),
                int(state[1]/self.gridsize[1]),
                int(state[2]/self.gridsize[2]))

    def in_bounds(self, state):
        return 0 <= state[0] < self.width and 0 <= state[1] < self.height

    def cost(self, a, b):
        return self.weights.get((b[0], b[1]),1)

    def passable(self, state):
        # TODO Rename or change? Only returns true if object is _inside_ obstacle
        # Polygons add safety zone by default now.
        if not self.map.is_occupied(state[0:2]):
            return True
        else:
            return False

    def neighbors(self, state):
        """
        Applies rudder commands to find the neighbors of the given state.

        For the Viknes 830, the maximum rudder deflection is 15 deg.
        """
        trajectories = np.array([[ 5.45327685, -0.43705761, -0.16338934],
                                 [ 5.46279187,  0.        ,  0.        ],
                                 [ 5.45327685,  0.43705761,  0.16338934]])

        results = []
        for traj in trajectories:
            newpoint = state + np.dot(Rz(state[2]), traj)
            results.append(newpoint)

        #results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

class HybridAStar(object):
    def __init__(self, x0, xg, the_map):
        self.start = x0[0:3]
        self.goal  = xg[0:3]

        self.graph = SearchGrid(the_map, [5.0, 5.0, 15.0/360.0], N=3)

        self.map = the_map
        self.eps = 10.0
        self.to_be_updated = True
        self.path_found = False

    def update(self, vessel_object):
        if self.to_be_updated:

            vessel_object.waypoints = self.search()

            self.to_be_updated = False
            self.map.disable_safety_region()


    def search(self):
        """The Hybrid State A* search algorithm."""

        # To clean up a bit
        get_grid_id = self.graph.get_grid_id

        frontier = PriorityQueue()
        frontier.put(self.start, 0)
        came_from = {}
        cost_so_far = {}

        came_from[tuple(self.start)] = None
        cost_so_far[get_grid_id(self.start)] = 0

        path_found = False

        while not frontier.empty():
            current = frontier.get()

            if np.linalg.norm(current[0:2] - self.goal[0:2]) < self.eps \
               and np.abs(current[2]-self.goal[2]) < np.pi/8:
                self.path_found = True
                break

            for next in self.graph.neighbors(current):
                new_cost = cost_so_far[get_grid_id(current)] + \
                           self.graph.cost(current, next)

                if get_grid_id(next) not in cost_so_far or new_cost < cost_so_far[get_grid_id(next)]:
                    cost_so_far[get_grid_id(next)] = new_cost
                    priority = new_cost + heuristic(self.goal, next)
                    frontier.put(next, priority)
                    came_from[tuple(next)] = current


        # Reconstruct path
        path = [current]
        while tuple(current) != tuple(self.start):
            current = came_from[tuple(current)]
            path.append(current)
        print self.path_found
        return np.copy(np.asarray(path[::-1]))

def heuristic(a, b):
    """The search heuristics function."""
    return np.linalg.norm(a-b)

# def hybrid_astar(scenario):

#     frontier = PriorityQueue()
#     frontier.put(self.start, 0)
#     came_from = {}
#     cost_so_far = {}
#     came_from[start.grid_xy] = None
#     cost_so_far[start.grid_xy] = 0

#     path_found = False

#     while not frontier.empty():
#         current = frontier.get()

#         if current == goal:
#             path_found = True
#             break

#         for next in graph.neighbors(current):
#             new_cost = cost_so_far[current.grid_xy] + graph.cost(current, next)

#             if next.grid_xy not in cost_so_far or new_cost < cost_so_far[next.grid_xy]:
#                 cost_so_far[next.grid_xy] = new_cost
#                 priority = new_cost + heuristic(goal, next)
#                 frontier.put(next, priority)
#                 came_from[next] = current


#     path = reconstruct_path(came_from, start, current)

#     return path, path_found




if __name__ == "__main__":
    mymap = Map('s1')
    #mymap.load_map('s1')

    myvessel = Vessel('viknes')

    x0 = np.array([0, 0, 0, 3.0, 0, 0])
    xg = np.array([100, 100, np.pi/4])

    myscenario   = Scenario(mymap, x0, xg)
    myastar = HybridAStar(myscenario)

    myastar.update(myscenario)

    print "ok"
    fig, ax = plt.subplots()
    ax.plot(myscenario.waypoints[:,0],
            myscenario.waypoints[:,1],
            '.')

    ax.plot(x0[0], x0[1], 'bo')
    ax.plot(xg[0], xg[1], 'ro')

    ax.axis([-10, 160, -10, 160], 'equal')
    mymap.draw(ax, 'g', 'k')

    plt.show()
