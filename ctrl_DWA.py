#!/usr/bin/env python

"""
Dynamic Window controller

This module implements the Dynamic Window controller as proposed by Fox et. al., 1997.

"""

import time, cProfile
from matplotlib import pyplot as plt

import numpy as np
import copy

from world import World
from map import Map, Polygon
from vessel import Vessel

from utils import *

class DynamicWindow(Controller):
    def __init__(self, dT, N, gridsize=0.5):

        self.window_res = [5, 51]  # xy Dynamic Window resolution
        self.xStride = gridsize
        self.yStride = gridsize
        self.dT = dT
        self.N  = N
        self.n  = 0

        self.window          = np.zeros(self.window_res)
        self.velocity_map    = np.zeros(self.window_res)
        self.heading_map     = np.zeros(self.window_res)
        self.dist_map        = np.zeros(self.window_res)
        self.scaled_dist_map = np.zeros(self.window_res)

        self.u_range = None
        self.r_range = None

        self.current_arches  = [[None for x in xrange(self.window_res[1])] for x in xrange(self.window_res[0])]
        self.arches          = [None] * N
        self.best_arches     = [None] * N

        self.test_arch = True

        self.MAX_REVERSE_SP  = -99999

        # Temporary variables for plotting
        self.rk_best = 5
        self.uk_best = 5
        self.cur_uk = 5
        self.cur_rk = 8

        self.win_radius = 40
        self.pred_horiz = 30
        self.time_step  = 5
        
        self.alpha = 0.3#0.7
        self.beta  = 0.2#0.2
        self.gamma = 0.5#0.5

        self.sigma = 0.0 # Low pass filter constant

        self.u_max = 3.0
         
        # :todo: Some circular logic here. Need set this after World object is
        # created, which depends on Vessel and Controller objects.
        self.the_world = None
        self.last_update = -self.dT
        self.psi_target = None

    def update(self, vessel_object):

        x = vessel_object.x[0]
        y = vessel_object.x[1]
        psi = vessel_object.x[2]
        u = vessel_object.x[3]  # body frame forward velocity
        r = vessel_object.x[5]

        self.psi_target = np.arctan2(vessel_object.goal[1] - y,
                                     vessel_object.goal[0] - x)
        tic = time.clock()


        #self.window.fill(0)
        self.velocity_map.fill(0)
        self.heading_map.fill(0)
        self.dist_map.fill(0)
        self.scaled_dist_map.fill(0)

        # Determine reachable surge velocities
        u_rad_max = min(u + vessel_object.model.est_du_max * self.time_step,
                        min(self.u_max, vessel_object.model.est_u_max))
        u_rad_min = max(u + vessel_object.model.est_du_min * self.time_step,
                        max(-self.u_max, vessel_object.model.est_u_min))

        u_range = np.linspace(u_rad_max, u_rad_min, self.window_res[0])
        
        # Determine reachable yaw velocities
        r_rad_max = min(r + vessel_object.model.est_dr_max * self.time_step,
                        vessel_object.model.est_r_max)
        r_rad_min = max(r - vessel_object.model.est_dr_max * self.time_step,
                        -vessel_object.model.est_r_max)

        r_range = np.linspace(r_rad_max, r_rad_min, self.window_res[1])

        # print vessel_object.model.est_dr_max, vessel_object.model.est_r_max, r
        # print r_rad_max, r_rad_min
        # print u_range

        # Calculate distance map
        for uk in range(0, self.window_res[0]):
            u = u_range[uk]

            for rk in range(0, self.window_res[1]):
                r = r_range[rk]

                # Calculate distance map. The reachable points.
                self.calc_dist_map(uk, rk, x, y, psi, u, r)
                
                # Calculate the dynamic window
                self.calc_dyn_wind(uk, rk, x, y, psi, u, r,
                                   vessel_object.model.est_du_max,
                                   vessel_object.model.est_dr_max,
                                   vessel_object.current_goal,
                                   vessel_object.psi_d)

        # Normalize
        heading_min = np.amin(self.heading_map)
        heading_max = np.amax(self.heading_map)
        if heading_min == heading_max:
            self.heading_map.fill(0)
        else:
            self.heading_map  = (self.heading_map - heading_min) / float(heading_max - heading_min)

        velocity_min = np.amin(self.velocity_map)
        velocity_max = np.amax(self.velocity_map)
        if velocity_min == velocity_max:
            self.velocity_map.fill(0)
        else:
            self.velocity_map  = (self.velocity_map - velocity_min) / float(velocity_max - velocity_min)
    

        dist_min = np.amin(self.scaled_dist_map)
        dist_max = np.amax(self.scaled_dist_map)
        if dist_min == dist_max:
            self.scaled_dist_map.fill(0)
        else:
            self.scaled_dist_map  = (self.scaled_dist_map - dist_min) / float(dist_max - dist_min)

        
        # Compose window
        self.window = self.sigma * self.window + \
                      (1.-self.sigma) * (self.alpha*self.heading_map + \
                                        self.beta *self.scaled_dist_map + \
                                        self.gamma*self.velocity_map)

        # Find the best option
        n = np.argmax(self.window)
        uk_best = int(n / self.window_res[1])
        rk_best = n % self.window_res[1]

        if self.window[uk_best, rk_best] <= 0:
            # No admissible choice. Break with full force.
            vessel_object.psi_d = np.Inf
            vessel_object.u_d = self.MAX_REVERSE_SP
            vessel_object.r_d = 0

            uk_best = self.cur_uk
            rk_best = self.cur_rk
            self.best_arches[self.n] = np.zeros((1,2))
            print "SHITSHITSHIT!!!!"

        else:
            # Use best choice
            vessel_object.psi_d = np.Inf
            vessel_object.u_d = u_range[uk_best]
            vessel_object.r_d = r_range[rk_best]
            #print vessel_object.u_d, vessel_object.r_d
            self.uk_best = uk_best
            self.rk_best = rk_best
            self.best_arches[self.n] = copy.deepcopy(self.current_arches[self.uk_best][self.rk_best])

        # :todo: draw beautiful paths


        self.n += 1
        toc = time.clock()
        print "Dynamic window: (%.2f, %.2f, %.2f) CPU time: %.3f" %(x,y,psi,toc-tic)


        #print r_range, self.rk_best
        
    def calc_dist_map(self, uk, rk, x, y, psi, u, r):
        if np.abs(u) < 0.01:
            # No disatance
            self.scaled_dist_map[uk, rk] = 0
            self.dist_map[uk, rk] = 0
            self.current_arches[uk][rk] = np.zeros((1,2))
            return

        if np.abs(r) > 0.01:
            # The (u, r) circle is defined by:
            #    radius: u/r
            #    center: [x - (u/r)*sin(psi), y + (u/r)*cos(psi)]

            # Circle center
            center = np.array([x - (u/r)*np.sin(psi),
                               y + (u/r)*np.cos(psi)])

            # Angle from circle center to target
            beta = np.arctan2(y - center[1],
                              x - center[0])

            # Radius of turn
            radius = np.abs(u/r)

            # Size of steps taken along circle (in radians) when
            # checking for intersections.
            # Must be equal to the resolution of the world map.
            cstep = (r/np.abs(u)) * min(self.xStride, self.yStride)
            
            # Distance of each step
            step_dist = np.abs(cstep) * radius
            # Time along step
            step_time = step_dist / np.abs(u)

            # Find the last angle to test along the circle by
            # intersection the circle with a cicle in from of
            # our vehicle self.win_radius
            intersected, ints = int_circles(x + np.sign(u)*self.win_radius*np.cos(psi),
                                            y + np.sign(u)*self.win_radius*np.sin(psi),
                                            self.win_radius,
                                            center[0],
                                            center[1],
                                            radius)
            
            if not intersected:
                print "Error, error! Not two points in intersection"
                return

            # The intersection test should return two coordinates:
            # 1. The coordiante of our vehilce
            # 2. The coordinates of the limit of this trajectory
            # We use the manhattan distance to select the latter            
            if np.abs(ints[0,0]-x + ints[0,1]-y) < 0.001:
                coords = ints[1,:]
            else:
                coords = ints[0,:]
                
            # Find the angle of the given intersection.
            # It should be the last angle in the iteration below
            last_alpha = normalize_angle(np.arctan2(coords[1] - center[1],
                                                    coords[0] - center[0]),
                                         beta)
                    
            # Make sure last_alpha is "ahead" of beta in terms of cstep
            if cstep > 0 and last_alpha < beta:
                last_alpha += 2*np.pi
            elif cstep < 0 and last_alpha > beta:
                last_alpha -= 2*np.pi

            # Iterate along circle, testing for intersections
            alpha = beta
            max_dist = 0
            path = np.empty((int(np.around(self.pred_horiz/step_time))+1, 2))
            it = 0
            for t in np.arange(step_time, self.pred_horiz, step_time):
                alpha += cstep
                xk = center[0] + radius*np.cos(alpha)
                yk = center[1] + radius*np.sin(alpha)
                
                # :todo: draw paths?


                if (cstep > 0 and alpha >= last_alpha) or \
                   (cstep < 0 and alpha <= last_alpha):
                    # Travelled full path
                    alpha = last_alpha
                    break
                elif self.the_world.is_occupied(xk,
                                                yk,
                                                np.floor(t / self.dT)):
                    # Intersection
                    break

                path[it] = xk, yk
                it += 1

                max_dist += step_dist

            self.current_arches[uk][rk] = path[:(it)]

            # Update distance map with max distance along this path
            # relatice to possible distance
            self.scaled_dist_map[uk, rk] = np.abs((alpha-beta)/(last_alpha-beta))
            self.dist_map[uk, rk] = max_dist
        else:
            # Travelling at a straight line (u, 0)
            # Check this line for intersections with world
            
            # Distance of each step
            step_dist = min(self.xStride, self.yStride)
            
            # Time along step
            step_time = step_dist / abs(u)
            
            # Distance we can travel along line
            max_dist = 0
            
            # :todo: visualization of this path
            
            # Iterate over line
            x_step = np.sign(u)*step_dist*np.cos(psi)
            y_step = np.sign(u)*step_dist*np.sin(psi)
            xk = x
            yk = y
            path = np.empty((int(np.around(self.pred_horiz/step_time))+1, 2))
            it = 0
            for t in np.arange(step_time, self.pred_horiz, step_time):
                xk += x_step
                yk += y_step
                
                # :todo: draw/store paths?

                if max_dist >= 2*self.win_radius:
                    # We are done
                    max_dist = 2*self.win_radius
                    break
                elif self.the_world.is_occupied(xk,
                                                yk,
                                                t/self.dT):
                    # Found intersection
                    break
                    
                path[it] = xk, yk
                it += 1

                max_dist += step_dist
            
            self.current_arches[uk][rk] = path[:(it)]

            # :todo: Draw/update paths
            self.scaled_dist_map[uk, rk] = max_dist / (2*self.win_radius)
            self.dist_map[uk, rk] = max_dist
        
    def calc_dyn_wind(self, uk, rk, x, y, psi, u, r, est_du_max, est_dr_max, goal, psi_d):
        dist = self.dist_map[uk, rk]

        # Only proceed if this is an admissible velocity, i.e.,
        # the vehicle can come to a complete stop after choosing
        # this alternative
        if np.abs(u) > np.sqrt(2*dist * est_du_max) or \
           np.abs(r) > np.sqrt(2*dist * est_dr_max):
            # Not admissible
            self.window[uk, rk] = 0
            self.heading_map[uk, rk] = 0
            self.velocity_map[uk, rk] = 0
        else:
            # The value of this sector in the dynamic window is
            # value = alpha*heading + beta*dist + gamma*velocity
            #
            # The psi value should be calculated as the obtained psi
            # when applying maximum deceleration to the yaw after the
            # next time step.
            psi_next   = psi + r*self.time_step + 0.5*r*np.abs(r/est_dr_max)
            
            # :todo: Not working properly for now.
            # If psi_d is np.Inf, then it was set by DWA previously,
            # else it is set by LOS and we should weigh accordingly
            # if psi_d == np.Inf:
            #     psi_target = np.arctan2(goal[1] - y,
            #                             goal[0] - x)
            # else:
            #     psi_target = psi_d
                
            # If in reverse, heading is the opposite
            if u < 0:
                psi_next += np.pi

            heading = np.pi - np.abs(normalize_angle(psi_next - self.psi_target, 0))
            
            self.velocity_map[uk, rk] = u
            self.heading_map[uk, rk] = heading
            
    def draw(self, axes, n, fcolor='y', ecolor='k'):
        for ii in range(0, self.n, 8):
            axes.plot(self.best_arches[ii][:,0], self.best_arches[ii][:,1], 'r', alpha=0.5,
                      lw=2)

    def visualize(self, axes, t, n):
        """Visualize current arch. For real-time plotting."""
        axes.plot(self.best_arches[self.n-1][:,0],
                  self.best_arches[self.n-1][:,1], 'r', alpha=0.5)

def test():
    the_map = Map('s1')
    tic = time.clock()
    the_map.discretize_map()
    print time.clock() - tic
    #obstacle = Polygon([(40,15), (45,15), (45,20), (40,20)], safety_region_length=4.0)
    #the_map.add_obstacles([obstacle])

    tend = 10
    dT = 1
    h  = 0.05
    N  = int(tend/h)  + 1
    N2 = int(tend/dT) + 1

    x0 = np.array([10, 10, 0.0, 3.0, 0, 0])
    xg = np.array([50, 50, 0])

    myDynWnd = DynamicWindow(dT, N2)

    v = Vessel(x0, xg, h, dT, N, [myDynWnd], is_main_vessel=True, vesseltype='viknes')
    v.current_goal = np.array([50, 50])

    world = World([v], the_map)

    myDynWnd.the_world = world
    
    world.update_world(0,0, dT)


def simple_scenario_with_plot():
    the_map = Map('s1')
        
    #obstacle = Polygon([(30,15), (35,15), (35,20), (30,20)], safety_region_length=4.0)
    #the_map.add_obstacles([obstacle])

    tend = 50
    dT = 1
    h  = 0.05
    N  = int(tend/h) + 1
    N2 = int(tend/dT) + 1
    x0 = np.array([0, 0, 0.5, 3.0, 0, 0])
    xg = np.array([120, 120, 0])

    myDynWnd = DynamicWindow(dT, N2)

    v = Vessel(x0, xg, h, dT, N, [myDynWnd], is_main_vessel=True, vesseltype='viknes')
    v.current_goal = np.array([50, 50])

    world = World([v], the_map)

    myDynWnd.the_world = world

    n = 0
    for t in np.linspace(0, tend, N):
        world.update_world(t, n, dT)
        n += 1

    fig = plt.figure()
    ax  = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                          xlim=(-10, 10), ylim=(-10, 10))

    #fig, ax = plt.subplots()
    ax.grid()

    world.draw(ax, N)
    print "N, N2, n: ", N, N2, n
    myDynWnd.draw(ax, N2)


    plt.show()


if __name__ == "__main__":
    simple_scenario_with_plot()
    #test()
