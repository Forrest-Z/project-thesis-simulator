#!/usr/bin/env python

"""
Dynamic Window controller

This module implements the Dynamic Window controller as proposed by Fox et. al., 1997.

"""

import time, cProfile
from matplotlib import pyplot as plt

import numpy as np
#import utils

class DynamicWindow(object):
    def __init__(self, dT, N):

        self.window_res = [9, 51]  # xy Dynamic Window resolution
        self.xStride = 1.0
        self.yStride = 1.0
        self.dT = dT

        self.window          = np.zeros(self.window_res)
        self.velocity_map    = np.zeros(self.window_res)
        self.heading_map     = np.zeros(self.window_res)
        self.dist_map        = np.zeros(self.window_res)
        self.scaled_dist_map = np.zeros(self.window_res)

        self.arches          = [[None for x in xrange(self.window_res[1])] for x in xrange(self.window_res[0])]
        self.best_arches     = [None] * N

        self.test_arch = True

        # :todo: change
        self.goal = None

        self.MAX_REVERSE_SP  = -99999

        # Temporary variables for plotting
        self.rk_best = 5
        self.uk_best = 5
        self.cur_uk = 5
        self.cur_rk = 8

        self.win_radius = 40
        self.pred_horiz = 30
        self.time_step  = 5
        
        self.alpha = 1#0.3 * 180 / np.pi
        self.beta  = 1#0.2
        self.gamma = 1#0.5

        self.u_max = 3.0
         
        # :todo: Some circular logic here. Need set this after World object is
        # created, which depends on Vessel and Controller objects.
        self.the_world = None
        self.last_update = -self.dT

    def update(self, vessel_object):

        tic = time.clock()

        x = vessel_object.x[0]
        y = vessel_object.x[1]
        psi = vessel_object.x[2]
        u = vessel_object.x[3]  # body frame forward velocity
        r = vessel_object.x[5]

        self.window.fill(0)
        self.velocity_map.fill(0)
        self.heading_map.fill(0)
        self.dist_map.fill(0)
        self.scaled_dist_map.fill(0)

        # Determine reachable surge velocities
        u_rad_max = min(u + vessel_object.model.est_du_max * self.dT,
                        min(self.u_max, vessel_object.model.est_u_max))
        u_rad_min = max(u + vessel_object.model.est_du_min * self.dT,
                        max(-self.u_max, vessel_object.model.est_u_min))

        u_range = np.linspace(u_rad_max, u_rad_min, self.window_res[0])
        
        # Determine reachable yaw velocities
        r_rad_max = min(r + vessel_object.model.est_dr_max * self.dT,
                        vessel_object.model.est_r_max)
        r_rad_min = max(r - vessel_object.model.est_dr_max * self.dT,
                        -vessel_object.model.est_r_max)

        r_range = np.linspace(r_rad_max, r_rad_min, self.window_res[1])

        # print vessel_object.model.est_dr_max, vessel_object.model.est_r_max, r
        # print r_rad_max, r_rad_min
        # print u_range

        # Calculate distance map
        for uk in range(0, len(u_range)):
            u = u_range[uk]

            for rk in range(0, len(r_range)):
                r = r_range[rk]

                # Calculate distance map. The reachable points.
                self.calc_dist_map(u_range, r_range, uk, rk, x, y, psi, u, r)
                
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
            self.heading_map  = (self.heading_map - heading_min) / (heading_max - heading_min)

        velocity_min = np.amin(self.velocity_map)
        velocity_max = np.amax(self.velocity_map)
        if velocity_min == velocity_max:
            self.velocity_map.fill(0)
        else:
            self.velocity_map  = (self.velocity_map - velocity_min) / (velocity_max - velocity_min)        
    

        dist_min = np.amin(self.scaled_dist_map)
        dist_max = np.amax(self.scaled_dist_map)
        if dist_min == dist_max:
            self.scaled_dist_map.fill(0)
        else:
            self.scaled_dist_map  = (self.scaled_dist_map - dist_min) / (dist_max - dist_min)

        
        # Compose window
        self.window = self.alpha*self.heading_map + self.beta*self.scaled_dist_map + \
                      self.gamma*self.velocity_map

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
        else:
            # Use best choice
            vessel_object.psi_d = np.Inf
            vessel_object.u_d = u_range[uk_best]
            vessel_object.r_d = r_range[rk_best]
            #print vessel_object.u_d, vessel_object.r_d
            self.uk_best = uk_best
            self.rk_best = rk_best
        # :todo: draw beautiful paths
        n = int(vessel_object.time / self.dT)
        self.best_arches[n] = self.arches[self.uk_best][self.rk_best]
        
        toc = time.clock()
        print "Dynamic window CPU time: %.3f" %(toc-tic)


        #print r_range, self.rk_best
        
    def calc_dist_map(self, u_range, r_range, uk, rk, x, y, psi, u, r):
        if u == 0:
            # No disatance
            self.scaled_dist_map[uk, rk] = 0
            self.dist_map[uk, rk] = 0
            return

        if np.abs(r) > 0:
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
            path = np.empty((2, int(np.around(self.pred_horiz/step_time))-1))
            it = 0
            for t in np.linspace(step_time,
                                 self.pred_horiz,
                                 int(np.around(self.pred_horiz/step_time)) - 1):
                alpha += cstep
                xk = center[0] + radius*np.cos(alpha)
                yk = center[1] + radius*np.sin(alpha)
                
                # :todo: draw paths?
                path[:,it] = xk, yk
                it += 1

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

                max_dist += step_dist

            self.arches[uk][rk] = path[:,:it]

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
            path = np.empty((2, int(np.around(self.pred_horiz/step_time))-1))
            it = 0
            for t in np.linspace(step_time,
                                 self.pred_horiz,
                                 int(np.around(self.pred_horiz/step_time))-1):
                xk += x_step
                yk += y_step
                
                # :todo: draw/store paths?
                path[:,it] = xk, yk
                it += 1

                if max_dist >= 2*self.win_radius:
                    # We are done
                    max_dist = 2*self.win_radius
                    break
                elif self.the_world.is_occupied(xk,
                                                yk,
                                                t/self.dT):
                    # Found intersection
                    break
                max_dist += step_dist
            
            self.arches[uk][rk] = path[:,:it]

            # :todo: Draw/update paths
            self.scaled_dist_map[uk, rk] = max_dist / (2*self.win_radius)
            self.dist_map[uk, rk] = max_dist
        
    def calc_dyn_wind(self, uk, rk, x, y, psi, u, r, est_du_max, est_dr_max, goal, psi_d):
        dist = self.dist_map[uk, rk]

        # Only proceed if this is an admissible velocity, i.e.,
        # the vehicle can come to a complete stop after choosing
        # this alternative
        if np.abs(u) > np.sqrt(2*dist + est_du_max) or \
           np.abs(r) > np.sqrt(2*dist + est_dr_max):
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
            psi_next   = psi + r*self.dT + 0.5*r*np.abs(r/est_dr_max)
            
            # :todo: Not working properly for now.
            # If psi_d is np.Inf, then it was set by DWA previously,
            # else it is set by LOS and we should weigh accordingly
            if psi_d == np.Inf:
                psi_target = np.arctan2(goal[1] - y,
                                        goal[0] - x)
            else:
                psi_target = psi_d
                
            # If in reverse, heading is the opposite
            if u < 0:
                psi_next += np.pi

            heading = np.pi - np.abs(normalize_angle(psi_next - psi_target, 0))

            self.velocity_map[uk, rk] = u
            self.heading_map[uk, rk] = heading
            
    def draw(self, axes, n):
        print n
        for ii in range(0, n, 4):
            axes.plot(self.best_arches[ii][0],
                      self.best_arches[ii][1], 'r')


def normalize_angle(angle, angle_ref):
    """
    Makes 'angle' compatible with 'angle_ref' such that the numerical
    difference is at most PI
    """
    if angle_ref == np.Inf:
        return angle

    # Get angle within 2*PI of angle_ref
    diff = angle_ref - angle
    if diff > 0:
        new_angle = angle + (diff - np.fmod(diff, 2*np.pi))
    else:
        new_angle = angle + (diff + np.fmod(-diff, 2*np.pi))

    # Make sure angle is on the closest side of angle_ref
    diff = angle_ref - new_angle
    if diff > np.pi:
        new_angle += 2*np.pi
    elif diff < -np.pi:
        new_angle -= 2*np.pi

    return new_angle


def int_circles(x1, y1, r1, x2, y2, r2):
    """Tests for intersection between two circles.

    Returns:
        int = True on intersection
        coords = coordinates of intersection (numpy.array)

    http://local.wasp.uwa.edu.au/~pbourke/geometry/2circle/
    """

    # Distance between circle centers
    d = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    if (d > r1 + r2) or (d < np.abs(r1-r2)):
        # No solution
        return False, []

    a = (r1**2 - r2**2 + d**2 ) / (2*d)

    xp = x1 + a*(x2 - x1)/d
    yp = y1 + a*(y2 - y1)/d

    if (d == r1 + r2):
        # One solution
        coords = np.array([xp, yp])
        return True, coords
    else:
        # Two solutions
        h = np.sqrt(r1**2 - a**2);
        coords = np.array([[xp+h*(y2-y1)/d, yp-h*(x2-x1)/d],
                           [xp-h*(y2-y1)/d, yp+h*(x2-x1)/d]])
    return True, coords


def test():
    the_map = utils.Map()
    
    obstacle = utils.Polygon([(40,15), (45,15), (45,20), (40,20)], safety_region_length=4.0)
    the_map.add_obstacles([obstacle])

    tend = 100
    dT = 0.5
    N  = int(tend/dT) +1
    x0 = np.array([10, 10, 0.0, 1.0, 0, 0])
    xg = np.array([50, 50, 0])

    myDynWnd = DynamicWindow(dT, N)

    v = utils.Vessel(x0, xg, 0.1, 0.5, N, [myDynWnd], is_main_vessel=True, vesseltype='viknes')

    world = utils.World([v], the_map)

    myDynWnd.the_world = world
    
    world.update_world(0,0)


#cProfile.run('test()')

if __name__ == "__main__":
    # the_map = utils.Map()
    
    # obstacle = utils.Polygon([(40,15), (45,15), (45,20), (40,20)], safety_region_length=4.0)
    # the_map.add_obstacles([obstacle])

    # tend = 100
    # dT = 0.5
    # N  = int(tend/dT)
    # x0 = np.array([10, 10, 0.0, 1.0, 0, 0])
    # xg = np.array([50, 50, 0])

    # myDynWnd = DynamicWindow(dT)

    # v = utils.Vessel(x0, xg, dT, N, [myDynWnd], is_main_vessel=True, vesseltype='viknes')

    # world = utils.World([v], the_map)

    # myDynWnd.the_world = world
    
    # world.update_world(0)
    
    # fig, axes = plt.subplots()

    # # the_map.draw(axes)
    # # myDynWnd.draw(axes)
    # # axes.axis([-10, 50, -10, 50])

    # scen = utils.Scenario('dynwnd')
    # sim = utils.Simulation(scen)

    # sim.run_sim()
    
    # ani = sim.animate(fig, axes)
    # plt.show()
    pass
