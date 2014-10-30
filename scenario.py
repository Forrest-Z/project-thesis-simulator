#!/usr/bin/env python

import numpy as np

import matplotlib.pyplot as plt

from map import Map
from vessel import Vessel
from world import World
from simulation import Simulation

from ctrl_DWA import DynamicWindow
from ctrl_hybrid_astar import HybridAStar
from ctrl_LOS_Guidance import LOSGuidance

class Scenario(object):
    def __init__(self, name='s1'):
        if name == 's1':
            the_map = Map('s1')

            self.tend = 100
            self.dT   = 0.1
            self.N = int(self.tend / self.dT)

            # Vessel 1 (Main vessel)
            x01 = np.array([0, 0, 0, 1.0, 0, 0])
            xg1 = np.array([100, 100, np.pi/4])

            myLOS1 = LOSGuidance()
            myAstar = HybridAStar(x01, xg1, the_map)

            v1 = Vessel(x01, xg1, self.dT, self.N, [myLOS1], is_main_vessel=True, vesseltype='viknes')
            v1.waypoints = np.array([[0, 0], [50, 60], [70, 60], [100, 40], [100, 100]])
            #v1.waypoints = np.array([[0, 0], [140, 0], [120, 120]])

            # Vessel 2
            x02 = np.array([0, 120, 0, 3.0, 0, 0])
            xg2 = np.array([140, 0, 0])

            myLOS2 = LOSGuidance()

            v2 = Vessel(x02, xg2, self.dT, self.N, [myLOS2], is_main_vessel=False, vesseltype='viknes')
            v2.waypoints = np.array([[0, 120], [120, 120], [140, 0]])

            # Vessel 3
            x03 = np.array([120, 0, 0, 3.0, 0, 0])
            xg3 = np.array([0, 0, 0])

            myLOS3 = LOSGuidance()

            v3 = Vessel(x03, xg3, self.dT, self.N, [myLOS3], is_main_vessel=False, vesseltype='danskebaaten')
            v3.waypoints = np.array([[120, 0], [0, 0], [0, 120]])

            self.world = World([v1, v2], the_map)
        if name == 'collision':
            the_map = Map('s1')

            self.tend = 100
            self.dT   = 0.5
            self.N    = int(np.around(self.tend / self.dT)) + 1
            self.h    = 0.05
            # Vessel 1 (Main vessel)
            x01 = np.array([0, 0, 0, 3.0, 0, 0])
            xg1 = np.array([100, 100, np.pi/4])

            myLOS1 = LOSGuidance()
            myAstar = HybridAStar(x01, xg1, the_map)

            v1 = Vessel(x01, xg1,self.dT, self.dT, self.N, [myLOS1], is_main_vessel=True, vesseltype='viknes')
            v1.waypoints = np.array([[0, 0], [50, 60], [70, 60], [120, 20], [120, 120]])
            #v1.waypoints = np.array([[0, 0], [140, 0], [120, 120]])

            # Vessel 2
            x02 = np.array([0, 120, 0, 3.0, 0, 0])
            xg2 = np.array([140, 0, 0])

            myLOS2 = LOSGuidance()

            v2 = Vessel(x02, xg2,self.dT, self.dT, self.N, [myLOS2], is_main_vessel=False, vesseltype='viknes')
            v2.waypoints = np.array([[0, 120], [120, 120], [120, 0]])

            self.world = World([v1, v2], the_map)

        if name == 'dynwnd':
            the_map = Map()

            self.tend = 40   # Simulation time (seconds)
            self.h    = 0.05 # Integrator time step
            self.dT   = 0.5  # Controller time step
            self.N    = int(np.around(self.tend / self.h)) + 1

            # Vessel 1 (Main vessel)
            x01 = np.array([0, 0, np.pi/4, 3.0, 0, 0])
            xg1 = np.array([100, 100, np.pi/4])

            myLOS1 = LOSGuidance()
            myDynWnd = DynamicWindow(self.dT, int(self.tend/self.dT) + 1)

            v1 = Vessel(x01, xg1, self.h, self.dT, self.N, [myLOS1, myDynWnd], is_main_vessel=True, vesseltype='viknes')
            v1.waypoints = np.array([[0, 0], [120, 120]])
            #v1.waypoints = np.array([[0, 0], [140, 0], [120, 120]])

            # Vessel 2
            x02 = np.array([120, 120, -3*np.pi/4, 3.0, 0, 0])
            xg2 = np.array([140, 0, 0])

            myLOS2 = LOSGuidance()

            v2 = Vessel(x02, xg2, self.h, self.dT, self.N, [myLOS2], is_main_vessel=False, vesseltype='viknes')
            v2.waypoints = np.array([[120, 120], [0, 0]])

            self.world = World([v1, v2], the_map)

            myDynWnd.the_world = self.world

    def is_occupied(self, point):
        return self.map.is_occupied(point)

    def draw(self, axes, n, scolor='b', fcolor='r', ocolor='g', ecolor='k'):
        self.world.draw(axes, n)

    def animate(self, fig, axes, n):
        return self.world.animate(fig, axes, n)


if __name__ == "__main__":
    scen = Scenario('collision')
    sim  = Simulation(scen)

    sim.run_sim()

    fig = plt.figure()
    ax  = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                          xlim=(-10, 160), ylim=(-10, 160))
    ax.grid()
    #ani = sim.animate(fig, ax)
    sim.draw(ax)
    plt.show()

    ani = sim.animate(fig,ax)
    plt.show()
