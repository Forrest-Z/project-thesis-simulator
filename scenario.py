#!/usr/bin/env python

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import animation

from map import Map
from vessel import Vessel
from world import World
from simulation import Simulation

from ctrl_DWA import DynamicWindow
from ctrl_hybrid_astar import HybridAStar
from ctrl_LOS_Guidance import LOSGuidance
from ctrl_PotField import PotentialFields

class Scenario(object):
    def __init__(self, name='s1'):
        if name == 's1':
            the_map = Map('s1')

            self.tend = 150
            self.h    = 0.05
            self.dT   = 0.5
            self.N    = int(np.around(self.tend / self.h)) +1

            # Vessel 1 (Main vessel)
            x01 = np.array([0, 0, 0, 1.0, 0, 0])
            xg1 = np.array([100, 100, np.pi/4])

            myLOS1 = LOSGuidance()

            v1 = Vessel(x01, xg1, self.h, self.dT, self.N, [myLOS1], is_main_vessel=True, vesseltype='viknes')
            v1.waypoints = np.array([[0, 0], [50, 60], [70, 60], [100, 40], [100, 100]])
            #v1.waypoints = np.array([[0, 0], [140, 0], [120, 120]])

            # Vessel 2
            x02 = np.array([0, 120, 0, 3.0, 0, 0])
            xg2 = np.array([140, 0, 0])

            myLOS2 = LOSGuidance()

            v2 = Vessel(x02, xg2, self.h, self.dT, self.N, [myLOS2], is_main_vessel=False, vesseltype='viknes')
            v2.waypoints = np.array([[0, 120], [120, 120], [140, 0]])

            # Vessel 3
            x03 = np.array([120, 0, 0, 3.0, 0, 0])
            xg3 = np.array([0, 0, 0])

            myLOS3 = LOSGuidance()

            v3 = Vessel(x03, xg3, self.h, self.dT, self.N, [myLOS3], is_main_vessel=False, vesseltype='viknes')
            v3.waypoints = np.array([[120, 0], [0, 0], [0, 120]])

            self.world = World([v1, v2], the_map)

        elif name == 'collision':
            the_map = Map('s1')

            self.tend = 100
            self.h    = 0.05
            self.dT   = 0.5
            self.N    = int(np.around(self.tend / self.h)) + 1
            self.h    = 0.05
            # Vessel 1 (Main vessel)
            x01 = np.array([0, 0, 0, 3.0, 0, 0])
            xg1 = np.array([120, 120, np.pi/4])

            myLOS1 = LOSGuidance()
            myAstar = HybridAStar(x01, xg1, the_map)

            v1 = Vessel(x01, xg1,self.h, self.dT, self.N, [myLOS1], is_main_vessel=True, vesseltype='viknes')
            v1.waypoints = np.array([[0, 0], [50, 60], [70, 60], [120, 10], [120, 120]])
            #v1.waypoints = np.array([[0, 0], [140, 0], [120, 120]])

            # Vessel 2
            x02 = np.array([0, 120, 0, 3.0, 0, 0])
            xg2 = np.array([120, 0, 0])

            myLOS2 = LOSGuidance()

            v2 = Vessel(x02, xg2,self.h, self.dT, self.N, [myLOS2], is_main_vessel=False, vesseltype='viknes')
            v2.waypoints = np.array([[0, 120], [120, 120], [120, 0]])

            # Vessel 3
            x03 = np.array([0, 50, np.pi/2, 3.0, 0, 0])
            xg3 = np.array([140, 0, 0])

            myLOS3 = LOSGuidance()

            v3 = Vessel(x03, xg3, self.h, self.dT, self.N, [myLOS3], is_main_vessel=False, vesseltype='viknes')
            v3.waypoints = np.array([[0, 50], [0, 120], [120, 120]])


            self.world = World([v1, v2], the_map)

        elif name == 'dynwnd':
            the_map = Map('polygon', gridsize=0.5,safety_region_length=4.5)

            self.tend = 100   # Simulation time (seconds)
            self.h    = 0.05 # Integrator time step
            self.dT   = 0.5  # Controller time step
            self.N    = int(np.around(self.tend / self.h)) + 1

            # Vessel 1 (Main vessel)
            x01 = np.array([5, 5, np.pi/12, 3.0, 0, 0])
            xg1 = np.array([120, 120, np.pi/4])

            myLOS1 = LOSGuidance()
            myDynWnd = DynamicWindow(self.dT, int(self.tend/self.dT) + 1)

            v1 = Vessel(x01, xg1, self.h, self.dT, self.N, [myDynWnd], is_main_vessel=True, vesseltype='viknes')
            v1.waypoints = np.array([[0, 0], [120, 120]])
            #v1.waypoints = np.array([[0, 0], [140, 0], [120, 120]])

            # Vessel 2
            #x02 = np.array([120, 120, -3*np.pi/4, 3.0, 0, 0])
            #xg2 = np.array([140, 0, 0])

            #myLOS2 = LOSGuidance()

            #v2 = Vessel(x02, xg2, self.h, self.dT, self.N, [myLOS2], is_main_vessel=False, vesseltype='viknes')
            #v2.waypoints = np.array([[120, 120], [0, 0]])

            self.world = World([v1], the_map)

            myDynWnd.the_world = self.world

        elif name == 'dwacollision':
            the_map = Map(gridsize=0.5,safety_region_length=4.5)

            self.tend = 60   # Simulation time (seconds)
            self.h    = 0.05 # Integrator time step
            self.dT   = 0.5  # Controller time step
            self.N    = int(np.around(self.tend / self.h)) + 1

            # Vessel 1 (Main vessel)
            x01 = np.array([5, 5, np.pi/4, 3.0, 0, 0])
            xg1 = np.array([120, 120, np.pi/4])

            myLOS1 = LOSGuidance()
            myDynWnd = DynamicWindow(self.dT, int(self.tend/self.dT) + 1)

            v1 = Vessel(x01, xg1, self.h, self.dT, self.N, [myDynWnd], is_main_vessel=True, vesseltype='viknes')
            v1.waypoints = np.array([[0, 0], [120, 120]])
            #v1.waypoints = np.array([[0, 0], [140, 0], [120, 120]])

            # Vessel 2
            x02 = np.array([80, 80, -3*np.pi/4, 3.0, 0, 0])
            xg2 = np.array([0, 0, -3*np.pi/4])

            myLOS2 = LOSGuidance()

            v2 = Vessel(x02, xg2, self.h, self.dT, self.N, [myLOS2], is_main_vessel=False, vesseltype='viknes')
            v2.waypoints = np.array([[120, 120], [0, 0]])

            self.world = World([v1, v2], the_map)

            myDynWnd.the_world = self.world

        elif name == 'potfield':
            the_map = Map('s1', gridsize=0.5, safety_region_length=4.0)

            self.tend = 140.0
            self.dT   = 0.5
            self.h    = 0.1
            self.N    = int(np.around(self.tend/self.h)) + 1

            N2   = int(np.around(self.tend/self.dT)) + 1
            x0   = np.array([5,5,0, 2.0,0,0])
            xg   = np.array([120, 120, 0])

            potfield = PotentialFields(the_map, N2)

            vobj = Vessel(x0, xg, self.h, self.dT, self.N, [potfield], is_main_vessel=True, vesseltype='viknes')

            self.world = World([vobj], the_map)

        else:
            print "You might have spelled the scenario name wrong..."
            self.tend = 0
            self.dT = 0.1
            self.h  = 0.05
            self.N  = 0
            self.world = None

    def is_occupied(self, point):
        return self.map.is_occupied(point)

    def draw(self, axes, n, scolor='b', fcolor='r', ocolor='g', ecolor='k'):
        self.world.draw(axes, n)

    def animate(self, fig, axes, n):
        return self.world.animate(fig, axes, n)

def harry_plotter(sim):
    fig = plt.figure()
    ax  = fig.add_subplot(111, autoscale_on=False)

    #ani = sim.animate(fig, ax)
    sim.draw(ax)

    ax.axis('scaled')
    ax.set_xlim((-10, 160))
    ax.set_ylim((-10, 160))
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')

    ax.grid()
#    plt.tight_layout()
    plt.show()

def harry_anim(sim):
    fig2 = plt.figure()
    ax2  = fig2.add_subplot(111, autoscale_on=False, )


    ani = sim.animate(fig2,ax2)
    ax2.axis('scaled')
    ax2.set_xlim((-10, 160))
    ax2.set_ylim((-10, 160))
    ax2.set_xlabel('x [m]')
    ax2.set_ylabel('y [m]')

    ax2.grid()

    #Writer = animation.writers['ffmpeg']
    #writer = Writer(fps=33, metadata=dict(artist='Thomas Stenersen'), bitrate=1800)
    #ani.save('head_on_situation.mp4', writer=writer)
    plt.show()

if __name__ == "__main__":
    #ani = sim.animate(fig, ax)

    scen = Scenario('potfield')
    sim  = Simulation(scen)


    sim.run_sim()

    harry_plotter(sim)






