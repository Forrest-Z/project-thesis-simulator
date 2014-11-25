#!/usr/bin/env python

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import gridspec
from mpl_toolkits.mplot3d import Axes3D

from map import Map
from vessel import Vessel
from world import World
from simulation import Simulation

from ctrl_DWA import DynamicWindow
from ctrl_hybrid_astar import HybridAStar
from ctrl_LOS_Guidance import LOSGuidance
from ctrl_PotField import PotentialFields
from ctrl_astar import AStar
from ctrl_purepursuit import PurePursuit
from ctrl_constant_bearing import ConstantBearing

from matplotlib2tikz import save as tikz_save

class Scenario(object):
    def __init__(self, name='s1'):
        self.name = name

        if name == 's1':
            the_map = Map('s1', gridsize=2.0, safety_region_length=4.0)

            self.tend = 150
            self.h    = 0.05
            self.dT   = 0.5
            self.N    = int(np.around(self.tend / self.h)) +1

            # Vessel 1 (Main vessel)
            x01 = np.array([0, 0, 0, 1.0, 0, 0])
            xg1 = np.array([150, 150, np.pi/4])

            myastar = AStar(x01, xg1, the_map)
            mypp    = PurePursuit()

            v1 = Vessel(x01, xg1, self.h, self.dT, self.N, [myastar, mypp], is_main_vessel=True, vesseltype='viknes')

            self.world = World([v1], the_map)

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

        elif name == 's1-dynwnd':
            the_map = Map('s1', gridsize=0.5,safety_region_length=4.0)

            self.tend = 80   # Simulation time (seconds)
            self.h    = 0.05 # Integrator time step
            self.dT   = 0.5  # Controller time step
            self.N    = int(np.around(self.tend / self.h)) + 1

            # Vessel 1 (Main vessel)
            x01 = np.array([0.0, 0.0, 0.0, 2.5, 0, 0])
            xg1 = np.array([140, 140, 0])

            myDynWnd = DynamicWindow(self.dT, int(self.tend/self.dT) + 1, the_map)

            v1 = Vessel(x01, xg1, self.h, self.dT, self.N, [myDynWnd], is_main_vessel=True, vesseltype='viknes')
            v1.goal = np.array([140, 140, 0])

            self.world = World([v1], the_map)

            myDynWnd.the_world = self.world

        elif name == 's1-potfield':
            the_map = Map('s1', gridsize=0.5, safety_region_length=4.0)

            self.tend = 140.0
            self.dT   = 0.5
            self.h    = 0.05
            self.N    = int(np.around(self.tend/self.h)) + 1

            N2   = int(np.around(self.tend/self.dT)) + 1
            x0   = np.array([0, 0, 0, 2.5, 0, 0])
            xg   = np.array([140, 140, 0])

            potfield = PotentialFields(the_map, N2)

            vobj = Vessel(x0, xg, self.h, self.dT, self.N, [potfield], is_main_vessel=True, vesseltype='viknes')

            self.world = World([vobj], the_map)

        elif name=='s1-hybridastar':
            the_map = Map('s1', gridsize=2.0, safety_region_length=4.0)

            self.tend = 120.0
            self.dT   = 0.5
            self.h    = 0.05
            self.N    = int(np.around(self.tend/self.h)) + 1

            N2   = int(np.around(self.tend/self.dT)) + 1
            x0 = np.array([0, 0, 0.0, 2.5, 0.0, 0])
            xg = np.array([140, 140, 0.0])

            hastar = HybridAStar(x0, xg, the_map)
            pp    = PurePursuit(R2=50)

            vobj = Vessel(x0, xg, self.h, self.dT, self.N, [hastar, pp], is_main_vessel=True, vesseltype='viknes')

            self.world = World([vobj], the_map)

        elif name=='s1-astar':
            the_map = Map('s1', gridsize=2.0, safety_region_length=4.0)

            self.tend = 120.0
            self.dT   = 0.5
            self.h    = 0.05
            self.N    = int(np.around(self.tend/self.h)) + 1

            N2   = int(np.around(self.tend/self.dT)) + 1
            x0 = np.array([0, 0, 0.0, 2.5, 0.0, 0])
            xg = np.array([140, 140, np.pi/4])

            astar = AStar(x0, xg, the_map)
            pp    = PurePursuit(R2=50)

            vobj = Vessel(x0, xg, self.h, self.dT, self.N, [astar, pp], is_main_vessel=True, vesseltype='viknes')

            self.world = World([vobj], the_map)


        elif name=='s2-potfield':
            the_map = Map('s2', gridsize=1.0, safety_region_length=4.0)

            self.tend = 80.0
            self.dT   = 0.5
            self.h    = 0.05
            self.N    = int(np.around(self.tend/self.h)) + 1

            N2   = int(np.around(self.tend/self.dT)) + 1
            x0   = np.array([0, 0, 0, 2.5, 0, 0])
            xg   = np.array([140, 140, 0])

            potfield = PotentialFields(the_map, N2)

            vobj = Vessel(x0, xg, self.h, self.dT, self.N, [potfield], is_main_vessel=True, vesseltype='viknes')

            self.world = World([vobj], the_map)

        elif name == 's2-dynwnd':
            the_map = Map('s2', gridsize=0.5,safety_region_length=4.0)

            self.tend = 80   # Simulation time (seconds)
            self.h    = 0.05 # Integrator time step
            self.dT   = 0.5  # Controller time step
            self.N    = int(np.around(self.tend / self.h)) + 1

            # Vessel 1 (Main vessel)
            x01 = np.array([0.0, 0.0, 0.0, 2.5, 0, 0])
            xg1 = np.array([140, 140, 0])

            myDynWnd = DynamicWindow(self.dT, int(self.tend/self.dT) + 1)

            v1 = Vessel(x01, xg1, self.h, self.dT, self.N, [myDynWnd], is_main_vessel=True, vesseltype='viknes')
            v1.goal = np.array([140, 140, 0])

            self.world = World([v1], the_map)

            myDynWnd.the_world = self.world

        elif name=='s2-astar':
            the_map = Map('s2', gridsize=2.0, safety_region_length=4.0)

            self.tend = 120.0
            self.dT   = 0.5
            self.h    = 0.05
            self.N    = int(np.around(self.tend/self.h)) + 1

            N2   = int(np.around(self.tend/self.dT)) + 1
            x0 = np.array([0, 0, 0.0, 2.5, 0.0, 0])
            xg = np.array([140, 140, np.pi/4])

            astar = AStar(x0, xg, the_map)
            pp    = PurePursuit(R2=50)

            vobj = Vessel(x0, xg, self.h, self.dT, self.N, [astar, pp], is_main_vessel=True, vesseltype='viknes')

            self.world = World([vobj], the_map)

        elif name=='s2-hybridastar':
            the_map = Map('s2', gridsize=2.0, safety_region_length=4.0)

            self.tend = 120.0
            self.dT   = 0.5
            self.h    = 0.05
            self.N    = int(np.around(self.tend/self.h)) + 1

            N2   = int(np.around(self.tend/self.dT)) + 1
            x0 = np.array([0, 0, 0.0, 2.5, 0.0, 0])
            xg = np.array([140, 140, 0.0])

            hastar = HybridAStar(x0, xg, the_map)
            pp    = PurePursuit(R2=50)

            vobj = Vessel(x0, xg, self.h, self.dT, self.N, [hastar, pp], is_main_vessel=True, vesseltype='viknes')

            self.world = World([vobj], the_map)

        elif name=='s3-potfield':
            the_map = Map('s1', gridsize=1.0, safety_region_length=4.0)

            self.tend = 140.0
            self.dT   = 0.5
            self.h    = 0.05
            self.N    = int(np.around(self.tend/self.h)) + 1

            N2   = int(np.around(self.tend/self.dT)) + 1
            x0   = np.array([0, 0, 0, 2.5, 0, 0])
            xg   = np.array([140, 140, 0])

            potfield = PotentialFields(the_map, N2)
            vobj = Vessel(x0, xg, self.h, self.dT, self.N, [potfield], is_main_vessel=True, vesseltype='viknes')

            # Other vessel
            xo0 = np.array([0,120,0,2.5,0,0])
            xog = np.array([140,0,0])

            los = LOSGuidance()
            vobj2 = Vessel(xo0, xog, self.h, self.dT, self.N, [los], is_main_vessel=False, vesseltype='viknes')
            vobj2.waypoints = np.array([[0,120], [60,120], [90, 0], [140,0]])

            self.world = World([vobj, vobj2], the_map)

            potfield.world = self.world

        elif name == 's3-dynwnd':
            the_map = Map('', gridsize=1.0, safety_region_length=4.0)

            self.tend = 80   # Simulation time (seconds)
            self.h    = 0.05 # Integrator time step
            self.dT   = 0.5  # Controller time step
            self.N    = int(np.around(self.tend / self.h)) + 1

            # Vessel 1 (Main vessel)
            x01 = np.array([60.0, 0.0, 3.14/4, 2.5, 0, 0])
            xg1 = np.array([80, 145, 0])

            myDynWnd = DynamicWindow(self.dT, int(self.tend/self.dT) + 1, the_map)

            vobj = Vessel(x01, xg1, self.h, self.dT, self.N, [myDynWnd], is_main_vessel=True, vesseltype='viknes')
            #v1.goal = np.array([140, 140, 0])

            # Other vessel
            xo0 = np.array([40.,60,-np.pi/4,1.5,0,0])
            xog = np.array([250,110,0])

            cb  = ConstantBearing(vobj.x)
            vobj2 = Vessel(xo0, xog, self.h, self.dT, self.N, [cb], is_main_vessel=False, vesseltype='hurtigruta')
            #vobj2.waypoints = np.array([[0,100], [150,110]])

            # Follower
            x0f = np.array([100.,140,-np.pi,1.5,0,0])
            xgf = np.array([250,110,0])

            pp = PurePursuit(mode='pursuit')
            pp.cGoal = vobj.x
            vobj3 = Vessel(x0f, xgf, self.h, self.dT, self.N, [pp], is_main_vessel=False, vesseltype='viknes')
            vobj3.u_d = 2.5

            self.world = World([vobj, vobj2, vobj3], the_map)

            myDynWnd.world = self.world

        elif name == 'hastar+dynwnd':
            the_map = Map('s2', gridsize=1.0, safety_region_length=4.0)

            self.tend = 100   # Simulation time (seconds)
            self.h    = 0.05 # Integrator time step
            self.dT   = 0.5  # Controller time step
            self.N    = int(np.around(self.tend / self.h)) + 1

            # Vessel 1 (Main vessel)
            x01 = np.array([0.0, 0.0, 3.14/4, 2.5, 0, 0])
            xg1 = np.array([80, 145, 0])

            hastar   = HybridAStar(x01, xg1, the_map)
            pp       = PurePursuit(mode='goal-switcher')
            myDynWnd = DynamicWindow(self.dT, int(self.tend/self.dT) + 1, the_map)

            vobj = Vessel(x01, xg1, self.h, self.dT, self.N, [hastar, pp, myDynWnd], is_main_vessel=True, vesseltype='viknes')


            xo0 = np.array([50.,130,5*np.pi/4,0.0,0,0])
            xog = np.array([250,110,0])


            vobj2 = Vessel(xo0, xog, self.h, self.dT, self.N, [], is_main_vessel=False, vesseltype='hurtigruta')


            self.world = World([vobj, vobj2], the_map)
            myDynWnd.world = self.world

        elif name == 'cb-test':
            the_map = Map('', gridsize=1.0, safety_region_length=4.0)

            self.tend = 80   # Simulation time (seconds)
            self.h    = 0.05 # Integrator time step
            self.dT   = 0.5  # Controller time step
            self.N    = int(np.around(self.tend / self.h)) + 1

            # Vessel 1 (Main vessel)
            x01 = np.array([60.0, 0.0, 3.14/4, 2.5, 0, 0])
            xg1 = np.array([80, 145, 0])

            pp = PurePursuit()

            vobj = Vessel(x01, xg1, self.h, self.dT, self.N, [pp], is_main_vessel=True, vesseltype='viknes')

            # Other vessel
            xo0 = np.array([40.,60,-np.pi/4,1.5,0,0])
            xog = np.array([250,110,0])

            cb  = ConstantBearing(vobj.x)

            vobj2 = Vessel(xo0, xog, self.h, self.dT, self.N, [cb], is_main_vessel=False, vesseltype='hurtigruta')

            self.world = World([vobj, vobj2], the_map)


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

            myLOS2 = LOSGuidance(u_d = 2.0)

            v2 = Vessel(x02, xg2, self.h, self.dT, self.N, [myLOS2], is_main_vessel=False, vesseltype='viknes')
            v2.waypoints = np.array([[120, 120], [0, 0]])

            self.world = World([v1, v2], the_map)

            myDynWnd.the_world = self.world


        elif name=='hybridastar':
            the_map = Map('s1',gridsize=1.0, safety_region_length=6.0)

            self.tend = 140.0
            self.dT   = 0.5
            self.h    = 0.05
            self.N    = int(np.around(self.tend/self.h)) + 1

            N2   = int(np.around(self.tend/self.dT)) + 1
            x0 = np.array([0, 0, 0.0, 2.5, 0.0, 0])
            xg = np.array([130, 130, 8.0])

            hastar = HybridAStar(x0, xg, the_map)
            pp     = PurePursuit(R2=50, mode="goal-switcher")

            dynwnd = DynamicWindow(self.dT, N2, the_map)
            dynwnd.alpha = 0.8
            dynwnd.beta  = 0.1
            dynwnd.gamma = 0.1

            ptf    = PotentialFields(the_map, N2)
            ptf.mu    = 10
            ptf.d_max = 30
            ptf.k     = 10.

            vobj = Vessel(x0, xg, self.h, self.dT, self.N, [hastar, pp, dynwnd], is_main_vessel=True, vesseltype='viknes')

            self.world = World([vobj], the_map)
            dynwnd.the_world = self.world
            ptf.world = self.world
        elif name=='minima':
            the_map = Map('minima', gridsize=0.5, safety_region_length=4.0)

            self.tend = 120.0
            self.dT   = 0.5
            self.h    = 0.1
            self.N    = int(np.around(self.tend/self.h)) + 1

            N2   = int(np.around(self.tend/self.dT)) + 1
            x0   = np.array([30,30,0, 2.0,0,0])
            xg   = np.array([140, 140, 3*np.pi/4])

            hastar = HybridAStar(x0, xg, the_map)
            los    = LOSGuidance()

            vobj = Vessel(x0, xg, self.h, self.dT, self.N, [hastar, los], is_main_vessel=True, vesseltype='viknes')

            self.world = World([vobj], the_map)

        elif name=='pptest':
            the_map = Map('', gridsize=0.5, safety_region_length=4.0)

            self.tend = 120.0
            self.dT   = 0.5
            self.h    = 0.1
            self.N    = int(np.around(self.tend/self.h)) + 1

            N2   = int(np.around(self.tend/self.dT)) + 1
            x0   = np.array([0,0,0, 2.0,0,0])
            xg   = np.array([140, 140, 3*np.pi/4])

            pp   = PurePursuit()

            vobj = Vessel(x0, xg, self.h, self.dT, self.N, [pp], is_main_vessel=True, vesseltype='viknes')
            vobj.waypoints = np.array([(50.,50.), (50., 0.), (100., 100.)])
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

#    ax.grid()
#    plt.tight_layout()

    tikz_save('../../../latex/fig/'+scen.name+'.tikz',
              figureheight='1.5\\textwidth',
              figurewidth='1.5\\textwidth')
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

    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=33, metadata=dict(artist='Thomas Stenersen'), bitrate=1800)
    ani.save('guided_dyn_wnd_2.mp4', writer=writer)
    plt.show()

if __name__ == "__main__":
    plt.ion()

    fig = plt.figure()

    gs    = gridspec.GridSpec(2,4)
    axarr = [fig.add_subplot(gs[:,0:2], autoscale_on=False)]

    axarr[0].axis('scaled')
    axarr[0].set_xlim((-10, 160))
    axarr[0].set_ylim((-10, 160))
    axarr[0].set_xlabel('x [m]')
    axarr[0].set_ylabel('y [m]')

    axarr += [fig.add_subplot(gs[0, 2], projection='3d'),
              fig.add_subplot(gs[1, 2], projection='3d'),
              fig.add_subplot(gs[0, 3], projection='3d'),
              fig.add_subplot(gs[1, 3], projection='3d')]
        #axarr[ii+1].set_aspect('equal')

    scen = Scenario('s3-dynwnd')
    #sim  = Simulation(scen)
    sim  = Simulation(scen, fig, axarr)

    sim.run_sim()

    plt.show()
    #harry_anim(sim)






