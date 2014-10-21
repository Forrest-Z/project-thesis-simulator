#!/usr/bin/env python

import numpy as np

from utils import State, Vessel, Map, Scenario, Simulation, VesselModel, Polygon
from ctrl_LOS_Guidance import LOSGuidance
from ctrl_hybrid_astar import HybridAStar

import matplotlib.pyplot as plt

if __name__ == "__main__":
    mymap = Map('s1')

    myvessel = Vessel('viknes')
    initial_state = np.array([0, 0, np.pi/12,
                              3.0, 0, 0])
    goal_state    = np.array([100, 100, np.pi/4])

    waypoints     = np.array([[0, 0], [60, 60], [100, 60], [100, 100]])

    myscenario   = Scenario(mymap, initial_state, goal_state)
    #myscenario.waypoints = waypoints

    myAstar = HybridAStar(myscenario)
    myLOS = LOSGuidance(myscenario)

    controllers = [myAstar, myLOS]

    myModel = VesselModel(initial_state, 0.1)

    mysimulation = Simulation(myscenario, myvessel, controllers,  myModel, 65)

    mysimulation.run_sim()

    fig, ax = plt.subplots()


    mysimulation.draw(ax)

    plt.show()
