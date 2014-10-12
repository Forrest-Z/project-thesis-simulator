#!/usr/bin/env python

from ctrl_hybrid_astar import hybrid_astar
from utils import State, Vessel, Map, Scenario, Simulation

import matplotlib.pyplot as plt

if __name__ == "__main__":
    mymap = Map('s1')

    myvessel = Vessel('other')
    start_state = State(0,0,0, gridsize=1)
    goal_state  = State(100,100,0)

    myscenario   = Scenario(mymap, start_state, goal_state)
    mysimulation = Simulation(myscenario, hybrid_astar, myvessel)

    mysimulation.run_sim()

    mysimulation.print_sim_time()

    fig, ax = plt.subplots()
    mysimulation.draw(ax)


    plt.show()
