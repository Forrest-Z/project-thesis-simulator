#!/usr/bin/env python
import time

import numpy as np
from numpy import linalg as LA

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from matplotlib.patches import Circle

from ctrl_LOS_Guidance import LOSGuidance
from ctrl_hybrid_astar import HybridAStar
from ctrl_DWA import DynamicWindow

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

