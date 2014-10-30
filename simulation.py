#!/usr/bin/env python

import numpy as np

class Simulation(object):
    """This class combines the different classes needed to simulate a scenario.

    Args:
        scenario   (class): a Scenario object (contains map, etc.)
        vessel     (class): the vessel to be used (e.g. the Viknes 830)
        controllers (list): list of controller types (A*, LOS, etc.)
        model      (class): the mathematical model for the vessel
                                :todo: make vessel the parent for this class

    Kwargs:
        tend       (float): The end time of the simulation (default 100)

    """
    def __init__(self, scenario):
        """Initializes the simulation."""
        self.scenario = scenario

        self.tend = self.scenario.tend
        self.N    = self.scenario.N
        self.n    = 0

        self._end_reason = "SimOK"

    def run_sim(self):
        """Runs the simulation."""

        print "Starting simulation"
        for t in np.linspace(0, self.tend, self.N):
            """The actual simulation"""

            self.scenario.world.update_world(t, self.n, self.scenario.dT)
            if self.scenario.world.collision_detection():
                print "We have crashed!"
                self._end_reason = "Collision"
                break

            self.n += 1

    def draw(self, axes):
        """Draw the simulation.

        Plots the result of the simulation.

        Args:
            axes (class): the matplotlib axes to draw to.
        """
        self.scenario.draw(axes, self.n)

    def animate(self, fig, axes):
        return self.scenario.animate(fig, axes, self.n)

    def print_sim_time(self):
        """Prints the time taken to simulate the scenario.
        :todo: implement
        """
        print "Simulation time: %.3f seconds!" % (self.sim_time)
