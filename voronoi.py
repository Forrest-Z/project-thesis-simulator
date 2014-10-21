#!/usr/bin/env python

from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import numpy as np

from utils import Map, Polygon


if __name__ == "__main__":
    mymap = Map('s1')

    points = mymap.get_obstacle_edge_samples(5)

    vor = Voronoi(points)

    vedges = []
    for v in vor.vertices:
        if not mymap.is_occupied(v):
            vedges.append(v.tolist())
    fig, ax = plt.subplots()
    voronoi_plot_2d(vor, ax)

    v2 = np.array(vedges)

    ax.plot(v2[:,0], v2[:,1], 'r.')
    mymap.draw(ax, pcolor='g', ecolor='k')
    plt.show()
