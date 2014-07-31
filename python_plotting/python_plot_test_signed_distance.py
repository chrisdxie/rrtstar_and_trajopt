""" Plots best trajectory output. This file only plots the states of the trajectory. """

import numpy as np
import matplotlib.pyplot as plt


def plot(t1, t2, points):

    plt.clf()
    plt.cla()

    fig = plt.figure(1)
    ax = fig.axes[0]
    ax.set_axis_bgcolor('white')
    ax.set_aspect('equal')

    # Plot polygons
    nV = t1.shape[0]
    for i in range(nV):
        plt.plot([t1[i,0], t1[(i+1)%nV,0]],[t1[i,1], t1[(i+1)%nV,1]], color='b')

    nV = t2.shape[0]
    for i in range(nV):
        plt.plot([t2[i,0], t2[(i+1)%nV,0]],[t2[i,1], t2[(i+1)%nV,1]], color='b')

    # Plot signed distance line
    dist_val = points[0,0]
    if dist_val < 0:
        c = 'r'
    else:
        c = 'g'

    print t1
    print t2
    print points

    # Plot points as circles
    plt.plot(points[1,0], points[1,1], marker='o', color=c,  markersize=6)
    plt.plot(points[2,0], points[2,1], marker='o', color=c, markersize=6)

    # Plot line between them
    plt.plot([points[1,0], points[2,0]], [points[1,1], points[2,1]], color=c, linewidth=3)

    plt.show(block=False)
    plt.pause(.1)
    raw_input("Press Enter to continue")
