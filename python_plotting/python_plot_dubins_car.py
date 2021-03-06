""" Plots best trajectory output. This file only plots the states of the trajectory. """

import numpy as np
import matplotlib.pyplot as plt
from math import pi, atan


def plot(states, obstacles, goal_region):

    plt.clf()
    plt.cla()

    fig = plt.figure(1)
    ax = fig.axes[0]
    ax.set_axis_bgcolor('white')
    ax.set_aspect('equal')

    # Plot goal_region
    if len(goal_region.shape) == 1: # If it is a 1D array, make it a 2D column array
        goal_region = np.transpose([goal_region])

    x_center = goal_region[0,0]
    x_size = goal_region[1,0] 
    y_center = goal_region[2,0]
    y_size = goal_region[3,0]

    # [x_min, x_min, x_max, x_max]
    x_min = x_center - .5*x_size; x_max = x_center + .5*x_size;
    x = [x_min, x_min, x_max, x_max]
    # [y_min, y_max, y_max, y_min]
    y_min = y_center - .5*y_size; y_max = y_center + .5*y_size;
    y = [y_min, y_max, y_max, y_min]

    ax.fill(x, y, 'g')

    # Plot obstacles
    if len(obstacles.shape) == 1: # If it is a 1D array, make it a 2D column array
        obstacles = np.transpose([obstacles])

    num_obstacles = obstacles.shape[1]

    for i in range(num_obstacles):
        x_center = obstacles[0,i]
        x_size = obstacles[1,i] 
        y_center = obstacles[2,i]
        y_size = obstacles[3,i]

        # Compute corners of obstacle region 
        # [x_min, x_min, x_max, x_max]
        x_min = x_center - .5*x_size; x_max = x_center + .5*x_size;
        x = [x_min, x_min, x_max, x_max]
        # [y_min, y_max, y_max, y_min]
        y_min = y_center - .5*y_size; y_max = y_center + .5*y_size;
        y = [y_min, y_max, y_max, y_min]

        ax.fill(x, y, 'r')

    # Plot sequence of states

    plt.plot(states[0,:], states[1,:], color='b')
    for i in range(states.shape[1]):
    	# since the output is in radians, I convert it to degrees
    	# the -90 is because I want the tip of the triangle to start facing the right
    	# as in 0 degrees of the unit circle.
        plt.plot(states[0,i], states[1,i], marker=(3,0,states[2,i]*180/pi-90), markersize=10, color='b')

    plt.show(block=False)
    plt.pause(.1)
    raw_input("Press Enter to continue")
