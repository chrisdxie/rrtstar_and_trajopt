""" Plots obstacle scene
"""

import numpy as np
import matplotlib.pyplot as plt
import sys

start_state = np.array([0,0])
goal_state = np.array([9,9])

X_MIN = -10
X_MAX = 10
Y_MIN = -10
Y_MAX = 10

def plot_scene():

	args = sys.argv[1:]

	if len(args) == 0:
		print "Must pass in an obstacle file. Call this function like this:\n\npython plot_scene.py <OBSTACLE_FILE_NAME>\n"
		exit(0)

	obs_file = args[0]
	obstacles = np.loadtxt(obs_file)

	plot_obstacles(obstacles)

def plot_obstacles(obstacles):

    plt.clf()
    plt.cla()

    fig = plt.figure(1)
    ax = fig.axes[0]
    ax.set_axis_bgcolor('white')
    ax.set_aspect('equal')

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

    plt.plot([start_state[0]], [start_state[1]], 'bo', markersize=8)
    plt.plot([goal_state[0]], [goal_state[1]], 'go', markersize=8)

    plt.xlim([X_MIN, X_MAX])
    plt.ylim([Y_MIN, Y_MAX])

    plt.show(block=False)
    plt.pause(.1)

    raw_input()


if __name__ == '__main__':
	plot_scene()