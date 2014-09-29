""" Generates a random scene and plots it. Asks to save obstacles as a matrix.

"""

import numpy as np
import matplotlib.pyplot as plt
import os

from random import randint, random

# Only worry about configuration for double integrator
start_state = np.array([0,0])
goal_state = np.array([9,9])

X_MIN = -10
X_MAX = 10
Y_MIN = -10
Y_MAX = 10

def uniform(a, b):
	return random()*(b-a) + a

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

    pause = True
    if (pause):
        print("Type 'save <file_name>' to save this figure into a pdf file as 'file_name.pdf'")
        print("Otherwise, press Enter to continue.")
        user_says = raw_input()
        user_says = user_says.strip().split()
        if user_says[0] == 'save':
            print "Obstacles:"
            print obstacles
            np.savetxt('{0}'.format(user_says[1]), obstacles)


def generate_obstacle():
	x_center = uniform(X_MIN, X_MAX)
	y_center = uniform(Y_MIN, Y_MAX)
	x_size = uniform(1, 3)
	y_size = uniform(1, 3)
	return np.array([x_center, x_size, y_center, y_size])

def blocking_start_or_goal(obs):
    x_center = obs[0]
    x_size = obs[1] 
    y_center = obs[2]
    y_size = obs[3]

    # Compute corners of obstacle region 
    # [x_min, x_min, x_max, x_max]
    x_min = x_center - .5*x_size; x_max = x_center + .5*x_size;
    x = [x_min, x_min, x_max, x_max]
    # [y_min, y_max, y_max, y_min]
    y_min = y_center - .5*y_size; y_max = y_center + .5*y_size;
    y = [y_min, y_max, y_max, y_min]

    if (x_min <= start_state[0] and start_state[0] <= x_max and y_min <= start_state[1] and start_state[1] <= y_max) or \
       (x_min <= goal_state[0] and goal_state[0] <= x_max and y_min <= goal_state[1] and goal_state[1] <= y_max):
    	return True
    else:
    	return False

def generate_random_scene():

	num_obstacles = randint(20, 30)
	obstacles = np.zeros([4, num_obstacles])

	for i in range(num_obstacles):

		obs = generate_obstacle();
		while blocking_start_or_goal(obs):
			obs = generate_obstacle();
		# End while loop

		obstacles[:,i] = obs

	# End for loop

	# Plot scene
	plot_obstacles(obstacles)

if __name__ == '__main__':
	generate_random_scene()
 