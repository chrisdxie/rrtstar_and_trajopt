""" Plots best trajectory output. This file only plots the states of the trajectory. """

import numpy as np
import matplotlib.pyplot as plt
from math import pi

def main():

    plt.clf()
    plt.cla()

    fig = plt.figure(1)
    ax = fig.axes[0]
    ax.set_axis_bgcolor('white')
    ax.set_aspect('equal')

    # Hard coding the obstacle and goal region as taken from the dubins car example
    obstacle = np.array([[2.5, 2.5, 7.5, 7.5],[2.5, 7.5, 7.5, 2.5]])
    goal_region = np.array([[7,7,9,9],[7,9,9,7]])

    # Fill in goal region first, so obstacle is shown to cover part of goal
    ax.fill(goal_region[0,:], goal_region[1,:], 'g')
    ax.fill(obstacle[0,:], obstacle[1,:], 'r')

    # Read in states file
    states = np.loadtxt('best_trajectory_states.txt')
    states = np.transpose(states)

    plt.plot(states[0,:], states[1,:], color='b')


    for i in range(states.shape[1]):
    	# since the output is in radians, I convert it to degrees
    	# the -90 is because I want the tip of the triangle to start facing the right
    	# as in 0 degrees of the unit circle.
    	plt.plot(states[0,i], states[1,i], marker=(3,0,states[2,i]*180/pi-90), markersize=10, color='b')

    plt.show(block=False)
    plt.pause(.1)
    raw_input("Press Enter to continue")


if __name__ == '__main__':
	main()