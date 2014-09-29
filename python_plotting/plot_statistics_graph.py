""" Plot statistics of files:

	Call it like this:

	$: python plot_statistics_graph.py <RRT*_STATS_FILE> <BIT*STATS_FILE> <SST_STATS_FILE>

	or 

	$: python plot_statistics_graph.py <BIT*_STATS_FILE> <SST_STATS_FILE>

	the number of arguments will determine which one to plot

"""

import numpy as np
import matplotlib.pyplot as plt
import os, sys

def main():

	args = sys.argv[1:]
	if len(args) == 3:
		plot_rrt_bit_sst_graphs(args)
	elif len(args) == 2:
		plot_bit_sst_graphs(args)

def plot_rrt_bit_sst_graphs(file_names):

	rrt_file = file_names[0]
	bit_file = file_names[1]
	sst_file = file_names[2]

	# Load the csv files
	rrt_stats = np.loadtxt(open(rrt_file,"rb"),delimiter=", ") 
	bit_stats = np.loadtxt(open(bit_file,"rb"),delimiter=", ") 
	sst_stats = np.loadtxt(open(sst_file,"rb"),delimiter=", ") 

	# Transpose them
	rrt_stats = rrt_stats.transpose()
	bit_stats = bit_stats.transpose()
	sst_stats = sst_stats.transpose()

	# Find where they start having solutions
	num_rrt_no_sols = rrt_stats[rrt_stats == -1].size; 
	num_bit_no_sols = bit_stats[bit_stats == 1e10].size; 
	num_sst_no_sols = sst_stats[sst_stats == 0].size; 

	# Remove any thing that has no solution
	rrt_stats = rrt_stats[:, num_rrt_no_sols:-1]
	bit_stats = bit_stats[:, num_bit_no_sols:-1]
	sst_stats = sst_stats[:, num_sst_no_sols:-1]

	# Plot
	plt.plot(rrt_stats[0,:], rrt_stats[2,:], color='b', linewidth=3.5, label='RRT*')
	plt.plot(bit_stats[0,:], bit_stats[2,:], color='g', linewidth=3.5, label='BIT* + TrajOpt')
	plt.plot(sst_stats[0,:], sst_stats[2,:], color='r', linewidth=3.5, label='SST')

	plt.xlabel('Time in seconds')
	plt.ylabel('Cost of best solution')
	plt.legend()

	#plt.xlim(0,600)

	plt.show(block=False)
	raw_input("What do you think?\n")




def plot_bit_sst_graphs(file_names):

	bit_file = file_names[1]
	sst_file = file_names[2]

	# Load the csv files
	bit_stats = np.loadtxt(open(bit_file,"rb"),delimiter=", ") 
	sst_stats = np.loadtxt(open(sst_file,"rb"),delimiter=", ") 

	# Transpose them
	bit_stats = bit_stats.transpose()
	sst_stats = sst_stats.transpose()

	# Find where they start having solutions
	num_bit_no_sols = bit_stats[bit_stats == 1e10].size; 
	num_sst_no_sols = sst_stats[sst_stats == 0].size; 

	# Remove any thing that has no solution
	bit_stats = bit_stats[:, num_bit_no_sols:-1]
	sst_stats = sst_stats[:, num_sst_no_sols:-1]

	# Plot
	plt.plot(bit_stats[0,:], bit_stats[2,:], color='g', linewidth=2, label='BIT* + TrajOpt')
	plt.plot(sst_stats[0,:], sst_stats[2,:], color='r', linewidth=2, label='SST')

	plt.show()
	raw_input("What do you think?")

if __name__ == '__main__':
	main()
