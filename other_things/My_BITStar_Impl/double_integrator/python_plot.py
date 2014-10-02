import numpy as np
import matplotlib.pyplot as plt
import os

from math import pi, atan

#import IPython

def plot(states, parents, goal_path, obstacles, seconds, cost):
    #return
    #print goal_path
    
    plt.clf()
    plt.cla()

    fig = plt.figure(1)
    ax = fig.axes[0]
    ax.set_axis_bgcolor('white')
    ax.set_aspect('equal')

    # Plot goal_region
    # if len(goal_region.shape) == 1: # If it is a 1D array, make it a 2D column array
    #     goal_region = np.transpose([goal_region])

    # x_center = goal_region[0,0]
    # x_size = goal_region[1,0] 
    # y_center = goal_region[2,0]
    # y_size = goal_region[3,0]

    # # [x_min, x_min, x_max, x_max]
    # x_min = x_center - .5*x_size; x_max = x_center + .5*x_size;
    # x = [x_min, x_min, x_max, x_max]
    # # [y_min, y_max, y_max, y_min]
    # y_min = y_center - .5*y_size; y_max = y_center + .5*y_size;
    # y = [y_min, y_max, y_max, y_min]

    # ax.fill(x, y, 'c')

    # Plot obstacles
    if len(obstacles.shape) == 1: # If it is a 1D array, make it a 2D column array
        obstacles = np.transpose([obstacles])

    num_obstacles = obstacles.shape[1]

    # import pdb
    # pdb.set_trace()

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
    
    # way to manually set axes
    #ax.axis([x_min, x_max, y_min, y_max])
    
    # Plot BIT* graph
    for i in range(0, states.shape[1] - 1): # second dimension of shape, because plotting in 2D
        plt.plot([states[0,i], parents[0,i]], [states[1,i], parents[1,i]],color='blue')

    # Plot sequence of goal states
    plt.plot(goal_path[0,:], goal_path[1,:], color='g')
    for i in range(goal_path.shape[1]):
        # since the output is in radians, I convert it to degrees
        # the -90 is because I want the tip of the triangle to start facing the right
        # as in 0 degrees of the unit circle.
        orientation = float(atan(goal_path[3,i]/goal_path[2,i]))
        if goal_path[2,i] < 0 and goal_path[3,i] < 0: # dx and dy are negative
            orientation = pi + orientation
        elif goal_path[2,i] < 0: # If dx is negative, the is starting from pi, not 0
            orientation = pi - abs(orientation)
        elif goal_path[3,i] < 0: # If dy is negative, everything is okay
            pass
        if np.isnan(orientation): # intitial state is [0,0,0,0], orientation is nan
            continue
        else:
            plt.plot(goal_path[0,i], goal_path[1,i], marker=(3,0,orientation*180/pi-90), markersize=10, color='g')
    
    plt.title('Cost: ' + str(cost))
    plt.ylim(-10,10)
    plt.xlim(-10,10)

    #plt.show(block=False)
    #plt.pause(.1)
    
    # way to save the plot
    #save('pics/{0}_seconds'.format(seconds), 'pdf')
    
    # do pause in python so plot can be resized etc.
   
    pause = True
    if (pause):
        print("Type 'save' to save this figure into a pdf file as 'pics/<number of seconds>_seconds.pdf'")
        print("Otherwise, press Enter to continue.")
        user_says = raw_input()
        if user_says.strip() == 'save':
            save('pics/{0}_iters'.format(seconds), 'pdf')
   
       
def save(path, ext='png', close=True, verbose=True):
    """Save a figure from pyplot.
     
    Parameters
    ----------
    path : string
    The path (and filename, without the extension) to save the
    figure to.
     
    ext : string (default='png')
    The file extension. This must be supported by the active
    matplotlib backend (see matplotlib.backends module). Most
    backends support 'png', 'pdf', 'ps', 'eps', and 'svg'.
     
    close : boolean (default=True)
    Whether to close the figure after saving. If you want to save
    the figure multiple times (e.g., to multiple formats), you
    should NOT close it in between saves or you will have to
    re-plot it.
     
    verbose : boolean (default=True)
    Whether to print information about when and where the image
    has been saved.
     
    """
    # Extract the directory and filename from the given path
    directory = os.path.split(path)[0]
    filename = "%s.%s" % (os.path.split(path)[1], ext)
    if directory == '':
        directory = '.'
     
    # If the directory does not exist, create it
    if not os.path.exists(directory):
        os.makedirs(directory)
     
    # The final path to save to
    savepath = os.path.join(directory, filename)
     
    if verbose:
        print("Saving figure to '%s'..." % savepath),
     
    # Actually save the figure
    plt.savefig(savepath)
    # Close it
    if close:
        plt.close()
     
    if verbose:
        print("Done")
    
