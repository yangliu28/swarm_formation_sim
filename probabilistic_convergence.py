# This simulation tests the probabilistic convergence algorithm (a collective decision
# making algorithm) on randomly generated 2D honeycomb network. The algorithm is extracted
# from previous loop reshape simulations, for further testing if it can be generally used
# on 2D random network topologies.

# input arguments:
# '-f': filename of the honeycomb network
# '-d': number of decisions each node can choose from

# Pygame will be used to animate the dynamic subgroup changes in the network;
# Matplotlib will be used to draw the unipolarity in a 3D histogram, full distribution is
# not necessary.

import math, sys, os, getopt, time
import matplotlib.pyplot as plt
from network_generator_2D_swarm import *
import numpy as np
import pygame
from formation_functions import *

net_size = 30  # default size of the honeycomb network
net_folder = 'honeycomb-networks'  # folder for all network files
net_filename = '30-1'  # defautl filename of the network file, if no input
net_filepath = os.path.join(os.getcwd(), net_folder, net_filename)  # corresponding filepath

deci_num = 30  # default number of decisions each node can choose from

# read command line options
try:
    opts, args = getopt.getopt(sys.argv[1:], 'f:d:')
    # The colon after 'f' means '-f' requires an argument, it will raise an error if no
    # argument followed by '-f'. But if '-f' is not even in the arguments, this won't raise
    # an error. So it's necessary to define the default network filename
except getopt.GetoptError as err:
    print str(err)
    sys.exit()
for opt,arg in opts:
    if opt == '-f':
        # get the filename of the network
        net_filename = arg
        # check if this file exists
        net_filepath = os.path.join(os.getcwd(), net_folder, net_filename)
        if not os.path.isfile(net_filepath):
            print "{} does not exist".format(net_filename)
            sys.exit()
        # parse the network size
        net_size = int(net_filename.split('-')[0])  # the number before first dash
    elif opt == '-d':
        # get the number of decisions
        deci_num = int(arg)

# read the network from file
nodes = []  # integers only is necessary to describe the network's node positions
f = open(net_filepath, 'r')
new_line = f.readline()
while len(new_line) != 0:  # not the end of the file yet
    pos_str = new_line[0:-1].split(' ')  # get rid of '\n' at end
    pos = [int(pos_str[0]), int(pos_str[1])]  # convert to integers
    nodes.append(pos)
    new_line = f.readline()

# generate the connection variable, 0 for not connected, 1 for connected
connections = [[0 for j in range(net_size)] for i in range(net_size)]  # all zeros
for i in range(net_size):
    for j in range(i+1, net_size):
        # find if nodes[i] and nodes[j] are neighbors
        diff_x = nodes[i][0] - nodes[j][0]
        diff_y = nodes[i][1] - nodes[j][1]
        if abs(diff_x) + abs(diff_y) == 1 or diff_x * diff_y == -1:
            # condition 1: one of the axis value difference is 1, the other is 0
            # condition 2: one of the axis value difference is 1, the other is -1
            connections[i][j] = 1
            connections[j][i] = 1

# plot the network as dots and lines in pygame window
pygame.init()  # initialize the pygame
# find appropriate window size from current network
# convert node positions from honeycomb coordinates to Cartesian coordinates, for plotting
nodes_plt = np.array([honeycomb_to_cartesian(pos) for pos in nodes])
(xmin, ymin) = np.amin(nodes_plt, axis=0)
(xmax, ymax) = np.amax(nodes_plt, axis=0)
world_size = (xmax-xmin + 2.0, ymax-ymin + 2.0)  # extra length for clearance
pixels_per_length = 50  # resolution for converting from world to display
# display origin is at left top corner
screen_size = (int(round(world_size[0] * pixels_per_length)),
               int(round(world_size[1] * pixels_per_length)))
background_color = (0,0,0)
node_color = (255,0,0)  # red for nodes and connecting lines
subgroup_color = (255,255,0)  # yellow for connecting lines in the subgroups
node_size = 5  # node modeled as dot, number of pixels for radius
subgroup_thickness = 3  # thickness of connecting lines in the subgroups
# set up the simulation window and surface object
icon = pygame.image.load("icon_geometry_art.jpg")
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Probabilistic Convergence of 2D Honeycomb Network")

# shift the node display positions to the middle of the window
centroid_temp = np.mean(nodes_plt, axis=0)
nodes_plt = nodes_plt - centroid_temp + (world_size[0]/2.0, world_size[1]/2.0)
nodes_disp = [world_to_display(nodes_plt[i], world_size, screen_size)
              for i in range(net_size)]

# draw the network for the first time
screen.fill(background_color)
# draw the connecting lines
for i in range(net_size):
    for j in range(i+1, net_size):
        if connections[i][j]:
            pygame.draw.line(screen, node_color, nodes_disp[i], nodes_disp[j])
# draw the nodes as dots
for i in range(net_size):
    pygame.draw.circle(screen, node_color, nodes_disp[i], node_size, 0)
pygame.display.update()
time.sleep(1)

############### the probabilistic convergence ###############

# variable for decision distribution of all individuals
deci_dist = np.random.rand(net_size, deci_num)
# normalize the random numbers such that the sum is 1.0
sum_temp = np.sum(deci_dist, axis=1)
for i in range(net_size):
    deci_dist[i][:] = deci_dist[i][:] / sum_temp[i]
# variable for the dominant decision
deci_domi = np.argmax(deci_dist, axis=1)
# only adjacent block of nodes sharing same dominant decision belongs to same subgroup
subgroups = []  # seperate lists of node indices for all subgroups
subsize = []  # the subgroup size that each node belongs to
# Difference of two distributions is the sum of absolute values of differences
# of all individual probabilities.
# Overflow threshold for the distribution difference. Distribution difference larger than
# this means neighbors are not quite agree with each other, so no further improvement on
# unipolarity will be performed. If distribution difference is lower than the threshold,
# linear multiplier will be used to improve unipolarity on the result distribution.
dist_diff_thres = 0.3
# Variable for ratio of distribution difference to distribution difference threshold.
# The ratio is in range of [0,1], it will be used for constructing the corresponding linear
# multiplier. At one side, the smaller the ratio, the smaller the distribution difference,
# and faster the unipolarity should increase. At the other side, the ratio will be positively
# related to the small end of the linear multiplier. The smallee the ratio gets, the steeper
# the linear multiplier will be, and the faster the unipolarity increases.
dist_diff_ratio = [0.0 for i in range(net_size)]
# Exponent of a power function to map the distribution difference ratio to a larger value,
# and therefore slow donw the growing rate.
dist_diff_power = 0.3





# Algorithm to update the subgroups


# update the subgroups initially
subgroups = [[0]]  # start with node 0
# a diminishing global pool for node indices, for nodes not yet assigned into subgroups
n_pool = range(1,net_size)  # 0 is already out of the pool
# convert the connection matrix to connection lists
connection_lists = []  # the lists of connecting nodes for each node
for i in range(net_size):
    connection_lists_temp = []
    for j in range(net_size):
        if connections[i][j]: connection_lists_temp.append(j)
    connection_lists.append(connection_lists_temp)
# start searching subgroups one by one from the global node pool
while len(n_pool) != 0:
    # start a new subgroup, with first node in the n_pool
    subgroup_temp = [n_pool[0]]  # current temporary subgroup
    n_pool.pop(0)  # remove first node in the pool
    # a list of potential members for current subgroup
    # this list may increase when new members of subgroup are discovered
    p_members = connection_lists[subgroup_temp[0]][:]
    # an index for iterating through p_members, in searching subgroup members
    p_index = 0  # if it climbs to the end, the searching ends
    # index of dominant decision for current subgroup
    current_domi = deci_domi[subgroup_temp[0]]
    # dynamically iterating through p_members with p_index
    while p_index < len(p_members):  # index still in valid range
        if deci_domi[p_members[p_index]] == current_domi:
            # a new member has been found
            new_member = p_members[p_index]  # get index of the new member
            p_members.remove(new_member)  # remove it from p_members list
                # but not increase p_index, because new value in p_members will flush in
            n_pool.remove(new_member)  # remove it from the global node pool
            subgroup_temp.append(new_member)  # add it to current subgroup
            # check if new potential members are available, due to new node discovery
            p_members_new = connection_lists[new_member]  # new potential members
            for member in p_members_new:
                if member not in p_members:  # should not already in p_members
                    if member not in subgroup_temp:  # should not in current subgroup
                        if member in n_pool:  # should be available in global pool
                            # if conditions satisfied, it is qualified as a potential member
                            p_members.append(member)  # append at the end
        else:
            # a boundary node(share different decision) has been met
            # leave it in p_members, will help to avoid checking back again on this node
            p_index = p_index + 1  # shift right one position
    # all connected members for this subgroup have been located
    subgroups.append(subgroup_temp)  # append the new subgroup
    # the end of searching for one subgroup

print deci_domi
print subgroups



