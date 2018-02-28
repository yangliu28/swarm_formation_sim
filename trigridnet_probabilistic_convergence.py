# This simulation tests the consensus achievement algorithm (collective decision making
# algorithm) on randomly generated 2D triangle grid network. The algorithm is extracted
# from previous loop reshape simulations for further testing, so that it can be generalized
# to 2D random network topologies. The distribution evolution method is similar to the
# loop reshape simulation, except more than three nodes are involved in the weighted
# averaging process.

# input arguments:
# '-f': filename of the triangle grid network
# '-d': number of decisions each node can choose from
# '-r': repeat times of simulation, with different initial random distribution; default=0
# '--nobargraph': option to skip the bar graph visualization

# Pygame will be used to animate the dynamic group changes in the network;
# Matplotlib will be used to draw the unipolarity in a 3D bar graph, the whole decision
# distribution is not necessary.

# Algorithm to update the groups in 2D triangle grid network:
# Using a pool of indices for nodes that have not been grouped, those labeled into a
# group will be remove from the pool. As long as the pool is not enpty, a while loop
# will continue searching groups one by one. In the loop, it initialize a new group
# with first node in the pool. It also initialize a fifo-like variable(p_members) for
# potential members of this group, and an index variable(p_index) for iterating through
# the potential members. If p_members[p_index] doesn't share same decidion with first node,
# p_index increases by 1, will check next value in p_members. If it does, then a new member
# for this group has been found, it will be removed from the pool and p_members, and added
# to current group. The new group member will also introduce its neighbors as new
# potential members, but they should not be in p_members and current group, and should be
# in node pool. The member search for this group will end if p_index iterates to the end
# of p_members.

# 01/19/2018
# Testing an invented concept called "holistic dependency", for measuring how much the most
# depended node is being depended on more than others for maintaining the network connectivity
# in the holistic view. The final name of the concept may change.

# 01/29/2018
# Dr. Lee suggest adding colors to different groups. A flashy idea I though at first, but
# after reconsidering, it has a much better visual effect.
# Link of a list of 20 distinct colors:
# https://sashat.me/2017/01/11/list-of-20-simple-distinct-colors/

# 02/06/2018
# Adding another experiment scenario: for 100 nodes, at the begining, forcing 10 closely
# located nodes to have a specific decision (colored in blue), and run simulation.

# 02/12/2018
# Adding another experiment scenario: for 100 nodes, forcing 10 closely located nodes to
# have a specific decision (colored in blue), and in the half way toward consensus, seed
# 20 "stubborn" nodes that won't change their mind at all. Ideally the collective decision
# will reverse to the one insisted by the new 20 nodes.
# (The program is really messy now.)

# 02/13/2018
# Replace the unipolarity with discrete entropy for evaluating the decisiveness of the
# preference distributions. Adding same colors of the consensus groups to the bar graph.

# 02/26/2018
# (an excursion of thought)
# Writing a paper and keep revising it totally ruin the fun of developping this simulation.
# Writing paper is boring to many people I know, but it's specially painful to me in a
# subconscious way. I have very strong personal style which runs through every aspect of my
# life, and the style does not mix well with academia. I love simplicity, elegance, and truth.
# I rarely claim something in my belief, but these I am very proud to have, and have no
# intention to compromise them with anything. Academic paper as a form to present the
# state-of-the-art, and to communicate thought, should contain succinct and honest description
# of the work. However the two advisors I have worked with (I respect them from the deep of my
# heart) both revised my papers toward being more acceptable by the publishers. Most of the
# changes like pharsing and some of the restructuring are very good, reflecting their years of
# academic training. They make the paper more explicit and easy to understand. Others changes,
# IMHO, are simply trying to elevate the paper, so on first read people would think this is
# a very good one by those abstract words. This is what makes me suffer. These changes are
# steering the paper to the opposite of succinct and honesty. For example, if it is only a
# method to solve a specific problem, saying it is a systematic approach to solve a series
# of problem, and using abstract words for the technical terms, does not make the work any
# better. It would only confuse the readers and let them spend more time to dig out the idea.
# I could never fight my advisors on these changes, they would say it is just a style of
# writing, and it brings out the potential of the work. This is like to say, our presented
# technology can do this, can do that, and potentially it could do much more amazing things.
# I totally hate the last part! It just give people a chance to bragging things they would
# never reach. As I find out, quite a few published papers that I believe I understand every
# bit of it, are trying to elevate their content to a level they don't deserve. If I were to
# rewrite them, I would cut all the crap and leave only the naked truth and idea. Many
# researcher picked up this habit in their training with publishing papers, like an untold
# rule. As for my paper, I really wish my work were that good to be entitled to those words,
# but they are not. Academia is a sublime place in many people's thinking, as in mine. I have
# very high standards for the graduation of a PhD. If I kept following my style, and be honest
# with myself, I knew long ago I would never graduate. I am loving it too much to hate it.
# (addition to the simulation)
# Requested by the paper, when visualizing the bar graph for the discrete entropy, adding the
# summation of all discrete entropy to show how the entropy reduction works.

# 02/28/2018
# Remove color grey, beige, and coral from the distinct color set, avoid blending with white
# background.


import pygame
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from trigridnet_generator import *
from formation_functions import *
import math, sys, os, getopt, time
import numpy as np

net_size = 30  # default size of the triangle grid network
net_folder = 'trigrid-networks'  # folder for triangle grid network files
net_filename = '30-1'  # default filename of the network file, if no input
net_filepath = os.path.join(os.getcwd(), net_folder, net_filename)  # corresponding filepath

deci_num = 30  # default number of decisions each node can choose from

repeat_times = 1  # default times of running the program repeatly

nobargraph = False  # option as to whether or not skipping the 3D bar graph

# read command line options
try:
    opts, args = getopt.getopt(sys.argv[1:], 'f:d:r:', ['nobargraph'])
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
    elif opt == '-r':
        # get the times of repeatence
        repeat_times = int(arg)
    elif opt == '--nobargraph':
        nobargraph = True

# read the network from file
nodes = []  # integers only is necessary to describe the network's node positions
f = open(net_filepath, 'r')
new_line = f.readline()
while len(new_line) != 0:  # not the end of the file yet
    pos_str = new_line[0:-1].split(' ')  # get rid of '\n' at end
    pos = [int(pos_str[0]), int(pos_str[1])]  # convert to integers
    nodes.append(pos)
    new_line = f.readline()

# generate the connection matrix, 0 for not connected, 1 for connected
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
# another list type variable for easily indexing from the nodes
# converted from the above connection matrix variable
connection_lists = []  # the lists of connecting nodes for each node
for i in range(net_size):
    connection_list_temp = []
    for j in range(net_size):
        if connections[i][j]: connection_list_temp.append(j)
    connection_lists.append(connection_list_temp)

# until here, the network information has been read and interpreted completely
# calculate the "holistic dependency"
calculate_h_dependency = False  # option for calculating holistic dependency
# this computation takes significant time when net_size is above 50
# the algorithm below has been optimized to the most efficient I can
if calculate_h_dependency:
    dependencies = [0.0 for i in range(net_size)]  # individual dependency for each robot
    holistic_dependency_abs = 0.0  # absolute holistic dependency
    holistic_dependency_rel = 0.0  # relative holistic dependency
    # Search the shortest path for every pair of nodes i and j.
    for i in range(net_size-1):  # i is the starting node
        for j in range(i+1, net_size):  # j is the target node
            # (There might be multiple path sharing the shortest length, which are totally fine,
            # all the path should count.)
            # Some useful tricks have been used to make the search efficient.
            path_potential = [[i]]  # the paths that are in progress, potentially the shortest
            path_succeed = []  # the shortest paths
            # start searching
            search_end = False  # flag indicating if the shortest path is found
            nodes_reached = {i}  # dynamic pool for nodes in at least one of the paths
                # key to speed up this search algorithm
            while not search_end:
                # increase one step for all paths in path_potential
                path_potential2 = []  # for the paths adding one step from path_potential
                nodes_reached_add = set()  # to be added to nodes_reached
                node_front_pool = dict()  # solutions for next qualified nodes of front node
                for path in path_potential:
                    node_front = path[-1]  # front node in this current path
                    nodes_next = []  # for nodes qualified as next one on path
                    if node_front in node_front_pool.keys():
                        nodes_next = node_front_pool[node_front]
                    else:
                        for node_n in connection_lists[node_front]:  # neighbor node
                            if node_n == j:  # the shortest path found, only these many steps needed
                                nodes_next.append(node_n)
                                search_end = True
                                continue
                            if node_n not in nodes_reached:
                                nodes_reached_add.add(node_n)
                                nodes_next.append(node_n)
                        node_front_pool[node_front] = nodes_next[:]  # add new solution
                    for node_next in nodes_next:
                        path_potential2.append(path + [node_next])
                for node in nodes_reached_add:
                    nodes_reached.add(node)
                # empty the old potential paths
                path_potential = []
                # assign the new potential paths, copy with list comprehension method
                path_potential = [[node for node in path] for path in path_potential2]
            # if here, the shortest paths have been found; locate them
            for path in path_potential:
                if path[-1] == j:
                    path_succeed.append(path)
            # distribute the dependency value evenly for each shortest paths
            d_value = 1.0 / len(path_succeed)
            for path in path_succeed:
                for node in path[1:-1]:  # exclude start and end nodes
                    dependencies[node] = dependencies[node] + d_value
    # print(dependencies)
    dependency_mean = sum(dependencies)/net_size
    node_max = dependencies.index(max(dependencies))
    holistic_dependency_abs = dependencies[node_max] - dependency_mean
    holistic_dependency_rel = dependencies[node_max] / dependency_mean
    print "absolute holistic dependency {}".format(holistic_dependency_abs)
    print "relative holistic dependency {}".format(holistic_dependency_rel)
# Also uncomment two lines somewhere below to highlight maximum individual dependency node,
# and halt the program after drawing the network.

# plot the network as dots and lines in pygame window
pygame.init()  # initialize the pygame
# find appropriate window size from current network
# convert node positions from triangle grid to Cartesian, for plotting
nodes_plt = np.array([trigrid_to_cartesian(pos) for pos in nodes])
(xmin, ymin) = np.amin(nodes_plt, axis=0)
(xmax, ymax) = np.amax(nodes_plt, axis=0)
world_size = (xmax-xmin + 5.0, ymax-ymin + 2.0)  # extra length for clearance
pixels_per_length = 50  # resolution for converting from world to display
# display origin is at left top corner
screen_size = (int(round(world_size[0] * pixels_per_length)),
               int(round(world_size[1] * pixels_per_length)))
color_white = (255,255,255)
color_black = (0,0,0)
# a set of 20 distinct colors (black and white excluded)
# distinct_color_set = ((230,25,75), (60,180,75), (255,225,25), (0,130,200), (245,130,48),
#     (145,30,180), (70,240,240), (240,50,230), (210,245,60), (250,190,190),
#     (0,128,128), (230,190,255), (170,110,40), (255,250,200), (128,0,0),
#     (170,255,195), (128,128,0), (255,215,180), (0,0,128), (128,128,128))
distinct_color_set = ((230,25,75), (60,180,75), (255,225,25), (0,130,200), (245,130,48),
    (145,30,180), (70,240,240), (240,50,230), (210,245,60), (250,190,190),
    (0,128,128), (230,190,255), (170,110,40), (128,0,0),
    (170,255,195), (128,128,0), (0,0,128))
color_quantity = 17  # grey is removed, 19 left
# convert the color set to float, for use in matplotlib
distinct_color_set_float = tuple([tuple([color[0]/255.0, color[1]/255.0, color[2]/255.0])
    for color in distinct_color_set])
node_size = 10  # node modeled as dot, number of pixels for radius
group_line_width = 5  # width of the connecting lines inside the groups
norm_line_width = 2  # width of other connections
# set up the simulation window and surface object
icon = pygame.image.load("icon_geometry_art.jpg")
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Probabilistic Convergence of 2D Triangle Grid Network")

# shift the node display positions to the middle of the window
centroid_temp = np.mean(nodes_plt, axis=0)
nodes_plt = nodes_plt - centroid_temp + (world_size[0]/2.0, world_size[1]/2.0)
nodes_disp = [world_to_display(nodes_plt[i], world_size, screen_size)
              for i in range(net_size)]

# draw the network for the first time
screen.fill(color_white)  # fill the background
# draw the connecting lines
for i in range(net_size):
    for j in range(i+1, net_size):
        if connections[i][j]:
            pygame.draw.line(screen, color_black, nodes_disp[i], nodes_disp[j], norm_line_width)
# draw the nodes as dots
for i in range(net_size):
    pygame.draw.circle(screen, color_black, nodes_disp[i], node_size, 0)
# highlight the node with maximum individual dependency
# pygame.draw.circle(screen, color_black, nodes_disp[node_max], node_size*2, 2)
pygame.display.update()

# # use the following to find the indices of the nodes on graph
# for i in range(net_size):
#     pygame.draw.circle(screen, color_black, nodes_disp[i], node_size*2,1)
#     pygame.display.update()
#     print "node {} is drawn".format(i)
#     raw_input("<Press enter to continue>")

# hold the program here to check the netwrok
# raw_input("Press the <ENTER> key to continue")

############### the probabilistic convergence ###############

# # for network 100-3
# # the closely located 10 nodes to command at beginning of the simulation
# command_nodes_10 = [0,1,2,5,7,10,11,12,13,35]  # in the middle of the network
# # command_nodes_10 = [22,34,36,50,57,61,72,87,91,92]  # in the top right corner
# # # the closely located 20 nodes to command during the simulation
# # command_nodes_20 = [8,9,20,22,24,27,34,36,44,45,
# #                    50,52,57,61,67,72,77,87,91,92]
# # iter_cutin = 5  # the 20 nodes command at this time stamp

all_steps = [0 for i in range(repeat_times)]  # steps taken to converge for all simulations
all_deci_orders = [0 for i in range(repeat_times)]  # the order of the final decisions
for sim_index in range(repeat_times):  # repeat the simulation for these times

    print("\n{}th simulation".format(sim_index))

    # variable for decision distribution of all individuals
    deci_dist = np.random.rand(net_size, deci_num)
    # # tweak the decision distribution for the command nodes
    # for i in command_nodes_10:
    #     deci_dist[i][0] = 1.0  # force the first probability to be the largest
    # normalize the random numbers such that the sum is 1.0
    sum_temp = np.sum(deci_dist, axis=1)
    for i in range(net_size):
        deci_dist[i][:] = deci_dist[i][:] / sum_temp[i]
    # calculate the average decision distribution
    mean_temp = np.mean(deci_dist, axis=0)
    avg_dist_sort = np.sort(mean_temp)[::-1]
    avg_dist_id_sort = np.argsort(mean_temp)[::-1]
    # the dominant decision of all nodes
    deci_domi = np.argmax(deci_dist, axis=1)
    print deci_domi
    # only adjacent block of nodes sharing same dominant decision belongs to same group
    groups = []  # put nodes in groups by their local convergence
    group_sizes = [0 for i in range(net_size)]  # the group size that each node belongs to
    color_initialized = False  # whether the color assignment has been done for the first time
    deci_colors = [-1 for i in range(deci_num)]  # color index for each exhibited decision
        # -1 for not assigned
    color_assigns = [0 for i in range(color_quantity)]  # number of assignments for each color
    group_colors = []  # color for the groups
    node_colors = [0 for i in range(net_size)]  # color for the nodes
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

    # start the matplotlib window first before the simulation cycle
    fig = plt.figure()
    # fig.canvas.set_window_title('Unipolarity of 2D Triangle Grid Network')
    fig.canvas.set_window_title('Discrete Entropy of the Preference Distributions')
    ax = fig.add_subplot(111, projection='3d')
    x_pos = [pos[0] for pos in nodes_plt]  # the positions of the bars
    y_pos = [pos[1] for pos in nodes_plt]
    z_pos = np.zeros(net_size)
    dx = 0.5 * np.ones(net_size)  # the sizes of the bars
    dy = 0.5 * np.ones(net_size)
    dz = np.zeros(net_size)
    if not nobargraph:
        fig.canvas.draw()
        fig.show()

    # the simulation cycle
    sim_exit = False  # simulation exit flag
    sim_pause = False  # simulation pause flag
    iter_count = 0
    time_now = pygame.time.get_ticks()  # return milliseconds
    time_last = time_now  # reset as right now
    time_period = 500  # simulation frequency control, will jump the delay if overflow
    skip_speed_control = False  # if skip speed control, run as fast as it can
    while not sim_exit:
        # exit the program by close window button, or Esc or Q on keyboard
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sim_exit = True  # exit with the close window button
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_SPACE:
                    sim_pause = not sim_pause  # reverse the pause flag
                if (event.key == pygame.K_ESCAPE) or (event.key == pygame.K_q):
                    sim_exit = True  # exit with ESC key or Q key

        # skip the rest if paused
        if sim_pause: continue

        # # the 20 nodes start to cut in here
        # if iter_count == iter_cutin:
        #     for i in command_nodes_20:
        #         deci_dist[i][1] = 1.0  # force the second probability to be the largest
        #         # normalize again
        #         sum_temp = np.sum(deci_dist[i])
        #         deci_dist[i][:] = deci_dist[i][:] / sum_temp

        # prepare information for the decision distribution evolution
        # including 1.dominant decisions, 2.groups, and 3.group sizes

        # 1.update the dominant decision for all nodes
        deci_domi = np.argmax(deci_dist, axis=1)

        # 2.update the groups
        groups = []  # empty the group container
        group_deci = []  # the exhibited decision of the groups
        # a diminishing global pool for node indices, for nodes not yet assigned into groups
        n_pool = range(net_size)
        # start searching groups one by one from the global node pool
        while len(n_pool) != 0:
            # start a new group, with first node in the n_pool
            first_member = n_pool[0]  # first member of this group
            group_temp = [first_member]  # current temporary group
            n_pool.pop(0)  # pop out first node in the pool
            # a list of potential members for current group
            # this list may increase when new members of group are discovered
            p_members = connection_lists[first_member][:]
            # an index for iterating through p_members, in searching group members
            p_index = 0  # if it climbs to the end, the searching ends
            # index of dominant decision for current group
            current_domi = deci_domi[first_member]
            # dynamically iterating through p_members with p_index
            while p_index < len(p_members):  # index still in valid range
                if deci_domi[p_members[p_index]] == current_domi:
                    # a new member has been found
                    new_member = p_members[p_index]  # get index of the new member
                    p_members.remove(new_member)  # remove it from p_members list
                        # but not increase p_index, because new value in p_members will flush in
                    n_pool.remove(new_member)  # remove it from the global node pool
                    group_temp.append(new_member)  # add it to current group
                    # check if new potential members are available, due to new node discovery
                    p_members_new = connection_lists[new_member]  # new potential members
                    for member in p_members_new:
                        if member not in p_members:  # should not already in p_members
                            if member not in group_temp:  # should not in current group
                                if member in n_pool:  # should be available in global pool
                                    # if conditions satisfied, it is qualified as a potential member
                                    p_members.append(member)  # append at the end
                else:
                    # a boundary node(share different decision) has been met
                    # leave it in p_members, will help to avoid checking back again on this node
                    p_index = p_index + 1  # shift right one position
            # all connected members for this group have been located
            groups.append(group_temp)  # append the new group
            group_deci.append(deci_domi[first_member])  # append new group's exhibited decision
        # update the colors for the exhibited decisions
        if not color_initialized:
            color_initialized = True
            select_set = range(color_quantity)  # the initial selecting set
            all_deci_set = set(group_deci)  # put all exhibited decisions in a set
            for deci in all_deci_set:  # avoid checking duplicate decisions
                if len(select_set) == 0:
                    select_set = range(color_quantity)  # start a new set to select from
                # # force color blue for first decision, color orange for second decision
                # if deci == 0:
                #     chosen_color = 3  # color blue
                # # elif deci == 1:
                # #     chosen_color = 4  # color orange
                # else:
                #     chosen_color = np.random.choice(select_set)
                chosen_color = np.random.choice(select_set)
                select_set.remove(chosen_color)
                deci_colors[deci] = chosen_color  # assign the chosen color to decision
                # increase the assignments of chosen color by 1
                color_assigns[chosen_color] = color_assigns[chosen_color] + 1
        else:
            # remove the color for a decision, if it's no longer the decision of any group
            all_deci_set = set(group_deci)
            for i in range(deci_num):
                if deci_colors[i] != -1:  # there was a color assigned before
                    if i not in all_deci_set:
                        # decrease the assignments of chosen color by 1
                        color_assigns[deci_colors[i]] = color_assigns[deci_colors[i]] - 1
                        deci_colors[i] = -1  # remove the assigned color
            # assign color for an exhibited decision if not assigned
            select_set = []  # set of colors to select from, start from empty
            for i in range(len(groups)):
                if deci_colors[group_deci[i]] == -1:
                    if len(select_set) == 0:
                        # construct a new select_set
                        color_assigns_min = min(color_assigns)
                        color_assigns_temp = [j - color_assigns_min for j in color_assigns]
                        select_set = range(color_quantity)
                        for j in range(color_quantity):
                            if color_assigns_temp[j] != 0:
                                select_set.remove(j)
                    # if here, the select_set is good to go
                    # # force color orange for second decision
                    # if group_deci[i] == 1:
                    #     chosen_color = 4  # color orange
                    # else:
                    #     chosen_color = np.random.choice(select_set)
                    # if chosen_color in select_set:
                    #     select_set.remove(chosen_color)
                    chosen_color = np.random.choice(select_set)
                    select_set.remove(chosen_color)
                    deci_colors[group_deci[i]] = chosen_color  # assign the chosen color
                    # increase the assignments of chosen color by 1
                    color_assigns[chosen_color] = color_assigns[chosen_color] + 1
        # update the colors for the groups
        group_colors = []
        for i in range(len(groups)):
            group_colors.append(deci_colors[group_deci[i]])

        # 3.update the group size for each node
        for i in range(len(groups)):
            size_temp = len(groups[i])
            color_temp = group_colors[i]
            for node in groups[i]:
                group_sizes[node] = size_temp
                node_colors[node] = color_temp  # update the color for each node

        # the decision distribution evolution
        converged_all = True  # flag for convergence of entire network
        deci_dist_t = np.copy(deci_dist)  # deep copy of the 'deci_dist'
        for i in range(net_size):
            # # skip updating the 20 commanding nodes, stubborn in their decisions
            # if i in command_nodes_20:
            #     if iter_count >= iter_cutin:
            #         continue
            host_domi = deci_domi[i]
            converged = True
            for neighbor in connection_lists[i]:
                if host_domi != deci_domi[neighbor]:
                    converged = False
                    break
            # action based on convergence of dominant decision
            if converged:  # all neighbors have converged with host
                # step 1: take equally weighted average on all distributions
                # including host and all neighbors
                deci_dist[i] = deci_dist_t[i]*1.0  # start with host itself
                for neighbor in connection_lists[i]:
                    # accumulate neighbor's distribution
                    deci_dist[i] = deci_dist[i] + deci_dist_t[neighbor]
                # normalize the distribution such that sum is 1.0
                sum_temp = np.sum(deci_dist[i])
                deci_dist[i] = deci_dist[i] / sum_temp
                # step 2: increase the unipolarity by applying the linear multiplier
                # (this step is only for when all neighbors are converged)
                # First find the largest difference between two distributions in this block
                # of nodes, including the host and all its neighbors.
                comb_pool = [i] + connection_lists[i]  # add host to a pool with its neighbors
                    # will be used to form combinations from this pool
                comb_pool_len = len(comb_pool)
                dist_diff = []
                for j in range(comb_pool_len):
                    for k in range(j+1, comb_pool_len):
                        j_node = comb_pool[j]
                        k_node = comb_pool[k]
                        dist_diff.append(np.sum(abs(deci_dist[j_node] - deci_dist[k_node])))
                dist_diff_max = max(dist_diff)  # maximum distribution difference of all
                if dist_diff_max < dist_diff_thres:
                    # distribution difference is small enough,
                    # that linear multiplier should be applied to increase unipolarity
                    dist_diff_ratio = dist_diff_max/dist_diff_thres
                    # Linear multiplier is generated from value of smaller and larger ends, the
                    # smaller end is positively related with dist_diff_ratio. The smaller the
                    # maximum distribution difference, the smaller the dist_diff_ratio, and the
                    # steeper the linear multiplier.
                    # '1.0/deci_num' is the average value of the linear multiplier
                    small_end = 1.0/deci_num * np.power(dist_diff_ratio, dist_diff_power)
                    large_end = 2.0/deci_num - small_end
                    # sort the magnitude of the current distribution
                    dist_temp = np.copy(deci_dist[i])  # temporary distribution
                    sort_index = range(deci_num)
                    for j in range(deci_num-1):  # bubble sort, ascending order
                        for k in range(deci_num-1-j):
                            if dist_temp[k] > dist_temp[k+1]:
                                # exchange values in 'dist_temp'
                                temp = dist_temp[k]
                                dist_temp[k] = dist_temp[k+1]
                                dist_temp[k+1] = temp
                                # exchange values in 'sort_index'
                                temp = sort_index[k]
                                sort_index[k] = sort_index[k+1]
                                sort_index[k+1] = temp
                    # applying the linear multiplier
                    for j in range(deci_num):
                        multiplier = small_end + float(j)/(deci_num-1) * (large_end-small_end)
                        deci_dist[i][sort_index[j]] = deci_dist[i][sort_index[j]] * multiplier
                    # normalize the distribution such that sum is 1.0
                    sum_temp = np.sum(deci_dist[i])
                    deci_dist[i] = deci_dist[i]/sum_temp
                else:
                    # not applying linear multiplier when distribution difference is large
                    pass
            else:  # at least one neighbor has different opinion with host
                converged_all = False  # the network is not converged
                # take unequal weights in the averaging process based on group sizes
                deci_dist[i] = deci_dist_t[i]*group_sizes[i]  # start with host itself
                for neighbor in connection_lists[i]:
                    # accumulate neighbor's distribution
                    deci_dist[i] = deci_dist[i] + deci_dist_t[neighbor]*group_sizes[neighbor]
                # normalize the distribution
                sum_temp = np.sum(deci_dist[i])
                deci_dist[i] = deci_dist[i] / sum_temp

        # graphics animation, both pygame window and matplotlib window
        # 1.pygame window for dynamics of network's groups
        screen.fill(color_white)
        # draw the regualr connecting lines
        for i in range(net_size):
            for j in range(i+1, net_size):
                if connections[i][j]:
                    pygame.draw.line(screen, color_black,
                                     nodes_disp[i], nodes_disp[j], norm_line_width)
        # draw the connecting lines marking the groups
        for i in range(len(groups)):
            group_len = len(groups[i])
            for j in range(group_len):
                for k in range(j+1, group_len):
                    j_node = groups[i][j]
                    k_node = groups[i][k]
                    # check if two nodes in one group is connected
                    if connections[j_node][k_node]:
                        # wider lines for group connections
                        pygame.draw.line(screen, distinct_color_set[group_colors[i]],
                            nodes_disp[j_node], nodes_disp[k_node], group_line_width)
        # draw the nodes as dots
        for i in range(net_size):
            pygame.draw.circle(screen, distinct_color_set[node_colors[i]],
                nodes_disp[i], node_size, 0)
        pygame.display.update()
        # # 2.matplotlib window for 3D bar graph of unipolarity of decision distribution
        # if not nobargraph:
        #     dz = [deci_dist[i][deci_domi[i]] for i in range(net_size)]
        #     ax.bar3d(x_pos, y_pos, z_pos, dx, dy, dz, color='b')
        #     fig.canvas.draw()
        #     fig.show()
        # 2.matplotlib window for 2D bar graph of discrete entropy
        if not nobargraph:
            # calculate the discrete entropy for all distributions
            ent_sum = 0
            for i in range(net_size):
                ent = 0
                for j in range(deci_num):
                    if deci_dist[i][j] == 0: continue
                    ent = ent - deci_dist[i][j] * math.log(deci_dist[i][j],2)
                dz[i] = ent
                ent_sum = ent_sum + ent
            print("summation of entropy of all distributions: {}".format(ent_sum))
            # draw the bar graph
                # somehow the old bars are overlapping the current ones, have to clear the
                # figure first, and rebuild the bar graph, as a temporary solution
            fig.clear()
            ax = fig.add_subplot(111, projection='3d')
            bar_colors = [distinct_color_set_float[node_colors[i]] for i in range(net_size)]
            barlist = ax.bar3d(x_pos, y_pos, z_pos, dx, dy, dz, bar_colors)
            fig.canvas.draw()
            fig.show()

        # simulation speed control
        if not skip_speed_control:
            # simulation updating frequency control
            time_now = pygame.time.get_ticks()
            time_past = time_now - time_last  # time past since last time_last
            # needs to delay a bit more if time past has not reach desired period
            # will skip if time is overdue
            if time_now - time_last < time_period:
                pygame.time.delay(time_period-time_past)
            time_last = pygame.time.get_ticks()  # reset time-last

        # iteration count
        print "iteration {}".format(iter_count)
        iter_count = iter_count + 1
        # hold the program to check the network
        # raw_input("<Press Enter to continue>")

        # exit as soon as the network is converged
        if converged_all:
            print("steps to converge: {}".format(iter_count-1))
            print("converged to the decision: {} ({} in order)".format(deci_domi[0],
                list(avg_dist_id_sort).index(deci_domi[0]) + 1))
            print("the order of average initial decision:")
            print(avg_dist_id_sort)
            break

        # record result of this simulation
        all_steps[sim_index] = iter_count - 1
        all_deci_orders[sim_index] = list(avg_dist_id_sort).index(deci_domi[0]) + 1

# report statistic result if simulation runs more than once
if repeat_times > 1:
    print("\nstatistics\nsteps to converge: {}".format(all_steps))
    print("final decision in order: {}".format(all_deci_orders))
    print("average steps: {}".format(np.mean(np.array(all_steps))))


