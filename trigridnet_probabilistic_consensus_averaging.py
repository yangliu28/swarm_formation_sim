# Modified from trigridnet_probabilistic_consensus_param.py on April 2019

# This program generate iteration data for a simple averaging consensus algorithm.
# It will be taken as a benchmark for the new algorithm.

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

# read command line options
try:
    opts, args = getopt.getopt(sys.argv[1:], 'f:d:r:')
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

# override parameters
# net_filename = "50-3"
# net_filepath = os.path.join(os.getcwd(), net_folder, net_filename)
# net_size = int(net_filename.split('-')[0])
# deci_num = 50
repeat_times = 100

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

# hold the program here to check the netwrok
# raw_input("Press the <ENTER> key to continue")

############### the probabilistic consensus ###############

# range_control = range(7)
# result = [0.0 for i in range_control]
# for index in range_control:
#     deci_num = (index + 4) * 100
for dummy in range(1):

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
        groups = []  # put nodes in groups by their local consensus
        group_sizes = [0 for i in range(net_size)]  # the group size that each node belongs to

        # Variable for ratio of distribution difference to distribution difference threshold.
        # The ratio is in range of [0,1], it will be used for constructing the corresponding linear
        # multiplier. At one side, the smaller the ratio, the smaller the distribution difference,
        # and faster the unipolarity should increase. At the other side, the ratio will be positively
        # related to the small end of the linear multiplier. The smallee the ratio gets, the steeper
        # the linear multiplier will be, and the faster the unipolarity increases.
        dist_diff_ratio = [0.0 for i in range(net_size)]

        # the simulation cycle
        iter_count = 0
        time_period = 500  # simulation frequency control, will jump the delay if overflow
        skip_speed_control = True  # if skip speed control, run as fast as it can
        while True:

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

            # 3.update the group size for each node
            for i in range(len(groups)):
                size_temp = len(groups[i])
                for node in groups[i]:
                    group_sizes[node] = size_temp

            # the decision distribution evolution
            converged_all = True  # flag for convergence of entire network
            deci_dist_t = np.copy(deci_dist)  # deep copy of the 'deci_dist'
            for i in range(net_size):
                # detect global convergence status change
                if converged_all:
                    host_domi = deci_domi[i]
                    converged = True
                    for neighbor in connection_lists[i]:
                        if host_domi != deci_domi[neighbor]:
                            converged = False
                            break
                    if not converged:
                        converged_all = False
                # take unequal weights in the averaging process based on group sizes
                deci_dist[i] = deci_dist_t[i]*group_sizes[i]  # start with host itself
                for neighbor in connection_lists[i]:
                    # accumulate neighbor's distribution
                    deci_dist[i] = deci_dist[i] + deci_dist_t[neighbor]*group_sizes[neighbor]
                # normalize the distribution
                sum_temp = np.sum(deci_dist[i])
                deci_dist[i] = deci_dist[i] / sum_temp
                # # action based on convergence of dominant decision
                # if converged:  # all neighbors have converged with host
                #     # step 1: take equally weighted average on all distributions
                #     # including host and all neighbors
                #     deci_dist[i] = deci_dist_t[i]*1.0  # start with host itself
                #     for neighbor in connection_lists[i]:
                #         # accumulate neighbor's distribution
                #         deci_dist[i] = deci_dist[i] + deci_dist_t[neighbor]
                #     # normalize the distribution such that sum is 1.0
                #     sum_temp = np.sum(deci_dist[i])
                #     deci_dist[i] = deci_dist[i] / sum_temp
                #     # step 2: increase the unipolarity by applying the linear multiplier
                #     # (this step is only for when all neighbors are converged)
                #     # First find the largest difference between two distributions in this block
                #     # of nodes, including the host and all its neighbors.
                #     comb_pool = [i] + connection_lists[i]  # add host to a pool with its neighbors
                #         # will be used to form combinations from this pool
                #     comb_pool_len = len(comb_pool)
                #     dist_diff = []
                #     for j in range(comb_pool_len):
                #         for k in range(j+1, comb_pool_len):
                #             j_node = comb_pool[j]
                #             k_node = comb_pool[k]
                #             dist_diff.append(np.sum(abs(deci_dist[j_node] - deci_dist[k_node])))
                #     dist_diff_max = max(dist_diff)  # maximum distribution difference of all
                #     if dist_diff_max < dist_diff_thres:
                #         # distribution difference is small enough,
                #         # that linear multiplier should be applied to increase unipolarity
                #         dist_diff_ratio = dist_diff_max/dist_diff_thres
                #         # Linear multiplier is generated from value of smaller and larger ends, the
                #         # smaller end is positively related with dist_diff_ratio. The smaller the
                #         # maximum distribution difference, the smaller the dist_diff_ratio, and the
                #         # steeper the linear multiplier.
                #         # '1.0/deci_num' is the average value of the linear multiplier
                #         small_end = 1.0/deci_num * np.power(dist_diff_ratio, dist_diff_power)
                #         large_end = 2.0/deci_num - small_end
                #         # sort the magnitude of the current distribution
                #         dist_temp = np.copy(deci_dist[i])  # temporary distribution
                #         sort_index = range(deci_num)
                #         for j in range(deci_num-1):  # bubble sort, ascending order
                #             for k in range(deci_num-1-j):
                #                 if dist_temp[k] > dist_temp[k+1]:
                #                     # exchange values in 'dist_temp'
                #                     temp = dist_temp[k]
                #                     dist_temp[k] = dist_temp[k+1]
                #                     dist_temp[k+1] = temp
                #                     # exchange values in 'sort_index'
                #                     temp = sort_index[k]
                #                     sort_index[k] = sort_index[k+1]
                #                     sort_index[k+1] = temp
                #         # applying the linear multiplier
                #         for j in range(deci_num):
                #             multiplier = small_end + float(j)/(deci_num-1) * (large_end-small_end)
                #             deci_dist[i][sort_index[j]] = deci_dist[i][sort_index[j]] * multiplier
                #         # normalize the distribution such that sum is 1.0
                #         sum_temp = np.sum(deci_dist[i])
                #         deci_dist[i] = deci_dist[i]/sum_temp
                #     else:
                #         # not applying linear multiplier when distribution difference is large
                #         pass
                # else:  # at least one neighbor has different opinion with host
                #     converged_all = False  # the network is not converged
                #     # take unequal weights in the averaging process based on group sizes
                #     deci_dist[i] = deci_dist_t[i]*group_sizes[i]  # start with host itself
                #     for neighbor in connection_lists[i]:
                #         # accumulate neighbor's distribution
                #         deci_dist[i] = deci_dist[i] + deci_dist_t[neighbor]*group_sizes[neighbor]
                #     # normalize the distribution
                #     sum_temp = np.sum(deci_dist[i])
                #     deci_dist[i] = deci_dist[i] / sum_temp

            # iteration count
            print "iteration {}".format(iter_count)
            iter_count = iter_count + 1

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
        print("maximum steps: {}".format(max(all_steps)))
        print("minimum steps: {}".format(min(all_steps)))
        print("std dev of steps: {}".format(np.std(np.array(all_steps))))


#     result[index] = np.mean(np.array(all_steps))

# print result
