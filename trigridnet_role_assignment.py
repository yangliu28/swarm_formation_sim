# This simulation tests a distributed role assignment algorithm, for robot swarm starting
# at arbitrary triangle grid network, to perform one-to-one role assignment. The assigned
# roles are abstract and represented as index numbers. The primary goal of this role
# assignment algorithm is to have all robots agree on the one-to-one assignment scheme,
# instead of coming up with the best assignment scheme.

# Inter-robot communication is used to let one robot know the status of another robot that
# is not directly connected. Enabling message convey is what I consider the most convenient
# way to resolve conflict when two robots decide on the same assigned role. Although in this
# way, it is possible for each robot to have the information of all the other robots, and
# thus it could become a distributed master control algorithm, we still consider the proposed
# algorithm a distributed one. Because we limit the information each robot will store, and
# the local computation performed, so the algorithm is still robust.

# The communication delay is also reflected in the simulation, one step in simulation will
# update one step of message transmission. Color is used to highlight the conflicts in the
# role assignment, instead of showing convergence of decisions.


import pygame
import matplotlib.pyplot as plt
from trigridnet_generator import *
from formation_functions import *
import numpy as np
import os, getopt, sys, time

import pandas as pd
pd.set_option('display.max_columns', None)
# print pd.DataFrame(gradients, range(net_size), range(net_size))

net_folder = 'trigrid-networks'
net_filename = '30-1'  # default network
net_size = 30  # default network size
net_filepath = os.path.join(os.getcwd(), net_folder, net_filename)

nobargraph = False

# read command line options
try:
    opts, args = getopt.getopt(sys.argv[1:], 'f:', ['nobargraph'])
except getopt.GetoptError as err:
    print str(err)
    sys.exit()
for opt,arg in opts:
    if opt == '-f':
        net_filename = arg
        net_filepath = os.path.join(os.getcwd(), net_folder, net_filename)
        # check if this file exists
        if not os.path.isfile(net_filepath):
            print "{} does not exist".format(net_filename)
            sys.exit()
        # parse the network size
        net_size = int(net_filename.split('-')[0])
    elif opt == '--nobargraph':
        nobargraph = True

# read the network from file
nodes_tri = []
# nodes_tri: node positions in the triangle grid network
# nodes_cart: node positions in Cartesian coordinates
# nodes_disp: node positions for display
f = open(net_filepath, 'r')
new_line = f.readline()
while len(new_line) != 0:
    pos_str = new_line[0:-1].split(' ')
    pos = [int(pos_str[0]), int(pos_str[1])]
    nodes_tri.append(pos)
    new_line = f.readline()

# generate the connection matrix, 0 for not connected, 1 for connected
connections = np.zeros((net_size, net_size))
for i in range(net_size):
    for j in range(i+1, net_size):
        diff_x = nodes_tri[i][0] - nodes_tri[j][0]
        diff_y = nodes_tri[i][1] - nodes_tri[j][1]
        if abs(diff_x) + abs(diff_y) == 1 or diff_x * diff_y == -1:
            connections[i,j] = 1
            connections[j,i] = 1
# connection list indexed by node
connection_lists = []
for i in range(net_size):
    connection_lists.append(list(np.where(connections[i] == 1)[0]))

# plot the network as dots and lines in pygame window
pygame.init()
font = pygame.font.SysFont("Cabin", 14)
nodes_cart = np.array([trigrid_to_cartesian(pos) for pos in nodes_tri])
# find appropriate window size to fit current network
(xmin, ymin) = np.amin(nodes_cart, axis=0)
(xmax, ymax) = np.amax(nodes_cart, axis=0)
clearance = 2.0
world_size = (xmax-xmin + clearance, ymax-ymin + clearance)
pixels_per_length = 50  # corresponds to 1.0 length in cartesian world
screen_size = (int(round(world_size[0] * pixels_per_length)),
               int(round(world_size[1] * pixels_per_length)))
node_size = 8
line_width = 4
# colors
color_white = (255,255,255)
color_black = (0,0,0)
distinct_color_set = ((230,25,75), (60,180,75), (255,225,25), (0,130,200), (245,130,48),
    (145,30,180), (70,240,240), (240,50,230), (210,245,60), (250,190,190),
    (0,128,128), (230,190,255), (170,110,40), (255,250,200), (128,0,0),
    (170,255,195), (128,128,0), (255,215,180), (0,0,128), (128,128,128))
# simulation window
icon = pygame.image.load("icon_geometry_art.jpg")
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Role Assignment on 2D Triangle Grid Network")

# node display positions
center_temp = (nodes_cart.max(axis=0) + nodes_cart.min(axis=0))/2.0
nodes_cart = nodes_cart - center_temp + (world_size[0]/2.0, world_size[1]/2.0)
nodes_disp = [world_to_display(nodes_cart[i], world_size, screen_size)
              for i in range(net_size)]

# draw the network for the first time
screen.fill(color_white)
for i in range(net_size):
    for j in range(i+1, net_size):
        if connections[i,j]:
            pygame.draw.line(screen, color_black, nodes_disp[i], nodes_disp[j], line_width)
for i in range(net_size):
    pygame.draw.circle(screen, color_black, nodes_disp[i], node_size, 0)
    text = font.render(str(i), True, color_black)
    screen.blit(text, (nodes_disp[i][0]+12, nodes_disp[i][1]-12))
pygame.display.update()

########## the role assignment algorithm ##########

# Gradient value method for information flow control:
# This gradient method was first used in the early simulations of Kilobot project, for a robot
# to localize itself in a swarm in order to do formation control. The gradient value indicates
# the distance from a particular source, and can be used here to guarantee the information
# flows only forward, instead of backward. If the source node has gradient value of 0, all the
# nodes next to it will have gradient value of 1, then nodes next to them have gradient value
# of 2. These nodes are forming nested-ring patterns. The message will only be transmitted from
# a low gradient node to a high gradient one. In this way, one message from source will travel
# through all other nodes, without resonating infinitely inside the network. Since every node
# in this application will transmit its own message, the node needs to calculate the gradient
# value of all message sources.

# To construct the gradient values in a distributed way, when messages are received from a new
# message source, the node will take the minimum gradient value plus 1 as its gradient for
# that message source. In this way each node will build the gradient values on-the-go for any
# other message source. A little more complicated algorithm for constructing gradient values
# is also developed to deal with any unstable communication for message transmissions.

# However, to simplify the role assignment simulation, the gradient map is pre-calculated.
# Although I could use algorithm similar in the holistic dependency calculation, a new one that
# searching the shortest path between any two nodes is investigated in the following.
gradients = np.copy(connections)  # build gradient map on the connection map
    # gradients[i,j] indicates gradient value of node j, to message source i
pool_gradient = 1  # gradients of the connections in the pool
pool_conn = {}
for i in range(net_size):
    pool_conn[i] = connection_lists[i][:]  # start with gradient 1 connections
while len(pool_conn.keys()) != 0:
    source_deactivate = []
    for source in pool_conn:
        targets_temp = []  # the new targets
        for target in pool_conn[source]:
            for target_new in connection_lists[target]:
                if target_new == source: continue  # skip itself
                if gradients[source, target_new] == 0:
                    gradients[source, target_new] = pool_gradient + 1
                    targets_temp.append(target_new)
        if len(targets_temp) == 0:
            source_deactivate.append(source)
        else:
            pool_conn[source] = targets_temp[:]  # update with new targets
    for source in source_deactivate:
        pool_conn.pop(source)  # remove the finished sources
    pool_gradient = pool_gradient + 1

# calculate the relative gradient values
gradients_rel = []
    # gradients_rel[i][j,k] refers to gradient of k relative to j with message source i
for i in range(net_size):  # message source i
    gradient_temp = np.zeros((net_size, net_size))
    for j in range(net_size):  # in the view point of j
        gradient_temp[j] = gradients[i] - gradients[i,j]
    gradients_rel.append(gradient_temp)

# list the neighbors a robot can send message to regarding a message source
neighbors_send = [[[] for j in range(net_size)] for i in range(net_size)]
    # neighbors_send[i][j][k] means, if message from source i is received in j,
    # it could be send to k
for i in range(net_size):  # message source i
    for j in range(net_size):  # in the view point of j
        for neighbor in connection_lists[j]:
            if gradients_rel[i][j,neighbor] == 1:
                neighbors_send[i][j].append(neighbor)

# generate the initial preference distribution
pref_dist = np.random.rand(net_size, net_size)
sum_temp = np.sum(pref_dist, axis=1)
for i in range(net_size):
    pref_dist[i,:] = pref_dist[i,:] / sum_temp[i]
roles = np.argmax(pref_dist)  # the chosen role

# received message container for all robots
message_rx = [[] for i in range(net_size)]
# for each message entry, it containts:
    # ID of message source
    # its preferred position
    # probability on preferred position
    # time stamp
# all robots transmit once their chosen role before the loop
transmission_total = 0  # count message transmissions for each iteration
iter_count = 0  # also used as time stamp in message
for source in range(net_size):
    message_temp = [source, roles[source], pref_dist[roles[source]], iter_count]
    for target in connection_lists[source]:  # send to all neighbors
        message_rx[target].append(message_temp)
        transmission_total = transmission_total + 1

# solid circle for undetermined role assignment scheme
# solid circle with color for conflicting in role assignment
# empty circle for converged role assignment scheme

assignment_chosen_position = [[] for i in range(net_size)]
assignment_choosing_robots

sim_exit = False
sim_pause = False
time_now = pygame.time.get_ticks()
time_last = time_now
time_period = 300
speed_control = True  # set False to skip speed control
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

    # process the received messages
    # transfer messages to the processing buffer, and empty the receiver
    message_rx_buf = [[[k for k in j] for j in i] for i in message_rx]
    message_rx = [[] for i in range(net_size)]
    for i in range(net_size):




# hold the simulation window to exit manually
raw_input("Role assignment finished, press <ENTER> to exit")


