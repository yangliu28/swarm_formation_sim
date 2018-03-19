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
pygame.display.update()

# raw_input("Press <ENTER> to continue")

########## the role assignment algorithm ##########

# Gradient value method for information flow control:
# This gradient method was first used in the early simulations of Kilobot project, for a robot
# to localize itself in a swarm in order to do formation control. The gradient value indicates
# the distance from a particular source, and can be used here to guarantee the information
# flows only forward, instead of backward. If the source robot has gradient value of 0, all the
# robots next to it will have gradient value of 1, then robots next to them have gradient value
# of 2. The robots are forming nested-ring patterns. The message will only be transmitted from
# a low gradient robot to a high gradient one. In this way, one message from source will travel
# through all other robots, without resonating inside the swarm. Since every robot in this
# application will transmit its own message, the robot needs to calculate the gradient value
# of all message sources.

# Distributed gradient map calculation:
# At the beginning, the robots don't know the gradient value to a particular message source.
# The message transmitted from the source robot contains a gradient header of value 0. When
# the robots next to it receives the message for the first time, they find the gradient value
# for this particular source has not been initialized yet, they will increase the gradient
# header by 1, and take it as its gradient value for that source. When transmitting it again,
# if it knows the neighbors' gradient to the same source, it will decide whether to transmit
# the received message based on the gradient(only transmit if neighbor has larger gradient).
# If gradient is unknown, it will transmit the message anyway. If a robot has already decided
# the gradient for the source, and receives the message that contains same gradient value,
# the robot knows in this way the gradient of that neighbor.


# pre-calculated



