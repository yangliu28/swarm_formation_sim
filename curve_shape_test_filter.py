# test program for smoothing open curves or closed loops using moving averaging filter

# will use deviation angle to characterize the shape of both open curve and closed loops
# dof=N-2 for open curves, only nodes inside the curve can have deviation angle
# dof=N-3 for closed loops, but all N deviation angles will be generated for filtering

# also rotate the curves or loops so it look more randomly placed

import pygame
import math, random
import numpy as np

from formation_functions import *

pygame.init()
screen_size = (600, 800)
background_color = (0,0,0)  # black background
node_color = (255,0,0)  # red for nodes
node_size = 5  # radius of dot in pixels in display coordinates

# continuous world coordinates, origin at left bottom corner
# x pointing right, y pointing up
world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])

# general variables specifying the simulation
N = 30  # number of nodes on the open curve or loop
node_space = 4.0  # node space in world coordinates
dev_lower = math.pi/3  # lower limit of deviation angle
dev_upper = 2*math.pi - dev_lower  # upper limit, symmetric about pi

# initialize the node positions variables
nodes = np.array([[0,0] for i in range(N)])  # originally generated node positions
nodes_f = np.array([[0,0] for i in range(N)])  # filtered node positions
# first node starts at origin, second node is node space away on the right
nodes[1] = [node_space, 0]
nodes_f[1] = [node_space, 0]

# change here to switch between open curve testing and closed loop testing
simulation_mode = 0  # 0 for open curve, 1 for close loop



if simulation_mode == 0:
	# section for the filter test of the open curve
    dev_ang = [0 for i in range(N-2)]
    forward_ang = 0.0  # forward angle of current line segment
    # randomly generate deviation angles for nodes not on two ends, and construct the curve
    for i in range(2, N):
        dev_satisfied = False  # flag indicating if a generated deviation is good
        while not dev_satisfied:
            dev_ang[i-2] = random.uniform(dev_lower, dev_upper)  # deviation angle on node i-1
            forward_ang_t = reset_radian(forward_ang + dev_ang[i-2])  # temporary
            # calculate position of node i
            nodes[i] = nodes[i-1] + node_space*np.array([math.cos(forward_ang_t),
                                                         math.sin(forward_ang_t)])
            # check if it is too close to all previous neighbors, except the one right before
            dev_satisfied = True  # will change to false if close nodes are detected
            for j in range(i-1):
                if np.linalg.norm(nodes[j]-nodes[i]) < node_space:
                    dev_satisfied = False
                    break
            # set the forward angle to this decided one
            if dev_satisfied: forward_ang = forward_ang_t

    # the moving averaging filter
    dev_ang_f = [0 for i in range(N-2)]  # filtered deviation angle
    for i in range(N-2):
        # if on two ends, take average with the only neighbor
        if i == 0: dev_ang_f[0] = (dev_ang[0]+dev_ang[1])/2
        elif i == N-3: dev_ang_f[N-3] = (dev_ang[N-4]+dev_ang[N-3])/2
        else: dev_ang_f[i] = (dev_ang[i-1]+dev_ang[i]+dev_ang[i+1])/3
    # construct the new curve with filtered deviation angle
    # it's possible but not likely that new curve has close non-neighbors nodes
    forward_ang = 0.0
    for i in range(2, N):
        forward_ang = reset_radian(forward_ang + dev_ang_f[i-2])
        nodes_f[i] = nodes_f[i-1] + node_space*np.array([math.cos(forward_ang),
                                                         maht.sin(forward_ang)])

    # shift the two curves to its geometric center, rotate a random angle
    # then shift the two curves again to top and bottom halves of the window
    geometric_center = np.mean(nodes, axis=0)
    nodes = nodes - geometry_center  # shift to its geometric center
    rotate_ang = random.uniform(-math.pi, math.pi)


    # visualize the result of the filter
    screen.fill(background_color)
    disp_pos = [[0,0] for i in range(N)]
    for i in range(N):
        disp_pos[i] = world_to_display(nodes[i], world_size, screen_size)
        pygame.draw.circle(screen, node_color, disp_pos)


else:
	# section for the filter test of close loops




