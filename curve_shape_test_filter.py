# test program for smoothing open curves or closed loops using moving averaging filter

# will use deviation angle to characterize the shape of both open curve and closed loops
# dof=N-2 for open curves, only nodes inside the curve can have deviation angle
# dof=N-3 for closed loops, but all N deviation angles will be generated for filtering

# comments on effects of moving averaging filter on open curves and closed loops:
# The moving averaging filter works by taking average of the deviation angle of host node
# and two neighbors. It works well on open curves, but not on closed loops. The filtered
# deviation angle won't guarantee the loop to be closed again.

# Because the loop has
# to be an equilateral polygon, not any combination of deviation angles can make a closed
# loop.

# another way of loop shape filter, just take the good old SMA algorithm
# a potential problem with this algorithm is that, in the result formation, the neighbor
# distance is not strictly the preset distance, the error is very small but exist, the
# accumulated error can potentially warp the polygon again, when reconstructing the loop
# from interior angles again.
# Need to test the method for polygons with increasing sides.

import pygame
import math, random
import numpy as np
from formation_functions import *

pygame.init()  # initialize pygame
screen_size = (600, 900)
background_color = (0,0,0)  # black background
node_color = (255,0,0)  # red for nodes
node_size = 5  # radius of dot in pixels in display coordinates

# set up the simulation window
icon = pygame.image.load("icon_geometry_art.jpg")
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Curve Shape Filter Test")

# continuous world coordinates, origin at left bottom corner
# x pointing right, y pointing up
world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])

# general variables specifying this simulation
N = 30  # number of nodes on the open curve or loop, at least 4 for the loop
node_space = 4.0  # node space in world coordinates
curve_dev_half = math.pi/2  # half range of randomly generated deviation angle for curve
# curve's middle point of deviation angle is 0
loop_dev_mid = 2*math.pi/N  # loop's middle point of deviation angle
loop_dev_std = math.pi/8  # standard deviation of deviation angle for closed loop

# initialize the node positions variables
nodes = np.array([[0,0] for i in range(N)])  # originally generated node positions
nodes_f = np.array([[0,0] for i in range(N)])  # filtered node positions
# first node starts at origin, second node is node space away on the right
nodes[1] = [node_space, 0]
nodes_f[1] = [node_space, 0]

# change here to switch between open curve testing and closed loop testing
simulation_mode = 1  # 0 for open curve, 1 for close loop

if simulation_mode == 0:
	# section for the filter test of the open curve
    dev_ang = [0 for i in range(N-2)]
    forward_ang = 0.0  # forward angle of current line segment
    # randomly generate deviation angles for nodes not on two ends, and construct the curve
    for i in range(2, N):
        dev_satisfied = False  # flag indicating if a generated deviation is good
        while not dev_satisfied:
            # deviation angle of node i-1
            dev_ang[i-2] = random.uniform(-curve_dev_half, curve_dev_half)
            forward_ang_t = reset_radian(forward_ang + dev_ang[i-2])  # temporary
            # calculate position of node i
            nodes[i] = nodes[i-1] + node_space*np.array([math.cos(forward_ang_t),
                                                         math.sin(forward_ang_t)])
            # check if it is too close to previous nodes, except the one right before
            dev_satisfied = True  # will change to false if close nodes are detected
            for j in range(i-1):
                if np.linalg.norm(nodes[i]-nodes[j]) < node_space:
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
    # construct the new curve with filtered deviation angles
    # it's possible but not likely that new curve has close non-neighbors nodes
    forward_ang = 0.0
    for i in range(2, N):
        forward_ang = reset_radian(forward_ang + dev_ang_f[i-2])
        nodes_f[i] = nodes_f[i-1] + node_space*np.array([math.cos(forward_ang),
                                                         math.sin(forward_ang)])

    # shift the two curves to its geometric center, rotate a random angle
    # then shift the two curves again to top and bottom halves of the window
    rot_ang = random.uniform(-math.pi, math.pi)  # two curves rotate the same angle
    rot_mat = np.array([[math.cos(rot_ang), math.sin(rot_ang)],
                        [-math.sin(rot_ang), math.cos(rot_ang)]])  # rotation matrix
    # shift old curve to origin
    geometric_center = np.mean(nodes, axis=0)
    nodes = nodes - geometric_center  # shift to its geometric center
    nodes = np.dot(nodes, rot_mat)  # rotate curve in angle of rot_ang
    # shift old curve to top halve of the window
    nodes = nodes + np.array([world_size[0]/2, 3*world_size[1]/4])
    # do all the above for the new filtered curve
    geometric_center = np.mean(nodes_f, axis=0)
    nodes_f = nodes_f - geometric_center  # shift to its geometric center
    nodes_f = np.dot(nodes_f, rot_mat)  # rotate curve in angle of rot_ang
    nodes_f = nodes_f + np.array([world_size[0]/2, world_size[1]/4])  # to bottom half

    # visualize the result of the filter
    screen.fill(background_color)
    disp_pos = [[0,0] for i in range(N)]
    for i in range(N):
        disp_pos[i] = world_to_display(nodes[i], world_size, screen_size)
        pygame.draw.circle(screen, node_color, disp_pos[i], node_size, 0)
    pygame.draw.circle(screen, node_color, disp_pos[0], int(node_size*1.5), 1)
    for i in range(N-1):
        pygame.draw.line(screen, node_color, disp_pos[i], disp_pos[i+1])
    # repeat above for the new filtered curve
    for i in range(N):
        disp_pos[i] = world_to_display(nodes_f[i], world_size, screen_size)
        pygame.draw.circle(screen, node_color, disp_pos[i], node_size, 0)
    pygame.draw.circle(screen, node_color, disp_pos[0], int(node_size*1.5), 1)
    for i in range(N-1):
        pygame.draw.line(screen, node_color, disp_pos[i], disp_pos[i+1])
    pygame.display.update()

    # simulation exit control
    sim_exit = False  # simulation exit flag
    while not sim_exit:
        # exit the program by close window button, or Esc or Q on keyboard
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sim_exit = True  # exit with the close window button
            if event.type == pygame.KEYUP:
                if (event.key == pygame.K_ESCAPE) or (event.key == pygame.K_q):
                    sim_exit = True  # exit with ESC key or Q key

else:
    # following commented block is a failed test of loop shape filter
    # # section for the filter test of close loops
    # # A new way of generating deviation angle is used here. Since random generation
    # # has no guarantee the mean value will be close to the middle of the range, a normal
    # # distribution will do a better work here.
    # dev_ang = []
    # while True:  # keep on generating deviation angles until success
    #     shape_good = True  # indicating if loop is still in good shape
    #     dev_ang = np.random.normal(loop_dev_mid, loop_dev_std, N-3).tolist()
    #     forward_ang = 0.0  # starting forward angle
    #     for i in range(2, N-1):
    #         forward_ang = reset_radian(forward_ang + dev_ang[i-2])
    #         nodes[i] = nodes[i-1] + node_space*np.array([math.cos(forward_ang),
    #                                                      math.sin(forward_ang)])
    #         # check if new node is too close to previous nodes, except the one right before
    #         for j in range(i-1):
    #             if np.linalg.norm(nodes[i]-nodes[j]) < node_space:
    #                 shape_good = False
    #                 break
    #         if not shape_good: break
    #     if not shape_good: continue
    #     # check if second last node is too far away from first node
    #     vect_temp = nodes[0]-nodes[N-2]  # from last second node to first node
    #     dist_temp = np.linalg.norm(vect_temp)
    #     if dist_temp > 2*node_space: continue
    #     # if here, the guess of deviation angles has been approved
    #     # calculate position of the last node
    #     forward_ang = math.atan2(vect_temp[1], vect_temp[0])
    #     rot_ang = math.acos(dist_temp/2/node_space)
    #     forward_ang = reset_radian(forward_ang - rot_ang)  # rotate cw of rot_ang
    #     nodes[N-1] = nodes[N-2] + node_space*np.array([math.cos(forward_ang),
    #                                                    math.sin(forward_ang)])
    #     # adding all the missing deviation angles
    #     # for deviation angle at node 0
    #     vect_b = nodes[0] - nodes[N-1]  # back vector
    #     vect_f = nodes[1] - nodes[0]  # front vector
    #     new_dev = reset_radian(math.atan2(vect_f[1], vect_f[0]) -
    #                            math.atan2(vect_b[1], vect_b[0]))
    #     dev_ang.insert(0, new_dev)
    #     # for deviation angle at node N-2
    #     vect_b = nodes[N-2] - nodes[N-3]
    #     vect_f = nodes[N-1] - nodes[N-2]
    #     dev_ang.append(reset_radian(math.atan2(vect_f[1], vect_f[0]) -
    #                                 math.atan2(vect_b[1], vect_b[0])))
    #     # for deviation angle at node N-1
    #     vect_b = nodes[N-1] - nodes[N-2]
    #     vect_f = nodes[0] - nodes[N-1]
    #     dev_ang.append(reset_radian(math.atan2(vect_f[1], vect_f[0]) -
    #                                 math.atan2(vect_b[1], vect_b[0])))
    #     break  # finish the constructing the loop, break

    # # the moving averaging filter
    # dev_ang_f = np.array([0 for i in range(N)])
    # for i in range(N):
    #     i_l = (i-1)%N  # neighbor on left
    #     i_r = (i+1)%N  # neighbor on right
    #     dev_ang_f[i] = (dev_ang[i_l] + dev_ang[i] + dev_ang[i_r])/3
    # # make sure the summation of deviation angles is 2*pi
    # dev_sum = np.sum(dev_ang_f)
    # dev_ang_f = dev_ang_f + (2*math.pi-dev_sum)/N
    # # construct the new loop with filtered deviation angles
    # forward_ang = 0.0
    # for i in range(2, N):
    #     forward_ang = reset_radian(forward_ang + dev_ang_f[i-1])
    #     nodes_f[i] = nodes_f[i-1] + node_space*np.array([math.cos(forward_ang),
    #                                                      math.sin(forward_ang)])

    # # shift the two loops to its geometric center, rotate a random angle
    # # then shift the two loops agian to top and bottom halves of the window
    # rot_ang = random.uniform(-math.pi, math.pi)  # two loops rotate the same angle
    # rot_mat = np.array([[math.cos(rot_ang), math.sin(rot_ang)],
    #                     [-math.sin(rot_ang), math.cos(rot_ang)]])  # rotation matrix
    # # shift old loop to origin
    # geometric_center = np.mean(nodes, axis=0)
    # nodes = nodes - geometric_center  # shift to its geometric center
    # nodes = np.dot(nodes, rot_mat)  # rotate curve in angle of rot_ang
    # # shift old loop to top halve of the window
    # nodes = nodes + np.array([world_size[0]/2, 3*world_size[1]/4])
    # # do all the above for the new filtered curve
    # geometric_center = np.mean(nodes_f, axis=0)
    # nodes_f = nodes_f - geometric_center  # shift to its geometric center
    # nodes_f = np.dot(nodes_f, rot_mat)  # rotate loop in angle of rot_ang
    # nodes_f = nodes_f + np.array([world_size[0]/2, world_size[1]/4])  # to bottom half

    # # visualize the result of the filter
    # screen.fill(background_color)
    # disp_pos = [[0,0] for i in range(N)]
    # for i in range(N):
    #     disp_pos[i] = world_to_display(nodes[i], world_size, screen_size)
    #     pygame.draw.circle(screen, node_color, disp_pos[i], node_size, 0)
    # pygame.draw.circle(screen, node_color, disp_pos[0], int(node_size*1.5), 1)
    # for i in range(N-1):
    #     pygame.draw.line(screen, node_color, disp_pos[i], disp_pos[i+1])
    # pygame.draw.line(screen, node_color, disp_pos[N-1], disp_pos[0])
    # # repeat above for the new filtered curve
    # for i in range(N):
    #     disp_pos[i] = world_to_display(nodes_f[i], world_size, screen_size)
    #     pygame.draw.circle(screen, node_color, disp_pos[i], node_size, 0)
    # pygame.draw.circle(screen, node_color, disp_pos[0], int(node_size*1.5), 1)
    # for i in range(N-1):
    #     pygame.draw.line(screen, node_color, disp_pos[i], disp_pos[i+1])
    # pygame.draw.line(screen, node_color, disp_pos[N-1], disp_pos[0])
    # pygame.display.update()

    # section for the filter test of close loops
    # The SMA algorithm is used here again to reshape the generated polygon to a better
    # filtered shape. The problem with the failed method above is that, the interior angle
    # summation rule is not the only constrait for equilaterial polygon. Not any combination
    # of interior angles can guarantee a closed polygon. Since I wasn't able to generalize
    # the hidden rule of interior angles for equilaterial polygon(what a shame, not happy),
    # I'm giving SMA algorithm a try




