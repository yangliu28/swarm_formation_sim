# test program for smoothing open curves or closed loops using moving averaging filter

# will use deviation angle to characterize the shape of both open curve and closed loops
# dof=N-2 for open curves, only nodes inside the curve can have deviation angle
# dof=N-3 for closed loops, but all N deviation angles will be generated for filtering

# comments on effects of moving averaging filter on open curves and closed loops:
# The moving averaging filter works by taking average of the deviation angle of host node
# and two neighbors. It works well on open curves, but not on closed loops. The filtered
# deviation angle won't guarantee the loop to be closed again. The interior angle summation
# rule is not the only constraint for equilaterial polygon, Because the loop has to be an
# equilateral polygon, not any combination of deviation angles can make a closed loop.



# I wasn't able to generalize
# the hidden rule of interior angles for equilaterial polygon.(what a shame, not happy)

# another way of loop shape filter, just take the good old SMA algorithm
# a potential problem with this algorithm is that, in the result formation, the neighbor
# distance is not strictly the preset distance, the error is very small but exist, the
# accumulated error can potentially warp the polygon again, when reconstructing the loop
# from interior angles again.
# Need to test the method for polygons with increasing sides.


import pygame
import math, random, sys
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
# moving filter weight of host node for loop reshape, two neighbors split the rest equaly
filter_weig = 0.618  # should be larger than 1/3, golden ratio seems like good
# SMA parameters, the spring constants should not be over 1.0, so the system can be stable
linear_const = 1.0  # linear spring constant
bend_const = 0.5  # bending spring constant
disp_coef = 0.5  # coefficient from feedback to displacement
# exit threshold for the feedback summation in loop SMA reshape process
fb_sum_threshold = 0.1

# initialize the node positions variables
nodes = np.array([[0.0, 0.0] for i in range(N)])  # originally generated node positions
nodes_f = np.array([[0.0, 0.0] for i in range(N)])  # filtered node positions
# first node starts at origin, second node is node space away on the right
nodes[1] = np.array([node_space, 0.0])
nodes_f[1] = np.array([node_space, 0.0])

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
    # # following commented block is a failed test of loop shape filter
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
    #     # calculate position of the last node
    #     forward_ang = math.atan2(vect_temp[1], vect_temp[0])
    #     rot_ang = math.acos(dist_temp/2/node_space)
    #     forward_ang = reset_radian(forward_ang - rot_ang)  # rotate cw of rot_ang
    #     nodes[N-1] = nodes[N-2] + node_space*np.array([math.cos(forward_ang),
    #                                                    math.sin(forward_ang)])
    #     # check if last node is too close to previous nodes
    #     for i in range(N-1):
    #         if np.linalg.norm(nodes[N-1]-nodes[i]) < node_space:
    #             shape_good = False
    #             break
    #     if not shape_good: continue
    #     # if here, the guess of deviation angles has been approved
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
    # dev_ang_f = np.array([0.0 for i in range(N)])
    # for i in range(N):
    #     dev_ang_f[i] = ((1.0-filter_weig)/2.0 * dev_ang[(i-1)%N] +
    #                     filter_weig * dev_ang[i] +
    #                     (1.0-filter_weig)/2.0 * dev_ang[(i+1)%N])
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

    # # simulation exit control
    # sim_exit = False  # simulation exit flag
    # while not sim_exit:
    #     # exit the program by close window button, or Esc or Q on keyboard
    #     for event in pygame.event.get():
    #         if event.type == pygame.QUIT:
    #             sim_exit = True  # exit with the close window button
    #         if event.type == pygame.KEYUP:
    #             if (event.key == pygame.K_ESCAPE) or (event.key == pygame.K_q):
    #                 sim_exit = True  # exit with ESC key or Q key

    # section for the filter test of close loops
    # The SMA algorithm is used here again to reshape the generated polygon to a better
    # filtered shape.
    dev_ang = []
    while True:  # keep on generating deviation angles until success
        shape_good = True  # indicating if loop is still in good shape
        dev_ang = np.random.normal(loop_dev_mid, loop_dev_std, N-3).tolist()
        forward_ang = 0.0  # starting forward angle
        for i in range(2, N-1):
            forward_ang = reset_radian(forward_ang + dev_ang[i-2])
            nodes[i] = nodes[i-1] + node_space*np.array([math.cos(forward_ang),
                                                         math.sin(forward_ang)])
            # check if new node is too close to previous nodes, except the one right before
            for j in range(i-1):
                if np.linalg.norm(nodes[i]-nodes[j]) < node_space:
                    shape_good = False
                    break
            if not shape_good: break
        if not shape_good: continue
        # check if second last node is too far away from first node
        vect_temp = nodes[0]-nodes[N-2]  # from last second node to first node
        dist_temp = np.linalg.norm(vect_temp)
        if dist_temp > 2*node_space: continue
        # calculate position of the last node
        forward_ang = math.atan2(vect_temp[1], vect_temp[0])
        rot_ang = math.acos(dist_temp/2/node_space)
        forward_ang = reset_radian(forward_ang - rot_ang)  # rotate cw of rot_ang
        nodes[N-1] = nodes[N-2] + node_space*np.array([math.cos(forward_ang),
                                                       math.sin(forward_ang)])
        # check if last node is too close to previous nodes
        for i in range(N-1):
            if np.linalg.norm(nodes[N-1]-nodes[i]) < node_space:
                shape_good = False
                break
        if not shape_good: continue
        # if here, the guess of deviation angles has been approved
        # adding all the missing deviation angles
        # for deviation angle at node 0
        vect_b = nodes[0] - nodes[N-1]  # back vector
        vect_f = nodes[1] - nodes[0]  # front vector
        new_dev = reset_radian(math.atan2(vect_f[1], vect_f[0]) -
                               math.atan2(vect_b[1], vect_b[0]))
        dev_ang.insert(0, new_dev)
        # for deviation angle at node N-2
        vect_b = nodes[N-2] - nodes[N-3]
        vect_f = nodes[N-1] - nodes[N-2]
        dev_ang.append(reset_radian(math.atan2(vect_f[1], vect_f[0]) -
                                    math.atan2(vect_b[1], vect_b[0])))
        # for deviation angle at node N-1
        vect_b = nodes[N-1] - nodes[N-2]
        vect_f = nodes[0] - nodes[N-1]
        dev_ang.append(reset_radian(math.atan2(vect_f[1], vect_f[0]) -
                                    math.atan2(vect_b[1], vect_b[0])))
        break  # finish the constructing the loop, break

    # the moving averaging filter, get the target deviation angles
    dev_ang_t = np.array([0.0 for i in range(N)])
    for i in range(N):
        dev_ang_t[i] = ((1.0-filter_weig)/2.0 * dev_ang[(i-1)%N] +
                        filter_weig * dev_ang[i] +
                        (1.0-filter_weig)/2.0 * dev_ang[(i+1)%N])
    # make sure the summation of deviation angles is 2*pi
    dev_sum = np.sum(dev_ang_t)
    dev_ang_t = dev_ang_t + (2*math.pi-dev_sum)/N

    # copy the old node positions and deviation angles to new loop, for starting with
    nodes_f = np.copy(nodes)  # deep copy
    dev_ang_f = np.copy(dev_ang)

    # shift the two loops to its geometric center, rotate a random angle
    # then shift the two loops agian to top and bottom halves of the window
    rot_ang = random.uniform(-math.pi, math.pi)  # two loops rotate the same angle
    rot_mat = np.array([[math.cos(rot_ang), math.sin(rot_ang)],
                        [-math.sin(rot_ang), math.cos(rot_ang)]])  # rotation matrix
    # shift old loop to origin
    geometric_center = np.mean(nodes, axis=0)
    nodes = nodes - geometric_center  # shift to its geometric center
    nodes = np.dot(nodes, rot_mat)  # rotate curve in angle of rot_ang
    # shift old loop to top halve of the window
    nodes = nodes + np.array([world_size[0]/2, 3*world_size[1]/4])  # to top half
    # do all the above for the new loop
    geometric_center = np.mean(nodes_f, axis=0)
    nodes_f = nodes_f - geometric_center  # shift to its geometric center
    nodes_f = np.dot(nodes_f, rot_mat)  # rotate loop in angle of rot_ang
    nodes_f = nodes_f + np.array([world_size[0]/2, world_size[1]/4])  # to bottom half

    # the loop, SMA algorithm to reshape the bottom polygon
    sim_exit = False  # simulation exit flag
    sim_pause = False  # simulation pause flag
    iter_count = 0
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

        # update the dynamic neighbor distances
        dist_neigh = [np.linalg.norm(nodes_f[(i+1)%N]-nodes_f[i]) for i in range(N)]
        # update the dynamic deviation angles
        for i in range(N):
            vect_l = nodes_f[(i-1)%N] - nodes_f[i]
            vect_r = nodes_f[(i+1)%N] - nodes_f[i]
            inter_ang = math.acos((vect_l[0]*vect_r[0] + vect_l[1]*vect_r[1])/
                                  (dist_neigh[(i-1)%N] * dist_neigh[i]))
            dev_ang_f[i] = math.pi - inter_ang  # from interior angle to deviation angle
            if (vect_r[0]*vect_l[1] - vect_r[1]*vect_l[0]) < 0:
                dev_ang_f[i] = -dev_ang_f[i]  # reverse side of deviation
        # the feedback from all spring effects
        fb_vect = [np.zeros([1,2]) for i in range(N)]
        for i in range(N):
            i_l = (i-1)%N
            i_r = (i+1)%N
            # unit vector from host to left
            vect_l = (nodes_f[i_l]-nodes_f[i])/dist_neigh[i_l]
            # unit vector from host to right
            vect_r = (nodes_f[i_r]-nodes_f[i])/dist_neigh[i]
            # unit vector along central axis pointing inward the polygon
            vect_lr = nodes_f[i_r]-nodes_f[i_l]  # from left neighbor to right
            dist_temp = math.sqrt(vect_lr[0]*vect_lr[0]+vect_lr[1]*vect_lr[1])
            vect_in = np.array([-vect_lr[1]/dist_temp, vect_lr[0]/dist_temp])  # rotate ccw pi/2

            # add the pulling or pushing effect from left neighbor
            fb_vect[i] = fb_vect[i] + (dist_neigh[i_l]-node_space) * linear_const * vect_l
            # add the pulling or pushing effect from right neighbor
            fb_vect[i] = fb_vect[i] + (dist_neigh[i]-node_space) * linear_const * vect_r
            # add the bending effect initialized by the host node itself
            fb_vect[i] = fb_vect[i] + ((dev_ang_f[i]-dev_ang_t[i]) * bend_const * vect_in)

            # update one step of position
            nodes_f[i] = nodes_f[i] + disp_coef * fb_vect[i]

        # print and update iteration count
        print "iteration count {}, ".format(iter_count),
        iter_count = iter_count + 1

        # use delay to slow down loop frequency
        time.sleep(0.2)  # second

        # graphics update
        screen.fill(background_color)
        disp_pos = [[0,0] for i in range(N)]
        for i in range(N):
            disp_pos[i] = world_to_display(nodes[i], world_size, screen_size)
            pygame.draw.circle(screen, node_color, disp_pos[i], node_size, 0)
        pygame.draw.circle(screen, node_color, disp_pos[0], int(node_size*1.5), 1)
        for i in range(N-1):
            pygame.draw.line(screen, node_color, disp_pos[i], disp_pos[i+1])
        pygame.draw.line(screen, node_color, disp_pos[N-1], disp_pos[0])
        # repeat above for the new filtered curve
        for i in range(N):
            disp_pos[i] = world_to_display(nodes_f[i], world_size, screen_size)
            pygame.draw.circle(screen, node_color, disp_pos[i], node_size, 0)
        pygame.draw.circle(screen, node_color, disp_pos[0], int(node_size*1.5), 1)
        for i in range(N-1):
            pygame.draw.line(screen, node_color, disp_pos[i], disp_pos[i+1])
        pygame.draw.line(screen, node_color, disp_pos[N-1], disp_pos[0])
        pygame.display.update()

        # auto-exit mechanism, summation of feedback is smaller than threshold
        fb_sum = 0.0
        for i in range(N): fb_sum = fb_sum + np.linalg.norm(fb_vect[i])
        print("fb_sum {}".format(fb_sum))
        if fb_sum < fb_sum_threshold:
            print("exit SMA reshape filter")
            break

    # if exit from above loop(forced exit from keyboard, or auto-exit condition satisfied)
    # then reconstruct the polygon from SMA result
    # update the neighbor distances
    dist_neigh = [np.linalg.norm(nodes_f[(i+1)%N]-nodes_f[i]) for i in range(N)]
    # calculate the resulting deviation angle set
    for i in range(N):
        vect_l = nodes_f[(i-1)%N] - nodes_f[i]
        vect_r = nodes_f[(i+1)%N] - nodes_f[i]
        inter_ang = math.acos((vect_l[0]*vect_r[0] + vect_l[1]*vect_r[1])/
                              (dist_neigh[(i-1)%N] * dist_neigh[i]))
        dev_ang_f[i] = math.pi - inter_ang  # from interior angle to deviation angle
        if (vect_r[0]*vect_l[1] - vect_r[1]*vect_l[0]) < 0:
            dev_ang_f[i] = -dev_ang_f[i]  # reverse side of deviation
    # reconstruct the polygon
    nodes_f[0] = np.array([0.0, 0.0])
    nodes_f[1] = np.array([node_space, 0.0])
    forward_ang = 0.0
    for i in range(2, N):  # for posiiton of node i
        forward_ang = reset_radian(forward_ang + dev_ang_f[i-1])
        nodes_f[i] = nodes_f[i-1] + node_space*np.array([math.cos(forward_ang),
                                                         math.sin(forward_ang)])

    # position shift of this loop
    geometric_center = np.mean(nodes_f, axis=0)
    nodes_f = nodes_f - geometric_center  # shift to its geometric center
    nodes_f = np.dot(nodes_f, rot_mat)  # use the same rotation angle from above
    nodes_f = nodes_f + np.array([world_size[0]/2, world_size[1]/4])  # to bottom half

    # graphics update
    screen.fill(background_color)
    disp_pos = [[0,0] for i in range(N)]
    for i in range(N):
        disp_pos[i] = world_to_display(nodes[i], world_size, screen_size)
        pygame.draw.circle(screen, node_color, disp_pos[i], node_size, 0)
    pygame.draw.circle(screen, node_color, disp_pos[0], int(node_size*1.5), 1)
    for i in range(N-1):
        pygame.draw.line(screen, node_color, disp_pos[i], disp_pos[i+1])
    pygame.draw.line(screen, node_color, disp_pos[N-1], disp_pos[0])
    # repeat above for the new filtered curve
    for i in range(N):
        disp_pos[i] = world_to_display(nodes_f[i], world_size, screen_size)
        pygame.draw.circle(screen, node_color, disp_pos[i], node_size, 0)
    pygame.draw.circle(screen, node_color, disp_pos[0], int(node_size*1.5), 1)
    for i in range(N-1):
        pygame.draw.line(screen, node_color, disp_pos[i], disp_pos[i+1])
    pygame.draw.line(screen, node_color, disp_pos[N-1], disp_pos[0])
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


