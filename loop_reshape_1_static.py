# static version of the probabilistic approach for the loop reshape formation

# random equilateral polygon generating method:
# Given all the side length of a n-side polygon, it can still varies in shape. The number of
# degree of freedom is (n-3). Equilateral polygon also has fixed side length, the way to
# generate such random polygon is to treat first (n-3) number of interior angles as DOFs.
# The rest of the polygon can be determined uniquely in either a convex or concave triangle.
# To make sure the polygon can be formed, the guesses for interior angles can not be too
# wild. Here a normal distribution is used to constrain the guesses within an appropriate
# range of the interior angle of corresponding regular polygon.
# Another check during the polygon generating is there should be no collapse or intersecting
# inside the polygon. That is, any non-neighbor nodes should not be closer than loop space.


# store data of one formation for the target


# show all the evolution of the preferability of all nodes
# plot the graph in histogram with matplotlib


import pygame
import math, random, numpy
from formation_functions import *

pygame.init()

# parameters for display, window origin is at left up corner
screen_size = (800, 600)  # width and height
background_color = (0,0,0)  # black background
robot_color = (255,0,0)  # red for robot and the line segments
robot_color_s = (255,153,153)  # pink for the start robot
robot_size = 5  # robot modeled as dot, number of pixels for radius

# set up the simulation window and surface object
icon = pygame.image.load("icon_geometry_art.jpg")
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Loop Reshape (static version)")

# for physics, continuous world, origin is at left bottom corner, starting (0, 0),
# with x axis pointing right, y axis pointing up.
# It's more natural to compute the physics in right hand coordiante system.
world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])

# variables to configure the simulation
poly_n = 30  # number of nodes for the polygon, also the robot quantity, at least 3
loop_space = 4.0  # side length of the equilateral polygon
# the following are for the guessing of the free interior angles
int_angle_reg = math.pi - 2*math.pi/poly_n  # interior angle of regular polygon
# standard deviation of the normal distribution of the guesses
int_angle_dev = (int_angle_reg - math.pi/3)/5

# instantiate the robots variable for the positions
nodes = []  # position of all nodes, index is the robot's identification
nodes.append([0, 0])  # first node start at origin
nodes.append([loop_space, 0])  # second node is loop_space distance away on the right
for i in range(2, poly_n):
    nodes.append([0, 0])  # fill the rest nodes with [0,0]

# process for generating the random equilateral polygon, two stages
poly_success = False  # flag for succeed in generating the polygon
trial_count = 0  # record number of trials until one is successful
while not poly_success:
    trial_count = trial_count + 1
    print "trial {}: ".format(trial_count),
    # continue trying until all the guesses can forming the desired polygon
    # stage 1: guessing all the free interior angles
    dof = poly_n-3  # number of free interior angles to be randomly generated
    if dof > 0:  # only continue guessing if at least one free interior angle
        # generate all the guesses from a normal distribution
        int_guesses = numpy.random.normal(int_angle_reg, int_angle_dev, dof)
        ori_current = 0  # orientation of the line segment
        no_irregular = True  # flag indicating if the polygon is irregular or not
            # irregular cases can be intersecting of line segments
            # or non neighbor nodes are closer than the loop space
        # construct the polygon based on these guesses
        for i in range(2, 2+dof):  # for the position of i-th node
            int_angle_t = int_guesses[i-2]  # interior angle of previous node
            ori_current = reset_radian(ori_current + (math.pi - int_angle_t))
            nodes[i][0] = nodes[i-1][0] + loop_space*math.cos(ori_current)
            nodes[i][1] = nodes[i-1][1] + loop_space*math.sin(ori_current)
            # check the distance of node i to all previous nodes
            # should not be closer than the loop space
            for j in range(i-1):  # exclude the previous neighbor
                vect_temp = [nodes[j][0]-nodes[i][0],
                             nodes[j][1]-nodes[i][1]]  # from i to j
                dist_temp = math.sqrt(vect_temp[0]*vect_temp[0]+
                                      vect_temp[1]*vect_temp[1])
                if dist_temp < loop_space:
                    no_irregular = False
                    print "node {} is too close to node {}".format(i, j)
                    break
            if not no_irregular:
                break
        if not no_irregular:
            continue  # continue the while loop, keep trying new polygon
    # stage 2: use convex triangle for the rest, and deciding if polygon is possible
    # solve the one last node
    vect_temp = [nodes[0][0]-nodes[poly_n-2][0],
                 nodes[0][1]-nodes[poly_n-2][1]]  # from n-2 to 0
    dist_temp = math.sqrt(vect_temp[0]*vect_temp[0]+
                          vect_temp[1]*vect_temp[1])
    # check distance between node n-2 and 0 to see if a convex triangle is possible
    # the situation that whether node n-2 and 0 are too close has been excluded
    if dist_temp > 2*loop_space:
        print("second last node is too far away from the starting node")
        continue
    else:
        # calculate the position of the last node
        midpoint = [(nodes[poly_n-2][0]+nodes[0][0])/2,
                    (nodes[poly_n-2][1]+nodes[0][1])/2]
        perp_dist = math.sqrt(loop_space*loop_space - dist_temp*dist_temp/4)
        perp_ori = math.atan2(vect_temp[1], vect_temp[0]) - math.pi/2
        nodes[poly_n-1][0] = midpoint[0] + perp_dist*math.cos(perp_ori)
        nodes[poly_n-1][1] = midpoint[1] + perp_dist*math.sin(perp_ori)
        # also check any irregularity for the last node
        no_irregular = True
        for i in range(1, poly_n-2):  # exclude starting node and second last node
            vect_temp = [nodes[i][0]-nodes[poly_n-1][0],
                         nodes[i][1]-nodes[poly_n-1][1]]  # from n-1 to i
            dist_temp = math.sqrt(vect_temp[0]*vect_temp[0]+
                                  vect_temp[1]*vect_temp[1])
            if dist_temp < loop_space:
                no_irregular = False
                print "last node is too close to node {}".format(i)
                break
        if no_irregular:
            poly_success = True  # reverse the flag
            print("successful!")

# calculate the geometry center of current polygon
geometry_center = [0, 0]
for i in range(poly_n):
    geometry_center[0] = geometry_center[0] + nodes[i][0]
    geometry_center[1] = geometry_center[1] + nodes[i][1]
geometry_center[0] = geometry_center[0]/poly_n
geometry_center[1] = geometry_center[1]/poly_n
# shift the polygon to the middle of the screen
for i in range(poly_n):
    nodes[i][0] = nodes[i][0] - geometry_center[0] + world_size[0]/2
    nodes[i][1] = nodes[i][1] - geometry_center[1] + world_size[1]/2

# draw the polygon
screen.fill(background_color)
# draw the nodes and line segments
disp_pos = [[0,0] for i in range(poly_n)]
# pink color for the first robot
disp_pos[0] = world_to_display(nodes[0], world_size, screen_size)
pygame.draw.circle(screen, robot_color_s, disp_pos[0], robot_size, 0)
# red color for the rest robots and line segments
for i in range(1, poly_n):
    disp_pos[i] = world_to_display(nodes[i], world_size, screen_size)
    pygame.draw.circle(screen, robot_color, disp_pos[i], robot_size, 0)
for i in range(poly_n-1):
    pygame.draw.line(screen, robot_color, disp_pos[i], disp_pos[i+1])
pygame.draw.line(screen, robot_color, disp_pos[poly_n-1], disp_pos[0])
pygame.display.update()

sim_exit = False  # simulation exit flag
while not sim_exit:
    # exit the program by close window button, or Esc or Q on keyboard
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sim_exit = True  # exit with the close window button
        if event.type == pygame.KEYUP:
            if (event.key == pygame.K_ESCAPE) or (event.key == pygame.K_q):
                sim_exit = True  # exit with ESC key or Q key

