# static version of the probabilistic approach for the loop reshape formation

# random equilateral polygon generating method:
# Given all the side length of a n-side polygon, it can still varies in shape. The number of
# degree of freedom is (n-3). Equilateral polygon also has fixed side length, the way to
# generate such random polygon is to treat first (n-3) number of interior angles as DOFs.
# The rest of the polygon can be determined uniquely in either a convex or concave triangle.
# To make sure the polygon can be formed, the guesses for interior angles can not be too
# wild. Here a normal distribution is used to constrain the guesses within an appropriate
# range of the interior angle of corresponding regular polygon.

# need two polygons, one for initial setup formation, one for target formation
# have the algorithm for this first, no matter in which graphic window

# the basic requirements:
# equal space polygon, concave is allowed
# no collapse inside, any pair of non-neighbor nodes should not be closer than loop space


# mouse click to select? or just show all of them (preferred)

# plot histogram with matplotlib


import pygame
import math, random, numpy
from formation_functions import *

pygame.init()

# parameters for display, window origin is at left up corner
screen_size = (1200, 1000)  # width and height
background_color = (0,0,0)  # black background
robot_color = (255,0,0)  # red for robot as well as link
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
int_angle_dev = (int_angle_reg - math.pi/3)/2

# instantiate the robots variable for the positions
nodes = []  # position of all nodes, index is the robot's identification
nodes.append([0, 0])  # first node start at origin
nodes.append([loop_space, 0])  # second node is loop_space distance away on the right
for i in range(2, poly_n):
    nodes.append([0, 0])  # fill the rest nodes with [0,0]

poly_success = False  # flag for succeed in generating the polygon
while not poly_success:
    # continue trying until all the guesses can forming the polygon
    # stage 1: guessing all the free interior angles
    dof = poly_n-3  # number of free interior angles to be randomly generated
    if dof > 0:  # only continue guessing if at least one free interior angle
        # generate all the guesses from a normal distribution
        int_guesses = numpy.random.normal(int_angle_reg, int_angle_dev, dof)
        ori_current = 0  # orientation of the line segment
        # construct the polygon based on these guesses
        for i in range(2, 2+dof):  # for the position of i-th node
            int_angle_t = int_guesses[i-2]  # interior angle of previous node
            ori_current = reset_radian(ori_current + (math.pi - int_angle_t))
            nodes[i][0] = nodes[i-1][0] + loop_space*math.cos(ori_current)
            nodes[i][1] = nodes[i-1][1] + loop_space*math.sin(ori_current)
    # stage 2: use convex triangle for the rest, and deciding if polygon is possible
    # solve the one last node
    vect_temp = [nodes[0][0]-nodes[poly_n-2][0],
                 nodes[0][1]-nodes[poly_n-2][1]]  # from n-2 to 0
    dist_temp = math.sqrt(vect_temp[0]*vect_temp[0]+
                          vect_temp[1]*vect_temp[1])
    # check distance between node n-2 and 0 to see if a convex triangle is possible
    if dist_temp > 2*loop_space:
        print("second last node is too far away from the first node")
        continue
    elif dist_temp < loop_space:
        print("second last node is too close to the first node")
        continue
    else:
        poly_success = True  # reverse the flag
        # calculate the position of the last node
        midpoint = [(nodes[poly_n-2][0]+nodes[0][0])/2,
                    (nodes[poly_n-2][1]+nodes[0][1])/2]
        perp_dist = math.sqrt(loop_space*loop_space - dist_temp*dist_temp/4)
        perp_ori = math.atan2(vect_temp[1], vect_temp[0]) - math.pi/2
        nodes[poly_n-1][0] = nodes[poly_n-2][0] + loop_space*math.cos(perp_ori)
        nodes[poly_n-1][1] = nodes[poly_n-2][1] + loop_space*math.sin(perp_ori)

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
for i in range(poly_n):
    disp_pos[i] = world_to_display(disp_pos[i], world_size, screen_size)
    pygame.draw.circle(screen, robot_color, disp_pos[i], robot_size, 0)
for i in range(poly_n-1):
    pygame.draw.line(screen, robot_color, nodes[i], nodes[i+1])
pygame.draw.line(screen, robot_color, nodes[poly_n-1], nodes[0])
pygame.display.update()


while True:
    pass

