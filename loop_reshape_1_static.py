# static version of the probabilistic approach for the loop reshape formation

# random equilateral polygon generating method:
# Given all the side length of a n-side polygon, it can still varies in shape. The number of
# degree of freedom is (n-3). Equilateral polygon also has fixed side length, the way to
# generate such random polygon is to treat first (n-3) number of interior angles as DOFs.
# The rest of the polygon can be determined uniquely in either a convex or concave triangle.
# To make sure the polygon can be formed, the guesses for interior angles can not be too
# wild. It will have upper and lower limitations for all guesses.

# need two polygons, one for initial setup formation, one for target formation
# have the algorithm for this first, no matter in which graphic window

# the basic requirements:
# equal space polygon, concave is allowed
# no collapse inside, any pair of non-neighbor nodes should not be closer than loop space


# mouse click to select? or just show all of them (preferred)

# plot histogram with matplotlib


import pygame
import math, random
from formation_functions import *

pygame.init()

# parameters for display, window origin is at left up corner
screen_size = (1200, 1000)  # width and height
background_color = (0,0,0)  # black background
robot_color = (255,0,0)  # red for robot as well as link

# set up the simulation window and surface object
icon = pygame.image.load("icon_geometry_art.jpg")
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Loop Reshape(static version)")

# for physics, continuous world, origin is at left bottom corner, starting (0, 0),
# with x axis pointing right, y axis pointing up.
# It's more natural to compute the physics in right hand coordiante system.
world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])

# variables to configure the simulation
poly_n = 30  # number of nodes/sides for the polygon, also the robot quantity, at least 3
loop_space = 4.0  # side length of the equilateral polygon
# the following are for the guessing of the free interior angles
int_angle_reg = math.pi - 2*math.pi/poly_n  # interior angle of regular polygon
int_angle_range = 2*(math.pi - reg_int_angle)  # half range of the interior angle
# lower limit of the interior angle
int_angle_lower = max(math.pi/3, int_angle_reg - int_angle_range)
# upper limit of the interior angle
int_angle_upper = int_angle_reg + int_angle_range

# instantiate the robots variable for the positions
nodes = []  # position of all nodes, index is the robot's identification
nodes.append([0, 0])  # first node start at origin
nodes.append([loop_space, 0])  # second node is loop_space distance away on the right
for i in range(2, poly_n):
    nodes.append([0, 0])  # fill the rest nodes with [0,0]

poly_succeed = False  # flag for succeed in generating the polygon
while not poly_succeed:
    # continue trying until all the guesses can forming the polygon
    # stage 1: guessing all the free interior angles
    dof = poly_n-3  # number of free interior angles to be randomly generated
    for i in range(2, 2+dof):  # will skip if dof is 0


    # stage 2: use convex triangle for the rest, deciding if polygon is possible


