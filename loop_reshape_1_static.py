# static version of the probabilistic approach for the loop reshape formation



# need two polygons, one for initial setup formation, one for target formation
# have the algorithm for this first, no matter in which graphic window

# the basic requirements:
# equal space polygon, concave is allowed
# no collapse inside, any pair of non-neighbor robots should not be closer than loop space


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
robot_quantity = 30  # the number of nodes for the polygon
loop_space = 4.0  # side length of the equilateral polygon
int_angle_reg = math.pi - 2*math.pi/robot_quantity  # interior angle of regular polygon
int_angle_range = 2*(math.pi - reg_int_angle)  # half range of the interior angle
# lower limit of the interior angle
int_angle_lower = max(math.pi/3, int_angle_reg - int_angle_range)
# upper limit of the interior angle
int_angle_upper = int_angle_reg + 

# instantiate the robots variable for the positions
robots = []  # for all robots, index is the robot's identification
for i in range(robot_quantity):
    robots.append([0,0])  # append a 2-D position



