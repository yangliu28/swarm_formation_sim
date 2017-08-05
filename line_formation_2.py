# second line formation simulation, using merging method instead of climbing

# merging method analysis(compared to climbing):
# The line may not be as straight as in first simulation, and it would take time to stretch
# the line so that robots are evenly spaced. But merging method has the flexibility to
# adjust the formation to walls and obstacles, so there is no need to use double walls to
# make sure the initial line segments start in the middle to avoid the line hit the wall.

# comments for research purpose:
# Same competing mechanism is used to ensure only one line is being formed.

# comments for programming purpose:
# Similar four statuses have been designed to serve different stages of the robot:
    # '0': free, wondering around, not in group, available to form a group or join one
    # '1': forming an initial line segment, or found a line and trying to merge into it
        # '1_0': initial an line segment, '0' found another '0' and formed a group
        # '1_1': join a line, merging into
    # '2': already in a line, dynamically adjust space and straightness
    # '-1': free, wondering around, no interaction with nearby robots
# Only allowed status transitions are:
    # forward: '-1'->'0', '0'->'1', '1'->'2'
    # backward: '1'->'-1', '2'->'-1'


import pygame
import math, random


pygame.init()

# for display, window origin is at left up corner
screen_size = (1200, 1000)  # width and height
background_color = (0, 0, 0)  # black background
robot_0_color = (0, 255, 0)  # for robot status '0', green
robot_1_color = (255, 153, 153)  # for robot status '1', pink
robot_2_color = (255, 51, 0)  # for robot status '2', red
robot_n1_color = (0, 51, 204)  # for robot status '-1', blue
robot_size = 5  # robot modeled as dot, number of pixels in radius

# set up the simulation window and surface object
icon = pygame.image.load('icon_geometry_art.jpg')
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption('Line Formation 1 Simulation')



