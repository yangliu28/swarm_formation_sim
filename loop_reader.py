# a program dedicated to read stored loop formation data file, visualize it in pygame

# pass the filename of the loop formation file, it should be placed under 'loop-data'

import pygame
import sys, os
from formation_functions import *

# initialize the pygame
pygame.init()

# name of the folder that stores loop formation files
loop_folder = 'loop-data'

# parameters for display, window origin is at left up corner
screen_size = (600, 800)  # width and height in pixels
    # top half for initial formation, bottom half for target formation
background_color = (0,0,0)  # black background
robot_color = (255,0,0)  # red for robot and the line segments
robot_color_s = (255,153,153)  # pink for the start robot
robot_size = 5  # robot modeled as dot, number of pixels for radius

# set up the simulation window and surface object
icon = pygame.image.load("icon_geometry_art.jpg")
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Loop Reader")

world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])
loop_space = 4.0

# read passed filename
filename = sys.argv[1]
filepath = os.path.join(os.getcwd(), loop_folder, filename)
if not os.path.isfile(filepath):
    print 'file "{}" does not exist'.format(filename)
    sys.exit()

# instantiate node positions variable
nodes = []
nodes.append([0, 0])
nodes.append([loop_space, 0])




