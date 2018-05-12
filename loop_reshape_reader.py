# a program dedicated to read stored loop formation data file, visualize it in pygame

# pass the filename of the loop formation file, it should be placed under 'loop-data'
# ex: "python loop_reshape_reader.py loop-filename"

import pygame
import sys, os
from formation_functions import *

# initialize the pygame
pygame.init()

# name of the folder that stores loop formation files
loop_folder = 'loop-data'

# parameters for display, window origin is at left up corner
screen_size = (600, 400)  # width and height in pixels
color_black = (0,0,0)
color_white = (255,255,255)
robot_size = 6  # robot modeled as dot, number of pixels for radius

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

f = open(filepath, 'r')  # read mode
poly_n = int(f.readline())  # get number of sides of the polygon
int_angles = []
newline = f.readline()
while len(newline) != 0:
    int_angles.append(float(newline))  # add new interior angle
    newline = f.readline()
# check if this file has appropriate number of interior angles as it indicated
if len(int_angles) != poly_n-3:
    print 'file "{}" has incorrect number of interior angles'.format(filename)
    sys.exit()
# construct the polygon with these interior angles
for i in range(2, poly_n):
    nodes.append([0, 0])  # filled with [0, 0]
ori_current = 0  # orientation of current line segment
for i in range(2, poly_n-1):
    int_angle_t = int_angles[i-2]  # interior angle of node i-1
    ori_current = reset_radian(ori_current + (math.pi - int_angle_t))
    nodes[i][0] = nodes[i-1][0] + loop_space*math.cos(ori_current)
    nodes[i][1] = nodes[i-1][1] + loop_space*math.sin(ori_current)
vect_temp = [nodes[0][0]-nodes[poly_n-2][0],
             nodes[0][1]-nodes[poly_n-2][1]]  # from node n-2 to 0
dist_temp = math.sqrt(vect_temp[0]*vect_temp[0]+
                      vect_temp[1]*vect_temp[1])
midpoint = [(nodes[poly_n-2][0]+nodes[0][0])/2,
            (nodes[poly_n-2][1]+nodes[0][1])/2]
perp_dist = math.sqrt(loop_space*loop_space - dist_temp*dist_temp/4)
perp_ori = math.atan2(vect_temp[1], vect_temp[0]) - math.pi/2
nodes[poly_n-1][0] = midpoint[0] + perp_dist*math.cos(perp_ori)
nodes[poly_n-1][1] = midpoint[1] + perp_dist*math.sin(perp_ori)

# shift the polygon to the middle
# calculate the geometric center
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
screen.fill(color_white)
disp_pos = [[0,0] for i in range(poly_n)]
for i in range(poly_n):
    disp_pos[i] = world_to_display(nodes[i], world_size, screen_size)
    pygame.draw.circle(screen, color_black, disp_pos[i], robot_size, 0)
for i in range(poly_n-1):
    pygame.draw.line(screen, color_black, disp_pos[i], disp_pos[i+1], 2)
pygame.draw.line(screen, color_black, disp_pos[poly_n-1], disp_pos[0], 2)
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


