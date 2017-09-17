# program for testing only the physical motion control algorithm of the loop reshape process
    # a new SMA algorithm inspired by the behavior of shape memory alloy
# the starting and ending formations will be read from file
# the goal is to reshape the initial loop to the shape of target loop

# Two arguments specifying the loop formation files needs to be passed, first one is for
# initial loop formation, second for target formation. The loop formation files should be
# located under 'loop-data' folder. A third argument is optional, specifying the shift of
# which node it prefers in the target formation, 0 is default if not specified.

import pygame
from formation_functions import *
import numpy as np
import sys, os
import math

if len(sys.argv) < 3:
    # three arguments at least, first one is this program's filename
    # the other two are for the loop formation files
    print("incorrect number of input arguments")
    sys.exit()
form_files = []
form_files.append(sys.argv[1])
form_files.append(sys.argv[2])
# make sure the files exist
loop_folder = "loop-data"
for i in range(2):
    new_filepath = os.path.join(os.getcwd(), loop_folder, form_files[0])
    if not os.path.isfile(new_filepath):
        if i == 0: print("incorrect filename for initial formation")
        else: print("incorrect filename for target formation")
        sys.exit()
# try to get another argument for shift in desired node
targ_shift = 0  # default
try:
    targ_shift = sys.argv[3]
except: pass

pygame.init()  # initialize the pygame
# parameters for display, window origin is at left up corner
screen_size = (600, 800)  # width and height in pixels
    # top half for initial formation, bottom half for target formation
background_color = (0,0,0)  # black background
robot_color = (255,0,0)  # red for robot and the line segments
robot_size = 5  # robot modeled as dot, number of pixels for radius
world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])

# set up the simulation window and surface object
icon = pygame.image.load("icon_geometry_art.jpg")
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Loop Reshape Motion Test")

# variables to configure the simulation
poly_n = 0  # will be decided when reading formation files
loop_space = 4.0  # side length of the equilateral polygon
# linear spring constant modeling the pulling and pushing effect of neighbor nodes
linear_const = 1.0
# angular spring constant modeling the bending effect of neighbor nodes
angular_const = 0.2
disp_coef = 0.5  # coefficient from feedback vector to displacement

# construct the polygons from formation data from file
nodes = [[], []]  # node positions for the two formation, index is the robot's ID
nodes[0].append([0, 0])  # first node starts at origin
nodes[0].append([loop_space, 0])  # second node is loop space away on the right
nodes[1].append([0, 0])
nodes[1].append([loop_space, 0])
for i in range(2):
    new_filepath = os.path.join(os.getcwd(), loop_folder, form_files[i])
    f = open(new_filepath, 'r')  # read only
    # check consistence of polygon size
    if i == 0:  # this is the first polygon
        poly_n = int(f.readline())  # initialize with first polygon size
    else:  # check consistence when reading second polygon
        if int(f.readline()) != poly_n:
            print("inconsistent polygon size from the formation files")
            sys.exit()
    # read all interior angles form the file
    inter_ang = []
    new_line = f.readline()
    while len(new_line) != 0:  # not the end of the file yet
        inter_ang.append(float(new_line))
        new_line = f.readline()
    # check if this file has the nubmer of interior angles as it promised
    if len(inter_ang) != poly_n-3:
        print 'file "{}" has incorrect number of interior angles'.format(form_files[i])
        sys.exit()
    # construct positions of all nodes from these interior angles
    ori_current = 0  # orientation of current line segment
    for j in range(2, poly_n-1):
        inter_ang_t = inter_ang[j-2]  # interior angle of previous node
        ori_current = reset_radian(ori_current + (math.pi - inter_ang_t))
        nodes[i].append([nodes[i][j-1][0] + loop_space*math.cos(ori_current),
                         nodes[i][j-1][1] + loop_space*math.sin(ori_current)])
    # calculate position of last node
    vect_temp = [nodes[i][poly_n-2][0] - nodes[i][0][0],
                 nodes[i][poly_n-2][1] - nodes[i][0][1]]  # from node 0 to n-2
    dist_temp = math.sqrt(vect_temp[0]*vect_temp[0]+
                          vect_temp[1]*vect_temp[1])
    midpoint = [(nodes[i][poly_n-2][0]+nodes[i][0][0])/2,
                (nodes[i][poly_n-2][1]+nodes[i][0][1])/2]
    perp_dist = math.sqrt(loop_space*loop_space - dist_temp*dist_temp/4)
    perp_ori = math.atan2(vect_temp[1], vect_temp[0]) + math.pi/2
    nodes[i].append([midpoint[0] + perp_dist*math.cos(perp_ori),
                     midpoint[1] + perp_dist*math.sin(perp_ori)])

# shift the two polygons to the top and bottom halves
nodes = np.array(nodes)  # convert to numpy array
for i in range(2):
    # calculate the geometry center of current polygon
    geometry_center = np.mean(nodes[i], axis=0)
    nodes[i,:,0] = nodes[i,:,0] - geometry_center[0] + world_size[0]/2
    if i == 0:  # initial formation shift to top half
        nodes[i,:,1] = nodes[i,:,1] - geometry_center[1] + 3*world_size[1]/4
    else:  # target formation shift to bottom half
        nodes[i,:,1] = nodes[i,:,1] - geometry_center[1] + world_size[1]/4

# calculate the interior angles of the target formation
inter_targ = [0 for i in range(poly_n)]
for i in range(poly_n):
    vect_l = nodes[1][(i-1)%poly_n] - nodes[1][i]  # from host to left
    vect_r = nodes[1][(i+1)%poly_n] - nodes[1][i]  # from host to right
    # get the small angle between vect_l and vect_r
    inter_targ[i] = math.acos((vect_l[0]*vect_r[0] + vect_l[1]*vect_r[1])/
                              (loop_space*loop_space))
    if (vect_r[0]*vect_l[1] - vect_r[1]*vect_l[0]) < 0:
        # cross product of vect_r to vect_l is smaller than 0
        # the resulting interior angle should be in range of [0, 2*pi)
        inter_targ[i] = 2*math.pi - inter_targ[i]

# loop for the physical motion update
sim_exit = False
sim_pause = False
frame_period = 100  # updating period of the simulation and graphics, in millisecond
timer_last = pygame.time.get_ticks()
timer_now = timer_last
# the interior angle of initial formation will be updated dynamically
inter_curr = [0 for i in range(poly_n)]
# the variable for the neighboring robots on loop
dist_neigh = [0 for i in range(poly_n)]
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

    timer_now = pygame.time.get_ticks()
    if (timer_now - timer_last) > frame_period:
        timer_last = timer_now  # reset timer

        # update the dynamic neighbor distances of the loop
        for i in range(poly_n):
            vect_r = nodes[0][(i+1)%poly_n] - nodes[0][i]  # from host to right
            dist_neigh[i] = math.sqrt(vect_r[0]*vect_r[0] + vect_r[1]*vect_r[1])

        # update the dynamic interior angles of the loop
        for i in range(poly_n):
            i_l = (i-1)%poly_n  # index of left neighbor
            i_r = (i+1)%poly_n  # index of right neighbor
            vect_l = nodes[0][i_l] - nodes[0][i]  # from host to left
            vect_r = nodes[0][i_r] - nodes[0][i]  # from host to right
            # get the small angle between vect_l and vect_r
            inter_curr[i] = math.acos((vect_l[0]*vect_r[0] + vect_l[1]*vect_r[1])/
                                      (dist_neigh[i_l] * dist_neigh[i]))
            if (vect_r[0]*vect_l[1] - vect_r[1]*vect_l[0]) < 0:
                # cross product of vect_r to vect_l is smaller than 0
                # the resulting interior angle should be in range of [0, 2*pi)
                inter_curr[i] = 2*math.pi - inter_curr[i]

        # variable for feedback from all spring effects
        fb_vect = [np.zeros([1,2]) for i in range(poly_n)]
        for i in range(poly_n):
            # get feedback from pulling and pushing of the linear spring
            i_l = (i-1)%poly_n  # index of left neighbor
            i_r = (i+1)%poly_n  # index of right neighbor
            # unit vector from host to left
            vect_l = (nodes[0][i_l]-nodes[0][i])/dist_neigh[i_l]
            # unit vector bending host node toward inside the polygon from left neighbor
            vect_lb = np.array([vect_l[1], -vect_l[0]])  # rotate vect_l cw for pi/2
            # unit vector from host to right
            vect_r = (nodes[0][i_r]-nodes[0][i])/dist_neigh[i]
            # unit vector bending host node toward inside the polygon from right neighbor
            vect_rb = np.array([-vect_r[1], vect_r[0]])  # rotate vect_r ccw for pi/2
            
            # add the pulling or pushing effect from left neighbor
            fb_vect[i] = fb_vect[i] + (dist_neigh[i_l]-loop_space) * linear_const * vect_l
            # add the pulling or pushing effect from right neighbor
            fb_vect[i] = fb_vect[i] + (dist_neigh[i]-loop_space) * linear_const * vect_r
            # add the bending effect from left neighbor
            fb_vect[i] = fb_vect[i] + ((inter_curr[i_l] - inter_targ[(i_l+targ_shift)%poly_n])*
                                       angular_const * vect_lb)
            # add the bending effect from right neighbor
            fb_vect[i] = fb_vect[i] + ((inter_curr[i_r] - inter_targ[(i_r+targ_shift)%poly_n])*
                                       angular_const * vect_rb)

            # update one step of position
            nodes[0][i] = nodes[0][i] + disp_coef * fb_vect[i]

        # graphics update
        screen.fill(background_color)
        for i in range(2):
            # draw the nodes and line segments
            disp_pos = [[0,0] for j in range(poly_n)]
            # calculate the display pos for all nodes, as red dots
            for j in range(0, poly_n):
                disp_pos[j] = world_to_display(nodes[i][j], world_size, screen_size)
                pygame.draw.circle(screen, robot_color, disp_pos[j], robot_size, 0)
            # draw an outer circle to mark the starting node
            pygame.draw.circle(screen, robot_color, disp_pos[0], int(robot_size*1.5), 1)
            for j in range(poly_n-1):
                pygame.draw.line(screen, robot_color, disp_pos[j], disp_pos[j+1])
            pygame.draw.line(screen, robot_color, disp_pos[poly_n-1], disp_pos[0])
        pygame.display.update()




