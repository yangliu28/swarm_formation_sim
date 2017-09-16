# program for testing only the physical motion control algorithm of the loop reshape process
    # a new SMA algorithm inspired by the behavior of shape memory alloy
# the starting and ending formations will be read from file

# two arguments specifying the loop formation files needs to be passed
# first one is for initial loop formation, second for target formation
# the loop formation files should be located under 'loop-data' folder


import pygame
from formation_functions import *
import numpy as np
import sys, os
import math

if len(sys.argv) != 3:
    # first argument is the program filename, the other two for loop formation files
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

# variables to configure the simulation
poly_n = 0  # will be decided when reading formation files
loop_space = 4.0  # side length of the equilateral polygon

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
    when len(new_line) != 0:  # not the end of the file yet
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

# calculate the interior angles of the two formations
inter_ang = [[9 for j in range(poly_n)] for i in range(2)]
for i in range(2):
    for j in range(poly_n):
        node_h = nodes[i][j]  # host node
        node_l = nodes[i][(j-1)%poly_n]  # node on the left
        node_r = nodes[i][(j+1)%poly_n]  # node on the right
        vect_l = [node_l[0]-node_h[0], node_l[1]-node_h[1]]  # from host to left
        vect_r = [node_r[0]-node_h[0], node_r[1]-node_h[1]]  # from host to right
        # get the angle rotating from vect_r to vect_l
        inter_ang[i][j] = math.



