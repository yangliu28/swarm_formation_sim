# script to generate files for the curve shapes
# each block of code generates one curve shape

# neon typeface reference: (Thanks, Emil!)
# http://fenotype.1001fonts.com/neon-fonts.html

import pickle
import math
import pygame
import numpy as np

# general function to reset radian angle to [-pi, pi)
def reset_radian(radian):
    while radian >= math.pi:
        radian = radian - 2*math.pi
    while radian < -math.pi:
        radian = radian + 2*math.pi
    return radian

# general function to calculate next position node along a heading direction
def cal_next_node(node_poses, index_curr, heading_angle, rep_times):
    for _ in range(rep_times):
        index_next = index_curr + 1
        x = node_poses[index_curr][0] + 1.0*math.cos(heading_angle)
        y = node_poses[index_curr][1] + 1.0*math.sin(heading_angle)
        node_poses[index_next] = np.array([x,y])
        index_curr = index_next
    return index_next

# ##### script to generate 30-squarehelix #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# print(node_index)
# print(node_poses)
# with open('30-squarehelix', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-squarehelix #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 9)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 9)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 9)
# print(node_index)
# print(node_poses)
# with open('100-squarehelix', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-ARM #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# heading_angle = (65*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = (-60*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = ((10-180)*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (-10*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (-60*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (-45*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -math.pi/2.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (-40*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = (-50*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = (50*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# print(node_index)
# print(node_poses)
# with open('30-ARM', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-ARM #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# arc_angle = math.pi/7.0/2.0
# heading_angle = (65*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 11)
# heading_angle = (-65*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = ((65-180)*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = (-65*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 10)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = arc_angle
# for _ in range(7):
#     heading_angle = reset_radian(heading_angle - 2*arc_angle)
#     node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = (-43*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 10)
# heading_angle = (-50*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = (50*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 10)
# print(node_index)
# print(node_poses)
# with open('100-ARM', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-KID #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# arc_angle = math.pi/4.0/2.0*0.9
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = 30*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = reset_radian(heading_angle - math.pi/2)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle - math.pi/2)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -30*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = arc_angle*0.5
# for _ in range(4):
#     heading_angle = reset_radian(heading_angle - 2*arc_angle)
#     node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# print(node_index)
# print(node_poses)
# with open('30-KID', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-KID #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# arc_segments = 17
# arc_angle = math.pi/float(arc_segments)/2.0
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 11)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = 40*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = reset_radian(heading_angle - math.pi/2)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle - math.pi/2)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = -40*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 9)
# heading_angle = (90-30)*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (90+30)*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = 15*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -15*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (-90-30)*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (-90+30)*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 9)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 11)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = arc_angle
# for _ in range(arc_segments):
#     heading_angle = reset_radian(heading_angle - 2*arc_angle)
#     node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# print(node_index)
# print(node_poses)
# with open('100-KID', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-MAD #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# arc_angle = math.pi/4.5/2.0
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = -50*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = 50*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (65*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = (-60*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = ((10-180)*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (-10*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (-60*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = arc_angle*1.4
# for _ in range(5):
#     heading_angle = reset_radian(heading_angle - 2*arc_angle)
#     node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# print(node_index)
# print(node_poses)
# with open('30-MAD', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-MAD #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# arc_segments = 17
# arc_angle = math.pi/float(arc_segments)/2.0
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 11)
# heading_angle = -45*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# heading_angle = 45*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 11)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = 63*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 12)
# heading_angle = -63*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = (63-180)*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = -63*math.pi/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 11)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = arc_angle
# for _ in range(arc_segments):
#     heading_angle = reset_radian(heading_angle - 2*arc_angle)
#     node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# print(node_index)
# print(node_poses)
# with open('100-MAD', 'w') as f:
#     pickle.dump(node_poses, f)








pygame.init()
# find the right world and screen sizes
x_max, y_max = np.max(node_poses, axis=0)
x_min, y_min = np.min(node_poses, axis=0)
pixel_per_length = 30
world_size = (x_max - x_min + 2.0, y_max - y_min + 2.0)
screen_size = (int(world_size[0])*pixel_per_length, int(world_size[1])*pixel_per_length)
# convert node poses in the world to disp poses on screen
def cal_disp_poses():
    poses_temp = np.zeros((swarm_size, 2))
    # shift the loop to the middle of the world
    middle = np.array([(x_max+x_min)/2.0, (y_max+y_min)/2.0])
    for i in range(swarm_size):
        poses_temp[i] = (node_poses[i] - middle +
            np.array([world_size[0]/2.0, world_size[1]/2.0]))
    # convert to display coordinates
    poses_temp[:,0] = poses_temp[:,0] / world_size[0]
    poses_temp[:,0] = poses_temp[:,0] * screen_size[0]
    poses_temp[:,1] = poses_temp[:,1] / world_size[1]
    poses_temp[:,1] = 1.0 - poses_temp[:,1]
    poses_temp[:,1] = poses_temp[:,1] * screen_size[1]
    return poses_temp.astype(int)
disp_poses = cal_disp_poses()
# draw the loop shape on pygame window
color_white = (255,255,255)
color_black = (0,0,0)
screen = pygame.display.set_mode(screen_size)
screen.fill(color_white)
for i in range(swarm_size):
    pygame.draw.circle(screen, color_black, disp_poses[i], 5, 0)
for i in range(swarm_size-1):
    pygame.draw.line(screen, color_black, disp_poses[i], disp_poses[i+1], 2)
pygame.display.update()
raw_input("<Press ENTER to exit>")


