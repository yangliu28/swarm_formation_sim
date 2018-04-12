# script to generate files for the loop shapes

import pickle
import math
import pygame
import numpy as np


##### script to generate 30-1-square #####
swarm_size = 30
node_poses = np.zeros((swarm_size, 2))
x = 0.0
y = 0.0
# bottom side line
node_poses[0] = np.array([0.0, 0.0])
for i in range(1,8):
    x = x + 1.0
    node_poses[i] = np.array([x,y])
# right line line
x = x + 1.0 / math.sqrt(2)
y = y + 1.0 / math.sqrt(2)
node_poses[8] = np.array([x,y])
for i in range(9,16):
    y = y + 1.0
    node_poses[i] = np.array([x,y])
# top side line
for i in range(16,23):
    x = x - 1.0
    node_poses[i] = np.array([x,y])
# left side line
x = x - 1.0 / math.sqrt(2)
y = y - 1.0 / math.sqrt(2)
node_poses[23] = np.array([x,y])
for i in range(24,30):
    y = y - 1.0
    node_poses[i] = np.array([x,y])
print("node_poses: {}".format(node_poses))
with open('30-1-square', 'w') as f:
    pickle.dump(node_poses, f)




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
pygame.draw.line(screen, color_black, disp_poses[0], disp_poses[swarm_size-1], 2)
pygame.display.update()
raw_input("<Press ENTER to exit>")


