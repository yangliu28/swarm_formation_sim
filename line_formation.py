# line formation simulation

import pygame
import math, random
from line_formation_robot import LFRobot
from line_formation_functions import *

pygame.init()  # initialize pygame

# for display, window origin is at left up corner
screen_size = (1200, 900)  # width and height
background_color = (0, 0, 0)  # black background
robot_color = (255, 255, 255)  # white robot
robot_size = 5  # robot modeled as dot, number of pixels in radius

# set up the simulation window and surface object
icon = pygame.image.load('geometry_art.jpg')
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption('Line Formation Simulation')

# for physics, origin is at left bottom corner, starting (0, 0)
# it's more natural to do the calculation in right hand coordiante system
# the continuous, physical world for the robots
world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])

# variables to configure the simulation
robot_quantity = 30
# coefficient to resize the robot distribution, but always keep close to center
distrib_coef = 0.5
const_vel = 2  # all moving robots are moving at a constant speed
frame_period = 100  # updating period of the simulation and graphics, in ms
comm_range = 7  # communication range, the radius


# instantiate the robot swarm
robots = []  # container for all robots, index is also the IDs
for i in range(robot_quantity):
    # random position, away from the window edges
    pos_temp = (((random.random()-0.5)*distrib_coef+0.5) * world_size[0],
                ((random.random()-0.5)*distrib_coef+0.5) * world_size[1])
    vel_temp = const_vel
    ori_temp = random.random() * 2*math.pi  # random in (0, 2*pi)
    object_temp = LFRobot(pos_temp, vel_temp, ori_temp)
    robots.append(object_temp)

# instantiate a distance table for every pair of robots
# will calculate once for symmetric data
dist_table = [[0 for i in range(robot_quantity)] for j in range(robot_quantity)]

# the loop
sim_exit = False  # simulation exit flag
sim_pause = False  # simulation pause flag
timer_last = pygame.time.get_ticks()  # return number of milliseconds after pygame.init()
timer_now = timer_last  # initialize it with timer_last
while not sim_exit:
    # exit the program
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sim_exit = True  # exit with the close window button
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_SPACE:
                sim_pause = not sim_pause  # reverse the pause flag
            if (event.key == pygame.K_ESCAPE) or (event.key == pygame.K_q):
                sim_exit = True  # exit with ESC key or Q key

    # skip the rest of the loop if paused
    if sim_pause: continue

    # update the physics, control rules and graphics at the same time
    timer_now = pygame.time.get_ticks()
    if (timer_now - timer_last) > frame_period:
        # calculate the distance of every pair of robots
        for i in range(robot_quantity):
            for j in range(i+1, robot_quantity):
                # it only covers the upper triangle without the diagonal
                pos_temp = (robots[i].pos[0]-robots[j].pos[0],
                            robots[i].pos[1]-robots[j].pos[1])
                dist_temp = math.sqrt(pos_temp[0]*pos_temp[0] +
                                      pos_temp[1]*pos_temp[1])
                if dist_temp <= comm_range:
                    dist_table[i][j] = dist_temp
                    dist_table[j][i] = dist_temp
                else:  # ignore the neighbors too far away
                    dist_table[i][j] = 0
                    dist_table[j][i] = 0
        # sort the distance in another table, record the index
        
        # check the status change with control rules for the line formation
        for i in range(robot_quantity):


    pygame.display.update()


pygame.quit()


