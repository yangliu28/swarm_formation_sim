# loop formation simulation of swarm robots, form a triangle, and build loop by merging

# comments for programming purpose:
# Similar status design with the second line formation:
    # '0': free, wondering around, available to form a pair, a triangle or joint a group
    # '1': transit status between '0' and '2', robot in a group, but not a loop yet
        # '1_0': forming status, either in a pair, or in a triangle formation
            # '1_0_0': forming status, in a pair
            # '1_0_1': forming status, in a triangle
        # '1_1': merging status, joining a robot '2' group
    # '2': formal members of a loop formation group
    # '-1': free, wondering around, ignoring all interactions
# Only allowed status transitions are:
    # forward: '-1'->'0', '0'->'1_0_0', '0'->'1_0_1', '0'->'1_1', '1_0_0'->'1_0_1'
    #          '1_0_1'->'2', '1_1'->2
    # backward: '1'->'-1', '2'->'-1'
# Status transition regarding robot '1':
    # When 2 '0' meets, they both become '1_0_0', and start adjust distance to the loop
    # space. When distance is good, they don't perform any status transition yet. Only
    # when a new '0' comes over, it will trigger all three robots into '1_0_1' status.
    # And during the entire time of '1_0_0', the two robots are always welcoming new '0',
    # even their distance is not good yet. When in '1_0_1', after the three robots form
    # the perfect triangle and good loop space, they will trigger a status transition
    # of becoming '2'.

import pygame
import math, random
from loop_formation_robot import LFRobot
from formation_functions import *

pygame.init()

# for display, window origin is at left up corner
screen_size = (1200, 1000)  # width and height
background_color = (0, 0, 0)  # black background
robot_0_color = (0, 255, 0)  # for robot status '0', green
robot_1_0_0_color = (255, 153, 153)  # for robot status '1_0_0', light pink
robot_1_0_1_color = (255, 153, 153)  # for robot status '1_0_1', dark pink
robot_1_1_color = (255, 153, 153)  # for robot status '1_1', dark pink
robot_2_color = (255, 51, 0)  # for robot status '2', red
robot_n1_color = (0, 51, 204)  # for robot status '-1', blue
robot_size = 5  # robot modeled as dot, number of pixels in radius

# set up the simulation window and surface object
icon = pygame.image.load('icon_geometry_art.jpg')
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption('Loop Formation Simulation')

# for physics, continuous world, origin is at left bottom corner, starting (0, 0),
# with x axis pointing right, y axis pointing up.
# It's more natural to compute the physics in right hand coordiante system.
world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])

# variables to configure the simulation
robot_quantity = 30
distrib_coef = 0.5  # coefficient to resize the initial robot distribution
const_vel = 3.0  # all robots except those in adjusting phase are moving at this speed
frame_period = 100  # updating period of the simulation and graphics, in millisecond
comm_range = 5.0  # sensing and communication share this same range, in radius
loop_space = comm_range * 0.7  # desired space of robots on the lop
space_error = loop_space * 0.1  # error to determine if a status transition is needed
life_incre = 8  # number of seconds a new member adds to a group
group_id_upper_limit = 1000  # upper limit of random integer for group id
n1_life_lower = 3  # lower limit of life time for status '-1'
n1_life_upper = 8  # upper limit of life time for status '-1'
# coefficient for calculating velocity of robot '2' on the loop for adjusting
# as the distance error decreases, the loop adjusting velocity also decreases
adjust_vel_coef = consst_vel/loop_space * 2

# instantiate the robot swarm as list
robots = []  # container for all robots, index is its identification
for i in range(robot_quantity):
    # random position, away from the window's edges
    pos_temp = (((random.random()-0.5)*distrib_coef+0.5) * world_size[0],
                ((random.random()-0.5)*distrib_coef+0.5) * world_size[1])
    vel_temp = const_vel
    ori_temp = random.random() * 2*math.pi - math.pi  # random in (-pi, pi)
    object_temp = LFRobot(pos_temp, vel_temp, ori_temp)
    robots.append(object_temp)
# instantiate the group variable as dictionary
groups = {}
    # key is the group id
    # value is a list
        # 0.first element: the group size, member includes both '2' and '1'
        # 1.second element: remaining life time
        # 2.third element: a list of robots on the loop in adjacent order, status '2'
        # 3.fourth element: a list of robots off the loop, not ordered, status '1'
        # 4.fifth element: true or false, being the dominant group
# instantiate a distance table for every pair of robots
# make sure all data in table is being written when updating
dist_table = [[0 for j in range(robot_quantity)] for i in range(robot_quantity)]




