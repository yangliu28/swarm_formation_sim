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


# explain status transition of '1'
# explain the indexes in the loop



import pygame
import math, random
from loop_formation_robot import LFRobot
from formation_functions import *

pygame.init()


