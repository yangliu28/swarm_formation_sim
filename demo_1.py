# This first demo shows how a robot swarm can autonomously choose a loop shape and form the
# shape in a distributed manner, without central control. Two consensus processes, decision
# making and role assignment, are performed consecutively with a fixed but arbitrary network.

# Description:
# Starting dispersed in random positions, the swarm aggregates together arbitrarily to form
# a connected network. Consensus decision making is performed with this network fixed, making
# a collective decision of which loop the swarm should form. Then with the same network, role
# assignment is performed, assigning the target positions to each robot. After these two
# consensus processes are done, the swarm disperses and aggregates again, this time aiming to
# form a loop with robots at their designated positions. The climbing method is used to permutate
# the robots on the loop. When the robots get their target positions, they dyanmically adjust
# the local shape so the loop deforms to the target one. The above steps will be ran repeatedly.

# Note that message transmission is simulated only in the role assignment, because communication
# is specialy depended on and message convey is used as well. While in consensus decision making
# and shape formation, the delay caused by communication are skipped.



# add interaction of mouse to show further interaction

# use text update in the terminal to show extra information, iterations

# when to terminate a process and continue next one
# consensus decision making -> subgroup size reaches to total number
# role assignment -> when there is no conflict
# loop formation -> when local shape accuracy is within a threshold

# the use of colors
# consensus decision making: same color for same chosen decision, of collective shape
    # guarantee all robots agreed on one choice
# role assignment: same color for same chosen decision, of target position
    # guarantee all robots agreed on all different choices
# loop formation: no colors, empty circle for dormant, filled circle for active


import pygame
from __future__ import print_function
import sys, getopt
import numpy as np

swarm_size = 30  # default number of robots in the swarm

# read command line options
try:
    opts, args = getopt.getopt(sys.argv[1:], 'n:')
except getopt.GetoptError as err:
    print str(err)
    sys.exit()
for opt,arg in opts:
    if opt = '-n':
        swarm_size = arg

# conversion between physical and display world sizes
# To best display any robot swarm in its appropriate window size, and have enough physical
# space for the robots to move around, it has been made that the ratio from unit world size
# to unit display size is fixed. The desired physical space between robots when they do shape
# formation is also fixed. So a small swarm will have small physical world, and a linearly
# small display window; vice versa for a large swarm.
# If the size of the swarm is proportional to the side length of the world, the area of the
# world will grow too fast. If the swarm size is proportional to the area of the world, when
# the size of the swarm grow large, it won't be able to be fitted in if performing a line or
# circle formation. A compromise is to make swarm size proportional to the side length to the
# power exponent between 1 and 2.

# automatically calculate world_side_coef
# from a good pair of swarm_size and its world_side_length

power_exponent = 1.5
world_side_coef = 0.5
world_side_length = world_side_coef * pow(swarm_size, 1/power_exponent)
world_size = (world_side_length)
pixels_per_length = 50  # for converting from physical world to display world

robot_poses = []


