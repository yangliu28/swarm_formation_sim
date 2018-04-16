# This second demo is a variation of first one, also shows how a robot swarm can autonomously
# choose a loop shape and form the shape. But the decision making and the role assignment are
# performed while the swarm is already in a loop shape. The role assignment method applies to
# loop network only, but has the advantage of using no message relay.

# Description:
# Starting dispersed in random positions, the swarm aggregates together to form a random loop.
# From here, the following steps will run repeatedly. The robots first make collective decision
# of which loop shape to form, then they perform role assignment to distribute target positions.
# At the same time of role assignment, the robots also adjust local shapes so the collective
# reshapes to the target loop shape.

# the simulations that run consecutively
# Simulation 1: aggregate together to form a random loop
    # Simulation 2: consensus decision making of target loop shape
    # Simulation 3: role assignment and loop reshape


from __future__ import print_function
import pygame
import sys, os, getopt, math
import numpy as numpy
import pickle

swarm_size = 30  # default swarm size

# read command line options
try:
    opts, args = getopt.getopt(sys.argv[1:], 'n:')
except getopt.GetoptError as err:
    print(str(err))
    sys.exit()
for opt,arg in opts:
    if opt == '-n':
        swarm_size = int(arg)





