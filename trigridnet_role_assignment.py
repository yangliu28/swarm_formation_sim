# This simulation tests a distributed role assignment algorithm, for robot swarm starting
# at arbitrary triangle grid network, to perform one-to-one role assignment. The assigned
# roles are abstract and represented as index numbers. The primary goal of this role
# assignment algorithm is to have all robots agree on the one-to-one assignment scheme,
# instead of coming up with the best assignment scheme.

# Inter-robot communication is used to let one robot know the status of another robot that
# is not directly connected. Enabling message convey is what I consider the most convenient
# way to resolve conflict when two robots decide on the same assigned role. Although in this
# way, it is possible for each robot to have the information of all the other robots, and
# thus it could become a distributed master control algorithm, we still consider the proposed
# algorithm a distributed one. Because we limit the information each robot will store, and
# the local computation performed, so the algorithm is still robust.

# The communication delay is also reflected in the simulation, one step in simulation will
# update one step of message transmission. Color is used to highlight the conflicts in the
# role assignment, instead of showing convergence of decisions.


import pygame
import matplotlib.pyplot as plt
from trigridnet_generator import *
import numpy as np
import os, getopt, sys, time

net_folder = 'trigrid-networks'
net_filename = '30-1'  # default network
net_size = 30  # default network size
net_filepath = os.path.join(os.getcwd(), net_folder, net_filename)

nobargraph = False

# read command line options
try:
    opts, args = getopt.getopt(sys.argv[1:], 'f:', ['nobargraph'])
except getopt.GetoptError as err:
    print str(err)
    sys.exit()
for opt,arg in opts:
    if opt = '-f':
        net_filename = arg
        net_filepath = os.path.join(os.getcwd(), net_folder, net_filename)
        # check if this file exists
        if not os.path.isfile(net_filepath):
            print "{} does not exist".format(net_filename)
            sys.exit()
        # parse the network size
        net_size = int(net_filename.split('-')[0])
    elif opt == '--nobargraph':
        nobargraph = True

# read the network from file
nodes = []  # the node positions in the form of triangle grid network


