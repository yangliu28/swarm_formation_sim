# This simulation tests the probabilistic convergence algorithm (a collective decision
# making algorithm) on randomly generated 2D honeycomb network. The algorithm is extracted
# from previous loop reshape simulations, for further testing if it can be generally used
# on 2D random network topologies.

import math, sys, os, getopt
import matplotlib.pyplot as plt
from network_generator_2D_swarm import *

net_size = 0  # size of the honeycomb network
net_filename = ''  # filename of the network to be read
net_filepath = ''
net_folder = 'honeycomb-networks'  # folder for all network files

# read command line options
try:
    opts, args = getopt.getopt(sys.argv[1:], 'f:')
except getopt.GetoptError as err:
    print str(err)
    sys.exit()
for opt,arg in opts:
    if opt == '-f':
        net_filename = arg
        # check if this file exists
        net_filepath = os.path.join(os.getcwd(), net_folder, net_filename)
        if not os.path.isfile(net_filepath):
            print "{} does not exist".format(net_filename)
            sys.exit()
        # parse the network size
        net_size = int(net_filename.split('-')[0])  # the number before first dash

# read the network from file
nodes = []



