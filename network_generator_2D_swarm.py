# random network generator for robot swarm on 2D honeycomb grid
# generated network should be one-connected, that is, no robot is isolated behind
# input: number of nodes for target network

# Constraint of honeycomb grid guaranteed the robots are uniformed spaced, and 6 connections
# at most for each robot. Robots will reside in the middle of each honeycomb grid cell, not
# the corner points. The lines representing connections are drew perpendicular to the edges
# of the hexagons. 

# Honeycomb grid coordinates:
# Position of a node on a 2D plane has two degrees of freedom. Following this rule, even three
# intersecting axes can be drew on the honeycomb grid, only two are chosen as the coordinate
# system. The two axes(randomly chosen) are angled at pi/3(or 2*pi/3), so it's like a warped 
# coordinate system. The nodes can be located by drawing parallel lines to the two axes. This
# system is very much like the Cartesian system, but warped and allowing more node connections.

# No topology duplication check:
# It's possible that two generated networks having different grid layouts may share the same
# topology. It's preferable to combine these two layouts, because the result of the algorithm
# test is only related to network topology. But since telling whether one grid layout has the
# same topology with another is with too much effort, I just use the grid layout as test
# subject of topology. Besides, it's not that often that two layouts are of same topology
# when network size is very large.


# relocate the network centroid to the middle of the graph

import math, random





