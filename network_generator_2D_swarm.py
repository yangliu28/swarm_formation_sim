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
# The position of a node is similarly described as (x,y) in integers.

# No topology duplication check:
# It's possible that two generated networks having different grid layouts may share the same
# topology. It's preferable to combine these two layouts, because the result of the algorithm
# test is only related to network topology. But since telling whether one grid layout has the
# same topology with another is with too much effort, I just use the grid layout as test
# subject of topology. Besides, it's not that often that two layouts are of same topology
# when network size is very large.



import math, random, sys
import numpy.matlib
import matplotlib.pyplot as plt
import getopt

def main():
    size = 0  # network size from input

    # read command line options
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'n:')
    except getopt.GetoptError as err:
        print str(err)
        sys.exit()
    for o,a in opts:
        if o == '-n':
            size = int(a)

    # use list to store the node information
    nodes_t = []  # target nodes pool, for decided nodes in target network
    nodes_a = []  # available nodes pool, for nodes available to be added to network
    # place the first node at the origin for the network
    nodes_t.append((0,0))
    for pos in get_neighbors(nodes_t[0]):
        nodes_a.append(pos)  # append all six neighbors to available pool

    # loop for randomly placing new nodes from available pool to generate the network
    for i in range(size-1):  # first node is decided and excluded
        # randomly choose one from the available pool
        pos_new = random.choice(nodes_a)
        nodes_a.remove(pos_new)  # remove selected node from available pool
        nodes_t.append(pos_new)  # add new node to the target pool
        # check and update every neighbor of newly selected node
        for pos in get_neighbors(pos_new):
            if pos in nodes_t: continue
            if pos in nodes_a: continue
            # if none of the above, add to the available pool
            nodes_a.append(pos)

    # generate the connection variable, 0 for not connected, 1 for connected
    connections = [[0 for j in range(size)] for i in range(size)]  # populated with zeros
    for i in range(size):
        for j in range(i+1, size):
            # find if nodes[i] and nodes[j] are neighbors
            diff_x = nodes_t[i][0] - nodes_t[j][0]
            diff_y = nodes_t[i][1] - nodes_t[j][1]
            if abs(diff_x) + abs(diff_y) == 1 or diff_x * diff_y == -1:
                # either one of the axis difference is 1, or combination of 1 and -1
                connections[i][j] = 1
                connections[j][i] = 1

    # plot the network as dots and lines
    fig = plt.figure()
    fig.canvas.set_window_title('Randomly generated 2D swarm network')
    splt = fig.add_subplot(1,1,1)
    splt.axis('equal')  # equal scale for x and y axis
    splt.tick_params(axis='both',
                     which='both',
                     bottom='off',
                     top='off',
                     left='off',
                     right='off',
                     labelbottom='off',
                     labelleft='off')  # turn off ticks and labels
    # convert node positions from honeycomb grid to Cartesian coordinates, for plotting
    nodes_t_plt = [honeycomb_to_cartesian(pos) for pos in nodes_t]
    # draw the connections as lines
    for i in range(size):
        for j in range(i+1, size):
            if connections[i][j] == 1:
                splt.plot([nodes_t_plt[i][0], nodes_t_plt[j][0]],
                          [nodes_t_plt[i][1], nodes_t_plt[j][1]], '-k')
    # draw the nodes as dots, origin node as red, rest blue
    splt.plot(nodes_t_plt[0][0], nodes_t_plt[0][1], 'o',
              markersize=10, markerfacecolor='red')
    for i in range(1,size):
        splt.plot(nodes_t_plt[i][0], nodes_t_plt[i][1], 'o',
                  markersize=10, markerfacecolor='blue')
    fig.show()

    while True:
        pass

# relocate the network centroid to the middle of the graph


# return the positions of the six neighbors of the input node on honeycomb grid
# The first four neighbors are just like the situation in the Cartesian coordinates, the last
# two neighbors are the two on the diagonal line along the y=-x axis, because the honeycomb
# grid is like askewing the y axis toward x, allowing more space in second and fourth quadrants.
def get_neighbors(pos):
    x = pos[0]
    y = pos[1]
    return [(x+1, y), (x-1, y),
            (x, y+1), (x, y-1),
            (x+1, y-1), (x-1, y+1)]

# return Cartesian coordinates of honeycomb grid nodes for plotting
def honeycomb_to_cartesian(pos):
    x = pos[0]
    y = pos[1]
    # askewing y axis to the right for pi/6
    return [x+y*math.sin(math.pi/6), y*math.cos(math.pi/6)]

if __name__ == '__main__':
    main()


