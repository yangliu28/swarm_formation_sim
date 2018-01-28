# this is an algorithm revision besed on 'loop_reshape_1_static.py'
# First section is copied verbatim, algorithm tests are carried on in second section for the
# preferability distribution evolution.

# arguments format:
# '-i': filename of the initial loop formation; default is generate the initial formation
# '-t': filename of the target loop formation; default is generate the target formation
# '--gensave': option to all or none of the generated formations; default is discard
# '--nobargraph': option to skip the bar graph visualization; default is with bargraph

# Revised algorithm in the second section:
# New algorithm combines weighted averaging, linear multiplier and power function methods.
# Since equal weighted averaging method can guarantee convergence(although unipolarity of
# resulting distribution can be very poor), a revised weighted averaging method was
# implemented here to ensure as much convergence as possible. Each node is given a new
# piece of information, that is how many nodes in the adjacent block agree on which target
# node they should be. (This adjacent block is referred as a subgroup.) This information
# will help to resolve any conflict on the boundary of two subgroups. And ideally, the
# subgroup with higher number of nodes should win. With this conflict resolving mechanism,
# the linear multiplier method is used to improve the unipolarity of the distributions. This
# method has similar effect of previous power function method with bigger than 1 exponent,
# but in a slower and constant growing rate. The linear multiplier is only used when two
# neighbors converges with the host on the formation. How much the unipolarity should grow
# depends on the largest difference of distributions when using averaging method. The larger
# the difference, the less the unipolarity should grow. A power function method with exponent
# smaller than 1 is used to slow down the increasing rate of the unipolarity.

# SMA motion control for the loop reshape process:
# Inspired by the shape memory alloy, each node maintain desired loop space by modeling the
# feedback from neighbors as stiff linear springs. Each node also trys to achived a certain
# interior angle, the difference from current interior angle to target interior angle acts
# as a potential energy, just like the memorized shape of the alloy. The node trys to
# accomodate this effect by moving itself along the central axis, the effect can be exerted
# from two neighbors, but they may have conflicting ideas, so it is simplier to just move
# host node along central axis. This feedback is also modeled as a rotating spring. This two
# combined spring force can reshape the loop to desired loop formation.


import pygame
from formation_functions import *
import matplotlib.pyplot as plt
from matplotlib import gridspec

import sys, os, math, random, time, getopt
import numpy as np

# read simulation options from arguments
form_opts = [0,0]  # indicating where the initial and target formation comes from
    # '0' for randomly generated
    # '1' for read from file
form_files = [0,0]  # filenames for the formations if read from file
gen_save = False  # whether saving all or none generated formations
show_bargraph = True  # whether showing bargraphs of the preference distributions
try:
    opts, args = getopt.getopt(sys.argv[1:], 'i:t:', ['gensave','nobargraph'])
except getopt.GetoptError as err:
    print str(err)
    sys.exit()
for opt,arg in opts:
    if opt == '-i':
        # read initial formation from file
        form_opts[0] = 1
        form_files[0] = arg  # get the filename of the initial formation
    elif opt == '-t':
        # read target formation from file
        form_opts[1] = 1
        form_files[1] = arg  # get the filename of the target formation
    elif opt == '--gensave':
        # save any generated formations
        gen_save = True
    elif opt == '--nobargraph':
        show_bargraph = False

########################### start of section 1 ###########################

# initialize the pygame
pygame.init()

# name of the folder under save directory that stores loop formation files
loop_folder = 'loop-data'

# parameters for display, window origin is at left up corner
screen_size = (600, 800)  # width and height in pixels
    # top half for initial formation, bottom half for target formation
background_color = (0,0,0)  # black background
robot_color = (255,0,0)  # red for robot and the line segments
robot_color_yellow = (255,255,0)  # yellow for robot
robot_size = 5  # robot modeled as dot, number of pixels for radius
sub_thick = 3  # thickness of line segments for connections in the subgroups

background_color = (255,255,255)
robot_color = (0,0,0)
robot_color_yellow = (0,0,0)
sub_thick = 4

# set up the simulation window and surface object
icon = pygame.image.load("icon_geometry_art.jpg")
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Loop Reshape 2 (dynamic version)")

# for physics, continuous world, origin is at left bottom corner, starting (0, 0),
# with x axis pointing right, y axis pointing up.
# It's more natural to compute the physics in right hand coordiante system.
world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])

# variables to configure the simulation
poly_n = 30  # number of nodes for the polygon, also the robot quantity, at least 3
loop_space = 4.0  # side length of the equilateral polygon
# desired loop space is a little over half of communication range
comm_range = loop_space/0.6
# upper and lower limits have equal difference to the desired loop space
space_upper = comm_range*0.9  # close but not equal to whole communication range
space_lower = comm_range*0.3
# ratio of speed to the distance difference
vel_dist_ratio = 1.0
# period for calculating new positions
# updating is not in real time, because bar graph animation takes too much time
physics_period = 500/1000.0  # second
# for the guessing of the free interior angles
int_angle_reg = math.pi - 2*math.pi/poly_n  # interior angle of regular polygon
# standard deviation of the normal distribution of the guesses
int_angle_dev = (int_angle_reg - math.pi/3)/5

# instantiate the node positions variable
nodes = [[],[]]  # node positions for two formation, index is the robot's identification
nodes[0].append([0, 0])  # first node starts at origin
nodes[0].append([loop_space, 0])  # second node is loop space away on the right
nodes[1].append([0, 0])
nodes[1].append([loop_space, 0])
for i in range(2, poly_n):
    nodes[0].append([0, 0])  # filled with [0,0]
    nodes[1].append([0, 0])

# construct the formation data for the two formation, either generating or from file
for i in range(2):
    if form_opts[i] == 0:  # option to generate a new formation
        # process for generating the random equilateral polygon, two stages
        poly_success = False  # flag for succeed in generating the polygon
        trial_count = 0  # record number of trials until a successful polygon is achieved
        int_final = []  # interior angles to be saved later in file
        while not poly_success:
            trial_count = trial_count + 1
            # print "trial {}: ".format(trial_count),
            # continue trying until all the guesses can forming the desired polygon
            # stage 1: guessing all the free interior angles
            dof = poly_n-3  # number of free interior angles to be randomly generated
            if dof > 0:  # only continue guessing if at least one free interior angle
                # generate all the guesses from a normal distribution
                int_guesses = np.random.normal(int_angle_reg, int_angle_dev, dof).tolist()
                ori_current = 0  # orientation of the line segment
                no_irregular = True  # flag indicating if the polygon is irregular or not
                    # example for irregular cases are intersecting of line segments
                    # or non neighbor nodes are closer than the loop space
                # construct the polygon based on these guessed angles
                for j in range(2, 2+dof):  # for the position of j-th node
                    int_angle_t = int_guesses[j-2]  # interior angle of previous node
                    ori_current = reset_radian(ori_current + (math.pi - int_angle_t))
                    nodes[i][j][0] = nodes[i][j-1][0] + loop_space*math.cos(ori_current)
                    nodes[i][j][1] = nodes[i][j-1][1] + loop_space*math.sin(ori_current)
                    # check the distance of node j to all previous nodes
                    # should not be closer than the loop space
                    for k in range(j-1):  # exclude the previous neighbor
                        vect_temp = [nodes[i][k][0]-nodes[i][j][0],
                                     nodes[i][k][1]-nodes[i][j][1]]  # from j to k
                        dist_temp = math.sqrt(vect_temp[0]*vect_temp[0]+
                                              vect_temp[1]*vect_temp[1])
                        if dist_temp < loop_space:
                            no_irregular = False
                            # print "node {} is too close to node {}".format(j, k)
                            break
                    if not no_irregular:
                        break
                if not no_irregular:
                    continue  # continue the while loop, keep trying new polygon
                else:  # if here, current interior angle guesses are good
                    int_final = int_guesses[:]
                    # although later check on the final node may still disqualify
                    # these guesses, the while loop will exit with a good int_final 
            # stage 2: use convex triangle for the rest, and deciding if polygon is possible
            # solve the one last node
            vect_temp = [nodes[i][0][0]-nodes[i][poly_n-2][0],
                         nodes[i][0][1]-nodes[i][poly_n-2][1]]  # from n-2 to 0
            dist_temp = math.sqrt(vect_temp[0]*vect_temp[0]+
                                  vect_temp[1]*vect_temp[1])
            # check distance between node n-2 and 0 to see if a convex triangle is possible
            # the situation that whether node n-2 and 0 are too close has been excluded
            if dist_temp > 2*loop_space:
                # print("second last node is too far away from the starting node")
                continue
            else:
                # calculate the position of the last node
                midpoint = [(nodes[i][poly_n-2][0]+nodes[i][0][0])/2,
                            (nodes[i][poly_n-2][1]+nodes[i][0][1])/2]
                perp_dist = math.sqrt(loop_space*loop_space - dist_temp*dist_temp/4)
                perp_ori = math.atan2(vect_temp[1], vect_temp[0]) - math.pi/2
                nodes[i][poly_n-1][0] = midpoint[0] + perp_dist*math.cos(perp_ori)
                nodes[i][poly_n-1][1] = midpoint[1] + perp_dist*math.sin(perp_ori)
                # also check any irregularity for the last node
                no_irregular = True
                for j in range(1, poly_n-2):  # exclude starting node and second last node
                    vect_temp = [nodes[i][j][0]-nodes[i][poly_n-1][0],
                                 nodes[i][j][1]-nodes[i][poly_n-1][1]]  # from n-1 to j
                    dist_temp = math.sqrt(vect_temp[0]*vect_temp[0]+
                                          vect_temp[1]*vect_temp[1])
                    if dist_temp < loop_space:
                        no_irregular = False
                        # print "last node is too close to node {}".format(j)
                        break
                if no_irregular:
                    poly_success = True  # reverse the flag
                    if i == 0:  # for print message
                        print "initial formation generated at trial {}".format(trial_count)
                    else:
                        print "target formation generated at trial {}".format(trial_count)
                    # print("successful!")
        # if here, a polygon has been successfully generated, save any new formation
        if not gen_save: continue  # skip following if option is not to save it
        new_filename = get_date_time()
        new_filepath = os.path.join(os.getcwd(), loop_folder, new_filename)
        if os.path.isfile(new_filepath):
            new_filename = new_filename + '-(1)'  # add a suffix to avoid overwrite
            new_filepath = new_filepath + '-(1)'
        f = open(new_filepath, 'w')
        f.write(str(poly_n) + '\n')  # first line is the number of sides of the polygon
        for j in int_final:  # only recorded guessed interior angles
            f.write(str(j) + '\n')  # convert float to string
        f.close()
        # message for a file has been saved
        if i == 0:
            print('initial formation saved as "' + new_filename + '"')
        else:
            print('target formation saved as "' + new_filename + '"')
    else:  # option to read formation from file
        new_filepath = os.path.join(os.getcwd(), loop_folder, form_files[i])
        f = open(new_filepath, 'r')  # read only
        # check if the loop has the same number of side
        if int(f.readline()) == poly_n:
            # continue getting the interior angles
            int_angles = []
            new_line = f.readline()
            while len(new_line) != 0:  # not the end of the file yet
                int_angles.append(float(new_line))  # add the new angle
                new_line = f.readline()
            # check if this file has the number of interior angles as it promised
            if len(int_angles) != poly_n-3:  # these many angles will determine the polygon
                # the number of sides is not consistent inside the file
                print 'file "{}" has incorrect number of interior angles'.format(form_files[i])
                sys.exit()
            # if here the data file is all fine, print message for this
            if i == 0:
                print 'initial formation read from file "{}"'.format(form_files[i])
            else:
                print 'target formation read from file "{}"'.format(form_files[i])
            # construct the polygon from these interior angles
            ori_current = 0  # orientation of current line segment
            for j in range(2, poly_n-1):
                int_angle_t = int_angles[j-2]  # interior angle of previous node
                ori_current = reset_radian(ori_current + (math.pi - int_angle_t))
                nodes[i][j][0] = nodes[i][j-1][0] + loop_space*math.cos(ori_current)
                nodes[i][j][1] = nodes[i][j-1][1] + loop_space*math.sin(ori_current)
                # no need to check any irregularities
            vect_temp = [nodes[i][0][0]-nodes[i][poly_n-2][0],
                         nodes[i][0][1]-nodes[i][poly_n-2][1]]  # from node n-2 to 0
            dist_temp = math.sqrt(vect_temp[0]*vect_temp[0]+
                                  vect_temp[1]*vect_temp[1])
            midpoint = [(nodes[i][poly_n-2][0]+nodes[i][0][0])/2,
                        (nodes[i][poly_n-2][1]+nodes[i][0][1])/2]
            perp_dist = math.sqrt(loop_space*loop_space - dist_temp*dist_temp/4)
            perp_ori = math.atan2(vect_temp[1], vect_temp[0]) - math.pi/2
            nodes[i][poly_n-1][0] = midpoint[0] + perp_dist*math.cos(perp_ori)
            nodes[i][poly_n-1][1] = midpoint[1] + perp_dist*math.sin(perp_ori)
        else:
            # the number of sides is not the same with poly_n specified here
            print 'file "{}" has incorrect number of sides of polygon'.format(form_files[i])
            sys.exit()

# shift the two polygon to the top and bottom halves
nodes = np.array(nodes)  # convert to numpy array
for i in range(2):
    # calculate the geometry center of current polygon
    geometry_center = np.mean(nodes[i], axis=0)
    # shift the polygon to the top or bottom half of the screen
    nodes[i,:,0] = nodes[i,:,0] - geometry_center[0] + world_size[0]/2
    if i == 0:  # initial formation shift to top half
        nodes[i,:,1] = nodes[i,:,1] - geometry_center[1] + 3*world_size[1]/4
    else:  # target formation shift to bottom half
        nodes[i,:,1] = nodes[i,:,1] - geometry_center[1] + world_size[1]/4

# draw the two polygons
screen.fill(background_color)
for i in range(2):
    # draw the nodes and line segments
    disp_pos = [[0,0] for j in range(poly_n)]
    # calculate the display pos for all nodes, draw them as red dots
    for j in range(0, poly_n):
        disp_pos[j] = world_to_display(nodes[i][j], world_size, screen_size)
        pygame.draw.circle(screen, robot_color, disp_pos[j], robot_size, 0)
    # draw an outer circle to mark the starting node
    pygame.draw.circle(screen, robot_color, disp_pos[0], int(robot_size*1.5), 1)
    # line segments for connecitons on the loop
    for j in range(poly_n-1):
        pygame.draw.line(screen, robot_color, disp_pos[j], disp_pos[j+1])
    pygame.draw.line(screen, robot_color, disp_pos[poly_n-1], disp_pos[0])
pygame.display.update()

########################### start of section 2 ###########################

# calculate the interior angles of the two formations
# It's not necessary to do the calculation again, but may have this part ready
# for the dynamic version of the program for the loop reshape simulation.
inter_ang = [[0 for j in range(poly_n)] for i in range(2)]
for i in range(2):
    for j in range(poly_n):
        # for the interior angles of initial setup formation
        node_h = nodes[i][j]  # host node
        node_l = nodes[i][(j-1)%poly_n]  # node on the left
        node_r = nodes[i][(j+1)%poly_n]  # node on the right
        vect_l = [node_l[0]-node_h[0], node_l[1]-node_h[1]]  # from host to left
        vect_r = [node_r[0]-node_h[0], node_r[1]-node_h[1]]  # from host to right
        # get the angle rotating from vect_r to vect_l
        inter_ang[i][j] = math.acos((vect_l[0]*vect_r[0] + vect_l[1]*vect_r[1])/
                                    (loop_space*loop_space))
        if (vect_r[0]*vect_l[1] - vect_r[1]*vect_l[0]) < 0:
            # cross product of vect_r to vect_l is smaller than 0
            inter_ang[i][j] = 2*math.pi - inter_ang[i][j]
        # the result interior angles should be in range of [0, 2*pi)

# rename the interior angle variables to be used later
# use interior angle instead of deviation angle because they should be equivalent
inter_curr = inter_ang[0][:]  # interior angles of initial(dynamic) setup formation
inter_targ = inter_ang[1][:]  # interior angles of target formation
# variable for the preferability distribution
pref_dist = np.zeros((poly_n, poly_n))
# variable indicating which target node has largest probability in the distributions
# this also represents which node it mostly prefers
domi_node = [0 for i in range(poly_n)]  # dominant node in the distributions
# divide nodes on loop to subgroups based on dominant node
# only adjacent block of nodes are in same subgroup if they agree on dominant node
subgroups = []  # lists of adjacent nodes inside
# variable indicating how many nodes are there in the same subgroup with host node
sub_size = []  # size of the subgroup the host node is in
# overflow threshold for the distribution difference
dist_diff_thres = 0.3
# variable for ratio of distribution difference to threshold, for tuning growing rate
# in range of [0,1], higher the ratio, slower it grows
dist_diff_ratio = [0 for i in range(poly_n)]
# exponent of the power function to map the ratio to a slower growing value
dist_diff_power = 0.3
# linear spring constant modeling the pulling and pushing effects of the neighbor nodes
linear_const = 1.0
# bending spring constant modeling the bending effect from host node itself
bend_const = 0.8
disp_coef = 0.5  # coefficient from feedback vector to displacement

# calculate the initial preferability distribution and dominant nodes
for i in range(poly_n):
    # the angle difference of inter_curr[i] to all angles in inter_targ
    ang_diff = [0 for j in range(poly_n)]
    ang_diff_sum = 0
    for j in range(poly_n):
        # angle difference represents the effort of change between two angles
        # the more effort, the less the preferability, so take reciprocal
        ang_diff[j] = 1/abs(inter_curr[i]-inter_targ[j])
        # summation of ang_diff
        ang_diff_sum = ang_diff_sum + ang_diff[j]
    # convert to preferability distribution
    for j in range(poly_n):
        # linearize all probabilities such that sum(pref_dist[i])=1
        pref_dist[i][j] = ang_diff[j]/ang_diff_sum

if show_bargraph:
    # matplotlib method of bar graph animation
    # adjust figure and grid size here
    fig = plt.figure(figsize=(16,12), tight_layout=True)
    fig.canvas.set_window_title('Evolution of Preferability Distribution')
    gs = gridspec.GridSpec(5, 6)
    ax = [fig.add_subplot(gs[i]) for i in range(poly_n)]
    rects = []  # bar chart subplot rectangle handler
    x_pos = range(poly_n)
    for i in range(poly_n):
        rects.append(ax[i].bar(x_pos, pref_dist[i], align='center'))
        ax[i].set_xlim(-1, poly_n)  # y limit depends on data set

# the loop
sim_exit = False  # simulation exit flag
sim_pause = False  # simulation pause flag
iter_count = 0
graph_iters = 1  # draw the distribution graphs every these many iterations
while not sim_exit:
    # exit the program by close window button, or Esc or Q on keyboard
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sim_exit = True  # exit with the close window button
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_SPACE:
                sim_pause = not sim_pause  # reverse the pause flag
            if (event.key == pygame.K_ESCAPE) or (event.key == pygame.K_q):
                sim_exit = True  # exit with ESC key or Q key

    # skip the rest if paused
    if sim_pause: continue

    # prepare information for the preferability distribution evolution
    # find the dominant node in each of the distributions
    for i in range(poly_n):
        domi_node_t = 0  # initialize the dominant node with the first one
        domi_prob_t = pref_dist[i][0]
        for j in range(1, poly_n):
            if pref_dist[i][j] > domi_prob_t:
                domi_node_t = j
                domi_prob_t = pref_dist[i][j]
        domi_node[i] = domi_node_t
    # update the subgroups
    subgroups = [[0]]  # initialize with a node 0 robot
    for i in range(1, poly_n):
        if (domi_node[i-1]+1)%poly_n == domi_node[i]:  # i-1 and i agree on dominant node
            # simply add i to same group with i-1
            subgroups[-1].append(i)
        else:
            # add a new group for node i in subgroups
            subgroups.append([i])
    # check if starting and ending robots should be in same subgroups
    if (domi_node[poly_n-1]+1)%poly_n == domi_node[0] and len(subgroups)>1:
        # add the first subgroup to the last subgroup
        for i in subgroups[0]:
            subgroups[-1].append(i)
        subgroups.pop(0)  # pop out the first subgroup
    # update subgroup size
    sub_size = [0 for i in range(poly_n)]  # initialize with all 0
    for sub in subgroups:
        sub_size_t = len(sub)
        for i in sub:
            sub_size[i] = sub_size_t

    # preferability distribution evolution
    pref_dist_t = np.copy(pref_dist)  # deep copy the 'pref_dist', intermediate variable
    for i in range(poly_n):
        i_l = (i-1)%poly_n  # index of neighbor on the left
        i_r = (i+1)%poly_n  # index of neighbor on the right        
        # shifted distribution from left neighbor
        dist_l = [pref_dist_t[i_l][-1]]  # first one copied from one at end
        for j in range(poly_n-1):
            dist_l.append(pref_dist_t[i_l][j])
        # shifted distribution from right neighbor
        dist_r = []
        for j in range(1, poly_n):
            dist_r.append(pref_dist_t[i_r][j])
        dist_r.append(pref_dist_t[i_r][0])  # last one copied from one at starting
        # calculating if two neighbors have converged ideas with host robot
        converge_l = False
        if (domi_node[i_l]+1)%poly_n == domi_node[i]: converge_l = True
        converge_r = False
        if (domi_node[i_r]-1)%poly_n == domi_node[i]: converge_r = True
        # weighted averaging depending on subgroup property
        if converge_l and converge_r:  # all three neighbors are in the same subgroup
            # step 1: take equal weighted average on all three distributions
            dist_sum = 0
            for j in range(poly_n):
                pref_dist[i][j] = dist_l[j] + pref_dist_t[i][j] + dist_r[j]
                dist_sum = dist_sum + pref_dist[i][j]
            # linearize the distribution
            pref_dist[i] = pref_dist[i]/dist_sum
            # step 2: increase the unipolarity by applying the linear multiplier
            # (step 2 is only for when both neighbors have converged opinions)
            # first find the largest difference in two of the three distributions
            dist_diff = [0, 0, 0]  # variable for difference of three distribution
            # distribution difference of left neighbor and host
            for j in range(poly_n):
                # difference of two distributions is sum of absolute individual differences
                # use current step's distribution for distribution difference
                dist_diff[0] = dist_diff[0] + abs(dist_l[j]-pref_dist_t[i][j])
            # distribution difference of host and right neighbor
            for j in range(poly_n):
                dist_diff[1] = dist_diff[1] + abs(pref_dist_t[i][j]-dist_r[j])
            # distribution difference of left and right neighbors
            for j in range(poly_n):
                dist_diff[2] = dist_diff[2] + abs(dist_l[j]-dist_r[j])
            # maximum distribution differences
            dist_diff_max = max(dist_diff)
            if dist_diff_max < dist_diff_thres:
                dist_diff_ratio[i] = dist_diff_max/dist_diff_thres  # for debugging
                # will skip step 2 if maximum difference is larger than the threshold
                # linear multiplier is generated from the smallest and largest probabilities
                # the smaller end is linearly mapped from largest distribution difference
                # '1.0/poly_n' is the average of the linear multiplier
                small_end = 1.0/poly_n * np.power(dist_diff_max/dist_diff_thres, dist_diff_power)
                large_end = 2.0/poly_n - small_end
                # sort the magnitude of processed distribution
                dist_t = np.copy(pref_dist[i])  # temporary distribution
                sort_index = range(poly_n)
                for j in range(poly_n-1):  # bubble sort, ascending order
                    for k in range(poly_n-1-j):
                        if dist_t[k] > dist_t[k+1]:
                            # exchange values in 'dist_t'
                            temp = dist_t[k]
                            dist_t[k] = dist_t[k+1]
                            dist_t[k+1] = temp
                            # exchange values in 'sort_index'
                            temp = sort_index[k]
                            sort_index[k] = sort_index[k+1]
                            sort_index[k+1] = temp
                # applying the linear multiplier
                dist_sum = 0
                for j in range(poly_n):
                    multiplier = small_end +  float(j)/(poly_n-1) * (large_end-small_end)
                    pref_dist[i][sort_index[j]] = pref_dist[i][sort_index[j]] * multiplier
                    dist_sum = dist_sum + pref_dist[i][sort_index[j]]
                # linearize the distribution
                pref_dist[i] = pref_dist[i]/dist_sum
            else:
                dist_diff_ratio[i] = 1.0  # for debugging, ratio overflowed
        else:  # at least one side has not converged yet
            dist_diff_ratio[i] = -1.0  # indicating linear multiplier was not used
            # take unequal weight in the averaging process based on subgroup property
            sub_size_l = sub_size[i_l]
            sub_size_r = sub_size[i_r]
            # taking the weighted average
            dist_sum = 0
            for j in range(poly_n):
                # weight on left is sub_size_l, on host is 1, on right is sub_size_r
                pref_dist[i][j] = (dist_l[j]*sub_size_l + pref_dist[i][j] +
                                   dist_r[j]*sub_size_r)
                dist_sum = dist_sum + pref_dist[i][j]
            pref_dist[i] = pref_dist[i]/dist_sum

    ##### previous motion strategy #####
    # # physics update, and carry out one iteration of position update
    # for i in range(poly_n):
    #     node_h = nodes[0][i]  # position of host node
    #     node_l = nodes[0][(i-1)%poly_n]  # position of left neighbor
    #     node_r = nodes[0][(i+1)%poly_n]  # position of right neighbor
    #     # find the central axis between the two neighbors
    #     pos_m = [(node_l[0]+node_r[0])/2, (node_l[1]+node_r[1])/2]  # the origin on the axis
    #     vect_rl = [node_l[0]-node_r[0], node_l[1]-node_r[1]]  # from node_r to node_l
    #     # distance of the two neighbors
    #     dist_rl = math.sqrt(vect_rl[0]*vect_rl[0]+vect_rl[1]*vect_rl[1])
    #     vect_rl = [vect_rl[0]/dist_rl, vect_rl[1]/dist_rl]  # become unit vector
    #     vect_ax = [-vect_rl[1], vect_rl[0]]  # central axis pointing outwords of the polygon
    #     vect_ax_ang = math.atan2(vect_ax[1], vect_ax[0])
    #     # all destinations will be defined as how much distance to pos_m along the axis

    #     # find the target destination that satisfies desired interior angle
    #     ang_targ = inter_targ[domi_node[i]]  # dynamic target interior angle
    #     # distance of target position along the axis
    #     targ_dist = loop_space*math.cos(ang_targ/2)
    #     # reverse distance if interior angle is over pi
    #     if ang_targ > math.pi: targ_dist = -targ_dist

    #     # find the stable destination that satisfies the desired loop space
    #     # then decide the final destination by comparing with target destination
    #     final_dist = 0  # variable for the final destination
    #     # Following discussion is based on the range of dist_rl being divided into four
    #     # regions by three points, they are 2*space_upper, 2*loop_space, and 2*space_lower.
    #     if dist_rl >= 2*space_upper:
    #         # two neighbors are too far away, over the upper space limit the host can reach
    #         # no need to compare with target destination, ensure connection first
    #         final_dist = 0  # final destination is at origin
    #     elif dist_rl >= 2*loop_space and dist_rl < 2*space_upper:
    #         # the final destination has a tight single range, stable destination is at origin
    #         stab_dist = 0
    #         # calculate the half range for the final destination
    #         # the rangeis symmetric about the origin, lower range is negative of upper range
    #         range_upper = math.sqrt(space_upper*space_upper-dist_rl*dist_rl/4)
    #         # provisional final destination, balance between interior angle and loop space
    #         # final_dist = (targ_dist+stab_dist)/2
    #         final_dist = targ_dist*0.618 + stab_dist*0.382
    #         # set final destination to limiting positions if exceeding them
    #         final_dist = max(min(final_dist, range_upper), -range_upper)
    #     elif dist_rl >= 2*space_lower and dist_rl < 2*loop_space:
    #         # the final destination has only one range
    #         # but two stable destinations, will choose one closer to target destination
    #         stab_dist = math.sqrt(loop_space*loop_space-dist_rl*dist_rl/4)
    #         range_upper = math.sqrt(space_upper*space_upper-dist_rl*dist_rl/4)
    #         # check which stable destination the target destination is closer to
    #         if abs(targ_dist-stab_dist) < abs(targ_dist+stab_dist):
    #             # closer to stable destination at positive side
    #             # final_dist = (targ_dist+stab_dist)/2
    #             final_dist = targ_dist*0.618 + stab_dist*0.382
    #         else:
    #             # closer to stable destination at negative side
    #             # final_dist = (targ_dist-stab_dist)/2
    #             final_dist = targ_dist*0.618 + (-stab_dist)*0.382
    #         # set final destination to limiting positions if exceeding them
    #         final_dist = max(min(final_dist, range_upper), -range_upper)
    #     elif dist_rl < 2*space_lower:
    #         # final destination has two possible ranges to choose from
    #         # will take one on the side where the node is currently located
    #         stab_dist = math.sqrt(loop_space*loop_space-dist_rl*dist_rl/4)
    #         range_upper = math.sqrt(space_upper*space_upper-dist_rl*dist_rl/4)
    #         range_lower = math.sqrt(space_lower*space_lower-dist_rl*dist_rl/4)
    #         # find out the current side of the node
    #         vect_lr = [-vect_rl[0], -vect_rl[1]]  # vector from left neighbor to right
    #         vect_lh = [node_h[0]-node_l[0], node_h[1]-node_l[1]]  # from left to host
    #         # cross product of vect_lh and vect_lr
    #         if vect_lh[0]*vect_lr[1]-vect_lh[1]*vect_lr[0] >= 0:
    #             # host node is at positive side
    #             # final_dist = (targ_dist+stab_dist)/2
    #             final_dist = targ_dist*0.618 + stab_dist*0.382
    #             # avoid overflow in range [range_lower, range_upper]
    #             final_dist = max(min(final_dist, range_upper), range_lower)
    #         else:
    #             # host node is at negative side
    #             # final_dist = (targ_dist-stab_dist)/2
    #             final_dist = targ_dist*0.618 + (-stab_dist)*0.382
    #             # avoid overflow in range [-range_upper, -range_lower]
    #             final_dist = max(min(final_dist, -range_lower), -range_upper)
    #     # calculate the position of the final destination, where the node desires to move to
    #     final_des = [pos_m[0] + final_dist*math.cos(vect_ax_ang),
    #                  pos_m[1] + final_dist*math.sin(vect_ax_ang)]

    #     # calculate the velocity and orientation based on the final destination
    #     vect_des = [final_des[0]-node_h[0], final_des[1]-node_h[1]]  # from host to des
    #     vect_des_dist = math.sqrt(vect_des[0]*vect_des[0] + vect_des[1]*vect_des[1])
    #     # velocity is proportional to the distance to the final destination
    #     vel = vel_dist_ratio*vect_des_dist
    #     ori = math.atan2(vect_des[1], vect_des[0])

    #     # carry out one step of update on the position
    #     nodes[0][i] = [node_h[0] + vel*physics_period*math.cos(ori),
    #                    node_h[1] + vel*physics_period*math.sin(ori)]

    ##### new SMA motion algorithm based on 'loop_reshape_test_moiton.py' #####
    # variable for the neighboring nodes on the loop
    dist_neigh = [0 for i in range(poly_n)]
    # update the dynamic neighbor distances of the loop
    for i in range(poly_n):
        vect_r = nodes[0][(i+1)%poly_n] - nodes[0][i]  # from host to right node
        dist_neigh[i] = math.sqrt(vect_r[0]*vect_r[0] + vect_r[1]*vect_r[1])
    # update the dynamic interior angles of the loop
    for i in range(poly_n):
        i_l = (i-1)%poly_n
        i_r = (i+1)%poly_n
        vect_l = nodes[0][i_l] - nodes[0][i]
        vect_r = nodes[0][i_r] - nodes[0][i]
        inter_curr[i] = math.acos((vect_l[0]*vect_r[0] + vect_l[1]*vect_r[1])/
                                  (dist_neigh[i_l] * dist_neigh[i]))
        if (vect_r[0]*vect_l[1] - vect_r[1]*vect_l[0]) < 0:
            inter_curr[i] = 2*math.pi - inter_curr[i]
    # the feedback from all spring effects
    fb_vect = [np.zeros([1,2]) for i in range(poly_n)]
    for i in range(poly_n):
        i_l = (i-1)%poly_n
        i_r = (i+1)%poly_n
        # unit vector from host to left
        vect_l = (nodes[0][i_l]-nodes[0][i])/dist_neigh[i_l]
        # unit vector from host to right
        vect_r = (nodes[0][i_r]-nodes[0][i])/dist_neigh[i]
        # unit vector along central axis pointing inward the polygon
        vect_lr = nodes[0][i_r]-nodes[0][i_l]  # from left neighbor to right
        dist_temp = math.sqrt(vect_lr[0]*vect_lr[0]+vect_lr[1]*vect_lr[1])
        vect_in = np.array([-vect_lr[1]/dist_temp, vect_lr[0]/dist_temp])  # rotate ccw pi/2

        # add the pulling or pushing effect from left neighbor
        fb_vect[i] = fb_vect[i] + (dist_neigh[i_l]-loop_space) * linear_const * vect_l
        # add the pulling or pushing effect from right neighbor
        fb_vect[i] = fb_vect[i] + (dist_neigh[i]-loop_space) * linear_const * vect_r
        # add the bending effect initialized by the host node itself
        fb_vect[i] = fb_vect[i] + ((inter_targ[domi_node[i]]-inter_curr[i])*
                                   bend_const * vect_in)

        # update one step of position
        # nodes[0][i] = nodes[0][i] + disp_coef * fb_vect[i]

    # use delay to slow down the physics update when bar graph animation is skpped
    # not clean buy quick way to adjust simulation speed
    time.sleep(0.2)  # in second

    # iteration count update
    print("iteration count {}".format(iter_count))
    iter_count = iter_count + 1  # update iteration count

    # graphics update
    if iter_count%graph_iters == 0:

        if show_bargraph:
            # graphics update for the bar graph
            # find the largest y data in all distributions as up limit in graphs
            y_lim = 0.0
            for i in range(poly_n):
                for j in range(poly_n):
                    if pref_dist[i][j] > y_lim:
                        y_lim = pref_dist[i][j]
            y_lim = min(1.0, y_lim*1.1)  # leave some gap
            # matplotlib method
            for i in range(poly_n):
                for j in range(poly_n):
                    rects[i][j].set_height(pref_dist[i][j])
                    ax[i].set_title('{} -> {} -> {:.2f}'.format(i, sub_size[i], dist_diff_ratio[i]))
                    ax[i].set_ylim(0.0, y_lim)
            fig.canvas.draw()
            fig.show()

        # graphics update for the pygame window
        screen.fill(background_color)
        for i in range(2):
            # calculate the display pos for all nodes
            disp_pos = [[0,0] for j in range(poly_n)]
            for j in range(0, poly_n):
                disp_pos[j] = world_to_display(nodes[i][j], world_size, screen_size)
            # draw the connecting lines
            for j in range(poly_n-1):
                pygame.draw.line(screen, robot_color, disp_pos[j], disp_pos[j+1])
            pygame.draw.line(screen, robot_color, disp_pos[poly_n-1], disp_pos[0])
            # highlight the subgroup connections
            if i == 0:  # only do this for top formation
                for sub in subgroups:
                    for j in range(len(sub)-1):
                        pair_l = sub[j]
                        pair_r = sub[j+1]
                        # use thick yellow lines for connecitons in subgroups
                        pygame.draw.line(screen, robot_color_yellow, disp_pos[pair_l],
                                         disp_pos[pair_r], sub_thick)
                if len(subgroups) == 1:
                    # draw extra segment for starting and end node
                    pair_l = subgroups[0][-1]
                    pair_r = subgroups[0][0]
                    pygame.draw.line(screen, robot_color_yellow, disp_pos[pair_l],
                                     disp_pos[pair_r], sub_thick)                
            # draw all nodes as red dots
            for j in range(0, poly_n):
                pygame.draw.circle(screen, robot_color, disp_pos[j], robot_size, 0)
            # draw an outer circle to mark the starting node
            pygame.draw.circle(screen, robot_color, disp_pos[0], int(robot_size*1.5), 1)
        pygame.display.update()

    raw_input("<Press Enter to continue>")
