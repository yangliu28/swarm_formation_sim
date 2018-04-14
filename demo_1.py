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

# the simulations that run alternatively in this program
# Simulation 1: aggregate together to form a random network
# Simulation 2: consensus decision making of target loop shape
# Simulation 3: consensus role assignment for the loop shape
# Simulation 4: loop formation with designated role assignment
# Simulation 5: loop reshaping to chosen shape

# Note that message transmission is simulated only in the role assignment, because communication
# is specialy depended on and message convey is used as well. While in consensus decision making
# and shape formation, the delay caused by communication are skipped.

# "seed" robot mechanism
# In simulation 1, a new mechanism is added to accelerate the aggregation. Previous, each
# robot in the swarm can start a new group with another robot. The result is that, often there
# are too many small local groups spread out the space, decrease the chance of a large group
# being formed. The new method is asigning certain number of robots to be "seed" robot. Only
# seed robot can initialize the forming of a new group, so as to avoid too many local groups.
# The percentage of seed robots is a new parameter to study, small percentage results in less
# robustness of the aggregation process, large percentage results in slow aggregation process.

# When robot travels almost parallel to the boundaries, sometimes it takes a long time for a
# robot to reach the group. To avoid that, every time a robot is bounced away by the wall, if
# the leaving direction is too perpendicular, a deviation angle is added to deviation the robot.

# 04/11/2018
# In simulation of aggregating to random network, the robots are not required to have a lower
# limit number of connections when in the group. The result is that, the final network topology
# is tree branch like, in which most robots are connected in serial. The unintended advantage
# is the robots are more easily being caught in the tree topology.
# Advised by Dr. Lee, it is better that the final network look like the triangle grid network.
# In this way the swarm robots will have more evenly distributed coverage over the space.
# To implement this: when in a group, if a robot has two or more connections, it moves to the
# destination calculated in the old way. If it has only one connection, it will rotate around
# counter-clockwise around that neighbor robot, untill it finds another neighbor. Again, there
# is an exception that the group itself has only two members.


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


from __future__ import print_function
import pygame
import sys, os, getopt, math, random
import numpy as np
import pickle  # for debugging multiple simulations, and file read/write

swarm_size = 30  # default size of the swarm

# read command line options
try:
    opts, args = getopt.getopt(sys.argv[1:], 'n:')
except getopt.GetoptError as err:
    print(str(err))
    sys.exit()
for opt,arg in opts:
    if opt == '-n':
        swarm_size = int(arg)

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
power_exponent = 1.95  # between 1.0 and 2.0
    # the larger the parameter, the slower the windows grows with swarm size; vice versa
# for converting from physical world to display world
pixels_per_length = 50  # this is to be fixed
# calculate world_side_coef from a desired screen size for 30 robots
def cal_world_side_coef():
    desired_screen_size = 400  # desired screen size for 30 robots
    desired_world_size = float(desired_screen_size) / pixels_per_length
    return desired_world_size / pow(30, 1/power_exponent)
world_side_coef = cal_world_side_coef()
world_side_length = world_side_coef * pow(swarm_size, 1/power_exponent)
world_size = (world_side_length, world_side_length)  # square physical world

# screen size calculated from world size
screen_side_length = int(pixels_per_length * world_side_length)
screen_size = (screen_side_length, screen_side_length)  # square display world

# print("world_side_length: {}".format(world_side_length))
# print("screen_side_length: {}".format(screen_side_length))

# simulation configuration
comm_range = 0.65  # communication range in the world
desired_space_ratio = 0.8  # ratio of the desired space to the communication range
    # should be larger than 1/1.414=0.71, to avoid connections crossing each other
desired_space = comm_range * desired_space_ratio
# deviate robot heading, so as to avoid robot travlling perpendicular to the walls
perp_thres = math.pi/18  # threshold, range from the perpendicular line
devia_angle = math.pi/9  # deviate these much angle from perpendicualr line

# robot properties
robot_poses = np.random.rand(swarm_size, 2) * world_side_length  # initialize the robot poses
dist_table = np.zeros((swarm_size, swarm_size))  # distances between robots
conn_table = np.zeros((swarm_size, swarm_size))  # connections between robots
    # 0 for disconnected, 1 for connected
conn_lists = [[] for i in range(swarm_size)]  # lists of robots connected
# function for all simulations, update the distances and connections between the robots
def dist_conn_update():
    global dist_table
    global conn_table
    global conn_lists
    conn_lists = [[] for i in range(swarm_size)]  # empty the lists
    for i in range(swarm_size):
        for j in range(i+1, swarm_size):
            dist_temp = np.linalg.norm(robot_poses[i] - robot_poses[j])
            dist_table[i,j] = dist_temp
            dist_table[j,i] = dist_temp
            if dist_temp > comm_range:
                conn_table[i,j] = 0
                conn_table[j,i] = 0
            else:
                conn_table[i,j] = 1
                conn_table[j,i] = 1
                conn_lists[i].append(j)
                conn_lists[j].append(i)
dist_conn_update()  # update the distances and connections
disp_poses = []  # display positions
# function for all simulations, update the display positions
def disp_poses_update():
    global disp_poses
    poses_temp = robot_poses / world_side_length
    poses_temp[:,1] = 1.0 - poses_temp[:,1]
    poses_temp = poses_temp * screen_side_length
    disp_poses = poses_temp.astype(int)  # convert to int and assign to disp_poses
disp_poses_update()
# deciding the seed robots, used in simulations with moving robots
seed_percentage = 0.1  # the percentage of seed robots in the swarm
seed_quantity = min(max(int(swarm_size*seed_percentage), 1), swarm_size)
    # no smaller than 1, and no larger than swarm_size
robot_seeds = [False for i in range(swarm_size)]  # whether a robot is a seed robot
    # only seed robot can initialize the forming a new group
seed_list_temp = np.arange(swarm_size)
np.random.shuffle(seed_list_temp)
for i in seed_list_temp[:seed_quantity]:
    robot_seeds[i] = True

# consensus configuration
shape_quantity = 30  # also the number of decisions
shape_decision = -1  # the index of chosen decision, in range(shape_quantity)
assignment_scheme = np.zeros(swarm_size)

# visualization configuration
color_white = (255,255,255)
color_black = (0,0,0)
color_grey = (128,128,128)
# distinct_color_set = ((230,25,75), (60,180,75), (255,225,25), (0,130,200), (245,130,48),
#     (145,30,180), (70,240,240), (240,50,230), (210,245,60), (250,190,190),
#     (0,128,128), (230,190,255), (170,110,40), (255,250,200), (128,0,0),
#     (170,255,195), (128,128,0), (255,215,180), (0,0,128), (128,128,128))
distinct_color_set = ((230,25,75), (60,180,75), (255,225,25), (0,130,200), (245,130,48),
    (145,30,180), (70,240,240), (240,50,230), (210,245,60), (250,190,190),
    (0,128,128), (230,190,255), (170,110,40), (128,0,0),
    (170,255,195), (128,128,0), (0,0,128))
color_quantity = 17
robot_size_formation = 5  # robot size in formation simulations
robot_width_empty = 2
conn_width_formation = 2  # connection line width in formation simulations
robot_size_consensus = 8  # robot size in consensus simulatiosn
conn_width_thin_consensus = 2  # thin connection line in consensus simulations
conn_width_thick_consensus = 5  # thick connection line in consensus simulations
robot_ring_size = 12  # extra ring on robot in consensus simulations

# set up the simulation window
pygame.init()
icon = pygame.image.load("icon_geometry_art.jpg")
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Demo 1")
# draw the network
screen.fill(color_white)
for i in range(swarm_size):
    pygame.draw.circle(screen, color_black, disp_poses[i], robot_size_formation,
        robot_width_empty)
    # pygame.draw.circle(screen, color_black, disp_poses[i],
    #     int(comm_range*pixels_per_length), 1)
pygame.display.update()

# pause to check the network before the simulations, or for screen recording
# raw_input("<Press Enter to continue>")

# function for simulation 1 and 4, group robots by their group ids, and find the largest group
def S14_robot_grouping(robot_list, robot_group_ids, groups):
    # "robot_list": the list of robot '1's, should not be empty
    # the input list 'robot_list' should not be empty
    groups_temp = {}  # key is group id, value is list of robots
    for i in robot_list:
        group_id_temp = robot_group_ids[i]
        if group_id_temp not in groups_temp.keys():
            groups_temp[group_id_temp] = [j]
        else:
            groups_temp[group_id_temp].append(j)
    group_id_max = -1  # the group with most members
        # regardless of only one group or multiple groups in groups_temp
    if len(groups_temp.keys()) > 1:  # there is more than one group
        # find the largest group and disassemble the rest
        group_id_max = groups_temp.keys()[0]
        size_max = len(groups[group_id_max][0])
        for group_id_temp in groups_temp.keys()[1:]:
            size_temp = len(groups[group_id_temp][0])
            if size_temp > size_max:
                group_id_max = group_id_temp
                size_max = size_temp
    else:  # only one group, automatically the largest one
        group_id_max = groups_temp.keys()[0]
    return groups_temp, group_id_max

# function for simulation 1 and 4, find the closest robot to a host robot
# use global variable "dist_table"
def S14_closest_robot(robot_host, robot_neighbors):
    # "robot_host": the robot to measure distance from
    # "robot_neighbors": a list of robots to be compared with
    robot_closest = robot_neighbors[0]
    dist_closest = dist_table[robot_host,robot_closest]
    for i in robot_neighbors[1:]:
        dist_temp = dist_table[robot_host,i]
        if dist_temp < dist_closest:
            robot_closest = i
            dist_closest = dist_temp
    return robot_closest

# general function to normalize a numpy vector
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v/norm

# general function to reset radian angle to [-pi, pi)
def reset_radian(radian):
    while radian >= math.pi:
        radian = radian - 2*math.pi
    while radian < -math.pi:
        radian = radian + 2*math.pi
    return radian

# general function to steer robot away from wall if out of boundary (following physics)
# use global variable "world_side_length"
def robot_boundary_check(robot_pos, robot_ori):
    new_ori = robot_ori
    if robot_pos[0] >= world_side_length:  # outside of right boundary
        if math.cos(new_ori) > 0:
            new_ori = reset_radian(2*(math.pi/2) - new_ori)
            # further check if new angle is too much perpendicular
            if new_ori > 0:
                if (math.pi - new_ori) < perp_thres:
                    new_ori = new_ori - devia_angle
            else:
                if (new_ori + math.pi) < perp_thres:
                    new_ori = new_ori + devia_angle
    elif robot_pos[0] <= 0:  # outside of left boundary
        if math.cos(new_ori) < 0:
            new_ori = reset_radian(2*(math.pi/2) - new_ori)
            if new_ori > 0:
                if new_ori < perp_thres:
                    new_ori = new_ori + devia_angle
            else:
                if (-new_ori) < perp_thres:
                    new_ori = new_ori - devia_angle
    if robot_pos[1] >= world_side_length:  # outside of top boundary
        if math.sin(new_ori) > 0:
            new_ori = reset_radian(2*(0) - new_ori)
            if new_ori > -math.pi/2:
                if (new_ori + math.pi/2) < perp_thres:
                    new_ori = new_ori + devia_angle
            else:
                if (-math.pi/2 - new_ori) < perp_thres:
                    new_ori = new_ori - devia_angle
    elif robot_pos[1] <= 0:  # outside of bottom boundary
        if math.sin(new_ori) < 0:
            new_ori = reset_radian(2*(0) - new_ori)
            if new_ori > math.pi/2:
                if (new_ori - math.pi/2) < perp_thres:
                    new_ori = new_ori + devia_angle
            else:
                if (math.pi/2 - new_ori) < perp_thres:
                    new_ori = new_ori - devia_angle
    return new_ori

# # general function to steer robot away from wall if out of boundary (in random direction)
# # use global variable "world_side_length"
# def robot_boundary_check(robot_pos, robot_ori):
#     new_ori = robot_ori
#     if robot_pos[0] >= world_side_length:  # outside of right boundary
#         if math.cos(new_ori) > 0:
#             new_ori = reset_radian(math.pi/2 + np.random.uniform(0,math.pi))
#     elif robot_pos[0] <= 0:  # outside of left boundary
#         if math.cos(new_ori) < 0:
#             new_ori = reset_radian(-math.pi/2 + np.random.uniform(0,math.pi))
#     if robot_pos[1] >= world_side_length:  # outside of top boundary
#         if math.sin(new_ori) > 0:
#             new_ori = reset_radian(-math.pi + np.random.uniform(0,math.pi))
#     elif robot_pos[1] <= 0:  # outside of bottom boundary
#         if math.sin(new_ori) < 0:
#             new_ori = reset_radian(0 + np.random.uniform(0,math.pi))
#     return new_ori

# flow control varialbes shared by all individual simulations
sim_haulted = False
time_last = 0.0
time_now = 0.0
frame_period = 100
sim_freq_control = True
iter_count = 0

# main loop of the program that run the set of simulations infinitely
# this loop does not exit unless error thrown out or manually terminated from terminal
while True:

    ########### simulation 1: aggregate together to form a random network ###########

    print("##### simulation 1: network aggregation #####")

    # (switching from using 'status' to using 'state': state here refers to being in one
    # condition from many options, like whether in a group, whether available for connection.
    # Status usually refers in a series of predefined stages, which goes one way from start
    # to the end, like referring the progress of a project. While my state may jump back and
    # forth. It's controversial of which one to use, but 'state' is what I choose.)

    # robot perperties
    # all robots start with state '-1', wandering around and ignoring connections
    robot_states = np.array([-1 for i in range(swarm_size)])
        # '-1' for being single, moving around, not available for connection
        # '0' for being single, moving around, available for connection
        # '1' for in a group, adjust position for maintaining connections
    n1_life_lower = 2  # inclusive
    n1_life_upper = 6  # exclusive
    robot_n1_lives = np.random.uniform(n1_life_lower, n1_life_upper, swarm_size)
    robot_oris = np.random.rand(swarm_size) * 2 * math.pi - math.pi  # in range of [-pi, pi)

    # group properties
    groups = {}
        # key is the group id, value is a list, in the list:
        # [0]: a list of robots in the group
        # [1]: remaining life time of the group
        # [2]: whether or not being the dominant group
    life_incre = 5  # number of seconds added to the life of a group when new robot joins
    group_id_upper = swarm_size  # group id is integer randomly chosen in [0, group_id_upper)
    robot_group_ids = np.array([-1 for i in range(swarm_size)])  # group id of the robots
        # '-1' for not in a group

    # use moving distance in each simulation step, instead of robot velocity
    # so to make it independent of simulation frequency control
    step_moving_dist = 0.05  # should be smaller than destination distance error
    destination_error = 0.08
    mov_vec_ratio = 0.5  # ratio used when calculating mov vector

    # the loop for simulation 1
    sim_haulted = False
    time_last = pygame.time.get_ticks()
    time_now = time_last
    frame_period = 100
    sim_freq_control = True
    iter_count = 0
    sys.stdout.write("iteration {}".format(iter_count))  # did nothing in iteration 0
    swarm_aggregated = False
    ending_period = 3.0  # leave this much time to let robots settle after aggregation is dine
    while False:
        # close window button to exit the entire program;
        # space key to pause this simulation
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # close window button is clicked
                print("program exit in simulation 1")
                sys.exit()  # exit the entire program
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_SPACE:
                    sim_haulted = not sim_haulted  # reverse the pause flag
        if sim_haulted: continue

        # simulation frequency control
        if sim_freq_control:
            time_now = pygame.time.get_ticks()
            if (time_now - time_last) > frame_period:
                time_last = time_now
            else:
                continue

        # increase iteration count
        iter_count = iter_count + 1
        sys.stdout.write("\riteration {}".format(iter_count))
        sys.stdout.flush()

        # state transition variables
        st_n1to0 = []  # robot '-1' gets back to '0' after life time ends
            # list of robots changing to '0' from '-1'
        st_gton1 = []  # group disassembles either life expires, or triggered by others
            # list of groups to be disassembled
        st_0to1_join = {}  # robot '0' detects robot '1' group, join the group
            # key is the robot '0', value is the group id
        st_0to1_new = {}  # robot '0' detects another robot '0', forming new group
            # key is the robot '0', value is the other neighbor robot '0'

        # update the "relations" of the robots
        dist_conn_update()
        # check any state transition, and schedule the tasks
        for i in range(swarm_size):
            if robot_states[i] == -1:  # for host robot with state '-1'
                if robot_n1_lives[i] < 0:
                    st_n1to0.append(i)  # life of '-1' ends, becoming '0'
                else:
                    conn_temp  = conn_lists[i][:]  # a list of connections with only state '1'
                    for j in conn_lists[i]:
                        if robot_states[j] != 1:
                            conn_temp.remove(j)
                    if len(conn_temp) != 0:
                        groups_local, group_id_max = S14_robot_grouping(conn_temp,
                            robot_group_ids, groups)
                        # disassmeble all groups except the largest one
                        for group_id_temp in groups_local.keys():
                            if (group_id_temp != group_id_max) and (group_id_temp not in st_gton1):
                                st_gton1.append(group_id_temp)  # schedule to disassemble this group
                        # find the closest neighbor in groups_local[group_id_max]
                        robot_closest = S14_closest_robot(i, groups_local[group_id_max])
                        # change moving direction opposing the closest robot
                        vect_temp = robot_poses[i] - robot_poses[robot_closest]
                        robot_oris[i] = math.atan2(vect_temp[1], vect_temp[0])
            elif robot_states[i] == 0:  # for host robot with state '0'
                state1_list = []  # list of state '1' robots in the connection list
                state0_list = []  # list of state '0' robots in teh connection list
                for j in conn_lists[i]:
                    # ignore state '-1' robots
                    if robot_states[j] == 1:
                        state1_list.append(j)
                    elif robot_states[j] == 0:
                        state0_list.append(j)
                if len(state1_list) != 0:
                    # there is state '1' robot in the list, ignoring state '0' robot
                    groups_local, group_id_max = S14_robot_grouping(state1_list,
                        robot_group_ids, groups)
                    # disassmeble all groups except the largest one
                    for group_id_temp in groups_local.keys():
                        if (group_id_temp != group_id_max) and (group_id_temp not in st_gton1):
                            st_gton1.append(group_id_temp)  # schedule to disassemble this group
                    # join the the group with the most members
                    st_0to1_join[i] = group_id_max
                elif len(state0_list) != 0:
                    # there is no robot '1', but has robot '0'
                    # find the closest robot, schedule to start a new group with it
                    st_0to1_new[i] = S14_closest_robot(i, state0_list)
            elif robot_states[i] == 1:  # for host robot with state '1'
                conn_temp  = conn_lists[i][:]  # a list of connections with only state '1'
                has_other_group = False  # whether there is robot '1' from other group
                host_group_id = robot_group_ids[i]  # group id of host robot
                for j in conn_lists[i]:
                    if robot_states[j] != 1:
                        conn_temp.remove(j)
                    else:
                        if robot_group_ids[j] != host_group_id:
                            has_other_group = True
                # disassemble the smaller groups
                if has_other_group:
                    groups_local, group_id_max = S14_robot_grouping(conn_temp,
                        robot_group_ids, groups)
                    # disassmeble all groups except the largest one
                    for group_id_temp in groups_local.keys():
                        if (group_id_temp != group_id_max) and (group_id_temp not in st_gton1):
                            st_gton1.append(group_id_temp)  # schedule to disassemble this group
            else:  # to be tested and deleted
                print("robot state error")
                sys.exit()

        # check the life time of the groups; if expired, schedule disassemble
        for group_id_temp in groups.keys():
            if groups[group_id_temp][1] < 0:  # life time of a group ends
                if group_id_temp not in st_gton1:
                    st_gton1.append(group_id_temp)

        # process the scheduled state transitions, different transition has different priority
        # 1.st_0to1_join, robot '0' joins a group, becomes '1'
        for robot_temp in st_0to1_join.keys():
            group_id_temp = st_0to1_join[robot_temp]  # the id of the group to join
            # update properties of the robot
            robot_states[robot_temp] = 1
            robot_group_ids[robot_temp] = group_id_temp
            # update properties of the group
            groups[group_id_temp][0].append(robot_temp)
            groups[group_id_temp][1] = groups[group_id_temp][1] + life_incre
        # 2.st_gton1
        for group_id_temp in st_gton1:
            for robot_temp in groups[group_id_temp][0]:
                robot_states[robot_temp] = -1
                robot_n1_lives[robot_temp] = np.random.uniform(n1_life_lower, n1_life_upper)
                robot_group_ids[robot_temp] = -1
                robot_oris[robot_temp] = np.random.rand() * 2 * math.pi - math.pi
            groups.pop(group_id_temp)
        # 3.st_0to1_new
        while len(st_0to1_new.keys()) != 0:
            pair0 = st_0to1_new.keys()[0]
            pair1 = st_0to1_new[pair0]
            st_0to1_new.pop(pair0)
            if (pair1 in st_0to1_new.keys()) and (st_0to1_new[pair1] == pair0):
                st_0to1_new.pop(pair1)
                # only forming a group if there is at least one seed robot in the pair
                if robot_seeds[pair0] or robot_seeds[pair1]:
                    # forming new group for robot pair0 and pair1
                    group_id_temp = np.random.randint(0, group_id_upper)
                    while group_id_temp in groups.keys():
                        group_id_temp = np.random.randint(0, group_id_upper)
                    # update properties of the robots
                    robot_states[pair0] = 1
                    robot_states[pair1] = 1
                    robot_group_ids[pair0] = group_id_temp
                    robot_group_ids[pair1] = group_id_temp
                    # update properties of the group
                    groups[group_id_temp] = [[pair0, pair1], life_incre*2, False]
        # 4.st_n1to0
        for robot_temp in st_n1to0:
            robot_states[robot_temp] = 0

        # check if a group becomes dominant
        for group_id_temp in groups.keys():
            if len(groups[group_id_temp][0]) > swarm_size/2.0:
                groups[group_id_temp][2] = True
            else:
                groups[group_id_temp][2] = False

        # update the physics
        local_conn_lists = [[] for i in range(swarm_size)]  # list of connections in same group
            # only for state '1' robot
        for i in range(swarm_size):
            # change move direction only for robot '1', for adjusting location in group
            if robot_states[i] == 1:
                # find the neighbors in the same group
                host_group_id = robot_group_ids[i]
                for j in conn_lists[i]:
                    if (robot_states[j] == 1) and (robot_group_ids[j] == host_group_id):
                        local_conn_lists[i].append(j)
                if len(local_conn_lists[i]) == 0:  # should not happen after parameter tuning
                    printf("robot {} loses its group {}".format(i, host_group_id))
                    sys.exit()
                # calculating the moving direction, based on neighbor situation
                if (len(local_conn_lists[i]) == 1) and (len(groups[host_group_id][0]) > 2):
                    # If the robot has only one neighbor, and it is not the case that the group
                    # has only members, then the robot will try to secure another neighbor, by
                    # rotating counter-clockwise around this only neighbor.
                    center = local_conn_lists[i][0]  # the center robot
                    # use the triangle of (desired_space, dist_table[i,center], step_moving_dist)
                    if dist_table[i,center] > (desired_space + step_moving_dist):
                        # moving toward the center robot
                        robot_oris[i] = math.atan2(robot_poses[center,1] - robot_poses[i,1],
                            robot_poses[center,0] - robot_poses[i,0])
                    elif (dist_table[i,center] + step_moving_dist) < desired_space:
                        # moving away from the center robot
                        robot_oris[i] = math.atan2(robot_poses[i,1] - robot_poses[center,1],
                            robot_poses[i,0] - robot_poses[center,0])
                    else:
                        # moving tangent along the circle of radius of "desired_space"
                        robot_oris[i] = math.atan2(robot_poses[i,1] - robot_poses[center,1],
                            robot_poses[i,0] - robot_poses[center,0])
                        # interior angle between 
                        int_angle_temp = math.acos((math.pow(dist_table[i,center],2) +
                            math.pow(step_moving_dist,2) - math.pow(desired_space,2)) /
                            (2.0*dist_table[i,center]*step_moving_dist))
                        robot_oris[i] = reset_radian(robot_oris[i] + (math.pi - int_angle_temp))
                else:  # the normal situation
                    # calculate the moving vector, and check if destination is within error range
                    mov_vec = np.zeros(2)
                    for j in local_conn_lists[i]:  # accumulate the influence from all neighbors
                        mov_vec = mov_vec + (mov_vec_ratio * (dist_table[i,j] - desired_space) *
                            normalize(robot_poses[j] - robot_poses[i]))
                    if np.linalg.norm(mov_vec) < destination_error:
                        continue  # skip the physics update if within destination error
                    else:
                        robot_oris[i] = math.atan2(mov_vec[1], mov_vec[0])  # change direction
            # check if out of boundaries
            robot_oris[i] = robot_boundary_check(robot_poses[i], robot_oris[i])
            # update one step of move
            robot_poses[i] = robot_poses[i] + (step_moving_dist *
                np.array([math.cos(robot_oris[i]), math.sin(robot_oris[i])]))

        # update the graphics
        disp_poses_update()
        screen.fill(color_white)
        # draw the robots of states '-1' and '0'
        for i in range(swarm_size):
            if robot_states[i] == -1:  # empty circle for state '-1' robot
                pygame.draw.circle(screen, color_grey, disp_poses[i],
                    robot_size_formation, robot_width_empty)
            elif robot_states[i] == 0:  # full circle for state '0' robot
                pygame.draw.circle(screen, color_grey, disp_poses[i],
                    robot_size_formation, 0)
        # draw the in-group robots by each group
        for group_id_temp in groups.keys():
            if groups[group_id_temp][2]:
                # highlight the dominant group with black color
                color_group = color_black
            else:
                color_group = color_grey
            conn_draw_sets = []  # avoid draw same connection two times
            # draw the robots and connections in the group
            for i in groups[group_id_temp][0]:
                pygame.draw.circle(screen, color_group, disp_poses[i],
                    robot_size_formation, 0)
                for j in local_conn_lists[i]:
                    if set([i,j]) not in conn_draw_sets:
                        pygame.draw.line(screen, color_group, disp_poses[i],
                            disp_poses[j], conn_width_formation)
                        conn_draw_sets.append(set([i,j]))
        pygame.display.update()

        # reduce life time of robot '-1' and groups
        for i in range(swarm_size):
            if robot_states[i] == -1:
                robot_n1_lives[i] = robot_n1_lives[i] - frame_period/1000.0
        for group_id_temp in groups.keys():
            if not groups[group_id_temp][2]:  # skip dominant group
                groups[group_id_temp][1] = groups[group_id_temp][1] - frame_period/1000.0

        # check exit condition of simulation 1
        if not swarm_aggregated:
            if (len(groups.keys()) == 1) and (len(groups.values()[0][0]) == swarm_size):
                swarm_aggregated = True  # once aggregated, there is no turning back
        if swarm_aggregated:
            if ending_period <= 0:
                print("")  # move cursor to the new line
                print("simulation 1 is finished")
                raw_input("<Press Enter to continue>")
                break
            else:
                ending_period = ending_period - frame_period/1000.0

    # # store the variable "robot_poses"
    # with open('v_robot_poses', 'w') as f:
    #     pickle.dump(robot_poses, f)
    # raw_input("<Press Enter to continue>")
    # break

    ########### simulation 2: consensus decision making of target loop shape ###########

    # restore variable "robot_poses"
    with open('v_robot_poses') as f:
        robot_poses = pickle.load(f)

    print("##### simulation 2: decision making #####")
    # "dist" in the variable may also refer to distribution

    dist_conn_update()  # need to update the network only once
    # draw the network first time
    disp_poses_update()
    screen.fill(color_white)
    for i in range(swarm_size):
        pygame.draw.circle(screen, color_black, disp_poses[i], robot_size_consensus, 0)
        for j in range(i+1, swarm_size):
            if conn_table[i,j]:
                pygame.draw.line(screen, color_black, disp_poses[i], disp_poses[j],
                    conn_width_thin_consensus)
    pygame.display.update()

    # initialize the decision making variables
    shape_decision = -1
    deci_dist = np.random.rand(swarm_size, shape_quantity)
    sum_temp = np.sum(deci_dist, axis=1)
    for i in range(swarm_size):
        deci_dist[i] = deci_dist[i] / sum_temp[i]
    deci_domi = np.argmax(deci_dist, axis=1)
    groups = []  # robots reach local consensus are in same group
    robot_group_sizes = [0 for i in range(swarm_size)]  # group size for each robot
    # color assignments for the robots and decisions
    color_initialized = False  # whether color assignment has been done for the first time
    deci_colors = [-1 for i in range(shape_quantity)]
    color_assigns = [0 for i in range(color_quantity)]
    group_colors = []
    robot_colors = [0 for i in range(swarm_size)]
    # decision making control variables
    dist_diff_thres = 0.3
    dist_diff_ratio = [0.0 for i in range(swarm_size)]
    dist_diff_power = 0.3

    # the loop for simulation 2
    sim_haulted = False
    time_last = pygame.time.get_ticks()
    time_now = time_last
    frame_period = 300
    sim_freq_control = True
    iter_count = 0
    sys.stdout.write("iteration {}".format(iter_count))
    while False:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # close window button is clicked
                print("program exit in simulation 2")
                sys.exit()  # exit the entire program
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_SPACE:
                    sim_haulted = not sim_haulted  # reverse the pause flag
        if sim_haulted: continue

        # simulation frequency control
        if sim_freq_control:
            time_now = pygame.time.get_ticks()
            if (time_now - time_last) > frame_period:
                time_last = time_now
            else:
                continue

        # increase iteration count
        iter_count = iter_count + 1
        sys.stdout.write("\riteration {}".format(iter_count))
        sys.stdout.flush()

        # 1.update the dominant decision for all robots
        deci_domi = np.argmax(deci_dist, axis=1)

        # 2.update the groups
        groups = []  # empty the group container
        group_deci = []  # the exhibited decision of the groups
        robot_pool = range(swarm_size)  # a robot pool, to be assigned into groups
        while len(robot_pool) != 0:  # searching groups one by one from the global robot pool
            # start a new group, with first robot in the robot_pool
            first_member = robot_pool[0]  # first member of this group
            group_temp = [first_member]  # current temporary group
            robot_pool.pop(0)  # pop out first robot in the pool
            # a list of potential members for current group
            # this list may increase when new members of group are discovered
            p_members = list(conn_lists[first_member])
            # an index for iterating through p_members, in searching group members
            p_index = 0  # if it climbs to the end, the searching ends
            # index of dominant decision for current group
            current_domi = deci_domi[first_member]
            # dynamically iterating through p_members with p_index
            while p_index < len(p_members):  # index still in valid range
                if deci_domi[p_members[p_index]] == current_domi:
                    # a new member has been found
                    new_member = p_members[p_index]  # get index of the new member
                    p_members.remove(new_member)  # remove it from p_members list
                        # but not increase p_index, because new value in p_members will flush in
                    robot_pool.remove(new_member)  # remove it from the global robot pool
                    group_temp.append(new_member)  # add it to current group
                    # check if new potential members are available, due to new robot discovery
                    p_members_new = conn_lists[new_member]  # new potential members
                    for member in p_members_new:
                        if member not in p_members:  # should not already in p_members
                            if member not in group_temp:  # should not in current group
                                if member in robot_pool:  # should be available in global pool
                                    # if conditions satisfied, it is qualified as a potential member
                                    p_members.append(member)  # append at the end
                else:
                    # a boundary robot(share different decision) has been met
                    # leave it in p_members, will help to avoid checking back again on this robot
                    p_index = p_index + 1  # shift right one position
            # all connected members for this group have been located
            groups.append(group_temp)  # append the new group
            group_deci.append(deci_domi[first_member])  # append new group's exhibited decision
        # update the colors for the exhibited decisions
        if not color_initialized:
            color_initialized = True
            select_set = range(color_quantity)  # the initial selecting set
            all_deci_set = set(group_deci)  # put all exhibited decisions in a set
            for deci in all_deci_set:  # avoid checking duplicate decisions
                if len(select_set) == 0:
                    select_set = range(color_quantity)  # start a new set to select from
                chosen_color = np.random.choice(select_set)
                select_set.remove(chosen_color)
                deci_colors[deci] = chosen_color  # assign the chosen color to decision
                # increase the assignments of chosen color by 1
                color_assigns[chosen_color] = color_assigns[chosen_color] + 1
        else:
            # remove the color for a decision, if it's no longer the decision of any group
            all_deci_set = set(group_deci)
            for i in range(shape_quantity):
                if deci_colors[i] != -1:  # there was a color assigned before
                    if i not in all_deci_set:
                        # decrease the assignments of chosen color by 1
                        color_assigns[deci_colors[i]] = color_assigns[deci_colors[i]] - 1
                        deci_colors[i] = -1  # remove the assigned color
            # assign color for an exhibited decision if not assigned
            select_set = []  # set of colors to select from, start from empty
            for i in range(len(groups)):
                if deci_colors[group_deci[i]] == -1:
                    if len(select_set) == 0:
                        # construct a new select_set
                        color_assigns_min = min(color_assigns)
                        color_assigns_temp = [j - color_assigns_min for j in color_assigns]
                        select_set = range(color_quantity)
                        for j in range(color_quantity):
                            if color_assigns_temp[j] != 0:
                                select_set.remove(j)
                    chosen_color = np.random.choice(select_set)
                    select_set.remove(chosen_color)
                    deci_colors[group_deci[i]] = chosen_color  # assign the chosen color
                    # increase the assignments of chosen color by 1
                    color_assigns[chosen_color] = color_assigns[chosen_color] + 1
        # update the colors for the groups
        group_colors = []
        for i in range(len(groups)):
            group_colors.append(deci_colors[group_deci[i]])

        # 3.update the group size for each robot
        for i in range(len(groups)):
            size_temp = len(groups[i])
            color_temp = group_colors[i]
            for robot in groups[i]:
                robot_group_sizes[robot] = size_temp
                robot_colors[robot] = color_temp  # update the color for each robot

        # the decision distribution evolution
        converged_all = True  # flag for convergence of entire network
        deci_dist_t = np.copy(deci_dist)  # deep copy of the 'deci_dist'
        for i in range(swarm_size):
            host_domi = deci_domi[i]
            converged = True
            for neighbor in conn_lists[i]:
                if host_domi != deci_domi[neighbor]:
                    converged = False
                    break
            # action based on convergence of dominant decision
            if converged:  # all neighbors have converged with host
                # step 1: take equally weighted average on all distributions
                # including host and all neighbors
                deci_dist[i] = deci_dist_t[i]*1.0  # start with host itself
                for neighbor in conn_lists[i]:
                    # accumulate neighbor's distribution
                    deci_dist[i] = deci_dist[i] + deci_dist_t[neighbor]
                # normalize the distribution such that sum is 1.0
                sum_temp = np.sum(deci_dist[i])
                deci_dist[i] = deci_dist[i] / sum_temp
                # step 2: increase the unipolarity by applying the linear multiplier
                # (this step is only for when all neighbors are converged)
                # First find the largest difference between two distributions in this block
                # of robots, including the host and all its neighbors.
                comb_pool = [i] + list(conn_lists[i])  # add host to a pool with its neighbors
                    # will be used to form combinations from this pool
                comb_pool_len = len(comb_pool)
                dist_diff = []
                for j in range(comb_pool_len):
                    for k in range(j+1, comb_pool_len):
                        j_robot = comb_pool[j]
                        k_robot = comb_pool[k]
                        dist_diff.append(np.sum(abs(deci_dist[j_robot] - deci_dist[k_robot])))
                dist_diff_max = max(dist_diff)  # maximum distribution difference of all
                if dist_diff_max < dist_diff_thres:
                    # distribution difference is small enough,
                    # that linear multiplier should be applied to increase unipolarity
                    dist_diff_ratio = dist_diff_max/dist_diff_thres
                    # Linear multiplier is generated from value of smaller and larger ends, the
                    # smaller end is positively related with dist_diff_ratio. The smaller the
                    # maximum distribution difference, the smaller the dist_diff_ratio, and the
                    # steeper the linear multiplier.
                    # '1.0/shape_quantity' is the average value of the linear multiplier
                    small_end = 1.0/shape_quantity * np.power(dist_diff_ratio, dist_diff_power)
                    large_end = 2.0/shape_quantity - small_end
                    # sort the magnitude of the current distribution
                    dist_temp = np.copy(deci_dist[i])  # temporary distribution
                    sort_index = range(shape_quantity)
                    for j in range(shape_quantity-1):  # bubble sort, ascending order
                        for k in range(shape_quantity-1-j):
                            if dist_temp[k] > dist_temp[k+1]:
                                # exchange values in 'dist_temp'
                                temp = dist_temp[k]
                                dist_temp[k] = dist_temp[k+1]
                                dist_temp[k+1] = temp
                                # exchange values in 'sort_index'
                                temp = sort_index[k]
                                sort_index[k] = sort_index[k+1]
                                sort_index[k+1] = temp
                    # applying the linear multiplier
                    for j in range(shape_quantity):
                        multiplier = small_end + float(j)/(shape_quantity-1) * (large_end-small_end)
                        deci_dist[i][sort_index[j]] = deci_dist[i][sort_index[j]] * multiplier
                    # normalize the distribution such that sum is 1.0
                    sum_temp = np.sum(deci_dist[i])
                    deci_dist[i] = deci_dist[i]/sum_temp
                else:
                    # not applying linear multiplier when distribution difference is large
                    pass
            else:  # at least one neighbor has different opinion with host
                converged_all = False  # the network is not converged
                # take unequal weights in the averaging process based on group sizes
                deci_dist[i] = deci_dist_t[i]*robot_group_sizes[i]  # start with host itself
                for neighbor in conn_lists[i]:
                    # accumulate neighbor's distribution
                    deci_dist[i] = deci_dist[i] + deci_dist_t[neighbor]*robot_group_sizes[neighbor]
                # normalize the distribution
                sum_temp = np.sum(deci_dist[i])
                deci_dist[i] = deci_dist[i] / sum_temp

        # update the graphics
        screen.fill(color_white)
        # draw the regualr connecting lines
        for i in range(swarm_size):
            for j in range(i+1, swarm_size):
                if conn_table[i,j]:
                    pygame.draw.line(screen, color_black, disp_poses[i], disp_poses[j],
                        conn_width_thin_consensus)
        # draw the connecting lines marking the groups
        for i in range(len(groups)):
            group_len = len(groups[i])
            for j in range(group_len):
                for k in range(j+1, group_len):
                    j_robot = groups[i][j]
                    k_robot = groups[i][k]
                    # check if two robots in one group is connected
                    if conn_table[j_robot,k_robot]:
                        pygame.draw.line(screen, distinct_color_set[group_colors[i]],
                            disp_poses[j_robot], disp_poses[k_robot], conn_width_thick_consensus)
        # draw the robots as dots
        for i in range(swarm_size):
            pygame.draw.circle(screen, distinct_color_set[robot_colors[i]],
                disp_poses[i], robot_size_consensus, 0)
        pygame.display.update()

        # check exit condition for simulations 2
        if converged_all:
            shape_decision = deci_domi[0]
            print("")  # move cursor to the new line
            print("converged to decision {}".format(shape_decision))
            print("simulation 2 is finished")
            raw_input("<Press Enter to continue>")
            break

    ########### simulation 3: consensus role assignment for the loop shape ###########

    print("##### simulation 3: role assignment #####")

    dist_conn_update()  # need to update the network only once
    # draw the network first time
    disp_poses_update()
    screen.fill(color_white)
    for i in range(swarm_size):
        pygame.draw.circle(screen, color_black, disp_poses[i], robot_size_consensus, 0)
        for j in range(i+1, swarm_size):
            if conn_table[i,j]:
                pygame.draw.line(screen, color_black, disp_poses[i], disp_poses[j],
                    conn_width_thin_consensus)
    pygame.display.update()

    # calculate the gradient map for message transmission
    gradients = np.copy(conn_table)  # build gradient map on connection map
    pool_gradient = 1  # gradients of the connections in the pool
    pool_conn = {}
    for i in range(swarm_size):
        pool_conn[i] = conn_lists[i][:]  # start with gradient 1 connections
    while len(pool_conn.keys()) != 0:
        source_deactivate = []
        for source in pool_conn:
            targets_temp = []  # the new targets
            for target in pool_conn[source]:
                for target_new in conn_lists[target]:
                    if target_new == source: continue  # skip itself
                    if gradients[source, target_new] == 0:
                        gradients[source, target_new] = pool_gradient + 1
                        targets_temp.append(target_new)
            if len(targets_temp) == 0:
                source_deactivate.append(source)
            else:
                pool_conn[source] = targets_temp[:]  # update with new targets
        for source in source_deactivate:
            pool_conn.pop(source)  # remove the finished sources
        pool_gradient = pool_gradient + 1
    # calculate the relative gradient values
    gradients_rel = []
        # gradients_rel[i][j,k] refers to gradient of k relative to j with message source i
    for i in range(swarm_size):  # message source i
        gradient_temp = np.zeros((swarm_size, swarm_size))
        for j in range(swarm_size):  # in the view point of j
            gradient_temp[j] = gradients[i] - gradients[i,j]
        gradients_rel.append(gradient_temp)
    # list the neighbors a robot can send message to regarding a message source
    neighbors_send = [[[] for j in range(swarm_size)] for i in range(swarm_size)]
        # neighbors_send[i][j][k] means, if message from source i is received in j,
        # it should be send to k
    for i in range(swarm_size):  # message source i
        for j in range(swarm_size):  # in the view point of j
            for neighbor in conn_lists[j]:
                if gradients_rel[i][j,neighbor] == 1:
                    neighbors_send[i][j].append(neighbor)

    # initialize the role assignment variables
    # preference distribution of all robots
    pref_dist = np.random.rand(swarm_size, swarm_size)  # no need to normalize it
    initial_roles = np.argmax(pref_dist, axis=1)  # the chosen role
    # the local assignment information
    local_role_assignment = [[[-1, 0, -1] for j in range(swarm_size)] for i in range(swarm_size)]
        # local_role_assignment[i][j] is local assignment information of robot i for robot j
        # first number is chosen role, second is probability, third is time stamp
    local_robot_assignment = [[[] for j in range(swarm_size)] for i in range(swarm_size)]
        # local_robot_assignment[i][j] is local assignment of robot i for role j
        # contains a list of robots that choose role j
    # populate the chosen role of itself to the local assignment information
    for i in range(swarm_size):
        local_role_assignment[i][i][0] = initial_roles[i]
        local_role_assignment[i][i][1] = pref_dist[i, initial_roles[i]]
        local_role_assignment[i][i][2] = 0
        local_robot_assignment[i][initial_roles[i]].append(i)

    # received message container for all robots
    message_rx = [[] for i in range(swarm_size)]
    # for each message entry, it containts:
        # message[0]: ID of message source
        # message[1]: its preferred role
        # message[2]: probability of chosen role
        # message[3]: time stamp
    # all robots transmit once their chosen role before the loop
    transmission_total = 0  # count message transmissions for each iteration
    iter_count = 0  # also used as time stamp in message
    for source in range(swarm_size):
        chosen_role = local_role_assignment[source][source][0]
        message_temp = [source, chosen_role, pref_dist[source, chosen_role], iter_count]
        for target in conn_lists[source]:  # send to all neighbors
            message_rx[target].append(message_temp)
            transmission_total = transmission_total + 1
    role_color = [0 for i in range(swarm_size)]  # colors for a conflicting role
    # Dynamically manage color for conflicting robots is unnecessarily complicated, might just
    # assign the colors in advance.
    role_index_pool = range(swarm_size)
    random.shuffle(role_index_pool)
    color_index_pool = range(color_quantity)
    random.shuffle(color_index_pool)
    while len(role_index_pool) != 0:
        role_color[role_index_pool[0]] = color_index_pool[0]
        role_index_pool.pop(0)
        color_index_pool.pop(0)
        if len(color_index_pool) == 0:
            color_index_pool = range(color_quantity)
            random.shuffle(color_index_pool)

    # flags
    transmit_flag = [[False for j in range(swarm_size)] for i in range(swarm_size)]
        # whether robot i should transmit received message of robot j
    change_flag = [False for i in range(swarm_size)]
        # whether robot i should change its chosen role
    scheme_converged = [False for i in range(swarm_size)]

    # the loop for simulation 3
    sim_haulted = False
    time_last = pygame.time.get_ticks()
    time_now = time_last
    time_period = 2000  # not frame_period
    sim_freq_control = True
    flash_delay = 200
    sys.stdout.write("iteration {}".format(iter_count))
    while False:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # close window button is clicked
                print("program exit in simulation 3")
                sys.exit()  # exit the entire program
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_SPACE:
                    sim_haulted = not sim_haulted  # reverse the pause flag
        if sim_haulted: continue

        # simulation frequency control
        if sim_freq_control:
            time_now = pygame.time.get_ticks()
            if (time_now - time_last) > time_period:
                time_last = time_now
            else:
                continue

        # increase iteration count
        iter_count = iter_count + 1
        sys.stdout.write("\riteration {}".format(iter_count))
        sys.stdout.flush()

        # process the received messages
        # transfer messages to the processing buffer, then empty the message receiver
        message_rx_buf = [[[k for k in j] for j in i] for i in message_rx]
        message_rx = [[] for i in range(swarm_size)]
        yield_robots = []  # the robots that are yielding on chosen roles
        yield_roles = []  # the old roles of yield_robots before yielding
        for i in range(swarm_size):  # messages received by robot i
            for message in message_rx_buf[i]:
                source = message[0]
                role = message[1]
                probability = message[2]
                time_stamp = message[3]
                if source == i:
                    print("error, robot {} receives message of itself".format(i))
                    sys.exit()
                if time_stamp > local_role_assignment[i][source][2]:
                    # received message will only take any effect if time stamp is new
                    # update local_robot_assignment
                    role_old = local_role_assignment[i][source][0]
                    if role_old >= 0:  # has been initialized before, not -1
                        local_robot_assignment[i][role_old].remove(source)
                    local_robot_assignment[i][role].append(source)
                    # update local_role_assignment
                    local_role_assignment[i][source][0] = role
                    local_role_assignment[i][source][1] = probability
                    local_role_assignment[i][source][2] = time_stamp
                    transmit_flag[i][source] = True
                    # check conflict with itself
                    if role == local_role_assignment[i][i][0]:
                        if probability >= pref_dist[i, local_role_assignment[i][i][0]]:
                            # change its choice after all message received
                            change_flag[i] = True
                            yield_robots.append(i)
                            yield_roles.append(local_role_assignment[i][i][0])
        # change the choice of role for those decide to
        for i in range(swarm_size):
            if change_flag[i]:
                change_flag[i] = False
                role_old = local_role_assignment[i][i][0]
                pref_dist_temp = np.copy(pref_dist[i])
                pref_dist_temp[local_role_assignment[i][i][0]] = -1
                    # set to negative to avoid being chosen
                for j in range(swarm_size):
                    if len(local_robot_assignment[i][j]) != 0:
                        # eliminate those choices that have been taken
                        pref_dist_temp[j] = -1
                role_new = np.argmax(pref_dist_temp)
                if pref_dist_temp[role_new] < 0:
                    print("error, robot {} has no available role".format(i))
                    sys.exit()
                # role_new is good to go
                # update local_robot_assignment
                local_robot_assignment[i][role_old].remove(i)
                local_robot_assignment[i][role_new].append(i)
                # update local_role_assignment
                local_role_assignment[i][i][0] = role_new
                local_role_assignment[i][i][1] = pref_dist[i][role_new]
                local_role_assignment[i][i][2] = iter_count
                transmit_flag[i][i] = True
        # transmit the received messages or initial new message transmission
        transmission_total = 0
        for transmitter in range(swarm_size):  # transmitter robot
            for source in range(swarm_size):  # message is for this source robot
                if transmit_flag[transmitter][source]:
                    transmit_flag[transmitter][source] = False
                    message_temp = [source, local_role_assignment[transmitter][source][0],
                                            local_role_assignment[transmitter][source][1],
                                            local_role_assignment[transmitter][source][2]]
                    for target in neighbors_send[source][transmitter]:
                        message_rx[target].append(message_temp)
                        transmission_total = transmission_total + 1

        # check if role assignment scheme is converged at every robot
        for i in range(swarm_size):
            if not scheme_converged[i]:
                converged = True
                for j in range(swarm_size):
                    if len(local_robot_assignment[i][j]) != 1:
                        converged  = False
                        break
                if converged:
                    scheme_converged[i] = True

        # for display, scan the robots that have detected conflict but not yielding
        persist_robots = []
        for i in range(swarm_size):
            if i in yield_robots: continue
            if len(local_robot_assignment[i][local_role_assignment[i][i][0]]) > 1:
                persist_robots.append(i)

        # update the display
        for i in range(swarm_size):
            for j in range(i+1, swarm_size):
                if conn_table[i,j]:
                    pygame.draw.line(screen, color_black, disp_poses[i], disp_poses[j],
                        conn_width_thin_consensus)
        for i in range(swarm_size):
            pygame.draw.circle(screen, color_black, disp_poses[i], robot_size_consensus, 0)
        # draw the persisting robots with color of conflicting role
        for i in persist_robots:
            pygame.draw.circle(screen, distinct_color_set[role_color[local_role_assignment[i][i][0]]],
                disp_poses[i], robot_size_consensus, 0)
        # draw extra ring on robot if local scheme has converged
        for i in range(swarm_size):
            if scheme_converged[i]:
                pygame.draw.circle(screen, color_black, disp_poses[i],
                    robot_ring_size, robot_width_empty)
        pygame.display.update()
        # flash the yielding robots with color of old role
        for _ in range(3):
            # change to color
            for i in range(len(yield_robots)):
                pygame.draw.circle(screen, distinct_color_set[role_color[yield_roles[i]]],
                    disp_poses[yield_robots[i]], robot_size_consensus, 0)
            pygame.display.update()
            pygame.time.delay(flash_delay)
            # change to black
            for i in range(len(yield_robots)):
                pygame.draw.circle(screen, color_black,
                    disp_poses[yield_robots[i]], robot_size_consensus, 0)
            pygame.display.update()
            pygame.time.delay(flash_delay)

        # exit the simulation if all role assignment schemes have converged
        all_converged = scheme_converged[0]
        for i in range(1, swarm_size):
            all_converged = all_converged and scheme_converged[i]
            if not all_converged: break
        if all_converged:
            for i in range(swarm_size):
                assignment_scheme[i] = local_role_assignment[0][i][0]
            print("")  # move cursor to the new line
            print("converged role assignment scheme: {}".format(assignment_scheme))
            print("simulation 3 is finished")
            raw_input("<Press Enter to continue>")
            break

    # # store the variable "assignment_scheme"
    # with open('v_assignment_scheme', 'w') as f:
    #     pickle.dump(assignment_scheme, f)
    # raw_input("<Press Enter to continue>")
    # break

    ########### simulation 4: loop formation with designated role assignment ###########

    # restore variable "assignment_scheme"
    with open('v_assignment_scheme') as f:
        assignment_scheme = pickle.load(f)

    print("##### simulation 4: loop formation #####")

    # robot perperties
    # all robots start with state '-1'
    robot_states = np.array([-1 for i in range(swarm_size)])
        # '-1' for wandering around, ignoring all connections
        # '0' for wandering around, available to connection
        # '1' for in a group, order on loop not satisfied, climbing CCW around loop
        # '2' for in a group, order on loop satisfied
    n1_life_lower = 2  # inclusive
    n1_life_upper = 6  # exclusive
    robot_n1_lives = np.random.uniform(n1_life_lower, n1_life_upper, swarm_size)
    robot_oris = np.random.rand(swarm_size) * 2 * math.pi - math.pi  # in range of [-pi, pi)
    robot_key_neighbors = [[] for i in range(swarm_size)]  # key neighbors for robot on loop
        # for state '1' robot: the robot that it is climbing around
        # for state '2' robot: the left and right neighbors in serial connection on the loop
            # exception is the group has only two members, key neighbor will be only one robot

    # group properties
    groups = {}
        # key is the group id, value is a list, in the list:
        # [0]: a list of robots in the group, both state '1' and '2'
        # [1]: remaining life time of the group
        # [2]: whether or not being the dominant group
    life_incre = 5  # number of seconds added to the life of a group when new robot joins
    group_id_upper = swarm_size  # upper limit of group id
    robot_group_ids = np.array([-1 for i in range(swarm_size)])  # group id for the robots
        # '-1' for not in a group

    # use step moving distance in each update, instead of calculating from robot velocity
    # so to make it independent of simulation frequency control
    step_moving_dist = 0.05  # should be smaller than destination distance error
    destination_error = 0.08
    mov_vec_ratio = 0.5  # ratio used when calculating mov vector
    # spring constants in SMA
    linear_const = 1.0
    bend_const = 0.8
    disp_coef = 0.3

    # the loop for simulation 1
    sim_haulted = False
    time_last = pygame.time.get_ticks()
    time_now = time_last
    frame_period = 100
    sim_freq_control = True
    iter_count = 0
    sys.stdout.write("iteration {}".format(iter_count))  # did nothing in iteration 0
    loop_formed = False
    ending_period = 3.0  # grace period
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # close window button is clicked
                print("program exit in simulation 4")
                sys.exit()  # exit the entire program
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_SPACE:
                    sim_haulted = not sim_haulted  # reverse the pause flag
        if sim_haulted: continue

        # simulation frequency control
        if sim_freq_control:
            time_now = pygame.time.get_ticks()
            if (time_now - time_last) > frame_period:
                time_last = time_now
            else:
                continue

        # increase iteration count
        iter_count = iter_count + 1
        sys.stdout.write("\riteration {}".format(iter_count))
        sys.stdout.flush()

        # state transition variables
        st_n1to0 = []  # robot '-1' gets back to '0' after life time ends
            # list of robots changing to '0' from '-1'
        st_gton1 = []  # group disassembles either life expires, or triggered by others
            # list of groups to be disassembled
        st_0to1 = {}  # robot '0' detects robot '2', join its group
            # key is the robot '0', value is the group id
        st_0to2 = {}  # robot '0' detects another robot '0', forming a new group
            # key is the robot '0', value is the other neighbor robot '0'
        st_1to2 = {}  # robot '1' is climbing around the loop, and finds its right position
            # key is the left side of the slot, value is a list of robots intend to join

        dist_conn_update()  # update "relations" of the robots
        # check state transitions, and schedule the tasks
        for i in range(swarm_size):
            if robot_states[i] == -1:  # for host robot with state '-1'
                if robot_n1_lives[i] < 0:
                    st_n1to0.append(i)
                else:
                    if len(conn_lists[i]) == 0: continue
                    conn_temp = conn_lists[i][:]
                    for j in conn_lists[i]:
                        if (robot_states[j] == -1) or (robot_states[j] == 0):
                            conn_temp.remove(j)
                    if len(conn_temp) != 0:
                        groups_local, group_id_max = S14_robot_grouping(conn_temp,
                            robot_group_ids, groups)
                        # disassemble all groups except the largest one
                        for group_id_temp in groups_local.keys():
                            if (group_id_temp != group_id_max) and (group_id_temp not in st_gton1):
                                st_gton1.append(group_id_temp)  # schedule to disassemble this group
                        # find the closest neighbor in groups_local[group_id_max]
                        robot_closest = S14_closest_robot(i, groups_local[group_id_max])
                        # change moving direction opposing the closest robot
                        vect_temp = robot_poses[i] - robot_poses[robot_closest]
                        robot_oris[i] = math.atan2(vect_temp[1], vect_temp[0])
            elif robot_states[i] == 0:  # for host robot with state '0'
                if len(conn_lists[i]) == 0: continue
                state2_list = []
                state1_list = []
                state0_list = []
                for j in conn_lists[i]:
                    if robot_states[j] == 2:
                        state2_list.append(j)
                    elif robot_states[j] == 1:
                        state1_list.append(j)
                    elif robot_states[j] == 0:
                        state0_list.append(j)
                state2_quantity = len(state2_list)
                state1_quantity = len(state1_list)
                state0_quantity = len(state0_list)
                # disassemble minority groups if there are multiple groups
                if state2_quantity + state1_quantity > 1:
                    # there is in-group robot in the neighbors
                    groups_local, group_id_max = S14_robot_grouping(state2_list+state1_list,
                        robot_group_ids, groups)
                    # disassmeble all groups except the largest one
                    for group_id_temp in groups_local.keys():
                        if (group_id_temp != group_id_max) and (group_id_temp not in st_gton1):
                            st_gton1.append(group_id_temp)  # schedule to disassemble this group
                # actions to the state '2', '1', and '0' robots
                if state2_quantity != 0:
                    # join the group with state '2' robots
                    if state2_quantity == 1:  # only one state '2' robot
                        # join the group of the state '2' robot
                        st_0to1[i] = robot_group_ids[state2_list[0]]
                        robot_key_neighbors[i] = [state2_list[0]]  # add key neighbor
                    else:  # multiple state '2' robots
                        # it's possible that the state '2' robots are in different groups
                        # find the closest one in the largest group, and join the group
                        groups_local, group_id_max = S14_robot_grouping(state2_list,
                            robot_group_ids, groups)
                        robot_closest = S14_closest_robot(i, groups_local[group_id_max])
                        st_0to1[i] = group_id_max
                        robot_key_neighbors[i] = [robot_closest]  # add key neighbor
                elif state1_quantity != 0:
                    # get repelled away from state '1' robot
                    if state1_quantity == 1:  # only one state '1' robot
                        vect_temp = robot_poses[i] - robot_poses[state1_list[0]]
                        robot_oris[i] = math.atan2(vect_temp[1], vect_temp[0])
                    else:
                        groups_local, group_id_max = S14_robot_grouping(state1_list,
                            robot_group_ids, groups)
                        robot_closest = S14_closest_robot(i, groups_local[group_id_max])
                        vect_temp = robot_poses[i] - robot_poses[robot_closest]
                        robot_oris[i] = math.atan2(vect_temp[1], vect_temp[0])
                elif state0_quantity != 0:
                    # form new group with state '0' robots
                    st_0to2[i] = S14_closest_robot(i, state0_list)
            elif (robot_states[i] == 1) or (robot_states[i] == 2):
                # disassemble the minority groups
                state12_list = []  # list of state '1' and '2' robots in the list
                has_other_group = False
                host_group_id = robot_group_ids[i]
                for j in conn_lists[i]:
                    if (robot_states[j] == 1) or (robot_states[j] == 2):
                        state12_list.append(j)
                        if robot_group_ids[j] != host_group_id:
                            has_other_group = True
                if has_other_group:
                    groups_local, group_id_max = S14_robot_grouping(state12_list,
                        robot_group_ids, groups)
                    for group_id_temp in groups_local.keys():
                        if (group_id_temp != group_id_max) and (group_id_temp not in st_gton1):
                            st_gton1.append(group_id_temp)  # schedule to disassemble this group
                # check if state '1' robot's order on loop is good
                if robot_states[i] == 1:
                    current_key = robot_key_neighbors[i][0]
                    next_key = robot_key_neighbors[current_key][-1]
                    if next_key in conn_lists[i]:  # next key neighbor is detected
                        role_i = assignment_scheme[i]
                        role_current_key = assignment_scheme[current_key]
                        role_next_key = assignment_scheme[next_key]
                        if len(robot_key_neighbors[current_key]) == 1:
                            # the situation that only two members are in the group
                            vect_temp = robot_poses[next_key] - robot_poses[current_key]
                            # deciding which side of vect_temp robot i is at
                            vect_i = robot_poses[i] - robot_poses[current_key]
                            side_value = np.cross(vect_i, vect_temp)
                            if side_value > 0:  # robot i on the right side
                                if role_next_key > role_current_key:
                                    if (role_i < role_current_key) or (role_i > role_next_key):
                                        # update with next key neighbor
                                        robot_key_neighbors[i] = next_key
                                    else:
                                        # its position on loop is achieved, becoming state '2'
                                        if current_key in st_1to2.keys():
                                            st_1to2[current_key].append(i)
                                        else:
                                            st_1to2[current_key] = [i]
                                else:
                                    if (role_i < role_current_key) and (role_i > role_next_key):
                                        # update with next key neighbor
                                        robot_key_neighbors[i] = next_key
                                    else:
                                        # its position on loop is achieved, becoming state '2'
                                        if current_key in st_1to2.keys():
                                            st_1to2[current_key].append(i)
                                        else:
                                            st_1to2[current_key] = [i]
                            else:  # robot i on the left side
                                pass
                        else:
                            # the situation that at least three members are in the group
                            if role_next_key > role_current_key:
                                # the roles of the two robots are on the same side
                                if (role_i < role_current_key) or (role_i > role_next_key):
                                    # update with next key neighbor
                                    robot_key_neighbors[i] = next_key
                                else:
                                    # its position on loop is achieved, becoming state '2'
                                    if current_key in st_1to2.keys():
                                        st_1to2[current_key].append(i)
                                    else:
                                        st_1to2[current_key] = [i]
                            else:
                                # the roles of the two robots are on different side
                                if (role_i < role_current_key) and (role_i > role_next_key):
                                    # update with next key neighbor
                                    robot_key_neighbors[i] = next_key
                                else:
                                    # its position on loop is achieved, becoming state '2'
                                    if current_key in st_1to2.keys():
                                        st_1to2[current_key].append(i)
                                    else:
                                        st_1to2[current_key] = [i]

        # check the life time of the groups; schedule disassembling if expired
        for group_id_temp in groups.keys():
            if groups[group_id_temp][1] < 0:  # life time of a group ends
                if group_id_temp not in st_gton1:
                    st_gton1.append(group_id_temp)

        # process the state transition tasks
        # 1.st_1to2, robot '1' locates its order on loop, becoming '2'
        for left_key in st_1to2.keys():
            right_key = robot_key_neighbors[left_key][1]
            role_left_key = assignment_scheme[left_key]
            role_right_key = assignment_scheme[right_key]
            joiner = -1  # the accepted joiner in this position
            if len(st_1to2[left_key]) == 1:
                joiner = st_1to2[left_key][0]
            else:
                joiner_list = st_1to2[left_key]
                role_dist_min = swarm_size
                for joiner_temp in joiner_list:
                    # find the joiner with closest role to left key neighbor
                    role_joiner_temp = assignment_scheme[joiner_temp]
                    role_dist_temp = role_joiner_temp - role_left_key
                    if role_dist_temp < 0:
                        role_dist_temp = role_dist_temp + swarm_size
                    if role_dist_temp < role_dist_min:
                        joiner = joiner_temp
                        role_dist_min = role_dist_temp
            # put in the new joiner
            # update the robot properties
            group_id_temp = robot_group_ids[left_key]
            robot_states[joiner] = 2
            robot_group_ids[joiner] = group_id_temp
            robot_key_neighbors[joiner] = [left_key, right_key]
            if len(groups[group_id_temp][0]) == 2:
                robot_key_neighbors[left_key] = [right_key, joiner]
                robot_key_neighbors[right_key] = [joiner, left_key]
            else:
                robot_key_neighbors[left_key][1] = joiner
                robot_key_neighbors[right_key][0] = joiner
            # update the group properties
            groups[group_id_temp][0].append(joiner)
            groups[group_id_temp][1] = groups[group_id_temp][1] + life_incre
        # 2.st_0to1, robot '0' joins a group, becoming '1'
        for joiner in st_0to1.keys():
            group_id_temp = st_0to1[joiner]
            # update the robot properties
            robot_states[joiner] = 1
            robot_group_ids[joiner] = group_id_temp
            # update the group properties
            groups[group_id_temp][0].append(joiner)
            groups[group_id_temp][1] = groups[group_id_temp][1] + life_incre
        # 3.st_0to2, robot '0' forms new group with '0', both becoming '2'
        while len(st_0to2.keys()) != 0:
            pair0 = st_0to2.keys()[0]
            pair1 = st_0to2[pair0]
            st_0to2.pop(pair0)
            if (pair1 in st_0to2.keys()) and (st_0to2[pair1] == pair0):
                st_0to2.pop(pair1)
                # only forming a group if there is at least one seed robot in the pair
                if robot_seeds[pair0] or robot_seeds[pair1]:
                    # forming new group for robot pair0 and pair1
                    group_id_temp = np.random.randint(0, group_id_upper)
                    while group_id_temp in groups.keys():
                        group_id_temp = np.random.randint(0, group_id_upper)
                    # update properties of the robots
                    robot_states[pair0] = 2
                    robot_states[pair1] = 2
                    robot_group_ids[pair0] = group_id_temp
                    robot_group_ids[pair1] = group_id_temp
                    robot_key_neighbors[pair0] = [pair1]
                    robot_key_neighbors[pair1] = [pair0]
                    # update properties of the group
                    groups[group_id_temp] = [[pair0, pair1], life_incre*2, False]
        # 4.st_gton1, groups get disassembled, life time ends or triggered by others
        for group_id_temp in st_gton1:
            for robot_temp in groups[group_id_temp][0]:
                robot_states[robot_temp] = -1
                robot_n1_lives[robot_temp] = np.random.uniform(n1_life_lower, n1_life_upper)
                robot_group_ids[robot_temp] = -1
                robot_oris[robot_temp] = np.random.rand() * 2 * math.pi - math.pi
                robot_key_neighbors[robot_temp] = []
            groups.pop(group_id_temp)
        # 5.st_n1to0, life time of robot '-1' ends, get back to '0'
        for robot_temp in st_n1to0:
            robot_states[robot_temp] = 0

        # check if a group becomes dominant
        for group_id_temp in groups.keys():
            if len(groups[group_id_temp][0]) > swarm_size/2.0:
                groups[group_id_temp][2] = True
            else:
                groups[group_id_temp][2] = False

        # update the physics
        for i in range(swarm_size):
            # adjusting moving direction for state '1' and '2' robots
            if robot_states[i] == 1:
                # rotating around its only key neighbor
                center = robot_key_neighbors[i][0]  # the center robot
                # use the triangle of (desired_space, dist_table[i,center], step_moving_dist)
                if dist_table[i,center] > (desired_space + step_moving_dist):
                    # moving toward the center robot
                    robot_oris[i] = math.atan2(robot_poses[center,1] - robot_poses[i,1],
                        robot_poses[center,0] - robot_poses[i,0])
                elif (dist_table[i,center] + step_moving_dist) < desired_space:
                    # moving away from the center robot
                    robot_oris[i] = math.atan2(robot_poses[i,1] - robot_poses[center,1],
                        robot_poses[i,0] - robot_poses[center,0])
                else:
                    # moving tangent along the circle of radius of "desired_space"
                    robot_oris[i] = math.atan2(robot_poses[i,1] - robot_poses[center,1],
                        robot_poses[i,0] - robot_poses[center,0])
                    # interior angle between 
                    int_angle_temp = math.acos((math.pow(dist_table[i,center],2) +
                        math.pow(step_moving_dist,2) - math.pow(desired_space,2)) /
                        (2.0*dist_table[i,center]*step_moving_dist))
                    robot_oris[i] = reset_radian(robot_oris[i] + (math.pi - int_angle_temp))
            elif robot_states[i] == 2:
                # adjusting position to maintain the loop
                if len(robot_key_neighbors[i]) == 1:
                    # situation that only two robots are in the group
                    j = robot_key_neighbors[i][0]
                    if abs(dist_table[i,j] - desired_space) < destination_error:
                        continue  # stay in position if within destination error
                    else:
                        if dist_table[i,j] > desired_space:
                            robot_oris[i] = math.atan2(robot_poses[j,1] - robot_poses[i,1],
                                robot_poses[j,0] - robot_poses[i,0])
                        else:
                            robot_oris[i] = math.atan2(robot_poses[i,1] - robot_poses[j,1],
                                robot_poses[i,0] - robot_poses[j,0])
                else:
                    # normal situation with at least three members in the group
                    group_id_temp = robot_group_ids[i]
                    state2_quantity = 0  # number of state '2' robots
                    for robot_temp in group[group_id_temp][0]:
                        if robot_states[robot_temp] == 2:
                            state2_quantity = state1_quantity + 1
                    desired_angle = math.pi - 2*math.pi / state2_quantity
                    # use the SMA algorithm to achieve the desired interior angle
                    left_key = robot_key_neighbors[i][0]
                    right_key = robot_key_neighbors[i][1]
                    vect_l = (robot_poses[left_key] - robot_poses[i]) / dist_table[i,left_key]
                    vect_r = (robot_poses[right_key] - robot_poses[i]) / dist_table[i,right_key]
                    vect_lr = robot_poses[right_key] - robot_poses[left_key]
                    dist_temp = math.sqrt(vect_lr[0]*vect_lr[0] + vect_lr[1]*vect_lr[1])
                    vect_in = np.array([-vect_lr[1], vect_lr[0]]) / dist_temp
                    inter_curr = math.acos(np.dot(vect_l, vect_r) /
                        (dist_table[i,left_key]*dist_table[i,right_key]))  # interior angle
                    if np.cross(vect_l, vect_r) > 0:
                        inter_curr = 2*math.pi - inter_curr
                    fb_vect = np.zeros(2)  # feedback vector to accumulate spring effects
                    fb_vect = fb_vect + ((dist_table[i,left_key] - desired_space) *
                        linear_const * vect_l)
                    fb_vect = fb_vect + ((dist_table[i,right_key] - desired_space) *
                        linear_const * vect_r)
                    fb_vect = fb_vect + ((desired_angle - inter_curr) *
                        bend_const * vect_in)
                    if np.linalg.norm(fb_vect)*disp_coef < destination_error:
                        continue  # stay in position if within destination error
                    else:
                        robot_oris[i] = math.atan2(fb_vect[1], fb_vect[0])
                # check if out of boundaries
                robot_oris[i] = robot_boundary_check(robot_poses[i], robot_oris[i])
                # update one step of move
                robot_poses[i] = robot_poses[i] + (step_moving_dist *
                    np.array([math.cos(robot_oris[i]), math.sin(robot_oris[i])]))

        # update the graphics
        disp_poses_update()
        screen.fill(color_white)
        # draw the robots of states '-1' and '0'
        for i in range(swarm_size):
            if robot_states[i] == -1:  # empty circle for state '-1' robot
                pygame.draw.circle(screen, color_grey, disp_poses[i],
                    robot_size_formation, robot_width_empty)
            elif robot_states[i] == 0:  # full circle for state '0' robot
                pygame.draw.circle(screen, color_grey, disp_poses[i],
                    robot_size_formation, 0)
        # draw the in-group robots by each group
        for group_id_temp in groups.keys():
            if groups[group_id_temp][2]:
                # highlight the dominant group with black color
                color_group = color_black
            else:
                color_group = color_grey
            conn_draw_sets = []  # avoid draw same connection two times
            # draw the robots and connections in the group
            for i in groups[group_id_temp][0]:
                pygame.draw.circle(screen, color_group, disp_poses[i],
                    robot_size_formation, 0)
                for j in robot_key_neighbors[i]:
                    if set([i,j]) not in conn_draw_sets:
                        pygame.draw.line(screen, color_group, disp_poses[i],
                            disp_poses[j], conn_width_formation)
                        conn_draw_sets.append(set([i,j]))
        pygame.display.update()

        # reduce life time of robot '-1' and groups
        for i in range(swarm_size):
            if robot_states[i] == -1:
                robot_n1_lives[i] = robot_n1_lives[i] - frame_period/1000.0
        for group_id_temp in groups.keys():
            if not groups[group_id_temp][2]:  # skip dominant group
                groups[group_id_temp][1] = groups[group_id_temp][1] - frame_period/1000.0

        # check exit condition of simulation 4
        if not loop_formed:
            if (len(groups.keys()) == 1) and (len(groups.values()[0][0]) == swarm_size):
                loop_formed = True
        if loop_formed:
            if ending_period <= 0:
                print("")  # move cursor to the new line
                print("simulation 4 is finished")
                raw_input("<Press Enter to continue>")
                break
            else:
                ending_period = ending_period - frame_period/1000.0

    ########### simulation 5: loop reshaping to chosen shape ###########

    print("##### simulation 5: loop reshaping #####")




