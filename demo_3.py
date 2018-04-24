# This third demo shows how a robot swarm can autonomously choose an open curve shape and form
# the shape in a distributed way. This simulation shares the same strategy with second demo in
# organizing the robots, but it needs no role assignment on the open curve.

# Description:
# Starting dispersed in random positions, the swarm aggregates together to form a straight line
# with the merging method. From here, the following steps run repeatedly. The robots first make
# collective decision of which open curve shape to form, then adjust the local shapes so the
# curve reshapes to the target shape.

# the simulations that run consecutively
# Simulation 1: aggregation together to form a straight line
    # Simulation 2: consensus decision making for target curve shape
    # Simulation 3: curve reshape


from __future__ import print_function
import pygame
import sys, os, getopt, math
import numpy as np
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

# calculate world size and screen size
power_exponent = 1.95  # between 1.0 and 2.0
    # the larger the parameter, the slower the windows grows with swarm size; vice versa
pixels_per_length = 50  # fixed
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

# formation configuration
comm_range = 0.65
desired_space_ratio = 0.8
desired_space = comm_range * desired_space_ratio
# deviate robot heading, so as to avoid robot travlling perpendicular to the walls
perp_thres = math.pi/18  # threshold, range from the perpendicular line
devia_angle = math.pi/9  # deviate these much angle from perpendicualr line
# consensus configuration
shape_quantity = 30  # the number of decisions
shape_decision = -1  # the index of chosen decision, in range(shape_quantity)
    # also the index in shape_catalog
curve_folder = "curve-data"  # folder to store the curve shapes
shape_catalog = ["circle", "square", "triangle", "star"]

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

# visualization configuration
color_white = (255,255,255)
color_black = (0,0,0)
color_grey = (128,128,128)
distinct_color_set = ((230,25,75), (60,180,75), (255,225,25), (0,130,200), (245,130,48),
    (145,30,180), (70,240,240), (240,50,230), (210,245,60), (250,190,190),
    (0,128,128), (230,190,255), (170,110,40), (128,0,0),
    (170,255,195), (128,128,0), (0,0,128))
color_quantity = 17
robot_size = 5
robot_empty_width = 2
conn_width = 2
robot_ring_size = 8

# set up the simulation window
pygame.init()
font = pygame.font.SysFont("Cabin", 12)
icon = pygame.image.load("icon_geometry_art.jpg")
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Demo 3")
# draw the network
screen.fill(color_white)
for i in range(swarm_size):
    pygame.draw.circle(screen, color_black, disp_poses[i], robot_size,
        robot_empty_width)
pygame.display.update()

# pause to check the network before the simulations, or for screen recording
raw_input("<Press Enter to continue>")

# function for simulation 1, group robots by their group ids, and find the largest group
def S1_robot_grouping(robot_list, robot_group_ids, groups):
    # the input list 'robot_list' should not be empty
    groups_temp = {}  # key is group id, value is list of robots
    for i in robot_list:
        group_id_temp = robot_group_ids[i]
        if group_id_temp not in groups_temp.keys():
            groups_temp[group_id_temp] = [i]
        else:
            groups_temp[group_id_temp].append(i)
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

########### simulation 1: aggregate together to form a straight line ###########

print("##### simulation 1: line formation #####")

# robot perperties
# all robots start with state '-1'
robot_states = np.array([-1 for i in range(swarm_size)])
    # '-1' for wandering around, ignoring all connections
    # '0' for wandering around, available to connection
    # '1' for in a group, transit state, only one key neighbor
    # '2' for in a group, both key neighbors secured
n1_life_lower = 2  # inclusive
n1_life_upper = 6  # exclusive
robot_n1_lives = np.random.uniform(n1_life_lower, n1_life_upper, swarm_size)
robot_oris = np.random.rand(swarm_size) * 2 * math.pi - math.pi  # in range of [-pi, pi)
robot_key_neighbors = [[] for i in range(swarm_size)]  # key neighbors for robot on the line
    # for state '1' robot: one key neighbor
    # for state '2' robot: two key neighbor on its left and right sides
        # except for robots on the two ends

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

# movement configuration
step_moving_dist = 0.05  # should be smaller than destination distance error
destination_error = 0.08
mov_vec_ratio = 0.5  # ratio used when calculating mov vector
# spring constants in SMA
linear_const = 1.0
bend_const = 0.8
disp_coef = 0.5

# the loop for simulation 1
sim_haulted = False
time_last = pygame.time.get_ticks()
time_now = time_last
frame_period = 50
sim_freq_control = True
iter_count = 0
line_formed = False
ending_period = 1.0  # grace period
print("swarm robots are forming a straight line ...")
while False:
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

    # state transition variables
    st_n1to0 = []  # robot '-1' gets back to '0' after life time ends
        # list of robots changing to '0' from '-1'
    st_gton1 = []  # group disassembles either because life expires, or triggered by others
        # list of groups to be disassembled
    st_0to1 = {}  # robot '0' detects robot'2', join its group
        # key is the robot '0', value is the group id
    st_0to2 = {}  # robot '0' detects another robot '0', forming a new group
        # key is the robot '0', value is the other neighbor robot '0'
    st_1to2 = {}  # robot '1' finds another key neighbor, becoming '2'
        # key is the left side of the slot, value is a list of robots intend to join
        # the left side of the slot may or may not be the current key neighbor of robot '1'

    # check state transitions, and schedule the tasks
    dist_conn_update()
    for i in range(swarm_size):
        if robot_states[i] == -1:  # for host robot with state '-1'
            if robot_n1_lives[i] < 0:
                st_n1to0.append(i)
            else:
                if len(conn_lists[i]) == 0: continue
                state12_list = []
                for j in conn_lists[i]:
                    if robot_states[j] == 1 or robot_states[j] == 2:
                        state12_list.append(j)
                # disassemble minority groups
                if len(state12_list) != 0:
                    groups_local, group_id_max = S1_robot_grouping(
                        state12_list, robot_group_ids, groups)
                    if len(groups_local.keys()) > 1:
                        # disassemble all groups except the largest one
                        for group_id_temp in groups_local.keys():
                            if ((group_id_temp != group_id_max) and
                                (group_id_temp not in st_gton1)):
                                # schedule to disassemble this group
                                st_gton1.append(group_id_temp)
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
                groups_local, group_id_max = S1_robot_grouping(state2_list+state1_list,
                    robot_group_ids, groups)
                # disassmeble all groups except the largest one
                for group_id_temp in groups_local.keys():
                    if (group_id_temp != group_id_max) and (group_id_temp not in st_gton1):
                        st_gton1.append(group_id_temp)  # schedule to disassemble this group
            # responses to the state '2' and '0' robots
            if state2_quantity != 0:
                # join the group with state '2' robots
                if state2_quantity == 1:  # only one state '2' robot
                    # join the group of the state '2' robot
                    st_0to1[i] = robot_group_ids[state2_list[0]]
                    robot_key_neighbors[i] = [state2_list[0]]  # add key neighbor
                else:  # multiple state '2' robots
                    # it's possible that the state '2' robots are in different groups
                    # find the closest one in the largest group, and join the group
                    groups_local, group_id_max = S1_robot_grouping(state2_list,
                        robot_group_ids, groups)
                    robot_closest = S1_closest_robot(i, groups_local[group_id_max])
                    st_0to1[i] = group_id_max
                    robot_key_neighbors[i] = [robot_closest]  # add key neighbor
            elif state0_quantity != 0:
                # form new group with state '0' robots
                st_0to2[i] = S1_closest_robot(i, state0_list)
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
                groups_local, group_id_max = S1_robot_grouping(state12_list,
                    robot_group_ids, groups)
                for group_id_temp in groups_local.keys():
                    if (group_id_temp != group_id_max) and (group_id_temp not in st_gton1):
                        st_gton1.append(group_id_temp)  # schedule to disassemble this group
            # check if state '1' robot is qualified for becoming state '2'
            if robot_states[i] == 1:
                key = robot_key_neighbors[i][0]
                if len(robot_key_neighbors[key]) == 1:



