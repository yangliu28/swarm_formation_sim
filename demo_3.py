# This third demo shows how a robot swarm can autonomously choose an open curve shape and form
# the shape in a distributed way. This simulation shares the same strategy with second demo in
# organizing the robots, but it needs no role assignment on the open curve.

# input arguments:
# '-n': number of robots
# '--manual': manual mode, press ENTER to proceed between simulations

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
manual_mode = False  # manually press enter key to proceed between simulations

# read command line options
try:
    opts, args = getopt.getopt(sys.argv[1:], 'n:', , ['manual'])
except getopt.GetoptError as err:
    print(str(err))
    sys.exit()
for opt,arg in opts:
    if opt == '-n':
        swarm_size = int(arg)
    elif opt == '--manual':
        manual_mode = True

# calculate world size and screen size
power_exponent = 1.3  # between 1.0 and 2.0
    # the larger the parameter, the slower the windows grows with swarm size; vice versa
pixels_per_length = 50  # fixed
# calculate world_side_coef from a desired screen size for 30 robots
def cal_world_side_coef():
    desired_screen_size = 600  # desired screen size for 30 robots
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
shape_decision = -1  # the index of chosen decision, in range(shape_quantity)
    # also the index in shape_catalog
curve_folder = "curve-data"  # folder to store the curve shapes
shape_catalog = ["ARM", "CWRU", "DIRL", "KID", "MAD", "squarehelix"]
shape_quantity = len(shape_catalog)

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
# sizes for consensus simulation
robot_size_consensus = 7
conn_width_thin_consensus = 2
conn_width_thick_consensus = 4

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

# function for simulation 1, find the closest robot to a host robot
# use global variable "dist_table"
def S1_closest_robot(robot_host, robot_neighbors):
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

# general function to reset radian angle to [-pi, pi)
def reset_radian(radian):
    while radian >= math.pi:
        radian = radian - 2*math.pi
    while radian < -math.pi:
        radian = radian + 2*math.pi
    return radian

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
        # robots on the two ends will have one key neighbor being '-1'

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
step_moving_dist = 0.05
destination_error = 0.08
# for adjusting line space
space_good_thres = desired_space * 0.9

# the loop for simulation 1
sim_haulted = False
time_last = pygame.time.get_ticks()
time_now = time_last
frame_period = 50
sim_freq_control = True
iter_count = 0
line_formed = False
ending_period = 5.0  # grace period
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
        # key is the robot '1', value is its key neighbor and merging side
        # sides: 0 for left side, 1 for right side

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
                if (robot_key_neighbors[key][0] == -1 or robot_key_neighbors[key][1] == -1):
                    # the key neighbor is on one end of the line
                    if (robot_key_neighbors[key][0] == -1 and
                        robot_key_neighbors[key][1] != -1):  # key is at left end
                        key_next = robot_key_neighbors[key][1]
                        vect_next = robot_poses[key_next] - robot_poses[key]
                        vect_i = robot_poses[i] - robot_poses[key]
                        # whether to merge depends on the side
                        if np.dot(vect_next,vect_i) > 0:
                            if key_next in conn_lists[i]:
                               st_1to2[i] = [key, 1]
                        else:
                            st_1to2[i] = [key, 0]
                    elif (robot_key_neighbors[key][0] != -1 and
                        robot_key_neighbors[key][1] == -1):  # key is at right end
                        key_next = robot_key_neighbors[key][0]
                        vect_next = robot_poses[key_next] - robot_poses[key]
                        vect_i = robot_poses[i] - robot_poses[key]
                        if np.dot(vect_next,vect_i) > 0:
                            if key_next in conn_lists[i]:
                               st_1to2[i] = [key, 0]
                        else:
                            st_1to2[i] = [key, 1]
                    else:
                        print("key neighbor error(st check)")
                        sys.exit()
                else:  # the key neighbor is in the middle of the line
                    key_left = robot_key_neighbors[key][0]
                    key_right = robot_key_neighbors[key][1]
                    if (key_left in conn_lists[i] and key_right in conn_lists[i]):
                        side = -1
                        if dist_table[i,key_left] < dist_table[i,key_right]: side = 0
                        else: side = 1
                        st_1to2[i] = [key, side]
                    elif (key_left in conn_lists[i] and key_right not in conn_lists[i]):
                        st_1to2[i] = [key, 0]
                    elif (key_left not in conn_lists[i] and key_right in conn_lists[i]):
                        st_1to2[i] = [key, 1]

    # check the life time of the groups; schedule disassembling if expired
    for group_id_temp in groups.keys():
        if groups[group_id_temp][1] < 0:  # life time of a group ends
            if group_id_temp not in st_gton1:
                st_gton1.append(group_id_temp)

    # process the state transitions
    # 1.st_1to2, state '1' becomes state '2'
    while len(st_1to2.keys()) != 0:
        joiner = st_1to2.keys()[0]
        key = st_1to2[joiner][0]
        side = st_1to2[joiner][1]
        side_other = 1 - side
        st_1to2.pop(joiner)
        if robot_key_neighbors[key][side] == -1:  # join at one end
            # check if other robots are join the same slot
            key_rest = st_1to2.keys()[:]
            for joiner_temp in key_rest:
                if (st_1to2[joiner_temp][0] == key and st_1to2[joiner_temp][1] == side):
                    # "joiner_temp" is joining same slot as "joiner"
                    st_1to2.pop(joiner_temp)
                    if dist_table[key,joiner] > dist_table[key,joiner_temp]:
                        joiner = joiner_temp
            # merge the robot at the end
            robot_states[joiner] = 2
            if side == 0: robot_key_neighbors[joiner] = [-1,key]
            else: robot_key_neighbors[joiner] = [key,-1]
            robot_key_neighbors[key][side] = joiner
        else:  # join in between
            key_other = robot_key_neighbors[key][side]
            side_other = 1 - side
            des_pos = (robot_poses[key] + robot_poses[key_other]) / 2.0
            # check if other robots are join the same slot
            key_rest = st_1to2.keys()[:]
            for joiner_temp in key_rest:
                if ((st_1to2[joiner_temp][0] == key and st_1to2[joiner_temp][1] == side) or
                    (st_1to2[joiner_temp][0] == key_next and st_1to2[joiner_temp][1] == side_other)):
                    # "joiner_temp" is joining same slot as "joiner"
                    st_1to2.pop(joiner_temp)
                    if dist_table[key,joiner] > dist_table[key,joiner_temp]:
                        joiner = joiner_temp
            # merge the robot in between
            robot_states[joiner] = 2
            if side == 0:
                robot_key_neighbors[joiner] = [key_other,key]
            else:
                robot_key_neighbors[joiner] = [key,key_other]
            robot_key_neighbors[key][side] = joiner
            robot_key_neighbors[key_other][side_other] = joiner
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
                robot_key_neighbors[pair0] = [-1,pair1]  # pair0 automatically becomes left end
                robot_key_neighbors[pair1] = [pair0,-1]  # pair1 becomes right end
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
    robot_poses_t = np.copy(robot_poses)  # as old poses
    no_state1_robot = True
    # space on the line is good(not too crowded)
    line_space_good = np.array([-1 for i in range(swarm_size)])
    for i in range(swarm_size):
        if robot_states[i] == 2:
            if robot_key_neighbors[i][0] == -1:
                key_next = robot_key_neighbors[i][1]
                if (dist_table[i,key_next] > space_good_thres):
                    line_space_good[i] = 1
                else:
                    line_space_good[i] = 0
            elif robot_key_neighbors[i][1] == -1:
                key_next = robot_key_neighbors[i][0]
                if (dist_table[i,key_next] > space_good_thres):
                    line_space_good[i] = 1
                else:
                    line_space_good[i] = 0
            else:
                key_left = robot_key_neighbors[i][0]
                key_right = robot_key_neighbors[i][1]
                if (dist_table[i,key_left] > space_good_thres and
                    dist_table[i,key_right] > space_good_thres):
                    line_space_good[i] = 1
                else:
                    line_space_good[i] = 0
    for i in range(swarm_size):
        # adjusting moving direction for state '1' and '2' robots
        if robot_states[i] == 1:
            no_state1_robot = False
            # rotating around its key neighbor, get closer to the other key neighbor
            center = robot_key_neighbors[i][0]  # the center robot
            if dist_table[i,center] > (desired_space + step_moving_dist):
                # moving toward the center robot
                vect_temp = robot_poses_t[center] - robot_poses_t[i]
                robot_oris[i] = math.atan2(vect_temp[1], vect_temp[0])
            elif (dist_table[i,center] + step_moving_dist) < desired_space:
                # moving away from the center robot
                vect_temp = robot_poses_t[i] - robot_poses_t[center]
                robot_oris[i] = math.atan2(vect_temp[1], vect_temp[0])
            else:
                # moving tangent along the circle of radius of "desired_space"
                # find the rotating direction to the closer potential neighbor
                rotate_dir = 0  # 1 for ccw, -1 for cw
                if (robot_key_neighbors[center][0] == -1 or
                    robot_key_neighbors[center][1] == -1):
                    key_next = -1  # rotating toward to key_next
                    if (robot_key_neighbors[center][0] == -1 and
                        robot_key_neighbors[center][1] != -1):
                        key_next = robot_key_neighbors[center][1]
                    elif (robot_key_neighbors[center][0] != -1 and
                        robot_key_neighbors[center][1] == -1):
                        key_next = robot_key_neighbors[center][0]
                    else:
                        print("key neighbor error(physics update1)")
                        sys.exit()
                    vect_next = robot_poses[key_next] - robot_poses[center]
                    vect_i = robot_poses[i] - robot_poses[center]
                    if np.dot(vect_next, vect_i) > 0:
                        if np.cross(vect_i, vect_next) > 0: rotate_dir = 1
                        else: rotate_dir = -1
                    else: continue  # stay in position if out at one end
                else:
                    key_left = robot_key_neighbors[center][0]
                    key_right = robot_key_neighbors[center][1]
                    key_next = -1
                    if dist_table[i,key_left] < dist_table[i,key_right]: key_next = key_left
                    else: key_next = key_right
                    vect_next = robot_poses[key_next] - robot_poses[center]
                    vect_i = robot_poses[i] - robot_poses[center]
                    if np.cross(vect_i, vect_next) > 0: rotate_dir = 1
                    else: rotate_dir = -1
                # calculate the new moving direction
                vect_i = robot_poses[i] - robot_poses[center]
                robot_oris[i] = math.atan2(vect_i[1], vect_i[0])
                int_angle_temp = math.acos((math.pow(dist_table[i,center],2) +
                    math.pow(step_moving_dist,2) - math.pow(desired_space,2)) /
                    (2.0*dist_table[i,center]*step_moving_dist))
                robot_oris[i] = reset_radian(robot_oris[i] +
                    rotate_dir*(math.pi - int_angle_temp))
        elif robot_states[i] == 2:
            # adjusting position to maintain the line
            if (robot_key_neighbors[i][0] == -1 or robot_key_neighbors[i][1] == -1):
                key = -1
                vect_line = np.zeros(2)
                if (robot_key_neighbors[i][0] == -1 and robot_key_neighbors[i][1] != -1):
                    key = robot_key_neighbors[i][1]
                    if robot_key_neighbors[key][1] == -1:
                        vect_line = robot_poses[i] - robot_poses[key]
                        vect_line = vect_line / np.linalg.norm(vect_line)
                    else:
                        key_other = robot_key_neighbors[key][1]
                        vect_line = robot_poses[key] - robot_poses[key_other]
                        vect_line = vect_line / np.linalg.norm(vect_line)
                elif (robot_key_neighbors[i][0] != -1 and robot_key_neighbors[i][1] == -1):
                    key = robot_key_neighbors[i][0]
                    if robot_key_neighbors[key][0] == -1:
                        vect_line = robot_poses[i] - robot_poses[key]
                        vect_line = vect_line / np.linalg.norm(vect_line)
                    else:
                        key_other = robot_key_neighbors[key][0]
                        vect_line = robot_poses[key] - robot_poses[key_other]
                        vect_line = vect_line / np.linalg.norm(vect_line)
                else:
                    print("key neighbor error(physics update2)")
                    sys.exit()
                des_pos = robot_poses[key] + vect_line*desired_space
                vect_des = des_pos - robot_poses[i]
                if np.linalg.norm(vect_des) < destination_error:
                    continue
                else:
                    robot_oris[i] = math.atan2(vect_des[1], vect_des[0])
            else:
                key_left = robot_key_neighbors[i][0]
                key_right = robot_key_neighbors[i][1]
                if (line_space_good[i] == 0 and
                    line_space_good[key_left] != line_space_good[key_right]):
                    if line_space_good[key_left] == 1:
                        vect_left = robot_poses[key_left] - robot_poses[i]
                        robot_oris[i] = math.atan2(vect_left[1], vect_left[0])
                    else:
                        vect_right = robot_poses[key_right] - robot_poses[i]
                        robot_oris[i] = math.atan2(vect_right[1], vect_right[0])
                else:
                    des_pos = (robot_poses[key_left] + robot_poses[key_right])/2.0
                    des_vect = des_pos - robot_poses[i]
                    if np.linalg.norm(des_vect) < destination_error:
                        continue  # stay in position if within destination error
                    else:
                        robot_oris[i] = math.atan2(des_vect[1], des_vect[0])
        # check if out of boundaries
        if (robot_states[i] == -1) or (robot_states[i] == 0):
            # only applies for state '-1' and '0'
            robot_oris[i] = robot_boundary_check(robot_poses_t[i], robot_oris[i])
        # update one step of move
        robot_poses[i] = robot_poses_t[i] + (step_moving_dist *
            np.array([math.cos(robot_oris[i]), math.sin(robot_oris[i])]))

    # update the graphics
    disp_poses_update()
    screen.fill(color_white)
    # draw the robots of states '-1' and '0'
    for i in range(swarm_size):
        if robot_states[i] == -1:  # empty circle for state '-1' robot
            pygame.draw.circle(screen, color_grey, disp_poses[i],
                robot_size, robot_empty_width)
        elif robot_states[i] == 0:  # full circle for state '0' robot
            pygame.draw.circle(screen, color_grey, disp_poses[i],
                robot_size, 0)
    # draw the in-group robots by group
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
                robot_size, 0)
            for j in robot_key_neighbors[i]:
                if j == -1: continue
                if set([i,j]) not in conn_draw_sets:
                    pygame.draw.line(screen, color_group, disp_poses[i],
                        disp_poses[j], conn_width)
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
    if not line_formed:
        if ((len(groups.keys()) == 1) and (len(groups.values()[0][0]) == swarm_size)
            and no_state1_robot):
            line_formed = True
    if line_formed:
        if ending_period <= 0:
            print("simulation 1 is finished")
            if manual_mode: raw_input("<Press Enter to continue>")
            print("")  # empty line
            break
        else:
            ending_period = ending_period - frame_period/1000.0

# # check if the line is complete; list robots' order on the line
# robot_starter = -1
# for i in range(swarm_size):
#     if robot_key_neighbors[i][0] == -1:
#         robot_starter = i
#         break
# line_robots = [robot_starter]  # robots on the line, in order
# robot_curr = robot_starter
# while (robot_key_neighbors[robot_curr][1] != -1):
#     robot_next = robot_key_neighbors[robot_curr][1]
#     line_robots.append(robot_next)
#     robot_curr = robot_next
# if (len(set(line_robots)) != swarm_size):
#     print("line is incomplete after line formation")
#     sys.exit()

# # store the variable "robot_poses", "robot_key_neighbors"
# tmp_filepath = os.path.join('tmp', 'demo3_30_robot_poses')
# # tmp_filepath = os.path.join('tmp', 'demo3_100_robot_poses')
# with open(tmp_filepath, 'w') as f:
#     pickle.dump([robot_poses, robot_key_neighbors, line_robots], f)
# raw_input("<Press Enter to continue>")
# sys.exit()

# simulation 2 and 3 will run repeatedly since here
while True:

    ########### simulation 2: consensus decision making for target curve shape ###########

    print("##### simulation 2: consensus decision making #####")

    # restore variable "robot_poses", "robot_key_neighbors"
    tmp_filepath = os.path.join('tmp', 'demo3_30_robot_poses')
    # tmp_filepath = os.path.join('tmp', 'demo3_100_robot_poses')
    with open(tmp_filepath) as f:
        robot_poses, robot_key_neighbors, line_robots = pickle.load(f)

    # shift the robots to the middle of the window
    x_max, y_max = np.amax(robot_poses, axis=0)
    x_min, y_min = np.amin(robot_poses, axis=0)
    robot_middle = np.array([(x_max+x_min)/2.0, (y_max+y_min)/2.0])
    world_middle = np.array([world_side_length/2.0, world_side_length/2.0])
    for i in range(swarm_size):
        robot_poses[i] = robot_poses[i] - robot_middle + world_middle

    # draw the network for the first time
    disp_poses_update()
    screen.fill(color_white)
    for i in range(swarm_size):
        pygame.draw.circle(screen, color_black, disp_poses[i], robot_size, 0)
        if robot_key_neighbors[i][1] == -1: continue
        pygame.draw.line(screen, color_black, disp_poses[i],
            disp_poses[robot_key_neighbors[i][1]], conn_width)
    pygame.display.update()

    # initialize the decision making variables
    shape_decision = -1
    deci_dist = np.random.rand(swarm_size, shape_quantity)
    sum_temp = np.sum(deci_dist, axis=1)
    for i in range(swarm_size):
        deci_dist[i] = deci_dist[i] / sum_temp[i]
    deci_domi = np.argmax(deci_dist, axis=1)
    groups = []  # group robots by local consensus
    robot_group_sizes = [0 for i in range(swarm_size)]
    # color assignment
    color_initialized = False
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
    frame_period = 500
    sim_freq_control = True
    iter_count = 0
    sys.stdout.write("iteration {}".format(iter_count))
    sys.stdout.flush()
    while True:
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

        # update the dominant decision for all robot
        deci_domi = np.argmax(deci_dist, axis=1)
        # update the groups
        robot_curr = line_robots[0]
        groups = [[robot_curr]]  # empty the group container
        # slice the loop at the connection before id '0' robot
        while (robot_key_neighbors[robot_curr][1] != -1):
            robot_next = robot_key_neighbors[robot_curr][1]
            if (deci_domi[robot_curr] == deci_domi[robot_next]):
                groups[-1].append(robot_next)
            else:
                groups.append([robot_next])
            robot_curr = robot_next
        # the decisions for the groups
        group_deci = [deci_domi[groups[i][0]] for i in range(len(groups))]
        # update group sizes for robots
        for group_temp in groups:
            group_temp_size = len(group_temp)
            for i in group_temp:
                robot_group_sizes[i] = group_temp_size

        # update colors for the decisions
        if not color_initialized:
            color_initialized = True
            color_set = range(color_quantity)
            deci_set = set(group_deci)
            for deci in deci_set:
                if len(color_set) == 0:
                    color_set = range(color_quantity)
                chosen_color = np.random.choice(color_set)
                color_set.remove(chosen_color)
                deci_colors[deci] = chosen_color
                color_assigns[chosen_color] = color_assigns[chosen_color] + 1
        else:
            # remove the color for a decision, if it's no longer the decision of any group
            deci_set = set(group_deci)
            for deci_temp in range(shape_quantity):
                color_temp = deci_colors[deci_temp]  # corresponding color for deci_temp
                if (color_temp != -1 and deci_temp not in deci_set):
                    color_assigns[color_temp] = color_assigns[color_temp] - 1
                    deci_colors[deci_temp] = -1
            # assign color for a new decision
            color_set = []
            for i in range(len(groups)):
                if deci_colors[group_deci[i]] == -1:
                    if len(color_set) == 0:
                        # construct a new color set
                        color_assigns_min = min(color_assigns)
                        for color_temp in range(color_quantity):
                            if color_assigns[color_temp] == color_assigns_min:
                                color_set.append(color_temp)
                    # if here, the color set is good to go
                    chosen_color = np.random.choice(color_set)
                    color_set.remove(chosen_color)
                    deci_colors[group_deci[i]] = chosen_color
                    color_assigns[chosen_color] = color_assigns[chosen_color] + 1
        # update the colors for the groups and robots
        group_colors = []
        for i in range(len(groups)):
            color_temp = deci_colors[group_deci[i]]
            group_colors.append(color_temp)
            for j in groups[i]:
                robot_colors[j] = color_temp

        # decision distribution evolution
        converged_all = True
        deci_dist_t = np.copy(deci_dist)  # deep copy the 'deci_dist'
        for i in range(swarm_size):
            if robot_key_neighbors[i][0] == -1 or robot_key_neighbors[i][1] == -1:
                # for robots on the two ends
                i_next = -1
                if robot_key_neighbors[i][0] == -1:
                    i_next = robot_key_neighbors[i][1]
                elif robot_key_neighbors[i][1] == -1:
                    i_next = robot_key_neighbors[i][0]
                if deci_domi[i] == deci_domi[i_next]:  # locally converged
                    # step 1: take equal weight average
                    deci_dist[i] = deci_dist_t[i] + deci_dist_t[i_next]
                    dist_sum = np.sum(deci_dist[i])
                    deci_dist[i] = deci_dist[i] / dist_sum
                    # step 2: increase the unipolarity by applying the linear multiplier
                    dist_diff = np.linalg.norm(deci_dist_t[i]-deci_dist_t[i_next], 1)
                    if dist_diff < dist_diff_thres:
                        dist_diff_ratio[i] = dist_diff/dist_diff_thres  # for debugging
                        small_end = 1.0/shape_quantity * np.power(dist_diff/dist_diff_thres,
                            dist_diff_power)
                        large_end = 2.0/shape_quantity - small_end
                        # sort the magnitude of processed distribution
                        dist_t = np.copy(deci_dist[i])  # temporary distribution
                        sort_index = range(shape_quantity)
                        for j in range(shape_quantity-1):  # bubble sort, ascending order
                            for k in range(shape_quantity-1-j):
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
                        for j in range(shape_quantity):
                            multiplier = (small_end +
                                float(j)/(shape_quantity-1) * (large_end-small_end))
                            deci_dist[i,sort_index[j]] = deci_dist[i,sort_index[j]] * multiplier
                        dist_sum = np.sum(deci_dist[i])
                        deci_dist[i] = deci_dist[i] / dist_sum
                    else:
                        dist_diff_ratio[i] = 1.0  # for debugging, ratio overflowed
                else:  # not converged on the ends
                    if converged_all: converged_all = False
                    dist_diff_ratio[i] = -1.0  # indicating linear multiplier was not used
                    # take unequal weight in the averaging process based on group property
                    deci_dist[i] = (deci_dist_t[i] +
                                    robot_group_sizes[i_next] * deci_dist_t[i_next])
                    dist_sum = np.sum(deci_dist[i])
                    deci_dist[i] = deci_dist[i] / dist_sum
            else:
                i_l = robot_key_neighbors[i][0]  # index of neighbor on the left
                i_r = robot_key_neighbors[i][1]  # index of neighbor on the right
                # deciding if two neighbors have converged ideas with host robot
                converged_l = False
                if (deci_domi[i_l] == deci_domi[i]): converged_l = True
                converged_r = False
                if (deci_domi[i_r] == deci_domi[i]): converged_r = True
                # weighted averaging depending on group property
                if converged_l and converged_r:  # all three robots are locally converged
                    # step 1: take equal weight average on all three distributions
                    deci_dist[i] = deci_dist_t[i_l] + deci_dist_t[i] + deci_dist_t[i_r]
                    dist_sum = np.sum(deci_dist[i])
                    deci_dist[i] = deci_dist[i] / dist_sum
                    # step 2: increase the unipolarity by applying the linear multiplier
                    dist_diff = [np.linalg.norm(deci_dist_t[i_l]-deci_dist_t[i], 1),
                                 np.linalg.norm(deci_dist_t[i_r]-deci_dist_t[i], 1),
                                 np.linalg.norm(deci_dist_t[i_l]-deci_dist_t[i_r], 1)]
                    dist_diff_max = max(dist_diff)  # maximum distribution difference
                    if dist_diff_max < dist_diff_thres:
                        dist_diff_ratio[i] = dist_diff_max/dist_diff_thres  # for debugging
                        small_end = 1.0/shape_quantity * np.power(dist_diff_max/dist_diff_thres,
                            dist_diff_power)
                        large_end = 2.0/shape_quantity - small_end
                        # sort the magnitude of processed distribution
                        dist_t = np.copy(deci_dist[i])  # temporary distribution
                        sort_index = range(shape_quantity)
                        for j in range(shape_quantity-1):  # bubble sort, ascending order
                            for k in range(shape_quantity-1-j):
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
                        for j in range(shape_quantity):
                            multiplier = (small_end +
                                float(j)/(shape_quantity-1) * (large_end-small_end))
                            deci_dist[i,sort_index[j]] = deci_dist[i,sort_index[j]] * multiplier
                        dist_sum = np.sum(deci_dist[i])
                        deci_dist[i] = deci_dist[i] / dist_sum
                    else:
                        dist_diff_ratio[i] = 1.0  # for debugging, ratio overflowed
                else:  # at least one side has not converged yet
                    if converged_all: converged_all = False
                    dist_diff_ratio[i] = -1.0  # indicating linear multiplier was not used
                    # take unequal weight in the averaging process based on group property
                    deci_dist[i] = (robot_group_sizes[i_l] * deci_dist_t[i_l] +
                                    deci_dist_t[i] +
                                    robot_group_sizes[i_r] * deci_dist_t[i_r])
                    dist_sum = np.sum(deci_dist[i])
                    deci_dist[i] = deci_dist[i] / dist_sum

        # update the graphics
        screen.fill(color_white)
        # draw the connections first
        for i in range(swarm_size):
            i_next = robot_key_neighbors[i][1]
            if i_next == -1: continue
            if (deci_domi[i] == deci_domi[i_next]):
                pygame.draw.line(screen, distinct_color_set[robot_colors[i]],
                    disp_poses[i], disp_poses[i_next], conn_width_thick_consensus)
            else:
                pygame.draw.line(screen, color_black, disp_poses[i], disp_poses[i_next],
                    conn_width_thin_consensus)
        # draw the robots
        for i in range(swarm_size):
            pygame.draw.circle(screen, distinct_color_set[robot_colors[i]], disp_poses[i],
                robot_size_consensus, 0)
        pygame.display.update()

        # check exit condition for simulations 2
        if converged_all:
            shape_decision = deci_domi[0]
            print("")  # move cursor to the new line
            print("converged to decision {}".format(shape_decision))
            print("simulation 2 is finished")
            if manual_mode: raw_input("<Press Enter to continue>")
            print("")  # empty line
            break

    ########### simulation 3: curve reshape ###########

    print("##### simulation 3: curve reshape #####")





