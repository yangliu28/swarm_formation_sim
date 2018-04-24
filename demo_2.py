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
    # Simulation 2: consensus decision making for target loop shape
    # Simulation 3: role assignment and loop reshape


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
comm_range = 0.65  # communication range in the world
desired_space_ratio = 0.8  # ratio of the desired space to the communication range
    # should be larger than 1/1.414=0.71, to avoid connections crossing each other
desired_space = comm_range * desired_space_ratio
# deviate robot heading, so as to avoid robot travlling perpendicular to the walls
perp_thres = math.pi/18  # threshold, range from the perpendicular line
devia_angle = math.pi/9  # deviate these much angle from perpendicualr line
# consensus configuration
shape_quantity = 30  # the number of decisions
shape_decision = -1  # the index of chosen decision, in range(shape_quantity)
    # also the index in shape_catalog
loop_folder = "loop-data2"  # folder to store the loop shapes
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
pygame.display.set_caption("Demo 2")
# draw the network
screen.fill(color_white)
for i in range(swarm_size):
    pygame.draw.circle(screen, color_black, disp_poses[i], robot_size,
        robot_empty_width)
pygame.display.update()

# pause to check the network before the simulations, or for screen recording
raw_input("<Press Enter to continue>")

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

########### simulation 1: aggregate together to form a random loop ###########

print("##### simulation 1: loop formation #####")

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
robot_key_neighbors = [[] for i in range(swarm_size)]  # key neighbors for robot on loop
    # for state '1' robot: one key neighbor
    # for state '2' robot: two key neighbor on its left and right sides
        # exception is the group has only two members, one key neighbor for each

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
loop_formed = False
ending_period = 1.0  # grace period
print("swarm robots are forming a random loop ...")
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
                state2_list = []
                state1_list = []
                for j in conn_lists[i]:
                    if robot_states[j] == 2:
                        state2_list.append(j)
                    elif robot_states[j] == 1:
                        state1_list.append(j)
                # either disassemble minority groups, or get repelled by robot '2'
                if len(state2_list) + len(state1_list) != 0:
                    groups_local, group_id_max = S1_robot_grouping(
                        state2_list + state1_list, robot_group_ids, groups)
                    if len(groups_local.keys()) > 1:
                        # disassemble all groups except the largest one
                        for group_id_temp in groups_local.keys():
                            if ((group_id_temp != group_id_max) and
                                (group_id_temp not in st_gton1)):
                                # schedule to disassemble this group
                                st_gton1.append(group_id_temp)
                    else:
                        # state '-1' robot can only be repelled away by state '2' robot
                        if len(state2_list) != 0:
                            # find the closest neighbor in groups_local[group_id_max]
                            robot_closest = S1_closest_robot(i, state2_list)
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
            # check if the other key neighbor of state '1' robot appears
            if robot_states[i] == 1:
                key = robot_key_neighbors[i][0]
                slot_left = -1  # to indicate if a slot is available
                if len(robot_key_neighbors[key]) == 1:
                    # this is a group of only two members
                    key_other = robot_key_neighbors[key][0]
                    if key_other in conn_lists[i]:
                        join_slot = [key, key_other]  # the slot robot i is going to join
                        # find the left side of the join slot
                        slot_vect = robot_poses[join_slot[1]] - robot_poses[join_slot[0]]
                        join_vect = robot_poses[i] - robot_poses[join_slot[0]]
                        if np.dot(join_vect, slot_vect) > 0:
                            slot_left = join_slot[0]
                        else:
                            slot_left = join_slot[1]
                else:  # a normal group with at least three members
                    key_left = robot_key_neighbors[key][0]
                    key_right = robot_key_neighbors[key][1]
                    key_left_avail = key_left in conn_lists[i]
                    key_right_avail = key_right in conn_lists[i]
                    if key_left_avail or key_right_avail:
                        if (not key_left_avail) and key_right_avail:
                            slot_left = key
                        elif key_left_avail and (not key_right_avail):
                            slot_left = key_left
                        else:  # both sides are available
                            robot_closest = S1_closest_robot(i, [key_left, key_right])
                            if robot_closest == key_left:
                                slot_left = key_left
                            else:
                                slot_left = key
                # schedule the task to join the slot
                if slot_left != -1:
                    # becoming state '2'
                    if slot_left in st_1to2.keys():
                        st_1to2[slot_left].append(i)
                    else:
                        st_1to2[slot_left] = [i]

    # check the life time of the groups; schedule disassembling if expired
    for group_id_temp in groups.keys():
        if groups[group_id_temp][1] < 0:  # life time of a group ends
            if group_id_temp not in st_gton1:
                st_gton1.append(group_id_temp)

    # process the state transitions
    # 1.st_1to2, state '1' becomes state '2'
    for slot_left in st_1to2.keys():
        slot_right = robot_key_neighbors[slot_left][-1]
        joiner = -1  # the accepted joiner in this position
        if len(st_1to2[slot_left]) == 1:
            joiner = st_1to2[slot_left][0]
        else:
            # find the joiner which is farthest from slot_left
            dist_max = 0
            for joiner_temp in st_1to2[slot_left]:
                dist_temp = dist_table[slot_left,joiner_temp]
                if dist_temp > dist_max:
                    dist_max = dist_temp
                    joiner = joiner_temp
        # plug in the new state '2' member
        # update the robot properties
        group_id_temp = robot_group_ids[slot_left]
        robot_states[joiner] = 2
        robot_group_ids[joiner] = group_id_temp
        robot_key_neighbors[joiner] = [slot_left, slot_right]
        if len(robot_key_neighbors[slot_left]) == 1:
            robot_key_neighbors[slot_left] = [slot_right, joiner]
            robot_key_neighbors[slot_right] = [joiner, slot_left]
        else:
            robot_key_neighbors[slot_left][1] = joiner
            robot_key_neighbors[slot_right][0] = joiner
        # no need to update the group properties
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
    robot_poses_temp = np.copy(robot_poses)  # temporary variable for robot positions
    no_state1_robot = True
    for i in range(swarm_size):
        # adjusting moving direction for state '1' and '2' robots
        if robot_states[i] == 1:
            no_state1_robot = False
            # rotating around its key neighbor, get closer to the other key neighbor
            center = robot_key_neighbors[i][0]  # the center robot
            if dist_table[i,center] > (desired_space + step_moving_dist):
                # moving toward the center robot
                vect_temp = robot_poses[center] - robot_poses[i]
                robot_oris[i] = math.atan2(vect_temp[1], vect_temp[0])
            elif (dist_table[i,center] + step_moving_dist) < desired_space:
                # moving away from the center robot
                vect_temp = robot_poses[i] - robot_poses[center]
                robot_oris[i] = math.atan2(vect_temp[1], vect_temp[0])
            else:
                # moving tangent along the circle of radius of "desired_space"
                # find the rotating direction to the closer potential neighbor
                rotate_dir = 0  # 1 for ccw, -1 for cw
                key_next = -1  # rotating toward to key_next
                if len(robot_key_neighbors[center]) == 1:
                    key_next = robot_key_neighbors[center][0]
                else:
                    key_next = S1_closest_robot(i, robot_key_neighbors[center])
                vect_i = robot_poses[i] - robot_poses[center]
                vect_next = robot_poses[key_next] - robot_poses[center]
                if np.cross(vect_i, vect_next) > 0:
                    rotate_dir = 1  # should rotate ccw
                else:
                    rotate_dir = -1  # should rotate cw
                # calculate the new moving direction
                robot_oris[i] = math.atan2(vect_i[1], vect_i[0])
                int_angle_temp = math.acos((math.pow(dist_table[i,center],2) +
                    math.pow(step_moving_dist,2) - math.pow(desired_space,2)) /
                    (2.0*dist_table[i,center]*step_moving_dist))
                robot_oris[i] = reset_radian(robot_oris[i] +
                    rotate_dir*(math.pi - int_angle_temp))
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
                for robot_temp in groups[group_id_temp][0]:
                    if robot_states[robot_temp] == 2:
                        state2_quantity = state2_quantity + 1
                desired_angle = math.pi - 2*math.pi / state2_quantity
                # use the SMA algorithm to achieve the desired interior angle
                left_key = robot_key_neighbors[i][0]
                right_key = robot_key_neighbors[i][1]
                vect_l = (robot_poses[left_key] - robot_poses[i]) / dist_table[i,left_key]
                vect_r = (robot_poses[right_key] - robot_poses[i]) / dist_table[i,right_key]
                vect_lr = robot_poses[right_key] - robot_poses[left_key]
                vect_lr_dist = np.linalg.norm(vect_lr)
                vect_in = np.array([-vect_lr[1], vect_lr[0]]) / vect_lr_dist
                inter_curr = math.acos(np.dot(vect_l, vect_r))  # interior angle
                if np.cross(vect_r, vect_l) < 0:
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
        if (robot_states[i] == -1) or (robot_states[i] == 0):
            # only applies for state '-1' and '0'
            robot_oris[i] = robot_boundary_check(robot_poses[i], robot_oris[i])
        # update one step of move
        robot_poses_temp[i] = robot_poses[i] + (step_moving_dist *
            np.array([math.cos(robot_oris[i]), math.sin(robot_oris[i])]))
    robot_poses = np.copy(robot_poses_temp)

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
    if not loop_formed:
        if ((len(groups.keys()) == 1) and (len(groups.values()[0][0]) == swarm_size)
            and no_state1_robot):
            loop_formed = True
    if loop_formed:
        if ending_period <= 0:
            print("simulation 1 is finished")
            raw_input("<Press Enter to continue>")
            print("")  # empty line
            break
        else:
            ending_period = ending_period - frame_period/1000.0

# # check if the loop is complete; calculate robots' order on loop
# loop_set = set()  # set of robots on the loop
# robot_starter = 0
# robot_curr = 0
# loop_set.add(robot_curr)
# robot_loop_orders = np.zeros(swarm_size)  # robot's order on loop
# order_count = 0
# while (robot_key_neighbors[robot_curr][1] != robot_starter):
#     robot_next = robot_key_neighbors[robot_curr][1]
#     loop_set.add(robot_next)
#     order_count = order_count + 1
#     robot_loop_orders[robot_next] = order_count
#     robot_curr = robot_next
# if (len(loop_set) != swarm_size):
#     print("loop is incomplete after loop formation")
#     sys.exit()

# # store the variable "robot_poses"
# with open('d2_robot_poses', 'w') as f:
#     pickle.dump([robot_poses, robot_key_neighbors, robot_loop_orders], f)
# raw_input("<Press Enter to continue>")
# sys.exit()

# simulation 2 and 3 will run repeatedly since here
while True:

    ########### simulation 2: consensus decision making for target loop shape ###########

    print("##### simulation 2: consensus decision making #####")

    # restore variable "robot_poses", "robot_key_neighbors"
    with open('d2_robot_poses') as f:
        robot_poses, robot_key_neighbors, robot_loop_orders = pickle.load(f)
        robot_loop_orders = robot_loop_orders.astype(int)

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

        # update the dominant decision for all robot
        deci_domi = np.argmax(deci_dist, axis=1)
        # update the groups
        robot_starter = 0
        robot_curr = 0
        groups = [[robot_curr]]  # empty the group container
        # slice the loop at the connection before id '0' robot
        while (robot_key_neighbors[robot_curr][1] != robot_starter):
            robot_next = robot_key_neighbors[robot_curr][1]
            if (deci_domi[robot_curr] == deci_domi[robot_next]):
                groups[-1].append(robot_next)
            else:
                groups.append([robot_next])
            robot_curr = robot_next
        # check if the two groups on the slicing point are in same group
        if (len(groups) > 1 and deci_domi[0] == deci_domi[robot_key_neighbors[0][0]]):
            # combine the last group to the first group
            for i in reversed(groups[-1]):
                groups[0].insert(0,i)
            groups.pop(-1)
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
                group_size_l = robot_group_sizes[i_l]
                group_size_r = robot_group_sizes[i_r]
                # weight on left is group_size_l, on host is 1, on right is group_size_r
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
            if (deci_domi[i] == deci_domi[i_next]):
                pygame.draw.line(screen, distinct_color_set[robot_colors[i]],
                    disp_poses[i], disp_poses[i_next], conn_width)
            else:
                pygame.draw.line(screen, color_black, disp_poses[i], disp_poses[i_next],
                    conn_width)
        # draw the robots
        for i in range(swarm_size):
            pygame.draw.circle(screen, distinct_color_set[robot_colors[i]], disp_poses[i],
                robot_size, 0)
        pygame.display.update()

        # check exit condition for simulations 2
        if converged_all:
            shape_decision = deci_domi[0]
            print("")  # move cursor to the new line
            print("converged to decision {}".format(shape_decision))
            print("simulation 2 is finished")
            raw_input("<Press Enter to continue>")
            print("")  # empty line
            break

    ########### simulation 3: role assignment and loop reshape ###########

    print("##### simulation 3: role assignment & loop reshape #####")

    print("chosen shape decision: {}".format(shape_decision))
    shape_decision = 3
    print("forced shape decision: {} - ".format(shape_decision) +
        shape_catalog[shape_decision])

    # read the loop shape from file
    filename = str(swarm_size) + "-" + shape_catalog[shape_decision]
    filepath = os.path.join(os.getcwd(), loop_folder, filename)
    if os.path.isfile(filepath):
        with open(filepath, 'r') as f:
            target_poses = pickle.load(f)
    else:
        print("fail to locate shape file: {}".format(filepath))
        sys.exit()
    # calculate the interior angles for the robots(instead of target positions)
    inter_target = np.zeros(swarm_size)
    for i in range(swarm_size):  # i on the target loop
        i_l = (i-1)%swarm_size
        i_r = (i+1)%swarm_size
        vect_l = target_poses[i_l] - target_poses[i]
        vect_r = target_poses[i_r] - target_poses[i]
        dist_l = np.linalg.norm(vect_l)
        dist_r = np.linalg.norm(vect_r)
        inter_target[i] = math.acos(np.around(
            np.dot(vect_l, vect_r) / (dist_l * dist_r) ,6))
        if np.cross(vect_r, vect_l) < 0:
            inter_target[i] = 2*math.pi - inter_target[i]

    # draw the network for the first time
    screen.fill(color_white)
    for i in range(swarm_size):
        pygame.draw.circle(screen, color_black, disp_poses[i], robot_size, 0)
        pygame.draw.line(screen, color_black, disp_poses[i],
            disp_poses[robot_key_neighbors[i][1]], conn_width)
    pygame.display.update()

    # initialize the decision making variables
    pref_dist = np.random.rand(swarm_size, swarm_size)
    sum_temp = np.sum(pref_dist, axis=1)
    for i in range(swarm_size):
        pref_dist[i] = pref_dist[i] / sum_temp[i]
    deci_domi = np.argmax(pref_dist, axis=1)
    groups = []  # group robots by local consensus
    robot_group_sizes = [0 for i in range(swarm_size)]
    # color assignment
    color_initialized = False
    deci_colors = [-1 for i in range(swarm_size)]
    color_assigns = [0 for i in range(color_quantity)]
    group_colors = []
    robot_colors = [0 for i in range(swarm_size)]
    # decision making control variables
    dist_diff_thres = 0.3
    dist_diff_ratio = [0.0 for i in range(swarm_size)]
    dist_diff_power = 0.3

    # spring constants in SMA
    linear_const = 1.0
    bend_const = 0.8
    disp_coef = 0.05

    # the loop for simulation 2
    sim_haulted = False
    time_last = pygame.time.get_ticks()
    time_now = time_last
    frame_period = 500
    sim_freq_control = True
    print("loop is reshaping to " + shape_catalog[shape_decision] + " with "
        + str(swarm_size) + " robots ...")
    iter_count = 0
    sys.stdout.write("iteration {}".format(iter_count))
    sys.stdout.flush()
    inter_err_thres = 0.2
    while True:
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
            if (time_now - time_last) > frame_period:
                time_last = time_now
            else:
                continue

        # increase iteration count
        iter_count = iter_count + 1
        sys.stdout.write("\riteration {}".format(iter_count))
        sys.stdout.flush()

        # update the dominant decision for all robot
        deci_domi = np.argmax(pref_dist, axis=1)
        # update the groups
        robot_starter = 0
        robot_curr = 0
        groups = [[robot_curr]]  # empty the group container
        # slice the loop at the connection before id '0' robot
        while (robot_key_neighbors[robot_curr][1] != robot_starter):
            robot_next = robot_key_neighbors[robot_curr][1]
            if (deci_domi[robot_curr]+1)%swarm_size == deci_domi[robot_next]:
                groups[-1].append(robot_next)
            else:
                groups.append([robot_next])
            robot_curr = robot_next
        # check if the two groups on the slicing point are in same group
        if len(groups) > 1 and (deci_domi[0] ==
            (deci_domi[robot_key_neighbors[0][0]]+1)%swarm_size):
            # combine the last group to the first group
            for i in reversed(groups[-1]):
                groups[0].insert(0,i)
            groups.pop(-1)
        # the decisions for the groups
        group_deci = [(deci_domi[group_temp[0]] -
            robot_loop_orders[group_temp[0]]) % swarm_size for group_temp in groups]
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
            for deci_temp in range(swarm_size):
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
        pref_dist_t = np.copy(pref_dist)  # deep copy the 'pref_dist'
        for i in range(swarm_size):
            i_l = robot_key_neighbors[i][0]  # index of neighbor on the left
            i_r = robot_key_neighbors[i][1]  # index of neighbor on the right
            # shift distribution from left neighbor
            dist_l = list(pref_dist_t[i_l, 0:swarm_size-1])
            dist_l.insert(0, pref_dist_t[i_l,-1])
            dist_l = np.array(dist_l)
            # shift distribution from right neighbor
            dist_r = list(pref_dist_t[i_r, 1:swarm_size])
            dist_r.append(pref_dist_t[i_r,0])
            dist_r = np.array(dist_r)
            # deciding if two neighbors have converged ideas with host robot
            converged_l = False
            if ((deci_domi[i_l]+1)%swarm_size == deci_domi[i]): converged_l = True
            converged_r = False
            if ((deci_domi[i_r]-1)%swarm_size == deci_domi[i]): converged_r = True
            # weighted averaging depending on group property
            if converged_l and converged_r:  # all three robots are locally converged
                # step 1: take equal weight average on all three distributions
                pref_dist[i] = dist_l + pref_dist_t[i] + dist_r
                dist_sum = np.sum(pref_dist[i])
                pref_dist[i] = pref_dist[i] / dist_sum
                # step 2: increase the unipolarity by applying the linear multiplier
                dist_diff = [np.linalg.norm(pref_dist_t[i]-dist_l, 1),
                             np.linalg.norm(pref_dist_t[i]-dist_r, 1),
                             np.linalg.norm(dist_l-dist_r, 1),]
                dist_diff_max = max(dist_diff)  # maximum distribution difference
                if dist_diff_max < dist_diff_thres:
                    dist_diff_ratio[i] = dist_diff_max/dist_diff_thres  # for debugging
                    small_end = 1.0/swarm_size * np.power(dist_diff_max/dist_diff_thres,
                        dist_diff_power)
                    large_end = 2.0/swarm_size - small_end
                    # sort the magnitude of processed distribution
                    dist_t = np.copy(pref_dist[i])  # temporary distribution
                    sort_index = range(swarm_size)
                    for j in range(swarm_size-1):  # bubble sort, ascending order
                        for k in range(swarm_size-1-j):
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
                    for j in range(swarm_size):
                        multiplier = (small_end + float(j)/(swarm_size-1) *
                            (large_end-small_end))
                        pref_dist[i,sort_index[j]] = pref_dist[i,sort_index[j]] * multiplier
                    dist_sum = np.sum(pref_dist[i])
                    pref_dist[i] = pref_dist[i] / dist_sum
                else:
                    dist_diff_ratio[i] = 1.0  # for debugging, ratio overflowed
            else:  # at least one side has not converged yet
                if converged_all: converged_all = False
                dist_diff_ratio[i] = -1.0  # indicating linear multiplier was not used
                # take unequal weight in the averaging process based on group property
                group_size_l = robot_group_sizes[i_l]
                group_size_r = robot_group_sizes[i_r]
                # weight on left is group_size_l, on host is 1, on right is group_size_r
                pref_dist[i] = (robot_group_sizes[i_l] * dist_l +
                                pref_dist_t[i] +
                                robot_group_sizes[i_r] * dist_r)
                dist_sum = np.sum(pref_dist[i])
                pref_dist[i] = pref_dist[i] / dist_sum

        # update the physics
        robot_poses_t = np.copy(robot_poses)  # as old poses
        inter_curr = np.zeros(swarm_size)
        for i in range(swarm_size):
            i_l = robot_key_neighbors[i][0]
            i_r = robot_key_neighbors[i][1]
            # vectors
            vect_l = robot_poses_t[i_l] - robot_poses_t[i]
            vect_r = robot_poses_t[i_r] - robot_poses_t[i]
            vect_lr = robot_poses_t[i_r] - robot_poses_t[i_l]
            # distances
            dist_l = np.linalg.norm(vect_l)
            dist_r = np.linalg.norm(vect_r)
            dist_lr = np.linalg.norm(vect_lr)
            # unit vectors
            u_vect_l = vect_l / dist_l
            u_vect_r = vect_r / dist_r
            u_vect_in = np.array([-vect_lr[1], vect_lr[0]]) / dist_lr
            # calculate current interior angle
            inter_curr[i] = math.acos(np.around(
                np.dot(vect_l, vect_r) / (dist_l * dist_r), 6))
            if np.cross(vect_r, vect_l) < 0:
                inter_curr[i] = 2*math.pi - inter_curr[i]
            # feedback vector for the SMA algorithm
            fb_vect = np.zeros(2)
            fb_vect = fb_vect + (dist_l - desired_space) * linear_const * u_vect_l
            fb_vect = fb_vect + (dist_r - desired_space) * linear_const * u_vect_r
            fb_vect = fb_vect + (inter_target[deci_domi[i]] - inter_curr[i]) * bend_const * u_vect_in
            # update one step of position
            robot_poses[i] = robot_poses_t[i] + disp_coef * fb_vect

        # update the graphics
        disp_poses_update()
        screen.fill(color_white)
        # draw the connections first
        for i in range(swarm_size):
            i_next = robot_key_neighbors[i][1]
            if ((deci_domi[i]+1)%swarm_size == deci_domi[i_next]):
                pygame.draw.line(screen, distinct_color_set[robot_colors[i]],
                    disp_poses[i], disp_poses[i_next], conn_width)
            else:
                pygame.draw.line(screen, color_black, disp_poses[i], disp_poses[i_next],
                    conn_width)
        # draw the robots
        for i in range(swarm_size):
            pygame.draw.circle(screen, distinct_color_set[robot_colors[i]], disp_poses[i],
                robot_size, 0)
        pygame.display.update()

        # calculate the maximum error of interior angle
        inter_err_max = 0
        for i in range(swarm_size):
            err_curr = abs(inter_curr[i] - inter_target[deci_domi[i]])
            if err_curr > inter_err_max: inter_err_max = err_curr

        # check exit condition of simulation 3
        if converged_all and inter_err_max < inter_err_thres:
            print("simulation 3 is finished")
            raw_input("<Press Enter to continue>")
            print("")  # empty line
            break


