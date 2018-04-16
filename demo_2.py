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
assignment_scheme = np.zeros(swarm_size)
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
while True:
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
                    groups_local, group_id_max = S14_robot_grouping(
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
                            robot_closest = S14_closest_robot(i, state2_list)
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
                    groups_local, group_id_max = S14_robot_grouping(state2_list,
                        robot_group_ids, groups)
                    robot_closest = S14_closest_robot(i, groups_local[group_id_max])
                    st_0to1[i] = group_id_max
                    robot_key_neighbors[i] = [robot_closest]  # add key neighbor
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
        if len(st_1to2[left_key]) == 1:
            joiner = st_1to2[slot_left][0]
        else:
            # find the joiner which is farthest from slot_left
            dist_max = 0
            for joiner_temp in st_1to2[left_key]:
                dist_temp = dist_table[slot_left,joiner_temp]
                if dist_temp > dist_max:
                    dist_max = dist_temp
                    joiner = joiner_temp
        # plug in the new state '2' member
        # update the robot properties
        group_id_temp = robot_group_ids[left_key]
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
            # rotating around its only key neighbor, in a direction to get closer to the
            # closest other key neighbor
            rotate_dir = 0  # 1 for ccw, -1 for cw



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




