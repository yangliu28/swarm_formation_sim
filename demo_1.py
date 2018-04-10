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
# Simulation 2: consensus 1 - decision making on which loop shape to form
# Simulation 3: consensus 2 - role assignment for the loop shape
# Simulation 4: form the loop with designated target positions

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
import sys, getopt, math
import numpy as np

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

# visualization configuration
color_white = (255,255,255)
color_black = (0,0,0)
color_grey = (128,128,128)
distinct_color_set = ((230,25,75), (60,180,75), (255,225,25), (0,130,200), (245,130,48),
    (145,30,180), (70,240,240), (240,50,230), (210,245,60), (250,190,190),
    (0,128,128), (230,190,255), (170,110,40), (255,250,200), (128,0,0),
    (170,255,195), (128,128,0), (255,215,180), (0,0,128), (128,128,128))
node_size = 5
node_empty_width = 2
connection_width = 2

# set up the simulation window
pygame.init()
icon = pygame.image.load("icon_geometry_art.jpg")
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("Demo 1")
# draw the network
screen.fill(color_white)
for i in range(swarm_size):
    pygame.draw.circle(screen, color_black, disp_poses[i], node_size, node_empty_width)
    # pygame.draw.circle(screen, color_black, disp_poses[i],
    #     int(comm_range*pixels_per_length), 1)
pygame.display.update()

# pause to check the network before the simulations
raw_input("<Press Enter to continue>")

# function for simulation 1, group robot '1's by their group ids, and find the largest group
def S1_robot1_grouping(robot_list, robot_group_ids, groups):
    # "robot_list": the list of robot '1's, should not be empty
    # the input list 'robot_list' should not be empty
    groups_temp = {}  # key is group id, value is list of robots
    for i in robot_list:
        group_id_temp = robot_group_ids[i]
        if group_id_temp == -1:  # to be tested and deleted
            print("group id error")
            sys.exit()
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

# general function to check if robot is out of boundary
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
    # deciding the seed robots
    seed_percentage = 0.1  # the percentage of seed robots in the swarm
    seed_quantity = min(max(int(swarm_size*seed_percentage), 1), swarm_size)
        # no smaller than 1, and no larger than swarm_size
    robot_seeds = [False for i in range(swarm_size)]  # whether a robot is a seed robot
        # only seed robot can initialize the forming a new group
    seed_list_temp = np.arange(swarm_size)
    np.random.shuffle(seed_list_temp)
    for i in seed_list_temp[:seed_quantity]:
        robot_seeds[i] = True

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
    while True:
        # close window button to exit the entire program;
        # space key to pause this simulation
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # close window button is clicked
                print("program exit in simulation 1 with close window button")
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
        st_1ton1 = []  # group disassembles either life expires, or triggered by others
            # list of group to be disassembled
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
                        groups_local, group_id_max = S1_robot1_grouping(conn_temp,
                            robot_group_ids, groups)
                        # disassmeble all groups except the largest one
                        for group_id_temp in groups_local.keys():
                            if (group_id_temp != group_id_max) and (group_id_temp not in st_1ton1):
                                st_1ton1.append(group_id_temp)  # schedule to disassemble this group
                        # find the closest neighbor in groups_local[group_id_max]
                        robot_closest = S1_closest_robot(i, groups_local[group_id_max])
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
                    groups_local, group_id_max = S1_robot1_grouping(state1_list,
                        robot_group_ids, groups)
                    # disassmeble all groups except the largest one
                    for group_id_temp in groups_local.keys():
                        if (group_id_temp != group_id_max) and (group_id_temp not in st_1ton1):
                            st_1ton1.append(group_id_temp)  # schedule to disassemble this group
                    # join the the group with the most members
                    st_0to1_join[i] = group_id_max
                elif len(state0_list) != 0:
                    # there is no robot '1', but has robot '0'
                    # find the closest robot, schedule to start a new group with it
                    st_0to1_new[i] = S1_closest_robot(i, state0_list)
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
                    groups_local, group_id_max = S1_robot1_grouping(conn_temp,
                        robot_group_ids, groups)
                    # disassmeble all groups except the largest one
                    for group_id_temp in groups_local.keys():
                        if (group_id_temp != group_id_max) and (group_id_temp not in st_1ton1):
                            st_1ton1.append(group_id_temp)  # schedule to disassemble this group
            else:  # to be tested and deleted
                print("robot state error")
                sys.exit()

        # check the life time of the groups; if expired, schedule disassemble
        for group_id_temp in groups.keys():
            if groups[group_id_temp][1] < 0:  # life time of a group ends
                if group_id_temp not in st_1ton1:
                    st_1ton1.append(group_id_temp)

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
        # 2.st_1ton1
        for group_id_temp in st_1ton1:
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
            if len(groups[group_id_temp][0]) > swarm_size/2:
                groups[group_id_temp][2] = True
            else:
                groups[group_id_temp][2] = False

        # update the physics
        local_conn_lists = [[] for i in range(swarm_size)]  # list of connections in same group
            # only for state '1' robot
        for i in range(swarm_size):
            # change move direction only for robot '1', for adjusting location in group
            if robot_states[i] == 1:
                host_group_id = robot_group_ids[i]
                for j in conn_lists[i]:
                    if (robot_states[j] == 1) and (robot_group_ids[j] == host_group_id):
                        local_conn_lists[i].append(j)
                if len(local_conn_lists[i]) == 0:  # should not happen after parameter tuning
                    printf("robot {} loses its group {}".format(i, host_group_id))
                    sys.exit()
                mov_vec = np.zeros(2)
                for j in local_conn_lists[i]:
                    mov_vec = mov_vec + (mov_vec_ratio * (dist_table[i,j] - desired_space) *
                        normalize(robot_poses[j] - robot_poses[i]))
                if np.linalg.norm(mov_vec) < destination_error:
                    # skip the physics update if within destination error
                    continue
                else:
                    robot_oris[i] = math.atan2(mov_vec[1], mov_vec[0])  # change direction
            # check if out of boundaries (from previous line formation program)
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
                    node_size, node_empty_width)
            elif robot_states[i] == 0:  # full circle for state '0' robot
                pygame.draw.circle(screen, color_grey, disp_poses[i],
                    node_size, 0)
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
                    node_size, 0)
                for j in local_conn_lists[i]:
                    if set([i,j]) not in conn_draw_sets:
                        pygame.draw.line(screen, color_group, disp_poses[i],
                            disp_poses[j], connection_width)
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
        if len(groups.keys()) == 1:
            if len(groups.values()[0][0]) == swarm_size:
                print("")  # move cursor to the new line
                print("simulation 1 is finished")
                # raw_input("<Press Enter to continue>")
                pygame.time.delay(10000)
                break




