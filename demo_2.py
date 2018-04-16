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
disp_coef = 0.5

# the loop for simulation 4
sim_haulted = False
time_last = pygame.time.get_ticks()
time_now = time_last
frame_period = 50
sim_freq_control = True
iter_count = 0
# sys.stdout.write("iteration {}".format(iter_count))  # did nothing in iteration 0
print("swarm robots are forming an ordered loop ...")
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
    # sys.stdout.write("\riteration {}".format(iter_count))
    # sys.stdout.flush()
