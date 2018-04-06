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

# Note that message transmission is simulated only in the role assignment, because communication
# is specialy depended on and message convey is used as well. While in consensus decision making
# and shape formation, the delay caused by communication are skipped.



# add interaction of mouse to show further interaction

# use text update in the terminal to show extra information, iterations

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
import sys, getopt
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
power_exponent = 1.9  # between 1.0 and 2.0
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

print("world_side_length: {}".format(world_side_length))
print("screen_side_length: {}".format(screen_side_length))

# robot positions
robot_poses = np.random.rand(swarm_size, 2) * world_side_length
# calculate and return the display positions from global variables
def cal_disp_poses():
    poses_temp = robot_poses / world_side_length
    poses_temp[:,1] = 1.0 - poses_temp[:,1]
    poses_temp = poses_temp * screen_side_length
    return poses_temp.astype(int)
disp_poses = cal_disp_poses()

# group properties
groups = {}

# simulation configuration
comm_range = 0.7  # communication range in the world
desired_space = comm_range * 0.75

# visualization configuration
color_white = (255,255,255)
color_black = (0,0,0)
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
    pygame.draw.circle(screen, color_black, disp_poses[i],
        int(comm_range*pixels_per_length), 1)
pygame.display.update()

raw_input("<Press Enter to continue>")

# the outer loop that run the designed set of operations infinitely
while True:
    ########### aggregate together to form a random network ###########


