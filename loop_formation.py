# loop formation simulation of swarm robots, form a triangle, and build loop by merging

# comments for programming purpose:
# Similar status design with the second line formation:
    # '0': free, wondering around, available to form a pair, a triangle or joint a group
    # '1': transit status between '0' and '2', robot in a group, but not a loop yet
        # '1_0': forming status, either in a pair, or in a triangle formation
            # '1_0_0': forming status, in a pair
            # '1_0_1': forming status, in a triangle
        # '1_1': merging status, joining a robot '2' group
    # '2': formal members of a loop formation group
    # '-1': free, wondering around, ignoring all interactions
# Only allowed status transitions are:
    # forward: '-1'->'0', '0'->'1_0_0', '0'->'1_0_1', '0'->'1_1', '1_0_0'->'1_0_1'
    #          '1_0_1'->'2', '1_1'->2
    # backward: '1'->'-1', '2'->'-1'
# Status transition regarding robot '1':
    # When 2 '0' meets, they both become '1_0_0', and start adjust distance to the loop
    # space. When distance is good, they don't perform any status transition yet. Only
    # when a new '0' comes over, it will trigger all three robots into '1_0_1' status.
    # And during the entire time of '1_0_0', the two robots are always welcoming new '0',
    # even their distance is not good yet. When in '1_0_1', after the three robots form
    # the perfect triangle and good loop space, they will trigger a status transition
    # of becoming '2'.

# reasons why still two stages forming, even when a '0' discovers two '0'

import pygame
import math, random
from loop_formation_robot import LFRobot
from formation_functions import *

pygame.init()

# for display, window origin is at left up corner
screen_size = (1200, 1000)  # width and height
background_color = (0, 0, 0)  # black background
robot_0_color = (0, 255, 0)  # for robot status '0', green
robot_1_0_0_color = (255, 153, 153)  # for robot status '1_0_0', light pink
robot_1_0_1_color = (255, 153, 153)  # for robot status '1_0_1', dark pink
robot_1_1_color = (255, 153, 153)  # for robot status '1_1', dark pink
robot_2_color = (255, 51, 0)  # for robot status '2', red
robot_n1_color = (0, 51, 204)  # for robot status '-1', blue
robot_size = 5  # robot modeled as dot, number of pixels in radius

# set up the simulation window and surface object
icon = pygame.image.load('icon_geometry_art.jpg')
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption('Loop Formation Simulation')

# for physics, continuous world, origin is at left bottom corner, starting (0, 0),
# with x axis pointing right, y axis pointing up.
# It's more natural to compute the physics in right hand coordiante system.
world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])

# variables to configure the simulation
robot_quantity = 30
distrib_coef = 0.5  # coefficient to resize the initial robot distribution
const_vel = 3.0  # all robots except those in adjusting phase are moving at this speed
frame_period = 100  # updating period of the simulation and graphics, in millisecond
comm_range = 5.0  # sensing and communication share this same range, in radius
loop_space = comm_range * 0.7  # desired space of robots on the lop
space_error = loop_space * 0.1  # error to determine if a status transition is needed
life_incre = 8  # number of seconds a new member adds to a group
group_id_upper_limit = 1000  # upper limit of random integer for group id
n1_life_lower = 3  # lower limit of life time for status '-1'
n1_life_upper = 8  # upper limit of life time for status '-1'
# coefficient for calculating velocity of robot '2' on the loop for adjusting
# as the distance error decreases, the loop adjusting velocity also decreases
adjust_vel_coef = consst_vel/loop_space * 2

# instantiate the robot swarm as list
robots = []  # container for all robots, index is its identification
for i in range(robot_quantity):
    # random position, away from the window's edges
    pos_temp = (((random.random()-0.5)*distrib_coef+0.5) * world_size[0],
                ((random.random()-0.5)*distrib_coef+0.5) * world_size[1])
    vel_temp = const_vel
    ori_temp = random.random() * 2*math.pi - math.pi  # random in (-pi, pi)
    object_temp = LFRobot(pos_temp, vel_temp, ori_temp)
    robots.append(object_temp)
# instantiate the group variable as dictionary
groups = {}
    # key is the group id
    # value is a list
        # 0.first element: the group size, member includes both '2' and '1'
        # 1.second element: remaining life time
        # 2.third element: a list of robots on the loop in adjacent order, status '2'
        # 3.fourth element: a list of robots off the loop, not ordered, status '1'
        # 4.fifth element: true or false, being the dominant group
# instantiate a distance table for every pair of robots
# make sure all data in table is being written when updating
dist_table = [[0 for j in range(robot_quantity)] for i in range(robot_quantity)]

# the loop
sim_exit = False  # simulation exit flag
sim_pause = False  # simulation pause flag
timer_last = pygame.time.get_ticks()  # return number of milliseconds after pygame.init()
timer_now = timer_last  # initialize it with timer_last
while not sim_exit:
    # exit the program
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sim_exit = True  # exit with the close window button
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_SPACE:
                sim_pause = not sim_pause  # reverse the pause flag
            if (event.key == pygame.K_ESCAPE) or (event.key == pygame.K_q):
                sim_exit = True  # exit with ESC key or Q key

    # skip the rest of the loop if paused
    if sim_pause: continue

    # update the physics, control rules and graphics at the same time
    timer_now = pygame.time.get_ticks()
    if (timer_now - timer_last) > frame_period:
        timer_last = timer_now  # reset timer
        # prepare the distance data for every pair of robots
        for i in range(robot_quantity):
            for j in range(i+1, robot_quantity):
                # status of '-1' does not involve in any connection, so skip
                if (robots[i].status == -1) or (robots[j].status == -1):
                    dist_table[i][j] = -1.0
                    dist_table[j][i] = -1.0
                    continue  # skip the rest
                # it only covers the upper triangle without the diagonal
                vect_temp = (robots[i].pos[0]-robots[j].pos[0],
                            robots[i].pos[1]-robots[j].pos[1])
                dist_temp = math.sqrt(vect_temp[0]*vect_temp[0] +
                                      vect_temp[1]*vect_temp[1])
                if dist_temp <= comm_range:
                    # only record distance smaller than communication range
                    dist_table[i][j] = dist_temp
                    dist_table[j][i] = dist_temp
                else:  # ignore the neighbors too far away
                    dist_table[i][j] = -1.0
                    dist_table[j][i] = -1.0
        # sort the distance in another table, record the index here
        index_list = [[] for i in range(robot_quantity)]  # index of neighbors in range
        # find all robots with non-zero distance
        for i in range(robot_quantity):
            for j in range(robot_quantity):
                if j == i: continue  # skip the self, not necessary
                if dist_table[i][j] > 0: index_list[i].append(j)
        # sort the index_list in the order of increasing distance
        for i in range(robot_quantity):
            len_temp = len(index_list[i])  # number of neighbors
            if (len_temp < 2): continue  # no need to sort
            else:
                # bubble sort
                for j in range(len_temp-1):
                    for k in range(len_temp-1-j):
                        if (dist_table[i][index_list[i][k]] >
                            dist_table[i][index_list[i][k+1]]):
                           # swap the position of the two index
                           index_temp = index_list[i][k]
                           index_list[i][k] = index_list[i][k+1]
                           index_list[i][k+1] = index_temp
        # get the status list corresponds to the sorted index_list
        status_list = [[] for i in range(robot_quantity)]
        for i in range(robot_quantity):
            for j in index_list[i]:
                status_list[i].append(robots[j].status)

        # instantiate the status transition variables, for the status check process
        # the priority is in the following order
        s_merging = {}  # robot '0' detects robot '2', merging into its group
            # key is id of robot '0', value is id of corresponding robot '2'
        s_forming0 = {}  # robot '0' forms the pair with another '0', becoming '1_0_0'
            # key is id of robot '0' that discovers other robots '0' in range
            # value is a list of robots '0' that are detected, in order of ascending dist
        s_forming1 = {}  # two '1_0_0' form the triangle with a '0', becoming '1_0_1'
            # key is group id of the two '1_0_0', value is the new robot '0'
            # it can be triggered from both ways, '1_0_0' to '0', or '0' to '1_0_0'
        s_form_done = {}  # robot '1_0_1' finishes initial forming, becoming '2'
            # key is group id of the robots '1_0_1'
            # value is a list of robots in group that agree froming is done, should be all
        s_merge_done = []  # robot '1_1' finishes merging, becoming '2'
            # list of robots '1' that finished merging
        s_group_exp = []  # life time of a group naturally expires, disassemble it
            # list of group ids
        s_disassemble = []  # disassemble trigger by robots '1' or '2'
            # list of lists of group id to be compared for disassembling
        s_back_0 = []  # robot '-1' gets back to '0' after life expires
            # list of robots '-1'
        # All the checkings for robots '1' and '2' getting lost during forming are removed,
        # such behaviors should be observable during tests, and program should not run into
        # any robot lost once it is free of bugs. Therefore it's not necessary for checking.

        # check 'robots' for any status change, schedule for processing in next step
        for i in range(robot_quantity):
            # for the host robot having status of '0'
            if robots[i].status == 0:
                # check if this robot has valid neighbors at all
                if len(index_list[i]) == 0: continue  # skip the neighbor check
                # process neighbors with status '2', highest priority
                if 2 in status_list[i]:
                    # check the group attribution of all the '2'
                    # get the robot id and group id of the first '2'
                    current_index = status_list[i].index(2)
                    current_robot = index_list[i][current_index]
                    current_group = robots[current_robot].group_id
                    groups_temp = {current_group: [current_robot]}
                    # check if there are still '2' in the list
                    while 2 in status_list[i][current_index+1:]:
                        # indexing the next '2' from current_index+1
                        # update the current_index, current_robot, current_group
                        current_index = status_list[i].index(2, current_index+1)
                        current_robot = index_list[i][current_index]
                        current_group = robots[current_robot].group_id
                        # update groups_temp
                        if current_group in groups_temp.keys():
                            # append this robot in the existing group
                            groups_temp[current_group].append(current_robot)
                        else:
                            # add new group id in groups_temp and add this robot
                            groups_temp[current_group] = [current_robot]
                    # check if there are multiple groups detected from the '2'
                    target_robot = 0  # the target robot '2' to merge into the group
                    if len(groups_temp.keys()) == 1:
                        # there is only one group detected
                        dist_min = 2*comm_range  # smallest distance, start with large one
                        robot_min = -1  # corresponding robot with min distance
                        # search the closest '2'
                        for j in groups_temp.values()[0]:
                            if dist_table[i][j] < dist_min:
                                if ((robots[j].status_2_avail1[0] & robots[j].status_2_avail2[0]) |
                                    (robots[j].status_2_avail1[1] & robots[j].status_2_avail2[1])):
                                    # check both avail1 and avail2 for both side
                                    # there is at least one side is available to be merged
                                    dist_min = dist_table[i][j]
                                    robot_min = j
                        target_robot = robot_min
                    else:
                        # there is more than one group detected
                        # it is designed that no disassembling trigger from robot '0'
                        # compare which group has the most members in it
                        member_max = 0  # start with 0 number of members in group
                        group_max = 0  # corresponding group id with most members
                        for j in groups_temp.keys():
                            ## 'line_formation_1.py' did wrong in the following line
                            # didn't fix it, not serious problem, program rarely goes here
                            if groups[j][0] > member_max:
                                member_max = groups[j][0]
                                group_max = j
                        # search the closest '2' inside that group
                        dist_min = 2*comm_range
                        robot_min = -1
                        for j in groups_temp[group_max]:
                            if dist_table[i][j] < dist_min:
                                if ((robots[j].status_2_avail1[0] & robots[j].status_2_avail2[0]) |
                                    (robots[j].status_2_avail1[1] & robots[j].status_2_avail2[1])):
                                    dist_min = dist_table[i][j]
                                    robot_min = j
                        target_robot = robot_min
                    # check if target robot has been located, prepare to grab on it
                    if target_robot != -1:  # not the initial value of robot_min
                        # the target robot should have at least one spot availbe to be merged
                        s_merging[i] = target_robot
                # process neighbors with status '1', second priority
                


