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
# The index of robots on the loop starts with 0 and goes counter-clockwise, the index 0
# robot never change its index, when a new robot merges, all robots after it on the loop
# will increase index by 1. Since a loop has no end, all robots have two key neighbors.
# The definition of key neighbors is that, the first one is for the robot before it on
# the loop, second one for robot after it.


# reasons why still two stages forming, even when a '0' discovers two '0'

import pygame
import math, random, sys
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

# function for solving destination on the loop based on positions of two neighbors
def solve_des(pos_l, pos_r, dist_0, l_d):
    # first input is 2D position of neighbor on the left, second for on the right
    # dist_0 is the distance between pos_l and pos_r
    # l_d is the desired distance to the two neighbors
    vect_0 = (pos_l[0]-pos_r[0], pos_l[1]-pos_r[1])  # vector from pos_r to pos_l
    midpoint = [(pos_l[0]+pos_r[0])/2, (pos_l[1]+pos_r[1])/2]
    if dist_0 >= 2*l_d:
        # two neighbors are too far away, destination will be just midpoint
        return midpoint
    else:
        # get direction perpendicular to the the line of pos_l and pos_r
        vect_1 = [-vect_0[1]/dist_0, vect_0[0]/dist_0]  # rotate vect_0 ccw for 90 degrees
        dist_1 = math.sqrt(l_d*l_d - dist_0*dist_0/4)
        # return the point from midpoint, goes along vect_1 for dist_1
        return [midpoint[0]+vect_1[0]*dist_1, midpoint[1]+vect_1[1]*dist_1]

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
        s_forming2 = {}  # two '1_0_0' form the triangle with a '0', becoming '1_0_1'
            # key is group id of the two '1_0_0', value is the new robot '0'
            # it is designed to be triggered by the new robot '0'
        s_forming1 = {}  # robot '0' forms the pair with another '0', becoming '1_0_0'
            # key is id of robot '0' that discovers other robots '0' in range
            # value is a list of robots '0' that are detected, in order of ascending dist
            # this works the same with "s_init_form" from previous simulations
        s_form_done = []  # robot '1_0_1' finishes initial forming, becoming '2'
            # list of group id that the triangle forming is done
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

        # check 'robots' variable for any status change, schedule for processing in next step
        for i in range(robot_quantity):
            # for the host robot having status of '0'
            if robots[i].status == 0:
                # check if this robot has valid neighbors at all
                if len(index_list[i]) == 0: continue  # skip the neighbor check
                # pre-process: classify the robot '1' neighbors into their substatus,
                # because robot '0' reacts differently to different substatuses of '1'
                # new statuses have been assigned as follows:
                    # '1_0_0' -> '1.1'
                    # '1_0_1' -> '1.2'
                    # '1_1'   -> '1.3'
                if 1 in status_list[i]:
                    index_temp = 0
                    while 1 in status_list[i]:
                        index_temp = status_list[i].index(1)
                        robot_temp = index_list[i][index_temp]
                        status_1_sub = robots[index_temp].status_1_sub
                        if status_1_sub == 0:
                            status_1_0_sub = robots[index_temp].status_1_0_sub
                            if status_1_0_sub == 0:
                                status_list[i][index_temp] = 1.1  # 1.1 for status '1_0_0'
                            elif status_1_0_sub == 1:
                                status_list[i][index_temp] = 1.2  # 1.2 for status '1_0_1'
                            else:  # just in case
                                print("wrong sub status of 1_0")
                                sys.exit()
                        elif status_1_sub == 1:
                            status_list[i][index_temp] = 1.3  # 1.3 for status '1_1'
                        else:  # just in case
                            prin("wrong sub status of 1")
                            sys.exit()
                # the sequence of processing the neighbors are:
                    # '2' -> '1.1' -> '0' -> '1.2'&'1.3'
                    # This means if there is a group of '2', join them; otherwise if there is a
                    # pair of '1_0_0', join them; otherwise if there is a '0', pair with it; if
                    # all the above failed, just get bounced away by closest '1_0_1' or '1_1'
                # start processing neighbors with status '2'
                if 2 in status_list[i]:
                    # check the group attribution of all the '2'
                    # get the robot id and group id of the first '2'
                    index_temp = status_list[i].index(2)
                    robot_temp = index_list[i][index_temp]
                    group_temp = robots[robot_temp].group_id
                    groups_temp = {group_temp: [robot_temp]}
                    # check if there are still '2' in the list
                    while 2 in status_list[i][index_temp+1:]:
                        # indexing the next '2' from index_temp+1
                        # update the index_temp, robot_temp, group_temp
                        index_temp = status_list[i].index(2, index_temp+1)
                        robot_temp = index_list[i][index_temp]
                        group_temp = robots[robot_temp].group_id
                        # update groups_temp
                        if group_temp in groups_temp.keys():
                            # append this robot in the existing group
                            groups_temp[group_temp].append(robot_temp)
                        else:
                            # add new group id in groups_temp and add this robot
                            groups_temp[group_temp] = [robot_temp]
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
                        # status transition scheduled, robot '0' is becoming '1_1'
                        s_merging[i] = target_robot
                # process neighbors with status '1.1'
                elif 1.1 in status_list[i]:
                    # check the group attribution of all '1.1'
                    # get the robot id and group id of the first 1.1
                    index_temp = status_list[i].index(1.1)
                    robot_temp = index_list[i][index_temp]
                    group_temp = robots[robot_temp].group_id
                    groups_temp = {group_temp: [robot_temp]}
                    # check if there are still '1.1' in the list
                    while 1.1 in status_list[i][index_temp+1:]:
                        # update the index_temp, robot_temp, group_temp
                        index_temp = status_list[i].index(1.1, index_temp+1)
                        robot_temp = index_list[i][index_temp]
                        group_temp = robots[robot_temp].group_id
                        # update groups_temp
                        if group_temp in groups_temp.keys():
                            # append this robot in the existing group
                            groups_temp[group_temp].append(robot_temp)
                        else:
                            # add new entry of group id in groups_temp and add this robot
                            groups_temp[group_temp] = [robot_temp]
                    # check if there are multiple groups detected from the '1.1'
                    target_robot = 0  # the target robot '1.1' to form triangle with
                    if len(groups_temp.keys()) == 1:
                        # there is only one group detected from the '1.1'
                        dist_min = 2*comm_range  # variable for smallest distance
                        robot_min = -1  # corresponding robot with dist_min
                        # search the closest '1.1'
                        for j in groups_temp.values()[0]:
                            if dist_table[i][j] < dist_min:
                                # there is no availability restriction here
                                dist_min = dist_table[i][j]
                                robot_min = j
                        target_robot = robot_min
                    else:
                        # there are more than one group detected from the '1.1'
                        # choose the group that has the most members in it
                        member_max = 0  # variable for the maximum number of members
                        group_max = 0  # corresponding group id with member_max
                        for j in groups_temp.keys():
                            if groups[j][0] > member_max:
                                member_max = groups[j][0]
                                group_max = j
                        # search the closest '1.1' inside that group "group_max"
                        dist_min = 2*comm_range
                        robot_min = -1
                        for j in groups_temp[group_max]:
                            if dist_table[i][j] < dist_min:
                                dist_min = dist_table[i][j]
                                robot_min = j
                        target_robot = robot_min
                    # check if target robot has been located, form the triangle with it
                    if target_robot != -1:  # not the initial value of "robot_min"
                        group_temp = robots[i].group_id
                        # status transition scheduled, robot '0' & '1_0_0' are becoming '1_0_1'
                        if group_temp in s_forming2.keys():
                            s_forming2[group_temp].append(target_robot)
                        else:  # most likely this happens
                            s_forming2[group_temp] = [target_robot]
                # process neighbors with status '0'
                elif 0 in status_list[i]:
                    # establish a list of all '0', in order of ascending distance
                    # the list is to be checked later if grouping is possible and no conflict
                    # remove possible '1.2' and '1,3' from status_list
                    while 1.2 in status_list[i]:
                        index_temp = status_list[i].index(1.2)
                        status_list[i].pop(index_temp)
                        index_list[i].pop(index_temp)
                    while 1.3 in status_list[i]:
                        index_temp = status_list[i].index(1.3)
                        status_list[i].pop(index_temp)
                        index_list[i].pop(index_temp)
                    # status transition scheduled, robot '0' is becoming '1_0_0'
                    s_forming1[i] = index_list[i][:]
                # process neighbors with status '1.2' and '1.3'
                elif 1.2 in stauts_list[i] or 1.3 in status_list[i]:
                    # find the closest '1.2' or '1.3', and get bounced away by it
                    dist_min = 2*comm_range
                    robot_min = -1
                    # if here, it means '1.2' and '1.3' are the only robot left in index_list
                    for j in index_list[i]:
                        if dist_table[i][j] < dist_min:
                            dist_min = dist_table[i][j]
                            robot_min = j
                    # target robot located, the robot_min, should not still be '-1' here
                    # get bounced away from this robot, update the moving direction
                    vect_temp = (robots[i].pos(0) - robots[robot_min].pos(0),
                                 robots[i].pos(1) - robots[robot_min].pos(1))
                    # new orientation is pointing from robot_min to the host robot
                    robots[i].ori = math.atan2(vect_temp[1], vect_temp[0])
            # for the host robot having status of '1'
            elif robots[i].status == 1:
                # disassemble check, get group attribution of all '1' and '2'
                # pop out the '0' from the status_list
                while 0 in status_list[i]:
                    index_temp = status_list[i].index(0)
                    status_list[i].pop(index_temp)
                    index_list[i].pop(index_temp)
                if len(index_list[i]) > 0:  # ensure at least one in-group robot around
                    # start the group attribution dictionaary with first robot
                    group_temp = robots[index_list[i][0]].group_id
                    groups_temp = {group_temp:[robot_temp]}
                    # then check the rest for group attribution
                    for j in index_list[i]:
                        group_temp = robots[j].group_id
                        if group_temp in groups_temp.keys():
                            groups_temp[group_temp].append(j)
                        else:
                            groups_temp[group_temp] = [j]
                    # check if there are multiple groups detected
                    if len(groups_temp.keys()) > 1:
                        # status transition scheduled, to disassemble the minority groups
                        s_disassemble.append(groups_temp.keys())
                        # may produce duplicates in s_disassemble, not big problem
                # check if any status transition needs to be scheduled
                if robots[i].status_1_sub == 0:
                    if status[i].status_1_0_sub = 1:
                        # host robot is in the triangle forming phase
                        # check if this group has already been scheduled for status transition
                        if robots[i].group_id not in s_form_done:
                            it0 = robots[i].key_neighbors[0]
                            it1 = robots[i].key_neighbors[1]
                            dist_satisfied = True  # flag indicating the distances are satisfied
                            if abs(dist_table[i][it0] - loop_space) > space_error:
                                dist_satisfied = False
                            elif abs(dist_table[i][it1] - loop_space) > space_error:
                                dist_satisfied = False
                            elif abs(dist_table[it0][it1] - loop_space) > space_error:
                                dist_satisfied = False
                            if dist_satisfied:
                                # status transition scheduled, robots '1_0_1' are becoming '2'
                                s_form_done.append(robots[i].group_id)
                elif robots[i].status_1_sub == 1:
                    # robot is in the merging phase
                    vect_temp = (robots[i].status_1_1_des[0] - robots[i].pos[0],
                                 robots[i].status_1_1_des[1] - robots[i].pos[1])
                    dist_temp = math.sqrt(vect_temp[0]*vect_temp[0]+
                                          vect_temp[1]*vect_temp[1])
                    if dist_temp < space_error:
                        # status transition scheduled, robot '1_1' is becoming '2'
                        s_merge_done.append(i)
            # for the host robot having status of '2'
            elif robots[i].status == 2:
                # disassemble check, get group attribution of all '1' and '2'
                # pop out the '0' from the status_list
                while 0 in status_list[i]:
                    index_temp = status_list[i].index(0)
                    status_list[i].pop(index_temp)
                    index_list[i].pop(index_temp)
                if len(index_list[i]) > 0:  # ensure at least one in-group robot around
                    # start the group attribution dictionaary with first robot
                    group_temp = robots[index_list[i][0]].group_id
                    groups_temp = {group_temp:[robot_temp]}
                    # then check the rest for group attribution
                    for j in index_list[i]:
                        group_temp = robots[j].group_id
                        if group_temp in groups_temp.keys():
                            groups_temp[group_temp].append(j)
                        else:
                            groups_temp[group_temp] = [j]
                    # check if there are multiple groups detected
                    if len(groups_temp.keys()) > 1:
                        # status transition scheduled, to disassemble the minority groups
                        s_disassemble.append(groups_temp.keys())
                        # may produce duplicates in s_disassemble, not big problem
            # for the host robot having status of '-1'
            elif robots[i].status == -1:
                # check if life time expires
                if robots[i].status_n1_life < 0:
                    # status transition scheduled, robot '-1' is becoming '0'
                    s_back_0.append(i)

        # check 'groups' variable for any status change
        for g_it in groups.keys():
            if groups[g_it][4]: continue  # already being dominant group
            if groups[g_it][0] > robot_quantity/2:
                # the group has more than half the totoal number of robots
                groups[g_it][4] = True  # becoming dominant group
                groups[g_it][1] = 100.0  # a large number
            if groups[g_it][1] < 0:  # life time of a group expires
                # schedule operation to disassemble this group
                s_group_exp.append(g_it)

        # process the scheduled status change, in the order of the designed priority
        # 1.s_merging, robot '2' merges into the group where the robot '2' is from
        for i in s_merging.keys():
            it0 = s_merging[i]  # 'it0' is the robot that robot 'i' tries to merge with
            # discuss the merging availability of robot 'it0'
            g_it = robots[it0].group_id
            # it0 was available when added to s_merging, but check in case occupied again
            # merge availablility on left side
            side0_avail = robots[it0].status_2_avail1[0] & robots[it0].status_2_avail2[0]
            side0_des = [-1,-1]  # destination if merging at left side
            if side0_avail:
                # calculate the merging destination
                it1 = robots[it0].key_neighbors[0]
                side0
