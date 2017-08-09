# second line formation simulation, using merging method instead of climbing
# this program shares a lot of code from first line formation sim program

# merging method analysis(compared to climbing):
# The line may not be as straight as in first simulation, and it would take time to stretch
# the line so that robots are evenly spaced. But merging method has the flexibility to
# adjust the formation to walls and obstacles, so there is no need to use double walls to
# make sure the initial line segments start in the middle to avoid the line hit the wall.

# comments for research purpose:
# Same competing mechanism is used to ensure only one line is being formed.

# comments for programming purpose:
# Similar four statuses have been designed to serve different stages of the robot:
    # '0': free, wondering around, not in group, available to form a group or join one
    # '1': forming an initial line segment, or found a line and trying to merge into it
        # '1_0': initial an line segment, '0' found another '0' and formed a group
        # '1_1': join a line, merging into
    # '2': already in a line, dynamically adjust space and straightness
    # '-1': free, wondering around, no interaction with nearby robots
# Only allowed status transitions are:
    # forward: '-1'->'0', '0'->'1', '1'->'2'
    # backward: '1'->'-1', '2'->'-1'

# just find sys.exit() can be useful to exit from not-in-the-plan errors
# will be used in this and future programs


import pygame
import math, random, sys
from line_formation_2_robot import LFRobot
from line_formation_2_functions import *

pygame.init()

# for display, window origin is at left up corner
screen_size = (1200, 1000)  # width and height
background_color = (0, 0, 0)  # black background
robot_0_color = (0, 255, 0)  # for robot status '0', green
robot_1_color = (255, 153, 153)  # for robot status '1', pink
robot_2_color = (255, 51, 0)  # for robot status '2', red
robot_n1_color = (0, 51, 204)  # for robot status '-1', blue
robot_size = 5  # robot modeled as dot, number of pixels in radius

# set up the simulation window and surface object
icon = pygame.image.load('icon_geometry_art.jpg')
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption('Line Formation 2 Simulation')

# for physics, continuous world, origin is at left bottom corner, starting (0, 0),
# with x axis pointing right, y axis pointing up.
# It's more natural to compute the physics in right hand coordiante system.
world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])

# varialbes to configure the simulation
robot_quantity = 30
# coefficient to resize the robot distribution, to keep initial positions to center
distrib_coef = 0.5
const_vel_1 = 3.0  # all robots except '2' are moving at this faster constant speed
const_vel_2 = 1.0  # robot '2' are moving at this speed to maintain the line
frame_period = 100  # updating period of the simulation and graphics, in ms
comm_range = 5.0  # sensing and communication range, the radius
line_space = comm_range * 0.7  # line space, between half of and whole communication range
space_error = line_space * 0.1  # the error to determine the space is good
life_incre = 8  # number of seconds a new member adds to a group
group_id_upper_limit = 1000  # upper limit of random integer for group id
n1_life_lower = 3  # lower limit of life time for status '-1'
n1_life_upper = 8  # upper limit of life time for status '-1'

# instantiate the robot swarm as list
robots = []  # container for all robots, index is its identification
for i in range(robot_quantity):
    # random position, away from the window's edges
    pos_temp = (((random.random()-0.5)*distrib_coef+0.5) * world_size[0],
                ((random.random()-0.5)*distrib_coef+0.5) * world_size[1])
    vel_temp = const_vel_1
    ori_temp = random.random() * 2*math.pi - math.pi  # random in (-pi, pi)
    object_temp = LFRobot(pos_temp, vel_temp, ori_temp)
    robots.append(object_temp)
# instantiate the group variable as dictionary
groups = {}
    # key is the group id
    # value is a list
        # 0.first element: the group size, member includes both '2' and '1'
        # 1.second element: remaining life time
        # 2.third element: a list of robots on the line in adjacent order, status '2'
        # 3.fourth element: a list of robots off the line, not in order, status '1'
        # 4.fifth element: true or false, being the domianant group
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
        # the priority to process them is in the following order
        s_grab_on = {}  # robot '0' grabs on robot '2', becoming '1'
            # key is id of robot '0', value is id of robot '2'
        s_init_form = {}  # robot '0' initial forms with another '0', becoming '1'
            # key is id of robot '0' that discovers other robot '0' in range
            # value is a list of robot '0's that are in range, in order of increasing distance
        s_form_done = {}  # robot '1' finishes initial forming, becoming '2'
            # key is group id
            # value is a list of id of robot '1's that have finished initial forming
        s_merge_done = {}  # robot '1' finishes merging, becoming '2'
            # key is group id
            # value is a list of id of robot '1's that have finished climbing
        s_form_lost = []  # robot '1' gets lost during initial forming
            # list of group id for the initial forming robots
        s_merge_lost = []  # robot '1' gets lost during merging
            # list of robot id for the merging robots
        s_line_lost = []  # robot '2' gets lost during adjusting pos
            # list of group id for the robots get lost in that group
        s_group_exp = []  # life time of a group naturally expires
            # life of group id
        s_disassemble = []  # disassemble triggerred by robot '1' or '2'
            # list of lists of group id to be compared for disassembling
        s_back_0 = []  # robot '-1' gets  back to '0'
            # list of robot id

        # check 'robots' for any status change, schedule them for processing in next step
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
                    # check if there is still '2' in the list
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
                            groups_tmep[current_group] = [current_robot]
                    # check if there are multiple groups detected from the '2'
                    target_robot = 0  # the target robot to grab on
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
                        s_grab_on[i] = target_robot
                # process neighbors with status '1', second priority
                elif 1 in status_list[i]:
                    # find the closest '1' and get bounced away by it
                    # still no trigger for disassembling if multiple groups exist in the '1's
                    dist_min = 2*comm_range
                    robot_min = -1
                    for j in range(len(status_list[i])):
                        if status_list[i][j] != 1: continue
                        if dist_table[i][index_list[i][j]] < dist_min:
                            dist_min = dist_table[i][index_list[i][j]]
                            robot_min = index_list[i][j]
                    # target robot located, the robot_min, should not still be -1 here
                    # get bounced away from this robot, update the moving direction
                    vect_temp = (robots[i].pos[0] - robots[robot_min].pos[0],
                                 robots[i].pos[1] - robots[robot_min].pos[1])
                    # orientation is pointing from robot_min to host
                    robots[i].ori = math.atan2(vect_temp[1], vect_temp[0])
                # process neighbors with status '0', least priority
                else:
                    # establish a list of all '0', in order of increasing distance
                    # to be checked later if grouping is possible and no conflict
                    # this list should be only '0's, already sorted
                    target_list = index_list[i][:]
                    # status transition scheduled, '0' forming initial group with '0'
                    s_init_form[i] = target_list[:]
            # for the host robot having status of '1'
            elif robot[i].status == 1:
                # status of '1' needs to be checked and maintained constantly
                # 1.check if the important group neighbors are still in range
                neighbors_secured = True
                for j in robots[i].key_neighbors:
                    if j not in index_list[i]:
                        neighbors_secured = False
                        break
                if neighbors_secured == False:
                    # status transition scheduled, robot '1' gets lost, becoming '-1'
                    if robots[i].status_1_sub == 0:
                        # append the group id, disassemble the entire group
                        s_form_lost.append(robots[i].group_id)
                    else:
                        s_merge_lost.append(i)  # append the robot id
                else:
                    # all key neighbors are in good position
                    # 2.disassemble check, get group attribution of all '1' and '2'
                    status_list_temp = status_list[i][:]
                    index_list_temp = index_list[i][:]
                    # pop out the '0' first
                    while 0 in status_list_temp:
                        index_temp = status_list_temp.index(0)
                        status_list_temp.pop(index_temp)
                        index_list_temp.pop(index_temp)
                    if len(index_list_temp) > 0:  # ensure at least one in-group robot around
                        # start the group attribution dictionary with first robot
                        groups_temp = {robots[index_list_temp[0]].group_id: [index_list_temp[0]]}
                        for j in index_list_temp[1:]:  # iterating from the second one
                            current_group = robots[j].group_id
                            if current_group in groups_temp.keys():
                                # append this robot in same group
                                groups_temp[current_group].append(j)
                            else:
                                # add new key in the groups_temp dictionary
                                groups_temp[current_group] = [j]
                        # check if there are multiple groups detected
                        if len(groups_temp.keys()) > 1:
                            # status transition scheduled, to disassemble groups
                            s_disassemble.append(groups_temp.keys())
                            # may produce duplicates in s_disassemble, not serious problem
                    # 3.check if any neighbor needs to be done
                    if robots[i].status_1_sub == 0:
                        # host robot is in the initial forming phase
                        # check if the neighbor robot is in appropriate distance
                        if abs(dist_table[i][robots[i].key_neighbors[0]] -
                               line_space) < space_error:
                            # status transition scheduled, finish initial forming, '1' to '2'
                            g_it = robots[i].group_id
                            if g_it in s_form_done.keys():
                                s_form_done[g_it].append(i)
                            else:
                                s_form_done[g_it] = [i]
                    elif robots[i].status_1_sub == 1:
                        # host robot is in the merging phase
                        # check if next key neighbor appears or not
                        if len(robots[i].status_1_1_next) != 0:
                            # the next key neighbor list should contains one member at most
                            next_neighbor = robots[i].status_1_1_next[0]
                            if next_neighbor in index_list[i]:
                                # the next expected neighbor appears
                                robots[i].status_1_1_next.pop(0)
                                robots[i].key_neighbors.append(next_neighbor)
                        # check if the merging robot reaches the destination
                        vect_temp = (robots[i].pos[0] - robots[i].status_1_1_des[0],
                                     robots[i].pos[1] - robots[i].status_1_1_des[1])
                        dist_temp = math.sqrt(vect_temp[0]*vect_temp[0] +
                                              vect_temp[1]*vect_temp[1])
                        if dist_temp < space_error:
                            # status transition scheduled, finish merging, '1' to '2'
                            g_it = robots[i].group_id
                            if g_it in s_merge_done.keys():
                                s_merge_done[g_it].append(i)
                            else:
                                s_merge_done[g_it] = [i]
            # for the host robot having status of '2'
            elif robots[i].status == 2:
                # check if all key neighbors are still in range
                neighbors_secured = True
                for j in robots[i].key_neighbors:
                    if j not in index_list[i]:
                        neighbors_secured = False
                        break
                if neighbors_secured == False:
                    # status transition scheduled, robot '2' gets lost, disassemble the group
                    s_line_lost.append(robots[i].group_id)
                else:
                    # all the key neighbors are in goood position
                    # disassemble check, for all the '1' and '2'
                    status_list_temp = status_list[i][:]
                    index_list_temp = index_list[i][:]
                    # pop out the '0' first
                    while 0 in status_list_temp:
                        index_temp = status_list_temp.index(0)
                        status_list_temp.pop(index_temp)
                        index_list_temp.pop(index_temp)
                    # start the group attribution dictionary with first robot
                    if len(index_list_temp) > 0:
                        groups_temp = {robots[index_list_temp[0]].group_id: [index_list_temp[0]]}
                        for j in index_list_temp[1:]:
                            current_group = robots[j].group_id
                            if current_group in groups_temp.keys():
                                groups_temp[current_group].append(j)
                            else:
                                groups_temp[current_group] = [j]
                        # check if there are multiple groups detected
                        if len(groups_temp.keys()) > 1:
                            # status transition scheduled, to disassemble groups
                            s_disassemble.append(groups_temp.keys())
            elif robots[i].status == -1:
                # check if life time expires, and get status back to '0'
                if robots[i].status_n1_life < 0:
                    s_back_0.append(i)

        # check 'groups' for any status change
        for g_it in groups.keys():
            if groups[g_it][4]: continue  # already becoming dominant
            if groups[g_it][0] > robot_quantity/2:
                # the group has more than half the totoal number of robots
                groups[g_it][4] = True  # becoming dominant
                groups[g_it][1] = 100.0  # a large number
            if groups[g_it][1] < 0:  # life time of a group expires
                # schedule operation to disassemble this group
                s_group_exp.append(g_it)

        # process the scheduled status change, in the order of the priority
        # 1.s_grab_on, robot '0' grabs on robot '2', becoming '1'
        for i in s_grab_on.keys():
            # discuss the merging availability of robot 'i'
            it0 = s_grab_on[i]  # 'it0' is the robot that robot 'i' tries to grab on
            # it0 was available when added to s_grab_on, but check again if occupied by others
            # merge availability of smaller index side
            side0_avail = robots[j].status_2_avail1[0] & robots[j].status_2_avail2[0]
            side0_des = [-1,-1]  # destination if merging at small index end
            side0_hang = False  # indicate if destination at side 0 is hanging
            side0_next = -1  # next key neighbor expected to meet at this side
            if side0_avail:
                # calculate the merging destination
                # if this side is beyond two ends of the line, it can only be the small end
                if robots[it0].status_2_sequence == 0:
                    # the grab on robot is the robot of index 0 on the line
                    # get its only neighbor to calculate the line direction
                    it1 = groups[robots[it0].group_id][2][1]  # second robot on the line
                    vect_temp = (robots[it0].pos[0] - robots[it1].pos[0],
                                 robots[it0].pos[1] - robots[it1].pos[1])  # from it1 to it0
                    ori_temp = math.atan2(vect_temp[1], vect_temp[0])
                    side0_des = [robots[it0].pos[0] + line_space*math.cos(ori_temp),
                                 robots[it0].pos[1] + line_space*math.sin(ori_temp)]
                    side0_hang = True
                else:
                    # the grab on robot is not at the start of the line
                    # get its smaller index neighbor
                    it1 = groups[robots[it0].group_id][2][robots[it0].status_2_sequence - 1]
                    side0_next = it1
                    # the destination is the middle position of it0 and it1
                    side0_des = [(robots[it0].pos[0]+robots[it1].pos[0])/2,
                                 (robots[it0].pos[1]+robots[it1].pos[1])/2]
            # merge availability of larger index side
            side1_avail = robots[j].status_2_avail1[1] & robots[j].status_2_avail2[1]
            side1_des = [-1,-1]
            side1_hang = False  # indicate if destination at side 1 is hanging
            side1_next = -1  # next key neighbor expected to meet at this side
            if side1_avail:
                # calculate the merging destination
                # if this side is beyond two ends of the line, it can only the large end
                if robots[it0].status_2_end:
                    # the grab on robot is at the larger index end
                    # get its only neighbor to calculate the line direction
                    it1 = groups[robots[it0].group_id][2][robots[it0].status_2_sequence - 1]
                    vect_temp = (robots[it0].pos[0] - robots[it1].pos[0],
                                 robots[it0].pos[1] - robots[it1].pos[1])  # from it1 to it0
                    ori_temp = math.atan2(vect_temp[1], vect_temp[0])
                    side1_des = [robots[it0].pos[0] + line_space*math.cos(ori_temp),
                                 robots[it0].pos[1] + line_space*math.sin(ori_temp)]
                    side1_hang = True
                else:
                    # the grab on robot is not at the larger index end
                    # get its larger index neighbor
                    it1 = groups[robots[it0].group_id][2][robots[it0].status_2_sequence + 1]
                    side1_next = it1
                    # the destination is the middle position of it0 and it1
                    side1_des = [(robots[it0].pos[0]+robots[it1].pos[0])/2,
                                 (robots[it0].pos[1]+robots[it1].pos[1])/2]
            # check if there is at least one side is available
            if side0_avail or side1_avail:
                # perform operations regardless of which side to merge
                robots[i].status = 1  # status becoming '1'
                g_it = robots[it0].group_id
                robots[i].group_id = g_it
                robots[i].status_1_sub = 1
                robots[i].key_neighbors = [it0]
                # perform operations regarding which side to merge
                if side0_avail and (not side1_avail):
                    # only small index side is available, perform corresponding operations





