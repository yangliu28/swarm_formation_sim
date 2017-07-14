# line formation simulation

import pygame
import math, random
from line_formation_robot import LFRobot
from line_formation_functions import *

pygame.init()  # initialize pygame

# for display, window origin is at left up corner
screen_size = (1200, 900)  # width and height
background_color = (0, 0, 0)  # black background
robot_color = (255, 255, 255)  # white robot
robot_size = 5  # robot modeled as dot, number of pixels in radius

# set up the simulation window and surface object
icon = pygame.image.load('geometry_art.jpg')
pygame.display.set_icon(icon)
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption('Line Formation Simulation')

# for physics, origin is at left bottom corner, starting (0, 0)
# it's more natural to do the calculation in right hand coordiante system
# the continuous, physical world for the robots
world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])

# variables to configure the simulation
robot_quantity = 30
# coefficient to resize the robot distribution, but always keep close to center
distrib_coef = 0.5
const_vel = 2  # all moving robots are moving at a constant speed
frame_period = 100  # updating period of the simulation and graphics, in ms
comm_range = 7  # communication range, the radius
line_space = comm_range * 0.7  # a little more than half the communication range
space_err = line_space * 0.1  # the error to determine the space is good
climb_space = line_space * 0.5  # climbing is half the line space along the line

# instantiate the robot swarm
robots = []  # container for all robots, index is also the identification
for i in range(robot_quantity):
    # random position, away from the window edges
    pos_temp = (((random.random()-0.5)*distrib_coef+0.5) * world_size[0],
                ((random.random()-0.5)*distrib_coef+0.5) * world_size[1])
    vel_temp = const_vel
    ori_temp = random.random() * 2*math.pi  # random in (0, 2*pi)
    object_temp = LFRobot(pos_temp, vel_temp, ori_temp)
    robots.append(object_temp)
# instantiate the group variable as dictionary
groups = {}
    # key is the group id, so two groups won't share same id
    # value is a list
        # 0.first element: the group size
        # 1.second element: life time remaining
        # 2.third element: a list of robots on the line in adjacent order, status '2'
        # 3.forth element: a list of robots off the line, not in order, status '1'

# instantiate a distance table for every pair of robots
# will calculate once for symmetric data
dist_table = [[-1.0 for i in range(robot_quantity)] for j in range(robot_quantity)]

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
        # prepare the distance data for every pair of robots
        for i in range(robot_quantity):
            for j in range(i+1, robot_quantity):
                # status of '-1' does not involve in any connection, so skip
                if (i == -1) or (j == -1):
                    dist_table[i][j] == -1.0
                    dist_table[j][i] == -1.0
                    continue  # skip the rest
                # it only covers the upper triangle without the diagonal
                pos_temp = (robots[i].pos[0]-robots[j].pos[0],
                            robots[i].pos[1]-robots[j].pos[1])
                dist_temp = math.sqrt(pos_temp[0]*pos_temp[0] +
                                      pos_temp[1]*pos_temp[1])
                if dist_temp <= comm_range:
                    # only record distance smaller than communication range
                    dist_table[i][j] = dist_temp
                    dist_table[j][i] = dist_temp
                else:  # ignore the neighbors too far away
                    dist_table[i][j] = -1.0
                    dist_table[j][i] = -1.0
        # sort the distance in another table, record the index here
        index_list = [[] for i in range(robot_quantity)]  # index of neighbors in range
        # find all non-zero distances
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
                        if dist_table[i][index_list[i][k]] >
                           dist_table[i][index_list[i][k+1]]:
                           # swap the position of the two index
                           index_temp = index_list[i][k]
                           index_list[i][k] = index_list[i][k+1]
                           index_list[i][k+1] = index_temp
        # get the status list corresponds to the sorted index_list
        status_list = [[] for i in range(robot_quantity)]
        for i in range(robot_quantity):
            for j in index_list[i]:
                status_list[i].append(robots[j].status)

        # instantiate the status transition variables, prepare for status check
        # the priority to process them is in the following order
        s_grab_on = {}  # robot '0' grabs on robot '2'
        s_init_form = {}  # robot '0' forms a group with another robot '0'
        s_form_done = []  # robot '1' finishes climbing, becoming '2'
        s_lost = []  # robot '1' gets lost while climbing
        s_disassemble = []  # disassembling trigger by robot '1' or '2'
        # other status transition needs to be regularly checked are:
            # natural expiration of a group
            # robot '-1' back in game after random delay, becoming '0'

        # check if any status change needs to be scheduled, and process in next step
        for i in range(robot_quantity):
            # check if this robot has valid neighbors at all
            if len(index_list[i]) == 0:
                continue;  # skip the neighbor check
            # for the host robot having status of '0'
            if robots[i].status == 0:
                # process neighbors with stauts '2', highest priority
                if 2 in status_list[i]:
                    # check the group attribution of all the '2'
                    # get the robot id and group id of the first '2'
                    current_index = status_list[i].index(2)
                    current_robot = index_list[i][current_index]
                    current_group = robots[current_robot].group_id
                    group_temp = {current_group:[current_robot]}
                    # check if there is still '2' in the list
                    while 2 in status_list[i][current_index+1:]:
                        # indexing the next '2' from current_index+1
                        # update the current_index, current_robot, current_group
                        current_index = status_list[i].index(2, current_index+1)
                        current_robot = index_list[i][current_index]
                        current_group = robots[current_robot].group_id
                        # update group_temp
                        if current_group in group_temp.keys():
                            # append this robot in the existing group
                            group_temp[current_group].append(current_robot)
                        else:
                            # add new group id in group_temp and add this robot
                            group_temp[current_group] = [current_robot]
                    # check if there are multiple groups detected from the '2'
                    target_robot = -1  # the target robot to grab on
                    if len(group_temp.keys()) == 1:
                        # there is only one group detected
                        dist_min = 2*comm_range  # start with a large dist
                        robot_min = -1  # corresponding robot with min distance
                        # search the closest '2'
                        for j in group_temp.values()[0]:
                            if dist_table[i][j] < dist_min:
                                dist_min = dist_table[i][j]
                                robot_min = j
                        target_robot = robot_min
                    else:
                        # there is more than one group detected
                        # no need to disassemble any group yet
                        # compare which group has the most members in it
                        member_max = 0  # start with 0 number of members
                        group_max = -1  # corresponding group id with most members
                        for j in group_temp.keys():
                            if len(group_temp[j]) > member_max:
                                member_max = len(group_temp[j])
                                group_max = j
                        # search the closeat '2' inside that group
                        dist_min = 2*comm_range
                        robot_min = -1
                        for j in group_temp[group_max]:
                            if dist_table[i][j] < dist_min:
                                dist_min = dist_table[i][j]
                                robot_min = j
                        target_robot = robot_min
                    # target_robot located, prepare to grab on it
                    # status transition scheduled, '0' to '1'
                    s_grab_on[i] = target_robot
                # process neighbors with status '1', second priority
                elif 1 in status_list[i]:
                    # find the closest '1' and get bounced away by it
                    # it doesn't matter if the '1's belong to different group
                    dist_min = 2*comm_range
                    robot_min = -1
                    for j in range(len(status_list[i])):
                        if status_list[i][j] != 1: continue
                        if dist_table[i][index_list[i][j]] < dist_min:
                            dist_min = dist_table[i][index_list[i][j]]
                            robot_min = index_list[i][j]
                    # target robot located, the robot_min
                    # get bounced away from this robot, update the moving direction
                    vec_temp = (robots[i].pos[0] - robots[robot_min].pos[0],
                                robots[i].pos[1] - robots[robot_min].pos[1])
                    # orientation is pointing from robot_min to host
                    robots[i].ori = math.atan2(vec_temp[1], vec_temp[0])
                # process neighbors with status '0', least priority
                else:
                    # establish a list of all '0', in order of increasing distance
                    # to be checked later if grouping is possible
                    # this list should be only '0's, already sorted
                    target_list = index_list[i][:]
                    # status transition scheduled, '0' forming intial group with '0'
                    s_init_form[i] = target_list
            # for the host robot having status of '1'
            elif robots[i].status == 1:
                # status of '1' needs to checked and maintained constantly
                # 1.check if the important group neighbors are still in range
                neighbors_secured = True
                for j in robots[i].key_neighbors:
                    if j not in index_list[i]:
                        neighbors_secured = False
                        break
                if neighbors_secured == False:
                    # status transition scheduled, robot '1' gets lost, becoming '-1'
                    s_lost.append(i)
                else:
                    # all the key neighbors are in good position
                    # 2.dissemble check, get group attribution of all '1' and '2'
                    status_list_temp = status_list[i][:]
                    index_list_temp = index_list[i][:]
                    # pop out the '0' first
                    while 0 in status_list_temp:
                        index_temp = status_list_temp.index(0)
                        status_list_temp.pop(index_temp)
                        index_list_temp.pop(index_temp)
                    # start the group attribution dictionary with first robot
                    group_temp = {robots[index_list_temp[0]].group_id: [index_list_temp[0]]}
                    for j in index_list_temp[1:]:  # then iterating from the second one
                        current_group = robots[j].group_id
                        if current_group in group_temp.keys():
                            # append this robot in same group
                            group_temp[current_group].append(j)
                        else:
                            # add new key in the group_temp dictionary
                            group_temp[current_group] = [j]
                    # check if there are multiple groups detected
                    if len(group_temp.keys()) > 1:
                        # status transition scheduled, to disassemble groups
                        s_disassemble.append([group_temp.keys()])
                        # may produce duplicates in s_disassemble, not big problem
                    # 3.check if any neighbor transition needs to be done
                    if robots[i].status_1_sub == 0:
                        # host robot is in the initial forming phase
                        # check if the neighbor robot is in appropriate distance
                        if abs(dist_table[i][robots[i].key_neighbors[0]] -
                               comm_range) < space_err:
                            # status transition scheduled, finish intial forming, '1' to '2'
                            s_form_done.append(i)
                    elif robots[i].status_1_sub == 1:
                        # host robot is in the climbing phase
                        # check if the grab-on robot is at the begining or end of the line
                        # if yes, no need to search new key neighbor, only check destination
                        if robots[robots[i].key_neighbors[0]].status_2_sequence == 0 or
                           robots[robots[i].key_neighbors[0]].status_2_end == True:
                           # check if reaching the destination coordinates
                            pos_temp = (robots[i].pos[0]-robots[i].status_1_1_des[0],
                                        robots[i].pos[1]-robots[i].status_1_1_des[1])
                            dist_temp = math.sqrt(pos_temp[0]*pos_temp[0] +
                                                  pos_temp[1]*pos_temp[1])
                            if dist_temp < space_err:
                                # status transition scheduled, finish climbing, '1' to '2'
                                s_form_done.append(i)
                        else:
                            # grab-on robot is not at the ends of the line yet, still climbing
                            # check if new key neighbor appears, if yes, update new key neighbor
                            # and update the new destination address
                            id_temp = -1
                            if robots[i].status_1_1_dir == 0:
                                id_temp = groups[robots[i].group_id][2][robots[i].status_2_sequence - 1]
                            else:
                                id_temp = groups[robots[i].group_id][2][robots[i].status_2_sequence + 1]
                            if id_temp in index_list[i]:
                                # update new grab-on robot as the only key neighbor
                                robots[i].key_neighbors = [id_temp]
                                it0 = robots[i].key_neighbors[0]  # 'it' stands for index temp
                                # calculate new destination for the climbing
                                if robots[robots[i].key_neighbors[0]].status_2_sequence == 0:
                                    # if new grab-on robot is at the begining of the line
                                    it1 = groups[robots[i].group_id][2][1]  # second one in the line
                                    robots[i].status_1_1_des = [2*robots[it0].pos[0] - robots[it1].pos[0],
                                                                2*robots[it0].pos[1] - robots[it1].pos[1]]
                                elif robots[robots[i].key_neighbors[0]].status_2_end == True:
                                    # if new grab-on robot is at the end of the line
                                    it1 = groups[robots[i].group_id][2][-2]  # second inversely in the line
                                    robots[i].status_1_1_des = [2*robots[it0].pos[0] - robots[it1].pos[0],
                                                                2*robots[it0].pos[1] - robots[it1].pos[1]]
                                else:  # new grab-on robot is not at any end
                                    it1 = 0  # index of the next promising key neighbor
                                    if robots[i].status_1_1_dir == 0:
                                        it1 = groups[robots[i].group_id][2][robots[robots[i].key_neighbors[0]].status_2_sequence - 1]
                                    else:
                                        it1 = groups[robots[i].group_id][2][robots[robots[i].key_neighbors[0]].status_2_sequence + 1]
                                    # direction from current key neighbor to next promising key neighbor
                                    dir_temp = math.atan2((robots[it1].pos[1]-robots[it0].pos[1]),
                                                          (robots[it1].pos[0]-robots[it0].pos[0]))
                                    # direction from next promising key neighbor to new destination
                                    if robots[i].status_1_1_side == 0:
                                        # climbing at the left of the line, rotate ccw of pi/2
                                        dir_temp = dir_temp + math.pi/2
                                    else:
                                        dir_temp = dir_temp - math.pi/2  # rotate cw of pi/2
                                    # calculate new destination address
                                    robots[i].status_1_1_des = [robots[it1].pos[0]+climb_space*math.cos(dir_temp),
                                                                robots[it1].pos[1]+climb_space*math.sin(dir_temp)]
            # for the host robot having status of '2'
            elif robots[i].status == 2:
                # it's ok to check if key neighbors are still in range
                # but key neighbors of '2' are also '2', all of them are static
                # so skip this step
                # 1.dissemble check, from all the '1' and '2'
                status_list_temp = status_list[i][:]
                index_list_temp = index_list[i][:]
                # pop out the '0' first
                while 0 in status_list_temp:
                    index_temp = status_list_temp.index(0)
                    status_list_temp.pop(index_temp)
                    index_list_temp.pop(index_temp)
                # start the group attribution dictionary with first robot
                group_temp = {robots[index_list_temp[0]].group_id: [index_list_temp[0]]}
                for j in index_list_temp[1:]:  # then iterating from the second one
                    current_group = robots[j].group_id
                    if current_group in group_temp:
                        group_temp[current_group].append(j)
                    else:
                        group_temp[current_group] = [j]
                # check if there are multiple groups detected
                if len(group_temp.keys()) > 1:
                    # status transition scheduled, to disassemble groups
                    s_disassemble.append([group_temp.keys()])
            # for the host robot having status of '-1'
            else:
                # do nothing

        # process the scheduled status change


        # update the position, and wall bouncing


    pygame.display.update()

pygame.quit()


