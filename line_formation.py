# line formation simulation of swarm robots

# comments for research purpose:
# My previous line formation simulations in ROS utilize the relative positions of nearby
# neighbors, and linear fit a line and move to the desired position. It only works when
# there are many robots within range or robot has a large sensing range, and preferrably
# the swarm is aggregated. No communication is required though.
# In this simulation, I extended the initial situation such that robots are in random
# positions in a bounded area. They can move around and communicate with robot in range.
# If not in a bounded area, they might just run too far away without knowing it. All
# communications are single hops, each robot only gives the other robot information about
# itself, not information it gets from a third robot. That means all information is first
# hand. The idea is that, start initial wondering, each robot is trying to form the first
# line segment, or find a line it can contribute to, depends on which comes first. There
# will be more than one line being formed. To control which line should survice, each line
# is given a time of life, it decreases by time and increases when a new member join the
# line. The line disassembles when it runs out of life time, and never expire if the number
# of members exceed half of the total number of robots in the simulation, and becomes the
# dominant group. Another way a line disassembles itself is, when a line detects the exist
# of another line, the line with less members will be disassembled.

# comments for programming purpose:
# Four statuses have been designed to serve different stages of the robot in the simulation:
    # '0': free, wondering around, not in group, available to form a group or join one
    # '1': forming an initial line segment, or found a line and trying to climbing to ends
        # '1_0': initial an line segment, '0' found another '0' and formed a group
        # '1_1': join a line, climbing to the closer end
    # '2': already in a line, remain in position
    # '-1': free, wondering around, no interaction with nearby robots
# Only allowed status transitions are:
    # forward: '-1'->'0', '0'->'1', '1'->'2'
    # backward: '1'->'-1', '2'->'-1'
# Purpose of '-1' is, when a line is disassembled, if '1' and '2' becomes '0', they will
# immediately form the old line because they are already in position. So '-1' which has no
# interaction with nearly robots, acts as a transition to status '0'.

# line formation runs out of boundary:
# It's possible that a line is trying to continue out of the boundary. This is no good way
# to solve it, if keeping the proposed control algorithm. But reducing the occurrence can
# be done, by enabling a relatively large wall detection range. Robot will be running away
# before it really hits the wall, so it's like smaller virtual boundaries inside the window.
# But this virtual boundaries only work on robot '0', such that robot '1' or '2' wants to
# form a line out of virtual boundaries, it can.


import pygame
import math, random
from line_formation_robot import LFRobot
from line_formation_functions import *

pygame.init()  # initialize pygame

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
pygame.display.set_caption('Line Formation Simulation')

# for physics, origin is at left bottom corner, starting (0, 0)
# it's more natural to do the calculation in right hand coordiante system
# the continuous, physical world for the robots
world_size = (100.0, 100.0 * screen_size[1]/screen_size[0])

# variables to configure the simulation
robot_quantity = 30
# coefficient to resize the robot distribution, but always keep close to center
distrib_coef = 0.5
const_vel = 3.0  # all moving robots are moving at a constant speed
frame_period = 100  # updating period of the simulation and graphics, in ms
comm_range = 5.0  # communication range, the radius
line_space = comm_range * 0.7  # a little more than half the communication range
space_err = line_space * 0.1  # the error to determine the space is good
climb_space = line_space * 0.5  # climbing is half the line space along the line
life_incre = 8  # each new member add these seconds to the life of the group
group_id_upper_limit = 1000  # random integer as group id from 0 to this limit
n1_life_lower = 3  # lower limit of life time of being status '-1'
n1_life_upper = 8  # upper limit of life time of being status '-1'
vbound_gap_ratio = 0.15  # for the virtual boundaries inside the window
    # ratio is the gap distance to the world size on that axis

# instantiate the robot swarm as list
robots = []  # container for all robots, index is also the identification
for i in range(robot_quantity):
    # random position, away from the window edges
    pos_temp = (((random.random()-0.5)*distrib_coef+0.5) * world_size[0],
                ((random.random()-0.5)*distrib_coef+0.5) * world_size[1])
    vel_temp = const_vel
    ori_temp = random.random() * 2*math.pi - math.pi  # random in (-pi, pi)
    object_temp = LFRobot(pos_temp, vel_temp, ori_temp)
    robots.append(object_temp)
# instantiate the group variable as dictionary
groups = {}
    # key is the group id, so two groups won't share same id
    # value is a list
        # 0.first element: the group size, including both status '2' and '1'
        # 1.second element: life time remaining
        # 2.third element: a list of robots on the line in adjacent order, status '2'
        # 3.forth element: a list of robots off the line, not in order, status '1'
        # 4.fifth element: true or false, being the dominant group

# instantiate a distance table for every pair of robots
# will calculate once for symmetric data
dist_table = [[-1.0 for j in range(robot_quantity)] for i in range(robot_quantity)]

# calculate the corner positions of virtual boundaries, for display
# left bottom corner
pos_lb = lf_world_to_display([world_size[0]*vbound_gap_ratio,
                              world_size[1]*vbound_gap_ratio], world_size, screen_size)
# right bottom corner
pos_rb = lf_world_to_display([world_size[0]*(1-vbound_gap_ratio),
                              world_size[1]*vbound_gap_ratio], world_size, screen_size)
# right top corner
pos_rt = lf_world_to_display([world_size[0]*(1-vbound_gap_ratio),
                              world_size[1]*(1-vbound_gap_ratio)], world_size, screen_size)
# left top corner
pos_lt = lf_world_to_display([world_size[0]*vbound_gap_ratio,
                              world_size[1]*(1-vbound_gap_ratio)], world_size, screen_size)

# the loop
sim_exit = False  # simulation exit flag
sim_pause = False  # simulation pause flag, pause at begining
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

        # instantiate the status transition variables, prepare for status check
        # the priority to process them is in the following order
        s_grab_on = {}  # robot '0' grabs on robot '2', becoming '1'
            # key is id of robot '0', value is id of robot '2'
        s_init_form = {}  # robot '0' initial forms with another '0', becoming '1'
            # key is id of robot '0' that discovers other robot '0' in range
            # value is a list of robot '0's that are in range, in order of increasing distance
        s_form_done = {}  # robot '1' finishes initial forming, becoming '2'
            # key is group id
            # value is a list of id of robot '1's that have finished initial forming
        s_climb_done = {}  # robot '1' finishes climbing, becoming '2'
            # key is group id
            # value is a list of id of robot '1's that have finished climbing
        s_form_lost = []  # robot '1' gets lost during initial forming
            # list of group id for the initial forming robots
        s_climb_lost = []  # robot '1' gets lost during climbing
            # list of robot id for the climbing robots
        s_group_exp = []  # life time of a group naturally expires
            # life of group id
        s_disassemble = []  # disassemble triggerred by robot '1' or '2'
            # list of lists of group id to be compared for disassembling
        s_back_0 = []  # robot '-1' gets  back to '0'
            # list of robot id

        # check 'robots' for any status change, and schedule and process in next step
        for i in range(robot_quantity):
            # for the host robot having status of '0'
            if robots[i].status == 0:
                # check if this robot has valid neighbors at all
                if len(index_list[i]) == 0: continue;  # skip the neighbor check
                # process neighbors with stauts '2', highest priority
                if 2 in status_list[i]:
                    # check the group attribution of all the '2'
                    # get the robot id and group id of the first '2'
                    current_index = status_list[i].index(2)
                    current_robot = index_list[i][current_index]
                    current_group = robots[current_robot].group_id
                    groups_temp = {current_group:[current_robot]}
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
                            groups_temp[current_group] = [current_robot]
                    # check if there are multiple groups detected from the '2'
                    target_robot = -1  # the target robot to grab on
                    if len(groups_temp.keys()) == 1:
                        # there is only one group detected
                        dist_min = 2*comm_range  # start with a large dist
                        robot_min = -1  # corresponding robot with min distance
                        # search the closest '2'
                        for j in groups_temp.values()[0]:
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
                        for j in groups_temp.keys():
                            if len(groups_temp[j]) > member_max:
                                member_max = len(groups_temp[j])
                                group_max = j
                        # search the closeat '2' inside that group
                        dist_min = 2*comm_range
                        robot_min = -1
                        for j in groups_temp[group_max]:
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
                    vect_temp = (robots[i].pos[0] - robots[robot_min].pos[0],
                                robots[i].pos[1] - robots[robot_min].pos[1])
                    # orientation is pointing from robot_min to host
                    robots[i].ori = math.atan2(vect_temp[1], vect_temp[0])
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
                    if robots[i].status_1_sub == 0:
                        # append the group id, disassemble the entire group
                        s_form_lost.append(robots[i].group_id)
                    else:
                        s_climb_lost.append(i)  # append the robot id
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
                    if len(index_list_temp) > 0:  # ensure there are in-group robots around
                        # start the group attribution dictionary with first robot
                        groups_temp = {robots[index_list_temp[0]].group_id: [index_list_temp[0]]}
                        for j in index_list_temp[1:]:  # then iterating from the second one
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
                            # may produce duplicates in s_disassemble, not big problem
                    # 3.check if any neighbor transition needs to be done
                    if robots[i].status_1_sub == 0:
                        # host robot is in the initial forming phase
                        # check if the neighbor robot is in appropriate distance
                        if abs(dist_table[i][robots[i].key_neighbors[0]] -
                               line_space) < space_err:
                            # status transition scheduled, finish intial forming, '1' to '2'
                            g_it = robots[i].group_id
                            if g_it in s_form_done.keys():
                                s_form_done[g_it].append(i)
                            else:
                                s_form_done[g_it] = [i]
                    elif robots[i].status_1_sub == 1:
                        # host robot is in the climbing phase
                        # check if the grab-on robot is at the begining or end of the line
                        # if yes, no need to search new key neighbor, only check destination
                        if (robots[robots[i].key_neighbors[0]].status_2_sequence == 0 or
                            robots[robots[i].key_neighbors[0]].status_2_end == True):
                            # check if reaching the destination coordinates
                            vect_temp = (robots[i].pos[0]-robots[i].status_1_1_des[0],
                                         robots[i].pos[1]-robots[i].status_1_1_des[1])
                            dist_temp = math.sqrt(vect_temp[0]*vect_temp[0] +
                                                  vect_temp[1]*vect_temp[1])
                            if dist_temp < space_err:
                                # status transition scheduled, finish climbing, '1' to '2'
                                g_it = robots[i].group_id
                                if g_it in s_climb_done.keys():
                                    s_climb_done[g_it].append(i)
                                else:
                                    s_climb_done[g_it] = [i]
                        else:
                            # grab-on robot is not at any end of the line yet, still climbing
                            # check if new key neighbor appears, if yes, update new key neighbor
                            # and update the new destination address
                            it0 = robots[i].key_neighbors[0]  # initialize with the old key neighbor
                                # 'it' stands for index temp
                            if robots[i].status_1_1_dir == 0:
                                # assign the new key neighbor
                                it0 = groups[robots[i].group_id][2][robots[it0].status_2_sequence - 1]
                            else:
                                it0 = groups[robots[i].group_id][2][robots[it0].status_2_sequence + 1]
                            if it0 in index_list[i]:
                                # update new grab-on robot as the only key neighbor
                                robots[i].key_neighbors = [it0]
                                # calculate new destination for the climbing
                                if robots[it0].status_2_sequence == 0:
                                    # if new grab-on robot is at the begining of the line
                                    it1 = groups[robots[i].group_id][2][1]  # second one in the line
                                    # to calculate the new destination, should not just mirror the des
                                    # from 'it1' with 'it0', it is not precise enough, error will accumulate
                                    # robots[i].status_1_1_des = [2*robots[it0].pos[0] - robots[it1].pos[0],
                                    #                             2*robots[it0].pos[1] - robots[it1].pos[1]]
                                    # ori_temp is pointing from 'it1' to 'it0'
                                    ori_temp = math.atan2(robots[it0].pos[1]-robots[it1].pos[1],
                                                          robots[it0].pos[0]-robots[it1].pos[0])
                                    robots[i].status_1_1_des = [robots[it0].pos[0] + line_space*math.cos(ori_temp),
                                                                robots[it0].pos[1] + line_space*math.sin(ori_temp)]
                                elif robots[it0].status_2_end == True:
                                    # if new grab-on robot is at the end of the line
                                    it1 = groups[robots[i].group_id][2][-2]  # second inversely in the line
                                    # robots[i].status_1_1_des = [2*robots[it0].pos[0] - robots[it1].pos[0],
                                    #                             2*robots[it0].pos[1] - robots[it1].pos[1]]
                                    ori_temp = math.atan2(robots[it0].pos[1]-robots[it1].pos[1],
                                                          robots[it0].pos[0]-robots[it1].pos[0])
                                    robots[i].status_1_1_des = [robots[it0].pos[0] + line_space*math.cos(ori_temp),
                                                                robots[it0].pos[1] + line_space*math.sin(ori_temp)]
                                else:  # new grab-on robot is not at any end
                                    it1 = 0  # index of the next promising key neighbor
                                    if robots[i].status_1_1_dir == 0:
                                        it1 = groups[robots[i].group_id][2][robots[it0].status_2_sequence - 1]
                                    else:
                                        it1 = groups[robots[i].group_id][2][robots[it0].status_2_sequence + 1]
                                    # direction from current key neighbor(it0) to next promising key neighbor(it1)
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
                # dissemble check, for all the '1' and '2'
                status_list_temp = status_list[i][:]
                index_list_temp = index_list[i][:]
                # pop out the '0' first
                while 0 in status_list_temp:
                    index_temp = status_list_temp.index(0)
                    status_list_temp.pop(index_temp)
                    index_list_temp.pop(index_temp)
                # start the group attribution dictionary with first robot
                if len(index_list_temp) > 0:  # ensure there are in-group robots around
                    groups_temp = {robots[index_list_temp[0]].group_id: [index_list_temp[0]]}
                    for j in index_list_temp[1:]:  # then iterating from the second one
                        current_group = robots[j].group_id
                        if current_group in groups_temp:
                            groups_temp[current_group].append(j)
                        else:
                            groups_temp[current_group] = [j]
                    # check if there are multiple groups detected
                    if len(groups_temp.keys()) > 1:
                        # status transition scheduled, to disassemble groups
                        s_disassemble.append(groups_temp.keys())
            # for the host robot having status of '-1'
            elif robots[i].status == -1:
                # check if life time expires, and get status back to '0'
                if robots[i].status_n1_life < 0:
                    s_back_0.append(i)

        # check 'groups' for any status change
        for g_it in groups.keys():
            if groups[g_it][4]: continue  # already being dominant
            if groups[g_it][0] > robot_quantity/2:
                # the group has more than half the total number of robots
                groups[g_it][4] = True  # becoming dominant
                groups[g_it][1] = 100.0  # a random large number
            if groups[g_it][1] < 0:  # life time of a group expires
                # schedule operation to disassemble this group
                s_group_exp.append(g_it)

        # process the scheduled status change, in the order of the priority
        # 1.s_grab_on, robot '0' grabs on robot '2', becoming '1'
        for i in s_grab_on.keys():
            # 1.update the 'robots' variable
            robots[i].status = 1  # status becoming '1'
            it0 = s_grab_on[i]  # for the robot being grabed on, new key neighbor
                # ie., robot 'i' grabs on robot 'it0'
            robots[i].key_neighbors = [it0]  # update the key neighbor
            g_it = robots[it0].group_id  # for the new group id, group index temp
            robots[i].group_id = g_it  # update group id
            robots[i].status_1_sub = 1  # sub status '1' for climbing
            # 2.update the 'groups' variable
            groups[g_it][0] = groups[g_it][0] + 1  # increase group size by 1
            groups[g_it][1] = groups[g_it][1] + life_incre  # increase life time
            groups[g_it][3].append(i)
            # 3.deciding the climbing direction
            if robots[it0].status_2_sequence > (len(groups[g_it][2])-1)/2:
                robots[i].status_1_1_dir = 1
            else:
                robots[i].status_1_1_dir = 0
            # 4.deciding which side the robot is climbing at
            if robots[i].status_1_1_dir == 0:  # search backward for the next robot
                it1 = groups[g_it][2][robots[it0].status_2_sequence + 1]
            else:
                it1 = groups[g_it][2][robots[it0].status_2_sequence - 1]
            # by calculating the determinant of vectors 'it1->i' and 'it1->it0'
            if ((robots[it0].pos[0]-robots[it1].pos[0])*(robots[i].pos[1]-robots[it1].pos[1]) -
                (robots[it0].pos[1]-robots[it1].pos[1])*(robots[i].pos[0]-robots[it1].pos[0])) >= 0:
                # it's rare that the above determinant should equal to 0
                robots[i].status_1_1_side = 0  # at left side
            else:
                robots[i].status_1_1_side = 1
            # 5.calculate the initial destination
            if robots[it0].status_2_sequence == 0:
                it1 = groups[g_it][2][1]  # adjacent robot id
                robots[i].status_1_1_des = [2*robots[it0].pos[0] - robots[it1].pos[0],
                                            2*robots[it0].pos[1] - robots[it1].pos[1]]
            elif robots[it0].status_2_end == True:
                it1 = groups[g_it][2][-2]  # adjacent robot id
                robots[i].status_1_1_des = [2*robots[it0].pos[0] - robots[it1].pos[0],
                                            2*robots[it0].pos[1] - robots[it1].pos[1]]
            else:
                # robot it0 is not at the begining or end of the line
                it1 = 0
                if robots[i].status_1_1_dir == 0:
                    it1 = groups[robots[i].group_id][2][robots[it0].status_2_sequence - 1]
                else:
                    it1 = groups[robots[i].group_id][2][robots[it0].status_2_sequence + 1]
                # direction from current key neighbor(it0) to next promising key neighbor(it1)
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
            # update the moving orientation
            robots[i].ori = math.atan2(robots[i].status_1_1_des[1]-robots[i].pos[1],
                                       robots[i].status_1_1_des[0]-robots[i].pos[0])
        # 2.s_init_form, robot '0' initial forms with another '0', becoming '1'
        s_pair = []  # the container for finalized initial forming pairs
        while len(s_init_form.keys()) != 0:  # there are still robots to be processed
            for i in s_init_form.keys():
                it = s_init_form[i][0]  # index temp
                if it in s_init_form.keys():
                    if s_init_form[it][0] == i:  # robot 'it' also recognizes 'i' as closest
                        s_pair.append([i, it])
                        s_init_form.pop(i)  # pop out both 'i' and 'it'
                        s_init_form.pop(it)
                        break
                    elif i not in s_init_form[it]:
                        # usually should not be here unless robots have different sensing range
                        s_init_form[i].remove(it)
                        if len(s_init_form[i]) == 0:
                            s_init_form.pop(i)
                        break
                    # have not consider the situation that 'i' in 'it' but not first one
                else:
                    # will be here if robot 'it' chooses to be bounced away by '1', or grab on '2'
                    s_init_form[i].remove(it)
                    if len(s_init_form[i]) == 0:
                        s_init_form.pop(i)
                    break
        # process the finalized pairs
        for pair in s_pair:
            it0 = pair[0]  # get indices of the pair
            it1 = pair[1]
            g_it = random.randint(0, group_id_upper_limit)
            while g_it in groups.keys():
                g_it = random.randint(0, group_id_upper_limit)  # generate again
            # update the 'robots' variable
            robots[it0].status = 1
            robots[it1].status = 1
            robots[it0].group_id = g_it
            robots[it1].group_id = g_it
            robots[it0].status_1_sub = 0  # sub status '0' for initial climbing
            robots[it1].status_1_sub = 0
            robots[it0].key_neighbors = [it1]  # name the other as the key neighbor
            robots[it1].key_neighbors = [it0]
            # update the 'groups' variable
            groups[g_it] = [2, 2*life_incre, [], [it0, it1], False]  # add new entry
            # deciding moving direction for the initial forming robots
            vect_temp = (robots[it1].pos[0]-robots[it0].pos[0],
                         robots[it1].pos[1]-robots[it0].pos[1])  # pointing from it0 to it1
            ori_temp = math.atan2(vect_temp[1], vect_temp[0])
            if dist_table[it0][it1] > line_space:  # equally dist_table[it1][it0]
                # attracting each other, most likely this happens
                robots[it0].ori = ori_temp
                robots[it1].ori = lf_reset_radian(ori_temp + math.pi)
            else:
                # repelling each other
                robots[it0].ori = lf_reset_radian(ori_temp + math.pi)
                robots[it1].ori = ori_temp
            # no need to check if they are already within the space error of line space
            # this will be done in the next round of status change check
        # 3.s_form_done, robot '1' finishes intial forming, becoming '2'
        for g_it in s_form_done.keys():
            if len(s_form_done[g_it]) == 2:  # double check, both robots agree forming is done
                it0 = s_form_done[g_it][0]
                it1 = s_form_done[g_it][1]
                # update 'robots' variable for 'it0' and 'it1'
                robots[it0].status = 2
                robots[it1].status = 2
                # randomly deciding which is begining and end of the line, random choice
                if random.random() > 0.5:  # half chance
                    # swap 'ito' and 'it1', 'it0' as begining, 'it1' as end
                    temp = it0
                    it0 = it1
                    it1 = temp
                # update the 'robots' variable
                robots[it0].status_2_sequence = 0
                robots[it1].status_2_sequence = 1
                robots[it0].status_2_end = False
                robots[it1].status_2_end = True
                robots[it0].key_neighbors = [it1]  # can skip this
                robots[it1].key_neighbors = [it0]
                # update the 'groups' variable
                g_it = robots[it0].group_id
                groups[g_it][2] = [it0, it1]  # add the robots for the line
                groups[g_it][3] = []  # empty the forming & climbing pool
        # 4.s_climb_done, robot '1' finishes climbing, becoming '2'
        for g_it in s_climb_done.keys():
            start_pool = []  # for robots finish at the begining
            end_pool = []  # for robots finish at the end
            for i in s_climb_done[g_it]:
                if robots[i].status_1_1_dir == 0:
                    start_pool.append(i)
                else:
                    end_pool.append(i)
            if len(start_pool) > 0:  # start pool not empty
                # most likely there is only one robot in the pool
                # randomly choose one just in case, may not be the best way
                it0 = random.choice(start_pool)
                # no need to care the not selected, they will continue climbing
                # upate the 'robots' variable
                robots[it0].status = 2
                robots[it0].status_2_sequence = 0  # make it begining of the line
                robots[it0].status_2_end = False
                robots[it0].key_neighbors = [groups[g_it][2][0]]
                robots[groups[g_it][2][0]].key_neighbors.append(it0)  # add a new key neighbor
                # update the 'groups' variable
                groups[g_it][3].remove(it0)  # remove from the climbing pool
                groups[g_it][2].insert(0, it0)  # insert new index at the begining
                # increase the status_2_sequence of other robots on the line by 1
                for i in groups[g_it][2][1:]:
                    robots[i].status_2_sequence = robots[i].status_2_sequence + 1
            if len(end_pool) > 0:  # end pool not empty
                it0 = random.choice(end_pool)
                # update the 'robots' variable
                robots[it0].status = 2
                robots[it0].status_2_sequence = len(groups[g_it][2])
                robots[it0].status_2_end = True
                it1 = groups[g_it][2][-1]  # the old end of the line
                robots[it1].status_2_end = False
                robots[it0].key_neighbors = [it1]
                robots[it1].key_neighbors.append(it0)
                # update the 'groups' variable
                groups[g_it][3].remove(it0)  # remove from the climbing pool
                groups[g_it][2].append(it0)  # append 'it0' at the end of the line
        # 5.s_form_lost, robot '1' gets lost during initial forming
        for g_it in s_form_lost:
            # can disassemble the group here, but may as well do it together in s_disassemble
            s_disassemble.append([g_it])
        # 6.s_climb_lost, robot '1' gets lost during climbing, becoming '-1'
        for i in s_climb_lost:
            # update the 'robots' variable
            robots[i].status = -1
            # a new random moving orientation
            robots[i].ori = random.random() * 2*math.pi - math.pi
            # a new random life time
            robots[i].status_n1_life = random.randint(n1_life_lower, n1_life_upper)
            # update the 'groups' variable
            g_it = robots[i].group_id
            groups[g_it][0] = groups[g_it][0] - 1  # reduce group size by 1
            # life time of a group doesn't decrease due to member lost
            groups[g_it][3].remove(i)  # remove the robot from the pool list
        # 7.s_group_exp, natural life expiration of the groups
        for g_it in s_group_exp:
            s_disassemble.append([g_it])  # leave it to s_disassemble
        # 8.s_disassemble, triggered by robot '1' or '2'
        s_dis_list = []  # list of group id that has been finalized for disassembling
        # compare number of members to decide which groups to disassemble
        for gs_it in s_disassemble:
            if len(gs_it) == 1:
                # from the s_form_lost
                if gs_it[0] not in s_dis_list:
                    s_dis_list.append(gs_it[0])
            else:
                # compare which group has the most members, and disassemble the rest
                g_temp = gs_it[:]
                member_max = 0  # number of members in the group
                group_max = -1  # corresponding group id with most members
                for g_it in g_temp:
                    if groups[g_it][0] > member_max:
                        member_max = groups[g_it][0]
                        group_max = g_it
                g_temp.remove(group_max)  # remove the group with the most members
                for g_it in g_temp:
                    if g_it not in s_dis_list:  # avoid multiple occurrence
                        s_dis_list.append(g_it)
        # start disassembling
        for g_it in s_dis_list:
            # update the 'robots' variable
            for i in groups[g_it][3]:  # for robots off the line
                robots[i].status = -1
                # give a random (integer) life time in designed range
                robots[i].status_n1_life = random.randint(n1_life_lower, n1_life_upper)
                robots[i].ori = random.random() * 2*math.pi - math.pi
            # give a random moving orientation is still not good enough
            # a new way for the robots on the line is 'explode' the first half to one side
            # and 'explode' the second half to the other side.
            num_online = len(groups[g_it][2])  # number of robots on the line
            num_1_half = num_online/2  # number of robots for the first half
            num_2_half = num_online - num_1_half  # number of robots for the second half
            for i in groups[g_it][2]:  # for robots on the line
                robots[i].status = -1
                robots[i].status_n1_life = random.randint(n1_life_lower, n1_life_upper)
                # robots[i].ori = random.random() * 2*math.pi - math.pi
                # first deciding the direction of the line, always pointing to the closer end
                ori_temp = 0
                seq_temp = robots[i].status_2_sequence  # sequence on the line
                if robots[i].status_2_sequence == 0:
                    # then get second one on the line
                    it0 = groups[robots[i].group_id][2][1]
                    # orientation points to the small index end
                    ori_temp = math.atan2(robots[i].pos[1]-robots[it0].pos[1],
                                          robots[i].pos[0]-robots[it0].pos[0])
                elif robots[i].status_2_end == True:
                    # then get inverse second one on the line
                    it0 = groups[robots[i].group_id][2][-2]
                    # orientation points to the large index end
                    ori_temp = math.atan2(robots[i].pos[1]-robots[it0].pos[1],
                                          robots[i].pos[0]-robots[it0].pos[0])
                else:
                    # get both neighbors
                    it0 = groups[g_it][2][seq_temp - 1]
                    it1 = groups[g_it][2][seq_temp + 1]
                    if seq_temp < num_1_half:  # robot in the first half
                        # orientation points to the small index end
                        ori_temp = math.atan2(robots[it0].pos[1]-robots[it1].pos[1],
                                              robots[it0].pos[0]-robots[it1].pos[0])
                    else:
                        # orientation points to the large index end
                        ori_temp = math.atan2(robots[it1].pos[1]-robots[it0].pos[1],
                                              robots[it1].pos[0]-robots[it0].pos[0])
                # then calculate the 'explosion' direction
                if seq_temp < num_1_half:
                    robots[i].ori = lf_reset_radian(ori_temp + math.pi*(seq_temp+1)/(num_1_half+1))
                    # always 'explode' to the left side
                else:
                    robots[i].ori = lf_reset_radian(ori_temp + math.pi*(num_online-seq_temp)/(num_2_half+1))
            # pop out this group in 'groups'
            groups.pop(g_it)
        # 9.s_back_0, life time of robot '-1' expires, becoming '0'
        for i in s_back_0:
            # still maintain the old moving direction
            robots[i].status = 0

        # update the physics(pos and ori), and wall bouncing, life decrease of '-1'
        for i in range(robot_quantity):
            if robots[i].status == 2:
                continue  # every robot moves except '2'
            # check if out of boundaries, algorithms revised from 'experiment_3_automaton'
            # change only direction of velocity
            if robots[i].status != 0:
                # for the robots '-1', '1' and '2', bounded by the real boundaries
                if robots[i].pos[0] >= world_size[0]:  # out of right boundary
                    if math.cos(robots[i].ori) > 0:  # velocity on x is pointing right
                        robots[i].ori = lf_reset_radian(2*(math.pi/2) - robots[i].ori)
                elif robots[i].pos[0] <= 0:  # out of left boundary
                    if math.cos(robots[i].ori) < 0:  # velocity on x is pointing left
                        robots[i].ori = lf_reset_radian(2*(math.pi/2) - robots[i].ori)
                if robots[i].pos[1] >= world_size[1]:  # out of top boundary
                    if math.sin(robots[i].ori) > 0:  # velocity on y is pointing up
                        robots[i].ori = lf_reset_radian(2*(0) - robots[i].ori)
                elif robots[i].pos[1] <= 0:  # out of bottom boundary
                    if math.sin(robots[i].ori) < 0:  # velocity on y is pointing down
                        robots[i].ori = lf_reset_radian(2*(0) - robots[i].ori)
            else:
                # for robot '0' only, bounded in a smaller area
                # so more lines will be formed in the center area, and line won't run out
                if robots[i].pos[0] >= world_size[0]*(1-vbound_gap_ratio):
                    if math.cos(robots[i].ori) > 0:
                        robots[i].ori = lf_reset_radian(2*(math.pi/2) - robots[i].ori)
                elif robots[i].pos[0] <= world_size[0]*vbound_gap_ratio:
                    if math.cos(robots[i].ori) < 0:
                        robots[i].ori = lf_reset_radian(2*(math.pi/2) - robots[i].ori)
                if robots[i].pos[1] >= world_size[1]*(1-vbound_gap_ratio):
                    if math.sin(robots[i].ori) > 0:
                        robots[i].ori = lf_reset_radian(2*(0) - robots[i].ori)
                elif robots[i].pos[1] <= world_size[1]*vbound_gap_ratio:
                    if math.sin(robots[i].ori) < 0:
                        robots[i].ori = lf_reset_radian(2*(0) - robots[i].ori)
            # update one step of distance
            travel_dist = robots[i].vel * frame_period/1000.0
            robots[i].pos[0] = robots[i].pos[0] + travel_dist*math.cos(robots[i].ori)
            robots[i].pos[1] = robots[i].pos[1] + travel_dist*math.sin(robots[i].ori)
            # update direction of velocity for robot '1',  conflict with boundary check?
            if robots[i].status == 1:
                if robots[i].status_1_sub == 0:
                    it0 = robots[i].key_neighbors[0]
                    vect_temp = (robots[it0].pos[0]-robots[i].pos[0],
                                 robots[it0].pos[1]-robots[i].pos[1])  # pointing from i to it0
                    ori_temp = math.atan2(vect_temp[1], vect_temp[0])
                    if dist_table[i][it0] > line_space:  # it's ok to use the out dated dist_table
                        # pointing toward the neighbor
                        robots[i].ori = ori_temp
                    else:
                        # pointing opposite the neighbor
                        robots[i].ori = lf_reset_radian(ori_temp + math.pi)
                else:
                    robots[i].ori = math.atan2(robots[i].status_1_1_des[1]-robots[i].pos[1],
                                               robots[i].status_1_1_des[0]-robots[i].pos[0])
            # decrease life time of robot with status '-1'
            if robots[i].status == -1:
                robots[i].status_n1_life = robots[i].status_n1_life - frame_period/1000.0
        # life time decrease of the groups
        for g_it in groups.keys():
            if groups[g_it][4]: continue  # not decrease life of the dominant
            groups[g_it][1] = groups[g_it][1] - frame_period/1000.0

        # graphics update
        screen.fill(background_color)
        # draw the virtual boundaries
        pygame.draw.lines(screen, (255, 255, 255), True, [pos_lb, pos_rb, pos_rt, pos_lt], 1)
        # draw the robots
        for i in range(robot_quantity):
            display_pos = lf_world_to_display(robots[i].pos, world_size, screen_size)
            # get color of the robot
            color_temp = ()
            if robots[i].status == 0:
                color_temp = robot_0_color
            elif robots[i].status == 1:
                color_temp = robot_1_color
            elif robots[i].status == 2:
                color_temp = robot_2_color
            elif robots[i].status == -1:
                color_temp = robot_n1_color
            # draw the robot as a small solid circle
            pygame.draw.circle(screen, color_temp, display_pos, robot_size, 0)  # fill the circle
        pygame.display.update()

pygame.quit()


