# script to generate files for the loop shapes
# each block of code generates one loop shape

import pickle
import math
import pygame
import numpy as np

# general function to reset radian angle to [-pi, pi)
def reset_radian(radian):
    while radian >= math.pi:
        radian = radian - 2*math.pi
    while radian < -math.pi:
        radian = radian + 2*math.pi
    return radian

# general function to calculate next position node along a heading direction
def cal_next_node(node_poses, index_curr, heading_angle, rep_times):
    for _ in range(rep_times):
        index_next = index_curr + 1
        x = node_poses[index_curr][0] + 1.0*math.cos(heading_angle)
        y = node_poses[index_curr][1] + 1.0*math.sin(heading_angle)
        node_poses[index_next] = np.array([x,y])
        index_curr = index_next
    return index_next

# ##### script to generate 30-square #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# x = 0.0
# y = 0.0
# # bottom side line
# # first node starting from bottom left corner
# node_poses[0] = np.array([0.0, 0.0])
# for i in range(1,8):
#     x = x + 1.0
#     node_poses[i] = np.array([x,y])
# # right line line
# x = x + 1.0 / math.sqrt(2)
# y = y + 1.0 / math.sqrt(2)
# node_poses[8] = np.array([x,y])
# for i in range(9,16):
#     y = y + 1.0
#     node_poses[i] = np.array([x,y])
# # top side line
# for i in range(16,23):
#     x = x - 1.0
#     node_poses[i] = np.array([x,y])
# # left side line
# x = x - 1.0 / math.sqrt(2)
# y = y - 1.0 / math.sqrt(2)
# node_poses[23] = np.array([x,y])
# for i in range(24,30):
#     y = y - 1.0
#     node_poses[i] = np.array([x,y])
# print("node_poses: {}".format(node_poses))
# with open('30-square', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-square #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# x = 0.0
# y = 0.0
# # bottom side line
# node_poses[0] = np.array([0.0, 0.0])
# for i in range(1,26):
#     x = x + 1.0
#     node_poses[i] = np.array([x,y])
# # right line line
# for i in range(26,51):
#     y = y + 1.0
#     node_poses[i] = np.array([x,y])
# # top side line
# for i in range(51,76):
#     x = x - 1.0
#     node_poses[i] = np.array([x,y])
# # left side line
# for i in range(76,100):
#     y = y - 1.0
#     node_poses[i] = np.array([x,y])
# print("node_poses: {}".format(node_poses))
# with open('100-square', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-circle #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# center = np.zeros(2)
# radius = 0.5 / math.sin(math.pi/swarm_size)
# # first node starting from left most position
# for i in range(swarm_size):
#     ori = -math.pi + 2*math.pi/swarm_size * i
#     node_poses[i] = center + np.array([radius*math.cos(ori), radius*math.sin(ori)])
# print("node_poses: {}".format(node_poses))
# with open('30-circle', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-circle #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# center = np.zeros(2)
# radius = 0.5 / math.sin(math.pi/swarm_size)
# # first node starting from left most position
# for i in range(swarm_size):
#     ori = -math.pi + 2*math.pi/swarm_size * i
#     node_poses[i] = center + np.array([radius*math.cos(ori), radius*math.sin(ori)])
# print("node_poses: {}".format(node_poses))
# with open('100-circle', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-triangle #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# # first node is at bottom left corner
# x = 0.0
# y = 0.0
# node_poses[0] = np.array([x,y])
# for i in range(1,11):
#     x = x + 1.0
#     node_poses[i] = np.array([x,y])
# for i in range(11,21):
#     x = x + 1.0 * math.cos(math.pi*2/3)
#     y = y + 1.0 * math.sin(math.pi*2/3)
#     node_poses[i] = np.array([x,y])
# for i in range(21, 30):
#     x = x + 1.0 * math.cos(-math.pi*2/3)
#     y = y + 1.0 * math.sin(-math.pi*2/3)
#     node_poses[i] = np.array([x,y])
# print("node_poses: {}".format(node_poses))
# with open('30-triangle', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-triangle #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# side_angle = math.acos(17.0/33.0)
# # first node is at bottom left corner
# x = 0.0
# y = 0.0
# node_poses[0] = np.array([x,y])
# for i in range(1,35):
#     x = x + 1.0
#     node_poses[i] = np.array([x,y])
# for i in range(35,68):
#     x = x + 1.0 * math.cos(math.pi-side_angle)
#     y = y + 1.0 * math.sin(math.pi-side_angle)
#     node_poses[i] = np.array([x,y])
# for i in range(68, 100):
#     x = x + 1.0 * math.cos(-math.pi+side_angle)
#     y = y + 1.0 * math.sin(-math.pi+side_angle)
#     node_poses[i] = np.array([x,y])
# print("node_poses: {}".format(node_poses))
# with open('100-triangle', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-star #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# outer_angle = 2*math.pi / 5.0
# devia_right = outer_angle
# devia_left = 2*outer_angle
# # first node is at bottom left corner
# heading_angle = outer_angle / 2.0  # current heading
# heading_dir = 0  # current heading direction: 0 for left, 1 for right
# seg_count = 0  # current segment count
# for i in range(1,swarm_size):
#     node_poses[i] = (node_poses[i-1] +
#         np.array([math.cos(heading_angle), math.sin(heading_angle)]))
#     seg_count = seg_count + 1
#     if seg_count == 3:
#         seg_count = 0
#         if heading_dir == 0:
#             heading_angle = reset_radian(heading_angle - devia_right)
#             heading_dir = 1
#         else:
#             heading_angle = reset_radian(heading_angle + devia_left)
#             heading_dir = 0
# print(node_poses)
# with open('30-star', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-star #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# outer_angle = 2*math.pi / 5.0
# devia_right = outer_angle
# devia_left = 2*outer_angle
# # first node is at bottom left corner
# heading_angle = outer_angle / 2.0  # current heading
# heading_dir = 0  # current heading direction: 0 for left, 1 for right
# seg_count = 0  # current segment count
# for i in range(1,swarm_size):
#     node_poses[i] = (node_poses[i-1] +
#         np.array([math.cos(heading_angle), math.sin(heading_angle)]))
#     seg_count = seg_count + 1
#     if seg_count == 10:
#         seg_count = 0
#         if heading_dir == 0:
#             heading_angle = reset_radian(heading_angle - devia_right)
#             heading_dir = 1
#         else:
#             heading_angle = reset_radian(heading_angle + devia_left)
#             heading_dir = 0
# print(node_poses)
# with open('100-star', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-airplane #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# # first node is at bottom center
# node_index = 0  # current sitting node
# heading_angle = - (14.0*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (130.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle - (42.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = reset_radian(heading_angle - (94.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = reset_radian(heading_angle + (106.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (55.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = reset_radian(heading_angle - (35.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# # half the airplane constructed, mirror the next half
# axis_vect = node_poses[node_index] - node_poses[0]
# axis_vect = axis_vect / np.linalg.norm(axis_vect)
# for i in range(1,15):
#     old_vect = node_poses[i] - node_poses[0]
#     node_poses[-i] = 2*np.dot(axis_vect, old_vect)*axis_vect - old_vect
# print(node_poses)
# with open('30-airplane', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-airplane #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# # first node is at bottom center
# node_index = 0  # current sitting node
# heading_angle = - (18.0*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# heading_angle = reset_radian(heading_angle + (135.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = reset_radian(heading_angle - (42.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = reset_radian(heading_angle - (94.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 9)
# heading_angle = reset_radian(heading_angle + (106.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = reset_radian(heading_angle + (55.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 12)
# heading_angle = reset_radian(heading_angle - (40.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 10)
# # half the airplane constructed, mirror the next half
# axis_vect = node_poses[node_index] - node_poses[0]
# axis_vect = axis_vect / np.linalg.norm(axis_vect)
# for i in range(1,50):
#     old_vect = node_poses[i] - node_poses[0]
#     node_poses[-i] = 2*np.dot(axis_vect, old_vect)*axis_vect - old_vect
# print(node_poses)
# with open('100-airplane', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-cross #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# print(node_index)
# print(node_poses)
# with open('30-cross', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-cross #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 17)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 16)
# print(node_index)
# print(node_poses)
# with open('100-cross', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-hand #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (20.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (20.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (55.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# # small finger
# heading_angle = reset_radian(heading_angle - (15.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = reset_radian(heading_angle + (90.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (85.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# # middle finger(no ring finger)
# heading_angle = reset_radian(heading_angle - (147.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = reset_radian(heading_angle + (90.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (85.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# # index finger
# heading_angle = reset_radian(heading_angle - (147.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = reset_radian(heading_angle + (90.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (85.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# # thumb
# heading_angle = reset_radian(heading_angle - (125.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = reset_radian(heading_angle + (85.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (85.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# print(node_index)
# print(node_poses)
# with open('30-hand', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-hand #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# heading_angle = reset_radian(heading_angle + (45.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = reset_radian(heading_angle + (35.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# # small finger
# heading_angle = reset_radian(heading_angle - (15.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# heading_angle = reset_radian(heading_angle + (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (80.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (44.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = reset_radian(heading_angle - (80.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# # ring finger
# heading_angle = reset_radian(heading_angle - (80.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = reset_radian(heading_angle + (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (80.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (44.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = reset_radian(heading_angle - (80.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# # middle finger
# heading_angle = reset_radian(heading_angle - (80.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = reset_radian(heading_angle + (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (80.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (44.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = reset_radian(heading_angle - (80.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# # index finger
# heading_angle = reset_radian(heading_angle - (80.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = reset_radian(heading_angle + (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (80.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (44.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = reset_radian(heading_angle - (10.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = reset_radian(heading_angle - (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# # thumb
# heading_angle = reset_radian(heading_angle - (80.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = reset_radian(heading_angle + (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (80.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (40.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 9)
# heading_angle = reset_radian(heading_angle + (20.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# print(node_index)
# print(node_poses)
# with open('100-hand', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-wrench #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle - (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (90.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (90.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = reset_radian(heading_angle + (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle - (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = reset_radian(heading_angle - (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = reset_radian(heading_angle + (90.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (90.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle - (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# # half the wrench is finished, mirror the next half
# axis_vect = node_poses[node_index] - node_poses[0]
# axis_vect = axis_vect / np.linalg.norm(axis_vect)
# for i in range(1,15):
#     old_vect = node_poses[i] - node_poses[0]
#     node_poses[-i] = 2*np.dot(axis_vect, old_vect)*axis_vect - old_vect
# print(node_index)
# print(node_poses)
# with open('30-wrench', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-wrench #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = reset_radian(heading_angle - (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = reset_radian(heading_angle + (90.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = reset_radian(heading_angle + (90.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = reset_radian(heading_angle + (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = reset_radian(heading_angle - (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 20)
# heading_angle = reset_radian(heading_angle - (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = reset_radian(heading_angle + (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = reset_radian(heading_angle + (90.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = reset_radian(heading_angle + (90.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = reset_radian(heading_angle - (50.0*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# # half the wrench is finished, mirror the next half
# axis_vect = node_poses[node_index] - node_poses[0]
# axis_vect = axis_vect / np.linalg.norm(axis_vect)
# for i in range(1,50):
#     old_vect = node_poses[i] - node_poses[0]
#     node_poses[-i] = 2*np.dot(axis_vect, old_vect)*axis_vect - old_vect
# print(node_index)
# print(node_poses)
# with open('100-wrench', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-goblet #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# arc_angle = 10.8  # default 11.25 deg
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = reset_radian(heading_angle + (135*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (30*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = (arc_angle*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (2*arc_angle*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (2*arc_angle*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(heading_angle + (2*arc_angle*math.pi)/180.0)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# # half the wrench is finished, mirror the next half
# axis_vect = node_poses[node_index] - node_poses[0]
# axis_vect = axis_vect / np.linalg.norm(axis_vect)
# for i in range(1,15):
#     old_vect = node_poses[i] - node_poses[0]
#     node_poses[-i] = 2*np.dot(axis_vect, old_vect)*axis_vect - old_vect
# print(node_index)
# print(node_poses)
# with open('30-goblet', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-goblet #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# arc_angle = 4.1
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = (120*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 10)
# heading_angle = -(arc_angle*math.pi)/180.0
# for _ in range(10):
#     heading_angle = reset_radian(heading_angle + (2*arc_angle*math.pi)/180.0)
#     node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# # half the wrench is finished, mirror the next half
# axis_vect = node_poses[node_index] - node_poses[0]
# axis_vect = axis_vect / np.linalg.norm(axis_vect)
# for i in range(1,50):
#     old_vect = node_poses[i] - node_poses[0]
#     node_poses[-i] = 2*np.dot(axis_vect, old_vect)*axis_vect - old_vect
# print(node_index)
# print(node_poses)
# with open('100-goblet', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-lamp #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = (132*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = ((180-15)*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = (110*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# # half the wrench is finished, mirror the next half
# axis_vect = node_poses[node_index] - node_poses[0]
# axis_vect = axis_vect / np.linalg.norm(axis_vect)
# for i in range(1,15):
#     old_vect = node_poses[i] - node_poses[0]
#     node_poses[-i] = 2*np.dot(axis_vect, old_vect)*axis_vect - old_vect
# print(node_index)
# print(node_poses)
# with open('30-lamp', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-lamp #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# heading_angle = (100*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (140*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (160*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = (175*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 7)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 10)
# heading_angle = (110*math.pi)/180.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 15)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# # half the wrench is finished, mirror the next half
# axis_vect = node_poses[node_index] - node_poses[0]
# axis_vect = axis_vect / np.linalg.norm(axis_vect)
# for i in range(1,50):
#     old_vect = node_poses[i] - node_poses[0]
#     node_poses[-i] = 2*np.dot(axis_vect, old_vect)*axis_vect - old_vect
# print(node_index)
# print(node_poses)
# with open('100-lamp', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 30-K #####
# swarm_size = 30
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# angle_up = (50*math.pi)/180.0
# angle_down = -(50*math.pi)/180.0
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = angle_down
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = angle_up
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(angle_down + math.pi)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = angle_up
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = reset_radian(angle_down + math.pi)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = reset_radian(angle_up - math.pi)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 2)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 1)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# print(node_index)
# print(node_poses)
# with open('30-K', 'w') as f:
#     pickle.dump(node_poses, f)


# ##### script to generate 100-K #####
# swarm_size = 100
# node_poses = np.zeros((swarm_size, 2))
# node_index = 0
# angle_up = (49*math.pi)/180.0
# angle_down = -(49*math.pi)/180.0
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 6)
# heading_angle = angle_up
# node_index = cal_next_node(node_poses, node_index, heading_angle, 3)
# heading_angle = angle_down
# node_index = cal_next_node(node_poses, node_index, heading_angle, 10)
# heading_angle = 0.0
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = reset_radian(angle_down + math.pi)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 14)
# heading_angle = angle_up
# node_index = cal_next_node(node_poses, node_index, heading_angle, 11)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 5)
# heading_angle = reset_radian(angle_up - math.pi)
# node_index = cal_next_node(node_poses, node_index, heading_angle, 10)
# heading_angle = math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 8)
# heading_angle = -math.pi
# node_index = cal_next_node(node_poses, node_index, heading_angle, 4)
# heading_angle = -math.pi/2
# node_index = cal_next_node(node_poses, node_index, heading_angle, 19)
# print(node_index)
# print(node_poses)
# with open('100-K', 'w') as f:
#     pickle.dump(node_poses, f)



pygame.init()
# find the right world and screen sizes
x_max, y_max = np.max(node_poses, axis=0)
x_min, y_min = np.min(node_poses, axis=0)
pixel_per_length = 30
world_size = (x_max - x_min + 2.0, y_max - y_min + 2.0)
screen_size = (int(world_size[0])*pixel_per_length, int(world_size[1])*pixel_per_length)
# convert node poses in the world to disp poses on screen
def cal_disp_poses():
    poses_temp = np.zeros((swarm_size, 2))
    # shift the loop to the middle of the world
    middle = np.array([(x_max+x_min)/2.0, (y_max+y_min)/2.0])
    for i in range(swarm_size):
        poses_temp[i] = (node_poses[i] - middle +
            np.array([world_size[0]/2.0, world_size[1]/2.0]))
    # convert to display coordinates
    poses_temp[:,0] = poses_temp[:,0] / world_size[0]
    poses_temp[:,0] = poses_temp[:,0] * screen_size[0]
    poses_temp[:,1] = poses_temp[:,1] / world_size[1]
    poses_temp[:,1] = 1.0 - poses_temp[:,1]
    poses_temp[:,1] = poses_temp[:,1] * screen_size[1]
    return poses_temp.astype(int)
disp_poses = cal_disp_poses()
# draw the loop shape on pygame window
color_white = (255,255,255)
color_black = (0,0,0)
screen = pygame.display.set_mode(screen_size)
screen.fill(color_white)
for i in range(swarm_size):
    pygame.draw.circle(screen, color_black, disp_poses[i], 5, 0)
for i in range(swarm_size-1):
    pygame.draw.line(screen, color_black, disp_poses[i], disp_poses[i+1], 2)
pygame.draw.line(screen, color_black, disp_poses[0], disp_poses[swarm_size-1], 2)
pygame.display.update()
raw_input("<Press ENTER to exit>")


