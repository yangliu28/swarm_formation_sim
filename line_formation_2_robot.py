# robot class for line formation 2 simulation

class LFRobot:  # LF for line formation
    def  __init__(self, pos, vel, ori):
        # pos, velocity, orientation for recording the physics
        self.pos = list(pos)  # convert to list
        self.vel = vel  # unsigned scalar
        self.ori = ori  # moving direction in the physical coordinates
        # variables for configuring indivudual robot's formation process
        self.status = 0  # key status for the formation, start with '0'
            # '0' for being single, moving around, available for joining a group
            # '1' for in a group, but not an indexed member, still on the way to the line
            # '2' for in a group, indexed member, position is good
            # '-1' for being single, moving around, and ignoring all connections
        self.group_id = 0  # for status '1' and '2'
        self.status_1_sub = 0  # sub status for status '1' robot
            # '0' for forming the initial line segment
            # '1' for merging into the line
        self.status_1_1_des = [0,0]  # destination for the merging robot
        self.status_2_sequence = 0  # sequence along the line for status '2' robot
            # index ranges from 0 to N-1
        self.status_2_end = False  # sub status deciding if robot is at larger index end
            # False: not at the larger index end of the line
            # True: at the larger index end, index should be N-1
        self.status_2_avail = [False,False]  # availility of the two sides to be merged
            # first value for small index side, second for large side
        self.key_neighbors = []  # key neighbors to secure the group formation
            # for '1_0' robot, key neighbor is the other initial forming robot
            # for '1_1' robot, key neighbors are the two new neighbors when merged
            # for '2' robot, key neighbors are the two adjacent neighbors along the line
        self.sttus_n1_life = 0  # life time of status '-1', randomly generated



