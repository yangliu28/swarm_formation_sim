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
        self.status_1_1_next = []  # next key neighbor expected to acquire
            # when '1' first starts to merge, it may sense a single '2' robot
            # most cases when '1' merges into the middle of two robots, it expects to
            # meet another '2', and add it to the key neighbor list.
            # This variable should contain one variable at most.
        self.status_2_sequence = 0  # sequence along the line for status '2' robot
            # index ranges from 0 to N-1
        self.status_2_end = False  # sub status deciding if robot is at larger index end
            # False: not at the larger index end of the line
            # True: at the larger index end, index should be N-1
        self.status_2_avail1 = [True,True]  # first availability for two sides to be merged
            # first availability indicates if the distance is large enough for merging
            # first value for small index side, second for large side
        self.status_2_avail2 = [True,True]  # second availability for two sides to be merged
            # second availability indicates if a robot '1' is allowed to merge
            # first value for the small index side, second for large side
        self.key_neighbors = [-1, -1]  # key neighbors to secure the group formation
            # two member list for all in-group robot, including '1' and '2'
            # first member is the robot on the smaller index side of the line
            # second member is the robot on the larger index side of the line
            # if no neighbor at one side, use -1 as a flag
        self.status_n1_life = 0  # life time of status '-1', randomly generated



