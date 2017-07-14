# robot class for line formation simulation

class LFRobot:  # LF stands for line formation
    def __init__(self, pos, vel, ori):
        # variables for the physical movement
        self.pos = list(pos)  # pos[0] for x, pos[1] for y, convert to list
        self.vel = vel  # unsigned scalar
        self.ori = ori  # global orientation of moving direction
        # variables for the line formation control
        self.status = 0  # robot's status for the line formation process
            # '0' for being single, and seeking connections
            # '1' for in a group, but climbing the line, not on the line yet
            # '2' for in a group, and in correct position for the line
            # '-1' for being single, and ignoring all connections
        self.group_id = -1  # random integer to uniquely identify the group
        self.status_1_sub = -1  # sub state for state '1' robot
            # '0' for initial forming
            # '1' for climbing
        self.status_1_0_dir = 0  # moving direction for the initial forming robots
            # an angle indicating the absolute moving direction
        self.status_1_1_dir = 0  # movind direction for the climbing robot
            # '0' for climbing toward the small index end
            # '1' for climbing toward the large index end
        self.status_1_1_des = [0,0]  # destination coordiantes for the climbign robots
            # decided by the current grab-on robot
        self.status_1_1_side = 0  # left or right side along the moving direction
            # '0' for left, '1' for right
        self.status_2_sequence = -1  # sequence along the line for state '2' robot
            # index starts from '0'
        self.status_2_end = False  # sub state deciding if robot is at the end of line
            # False: not at the end of line
            # True: at the end of line
        self.key_neighbors = []  # key neighbors to secure the group formation
            # for '1_0' robot, neighbor is the other initial forming robot
            # for '1_1' robot, neighbor is the line traveling robot
            # for '2' robot, neighbors are two robots adjacent on the line
                # one robot if the '2' is at the begining or end


