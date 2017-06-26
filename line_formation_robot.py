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
        self.group_id = -1  # random number to uniquely identify the group
        self.group_size = -1  # number of robots in the group
        self.group_index = -1  # sequence along the line


