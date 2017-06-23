# robot class for line formation simulation

class LFRobot:  # LF stands for line formation
    def __init__(self, pos, vel, ori):
        # variables for the movement
        self.pos = list(pos)  # pos[0] for x, pos[1] for y, convert to list
        self.vel = vel  # unsigned scalar
        self.ori = ori  # global orientation of moving direction
        # variables for the line formation control
        self.state = 0
            # '0' for being single, wondering around
            # '1' for in group but climbing the line, not on the line yet
            # '2' for in group and on the line
        self.group_id = 0  # random number to uniquely identify the group
        self.group_size = 0  # number of robots in the group
        self.group_index = 0  # sequence along the line


