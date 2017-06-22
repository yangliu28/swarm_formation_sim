# robot class for line formation simulation

class Robot:
    def __init__(self, pos, vel, ori):
        self.pos = list(pos)  # pos[0] for x, pos[1] for y, convert to list
        self.vel = vel  # unsigned scalar
        self.ori = ori  # global orientation of moving direction

        