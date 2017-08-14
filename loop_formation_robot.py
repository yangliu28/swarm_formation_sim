# robot class for loop formation simulation

class LFRobot: # LF stands for loop formation
    def __init__(self, pos, vel, ori):
        # position, velocity, and orientation for the physics
        self.pos = list(pos)  # convert to list
        self.vel = vel  # unsigned scalar
        self.ori = ori  # moving direction
        # variables for configuring individual robots's formation process
        self.status = 0  # key status for the formation, start with '0'
            # '0' for being single, moving around, available for grouping
            # '1' for on the way to a formal group, temporary transition status
            # '2' for being in a group, all members indexed, constantly adjusting position
            # '-1' for being single, moving around, ignoring all connections
            # see more details for substatuses in "loop_formation.py"
        self.group_id = 0  # for status '1' and '2'
        self.status_1_sub = 0  # substatus for status '1' robot
            # '0' for forming the initial triangle
            # '1' for merging into a loop
        self.status_1_0_sub = 0  # substatus for status '1_0' robot
            # '0' for forming the pair
            # '1' for forming the triangle
        self.status_1_0_1_des = [0,0]  # destination for the triangle forming robots
        self.status_1_1_des = [0,0]  # destination for the merging robots
        self.status_2_sequence = 0  # sequence along the line for status '2' robots
            # index range from 0 to N-1
        self.status_2_avail1 = [True,True]  # first availability for two sides to be merged
            # indicating if distances at two sides are large enough for accepting new member
            # first value for small index side, second for large side
        self.status_2_avail2 = [True,True]  # second availability for two sides to be merged
            # indicating if a robot is merging at one side, if yes, then availability is False
            # first value for small index side, second for large side
        self.key_neighbors = [-1,-1]  # key neighbors to secure the group formation
            # two members list for all '1' and '2' robots, except '1_0_0'
            # first value is the robot on the small index side, second for large side
            # if no neighbor at one side, value of '-1' will act as a flag
        self.status_n1_life = 0  # life time of status '-1', randomly generated


