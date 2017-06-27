# group class for line formation simulation

class LRGroup:
    def __init__(self, id, size):
        self.id = id  # must be an unique integer for each group
        self.size = size
        self.mem_online = []  # indices of robots already on the line, status '2'
        self.mem_offline = []  # indices of robots not yet on the line, status '1'


