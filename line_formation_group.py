# group class for line formation simulation

class LRGroup:
    def __init__(self, id, size):
        self.id = id
        self.size = size
        self.mem_online = []  # indices of robots already on the line
        self.mem_offline = []  # indices of robots not yet on the line


