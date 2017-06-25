# functions for the line formation simulation

import math

# reset radian angle to [0, 2*pi)
def lf_reset_radian(radian):
    while radian >= 2*math.pi:
        radian = radian - 2*math.pi
    while radian < 0:
        radian = radian + 2*math.pi
    return radian

# convert positions in physics coordinates to display coordinates
def lf_world_to_display(input_pos, world_size, display_size):
    # lf stands for line formation
    pos_display = [0, 0]
    pos_display[0] = int(input_pos[0]/world_size[0] * display_size[0])
    pos_display[1] = int((1-input_pos[1]/world_size[1]) * display_size[1])
    return pos_display


