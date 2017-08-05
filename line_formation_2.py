# second line formation simulation, using merging method instead of climbing

# merging method analysis(compared to climbing):
# The line may not be as straight as in first simulation, and it would take time to stretch
# the line so that robots are evenly spaced. But merging method has the flexibility to
# adjust the formation to walls and obstacles, so there is no need to use double walls to
# make sure the initial line segments start in the middle to avoid the line hit the wall.

# comments for research purpose:
# Same competing mechanism is used to ensure only one line being formed.


# comments for programming purpose:
# Similar four statuses have been designed to serve different stages of the robot:
    # '0': free, wondering around, not in group, available to form a group or join one
    # '1': forming an initial line segment, or found a line and trying to merge into it
        # '1_0': initial an line segment, '0' found another '0' and formed a group
        # '1_1': join a line, merging into
    # '2': already in a line, dynamically adjust space and straightness
    # '-1': free, wondering around, no interaction with nearby robots
# Only allowed status transitions are:
    # forward: '-1'->'0', '0'->'1', '1'->'2'
    # backward: '1'->'-1', '2'->'-1'
# The way merging method works is, when '0' runs into a '2', it inquires 


