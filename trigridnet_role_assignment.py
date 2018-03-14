# This simulation tests a distributed role assignment algorithm, for robots starting at
# arbitrary triangle grid network, and performing one-to-one role assignment.

# Inter-robot communication is used to convey message between unconnected robots. Although
# it is possible for each robot getting to know the information of all other robots,

# we still consider it a distributed algorithm, because we limited the local computation
# and information it will store, so the algorithm can be more robust.



# the step of communication is also considered in the simulation

# color is used to hightlight the conflicts, instead of showing convergence of decisions


