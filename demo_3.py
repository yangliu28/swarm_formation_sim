# This third demo shows how a robot swarm can autonomously choose an open curve shape and form
# the shape in a distributed way. This simulation shares the same strategy with second demo in
# organizing the robots, but it needs no role assignment on the open curve.

# Description:
# Starting dispersed in random positions, the swarm aggregates together to form a straight line.
# Merging method is chosen to form the line, the other is climbing method. From here, the
# following steps will ran repeatedly. The robots first make collective decision of which open
# curve shape to form, then adjust the local shapes so the curve reshapes to the chosen one.





