# This second demo is a variation of first one, also shows how a robot swarm can autonomously
# choose a loop shape and form the shape. But the decision making and the role assignment are
# performed while the swarm is already in a loop shape. The new role assignment method applies
# only to loop topology, but has the advantage of using no message convey.

# Description:
# Starting dispersed in random positions, the swarm aggregates together arbitrarily to form
# an arbitrary loop. From here, the following steps will ran repeatedly. The robots first make
# collective decision of which shape to form, then they perform role assignment to distribute
# target positions. Happening at the same time of role assignment, individual robot also
# adjusts the local shape so the loop reshapes to the chosen one.




