# This first demo shows how a robot swarm can autonomously choose a loop shape and form the
# shape in a distributed manner, without central control. Two consensus processes, decision
# making and role assignment, are performed consecutively with a fixed but arbitrary network.

# Description:
# Starting dispersed in random positions, the swarm aggregates together arbitrarily to form
# a connected network. Consensus decision making is performed with this network fixed, making
# a collective decision of which loop the swarm should form. Then with the same network, role
# assignment is performed, assigning the target positions to each robot. After these two
# consensus processes are done, the swarm disperses and aggregates again, this time aiming to
# form a loop with robots at their designated positions. The climbing method is used to permutate
# the robots on the loop. When the robots get their target positions, they dyanmically adjust
# the local shape so the loop deforms to the target one. The above steps will be ran repeatedly.


# use 'robot' instead of 'node here

# accomodate different network size, 

# note that communication is simulated in the role assignment, but not 

# how to display all the information needed along with the simulation
# consensus decision making: same color for same chosen decision
# role assignment
# loop formation


