# swarm_formation_sim
Formation simulations for swarm robots using Python and Pygame. I have done simulations in ROS before, but gave Python and Pygame a try for easiness in algorithm validation and graphics implementation.

Line formation simulation uses climbing method and competing mechanism, The requirements for the robots: basic wireless communication, limited communication range, direction and distance sensing of a robot in range, robot modeled as dot so no collision avoidance necessary, omnidirection movement, homeogeneous robots. The algorithm will rely heavily on communication to update robots' knowledge when algorithm is running on real robots. More details can be found in the comments in "line_formation.py".

Run the line formation simulation:

`python line_formation.py`

