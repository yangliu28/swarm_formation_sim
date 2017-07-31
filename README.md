# swarm_formation_sim
Formation simulations for swarm robots using Python and Pygame. I have done [simulations in ROS](https://github.com/yangliu28/swarm_robot_ros_sim.git) before, but decided to give Python and Pygame a try, for the simpilicity in algorithm validation and graphics implementation.

Line formation simulation uses climbing method and competing mechanism, The requirements for the robots: basic wireless communication, limited communication range, direction and distance sensing of a robot in range, robot modeled as dot so no collision avoidance necessary, omnidirection movement, homeogeneous robots. The algorithm will rely heavily on communication to update robots' knowledge when algorithm is running on real robots. More details can be found in the comments in "line_formation.py".

Run the line formation simulation:

`python line_formation.py`

