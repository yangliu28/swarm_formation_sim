# swarm_formation_sim
A collection of swarm robot formation simulations using Python with Pygame and Matplotlib. I have done [swarm robot simulations in ROS](https://github.com/yangliu28/swarm_robot_ros_sim.git) before, but the combination of Python, Pygame and Matplotlib turns out to better suit my needs for this research, plus the simpilicity in programming the alogrithms and implementing the graphics.

## Contents
All the formation control algorithms simulated here agree on a few conditions of the robots. The robots can sensing relative position of neighbors with in sensing range. The robots can communicate with robots in communication range. Both sensing range and communication range are very small, most time they are treated the same in the simulations. (This corresponds to the physical infrared sensor that does sensing and communicating at same time.) The robot swarm is homogeneous. The robots can do omnidirectional movements. The robots are modelled as dots, so no collision avoidance method is studied here.

*line_formation_1.py* is a line formation simulation features climbing method and competing mechanism.

*line_formation_2.py* is similar to the first line formation, except implementing merging method instead of climbing to form the line.

*loop_formation.py* uses same merging method to form a loop, the formation starts with a pair of robots, then a triangle formation as the initial loop.

*loop_reshape_2_dynamic.py* is the dynamic version of the loop reshape simulation. A new weighted averaging method is implemented to tolerate the conflict between distribution convergence and better distribution unipolarity. A new SMA-inspired motion strategy is used for the physical motion control of the loop reshape process.

## Run the simulations

Install corresponding Pygame version for your Python, optional dependencies include numpy, matplotlib. See the header of the desired '.py' to find the necessary dependencies.

Line formation simulation with climbing method:

`python line_formation_1.py`

Line formation simulation with merging method:

`python line_formation_2.py`

Loop formation simulation:

`python loop_formation.py`

Loop reshape simulation, dynamic version: (see more parameter options in the header comments of the file)

`python loop_reshape_2_dynamic.py gen_discard initial_read 30-1 target_gen`


