# swarm_formation_sim
A collection of swarm robot formation simulations using Python with Pygame and Matplotlib. I have done [swarm robot simulations in ROS](https://github.com/yangliu28/swarm_robot_ros_sim.git) before, but the combination of Python, Pygame and Matplotlib turns out to better suit my needs for this research, plus the simpilicity in programming the alogrithms and implementing the graphics.

## Contents
All the formation control algorithms simulated here agree on a few conditions of the robots. The robots can sensing relative position of neighbors with in sensing range. The robots can communicate with robots in communication range. Both sensing range and communication range are very small, most time they are treated the same in the simulations. (This corresponds to the physical infrared sensor that does sensing and communicating at same time.) The robot swarm is homogeneous. The robots can do omnidirectional movements. The robots are modelled as dots, so no collision avoidance method is studied here.

*line_formation_1.py* is the first line formation simulation featuring climbing method and competing mechanism. *line_formation_1_robot.py* containts the robot class for this simulation.

*line_formation_2.py* is similar to the first line formation, except implementing merging method instead of climbing to form the line. *line_formation_2_robot.py* containts the robot class for this simulation.

*loop_formation.py* uses same merging method to form a loop, the formation starts with a pair of robots, then a triangle formation as the initial loop. *loop_formation_robot.py* containts the robot class for this simulation.

*formation_functions.py* contains several frequeny used functions for the line and loop formations.

*loop_reshape_1_static.py* is the static version of the loop reshape simulation. Several preliminary algorithms have been tested here, in order to achieve an usable swarm decision convergence. The finalized algorithm has been tested in the dynamic version of the loop reshape simulation.

*loop_reshape_2_dynamic.py* is the dynamic version of the loop reshape simulation. A new weighted averaging method is implemented to tolerate the conflict between distribution convergence and better distribution unipolarity. A new SMA-inspired motion strategy is used for the physical motion control of the loop reshape process.

*loop_reshape_reader.py* is a useful tool to read stored loop formation files (randomly generated from the reshape simulations), visualize them in pygame.

*loop_reshape_test_power.py* is for testing how power function can increase the unipolarity of a random distribution. Linear multiplier was later found to be more mild and thus a better choice. *loop_reshape_test_motion* is for testing the physical motion controlalgorithm of the loop reshape process, the SMA algorithm was first tested here. *curve_shape_test_filter.py* is for smoothing open curves or closed curves to the effect of human drawing like curves.

*honeycomb_probabilistic_convergence.py* is the test program for probabilistic convergence algorithm, running on 2D honeycomb networks. *honeycomb_generator.py* is the corresponding 2D honeycomb network generator.


## Run the simulations
Install corresponding version of Pygame for your Python, optional dependencies include numpy, matplotlib, etc. See the header of the desired '.py' to find the necessary dependencies. Some simulation examples are listed below.

Line formation simulation with climbing method:

`python line_formation_1.py`

Line formation simulation with merging method:

`python line_formation_2.py`

Loop formation simulation:

`python loop_formation.py`

Loop reshape simulation:

`python loop_reshape_2_dynamic.py gen_discard initial_read 30-1 target_gen`

Probabilistic convergence algorithm simulation:

`python honeycomb_probabilistic_convergence.py -f 50-3 -d 30 --nobargraph`

## License
See the [LICENSE](LICENSE.md) file for license rights and limitations (MIT).

