# elisa3_node_cpp

# ROS node for Elisa-3 Robot
Elisa-3 ROS node based on roscpp; it support all the elisa-3 sensors and visualises the trajectory of 3 robots in RVIZ.

This package is maintained by [GCtronic](http://www.gctronic.com/).

##Adjustments to package
The communication protocol in the Elisa3 library has been adjusted to allow for the execution of three formation algorithms: ETC, PETC and phase. To adjust the type of formation, number of robots and/or the communication topology, adjust the inter-distance matrices Bx and By, the number of ROBOTS, and the Laplacian matric L respectively in the elisa3-lib.c file. 

Examples of L, Bx, By matrices have been provided in the formation folder for two types of communication topologies; hypercube (4 or 8 robots) and line (3 or 6 robots).

## How to use
First update the firmware of the robots with the hex file. Once the firmware is updated, one of the three algorithms can be selected by using the physical selector on the robots:
1. Selector 14: phase algorithm
2. Selector 13: ETC algorithm
3. Selector 12: PETC algorithm

Note that the Bx and By matrices for the phase algorithm need to contain the x and y distance in mm to the centre of the formatio, while for the ETC and PETC algorithms the Bx and By matrices contain the inter-robot distance in mm.

When the elisa3-lib.c file is update to allow for a different formation, do the following:
1. Build the library
   cd src/pc-side-elisa3-library/linux
   gcc -c ../usb-comm.c ../elisa3-lib.c
   ar -r libelisa3.a usb-comm.o elisa3-lib.o
2. Build the elisa3_node_cpp package
    cd ~
    cd catkin_ws
    catkin_make
    
    
Run the package by using: roslaunch elisa3_node_cpp elisa3_swarm.launch. 

## Errors
It often happens that the robots do not start up correctly or the communication fails. When this happens the RVIZ simulation will not show all robots. The best way to fix this is by using the following steps:
1. Make sure all robots are turned on and the selector is at the correct position
2. restart the package a couple of times
3. If all else fails, turn the robots on and off and start at step 1.

For detailed informations about the node refer to the [elisa-3 ROS wiki](http://www.gctronic.com/doc/index.php/Elisa-3#ROS).

