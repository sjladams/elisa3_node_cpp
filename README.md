# elisa3_node_cpp

# ROS node for Elisa-3 Robot
Elisa-3 ROS node based on roscpp; it support all the elisa-3 sensors.

This package is maintained by [GCtronic](http://www.gctronic.com/).

#Adjustments to package
The communication protocol in the Elisa3 library has been adjusted to allow for the execution of three formation algorithms: ETC, PETC and phase. To adjust the type of formation, number of robots and/or the communication topology, adjust the inter-distance matrices Bx and By, the number of ROBOTS, and the Laplacian matric L respectively in the elisa3-lib.c file. 

## How to use
First update the firmware of the robots with the hex file. Once the firmware is updated, one of the three algorithms can be selected by using the physical selector on the robots:
1. Selector 14: phase algorithm
2. Selector 13: ETC algorithm
3. Selector 12: PETC algorithm

Note that the Bx and By matrices for the phase algorithm need to contain the x and y distance in mm to the centre of the formatio, while for the ETC and PETC algorithms the Bx and By matrices contain the inter-robot distance in mm.

For detailed informations about the node refer to the [elisa-3 ROS wiki](http://www.gctronic.com/doc/index.php/Elisa-3#ROS).

