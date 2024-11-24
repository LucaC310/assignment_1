Research Track 1 - Assignment 1

This repository contains a ROS package that features two nodes: UI_node and Distance_node. The project is built within the turtlesim environment, providing a simple simulation to control and monitor turtle robots.

Nodes Overview
	1 -> UI_node
		Functionality:
		This node spawns a new turtle (turtle2) in turtlesim environments and allows the user to control the velocity of both turtles 		(turtle1 and turtle2).
		
		User Interaction:
		o The user can choose where to spawn the turtle2 and decide which turtle to control.
		o The user can issue velocity commands for a duration of one second.
		o After this time, the robot stops, and the user can input a new command.
			
	2 -> Distance_node
		Functionality:
		This node calculates the relative distance between turtle1 and turtle2 and publish this information on ROS topic.
		
		Safety Features:
		The turtles are stopped automatically if:
			o They come too close each other.
			o They approach the simulation boundaries (x or y > 10.0 or x or y < 1.0).
