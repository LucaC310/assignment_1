#include <iostream>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h> 
#include <turtlesim/Spawn.h> 

using namespace std;

ros:: Publisher pub;

int main (int argc, char **argv)
{
	ros::init(argc, argv, "UI_node");
	ros::NodeHandle n;
	
	pub = n.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1); 
	ros::ServiceClient client = n.serviceClient<turtlesim::Spawn>("/spawn");
	
	//User input for spawn position
	double x_position, y_position, theta_position;

	cout << "Set x position to spawn the turtle2: ";
	cin >> x_position;
	cout << "Set y position to spawn the turtle2: ";
	cin >> y_position;
	cout << "Set theta position to spawn the turtle2: ";
	cin >> theta_position;
	
	//Service request
	turtlesim::Spawn spawn;
	spawn.request.x = x_position;
	spawn.request.y = y_position;
	spawn.request.theta = theta_position;
	spawn.request.name = "turtle2";
	
	if(client.call(spawn)){
		ROS_INFO("Turtle spawned successfully at x: %f, y: %f, theta: %f", x_position, y_position, theta_position);
	} else{
		ROS_ERROR("Failed to spawn turtle.");
		return 1;
	}	
	
	
	//Variable for User Input
	string turtle_name;
	int turtle_choice;

	//Sets the loop rate to 1Hz (1 cycle per second)
	ros::Rate loop_rate(1);
	
	while (ros::ok()){
	                  
		double x_velocity, y_velocity, theta_velocity;
		
		//User Input for turtle selection
		cout << "Choose a turtle:\n";
		cout << "1. Turtle 1:\n";
		cout << "2. Turtle 2:\n";
		cout << "Enter your choice (1 or 2):";
		cin >> turtle_choice;
		
		//Set target turtle
		if (turtle_choice == 1){
			turtle_name = "/turtle1/cmd_vel";
		} else if (turtle_choice == 2) {
			turtle_name = "/turtle2/cmd_vel";
		} else {
			cerr << "Error.\n";
			return 1;
		}

		ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>(turtle_name, 1);
		geometry_msgs::Twist cmd_vel_msg;
		
		//User Input for velocities
		cout << "Enter x velocity (m/s): ";
		cin >> x_velocity;
		cout << "Enter y velocity (m/s): ";
		cin >> y_velocity;
		cout << "Enter theta velocity (rad/s): ";
		cin >> theta_velocity;
		
		//Create and publish velocity command
		cmd_vel_msg.linear.x = x_velocity;
		cmd_vel_msg.linear.y = y_velocity;
		cmd_vel_msg.angular.z = theta_velocity;
		velocity_publisher.publish(cmd_vel_msg);
		
		//Logs message
		ROS_INFO("Publish to %s -x: %f, -y: %f, -theta: %f",turtle_name.c_str(), x_velocity, y_velocity, theta_velocity);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;	
}
