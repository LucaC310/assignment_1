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
	turtlesim::Spawn spawn;
	spawn.request.x = 2.0;
	spawn.request.y = 1.0;
	spawn.request.theta = 0.0;
	spawn.request.name = "turtle2";	
	client.call(spawn);
	
	string turtle_name;
	int turtle_choice;

	ros::Rate loop_rate(1);
	
	while (ros::ok()){
	                  
		double x_velocity, y_velocity, theta_velocity;
		
		cout << "Choose a turtle:\n";
		cout << "1. Turtle 1:\n";
		cout << "2. Turtle 2:\n";
		cout << "Enter your choice (1 or 2):";
		cin >> turtle_choice;

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

		cout << "Enter x velocity (m/s): ";
		cin >> x_velocity;
		cout << "Enter y velocity (m/s): ";
		cin >> y_velocity;
		cout << "Enter theta velocity (rad/s): ";
		cin >> theta_velocity;
		
		cmd_vel_msg.linear.x = x_velocity;
		cmd_vel_msg.linear.y = y_velocity;
		cmd_vel_msg.angular.z = theta_velocity;
		velocity_publisher.publish(cmd_vel_msg);
		ROS_INFO("Publish to %s -x: %f, -y: %f, -theta: %f",
		turtle_name.c_str(), x_velocity, y_velocity, theta_velocity);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;	
}
