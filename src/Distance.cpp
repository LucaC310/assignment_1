#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <cmath>
turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;
bool turtle1_pose_received = false;
bool turtle2_pose_received = false;

ros::Publisher distance_pub;
ros::Publisher turtle2_stop_pub; 
float distance_threshold = 1.0; 
float boundary_threshold = 1.0; 

void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_pose = *msg;
    turtle1_pose_received = true;
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_pose = *msg;
    turtle2_pose_received = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_checker_node");
    ros::NodeHandle n;
    
    ros::Subscriber turtle1_pose_sub = n.subscribe("turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber turtle2_pose_sub = n.subscribe("turtle2/pose", 10, turtle2PoseCallback);
    
    distance_pub = n.advertise<std_msgs::Float32>("turtles_distance", 10);
    
    turtle2_stop_pub = n.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    ros::Rate rate(10); // Loop at 10 Hz

    while (ros::ok()) {
        ros::spinOnce(); 

        if (turtle1_pose_received && turtle2_pose_received) {
            
            float dx = turtle2_pose.x - turtle1_pose.x;
            float dy = turtle2_pose.y - turtle1_pose.y;
            float distance = std::sqrt(dx * dx + dy * dy);
            
            std_msgs::Float32 distance_msg;
            distance_msg.data = distance;
            distance_pub.publish(distance_msg);
          
            if (distance < distance_threshold) {
                ROS_WARN("Turtles are too close! Stopping turtle2.");

                
                geometry_msgs::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.angular.z = 0.0;
                turtle2_stop_pub.publish(stop_msg);
            }

            
            if (turtle2_pose.x > 10.0 || turtle2_pose.x < boundary_threshold ||
                turtle2_pose.y > 10.0 || turtle2_pose.y < boundary_threshold) {
                ROS_WARN("Turtle2 is near the boundaries! Stopping turtle2.");
                
                geometry_msgs::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.angular.z = 0.0;
                turtle2_stop_pub.publish(stop_msg);
            }
        }

        rate.sleep();
    }

    return 0;
}
