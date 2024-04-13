/// @file pedsim_tf_listener.cpp
/// @author Nguyen Van Hung (nvhung.v2k@gmail.com)
/// @date April 13, 2024
/// 
/// run with pedsim
/// INPUT: /tf
/// OUTPUT: current location, goal (geometry_msgs/Pose2D) --> steer2nextpose_note (./steer2nextpose.cpp)

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation

#include <iostream> // Enables command line input and output

// Remove the need to use std:: prefix
using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "pedsim_tf_listener");

  ros::NodeHandle node;

  ros::Publisher robot_loc =
    node.advertise<geometry_msgs::Pose2D>("/pedbot/pose", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/odom","/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Pose2D vel_msg;
    vel_msg.x = transform.getOrigin().x();
    vel_msg.y = transform.getOrigin().y();
    vel_msg.theta = tf::getYaw(transform.getRotation());
    robot_loc.publish(vel_msg);

    // Print the output to the console
    /*
    cout << "Current (x,y,theta) = " << "(" << vel_msg.x<< "," << vel_msg.y << ","
         << vel_msg.theta << ")"
         << endl << endl;
   */
    rate.sleep();
  }
  return 0;
};
