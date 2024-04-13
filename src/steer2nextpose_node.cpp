/// @file steer2nextpose.cpp
/// @author Nguyen Van Hung (nvhung.v2k@gmail.com)
/// @date April 13, 2024
/// 
/// run with pedsim
/// INPUT: current location, goal (geometry_msgs/Pose2D)
/// OUTPUT: cmd_vel (geometry_msgs/Twist) to pedsim' simulate_diff_drive_robot
 
#include <cstdlib> // Use for the absolute value method abs()
#include <iostream> // Enables command line input and output
 
#include "ros/ros.h" // Necessary header files for ROS
#include "geometry_msgs/Twist.h" // Twist messages (linear & angular velocity)
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation

// Remove the need to use std:: prefix
using namespace std;
 
// Key variable declarations 
geometry_msgs::Twist velCommand; // Linear and angular velocity in m/s 
geometry_msgs::Pose2D current; // Current x, y, and theta 
geometry_msgs::Pose2D nextposeGoal; // nextpose x, y, and theta (the nextpose)
ros::Publisher velocityPub; // Object used for publishing velocity command
 
const double PI = 3.141592654;
 
// The gain K, which is used to calculate the linear velocity
const double K_l = 0.5;
 
// The gain K, which is used to calculate the angular velocity
const double K_a = 0.5;
 
// The distance threshold in meters that will determine when 
// the turtlesim robot successfully reaches the goal.
const double distanceTolerance = 0.1;
 
// The angle threshold in radians that will determine when 
// the turtlesim robot successfully reaches the goal.
const double angleTolerance = 0.1;
 
// This flag determines when the robot needs to either 
// move towards a nextpose or stop.
bool goTonextpose = false;
 
// Initialized variables and take care of other setup tasks
void setup() {
 
  // We initialize with the default starting coordinate 
  // for the turtlesim simulator
  nextposeGoal.x = 5.0;	//Vi tri khoi tao cua robot tai pedsim
  nextposeGoal.y = 5.0;	//
   
  // Initialize the Twist message.
  // Initial linear and angular velocities are 0 m/s and rad/s, respectively.
  velCommand.linear.x = 0.0;
  velCommand.linear.y = 0.0;
  velCommand.linear.z = 0.0;
  velCommand.angular.x = 0.0;
  velCommand.angular.y = 0.0;
  velCommand.angular.z = 0.0;
}
 
// Get the distance between the current x,y coordinate and 
// the desired nextpose x,y coordinate.
double getDistanceTonextpose() {
  return sqrt(pow(nextposeGoal.x - current.x, 2) + pow(
    nextposeGoal.y - current.y, 2));
}
 
// Get the heading error
// i.e. how many radians does the robot need 
// to turn to head towards the nextpose  
double getHeadingError() {
 
  double deltaX = nextposeGoal.x - current.x;
  double deltaY = nextposeGoal.y - current.y;
  double nextposeHeading = atan2(deltaY, deltaX);
  double headingError = nextposeHeading - current.theta;   
  // Make sure heading error falls within -PI to PI range
  if (headingError > PI) {
    headingError = headingError - (2 * PI);
  } 
  if (headingError < -PI) {
    headingError = headingError + (2 * PI);
  } 
  return headingError;
}
 
// If we haven't yet reached the goal, set the velocity value.
// Otherwise, stop the robot.
void setVelocity() {
 
  double distanceTonextpose = getDistanceTonextpose();
  double headingError = getHeadingError();
 
  // If we are not yet at the nextpose
  if (goTonextpose == true && (abs(distanceTonextpose) > distanceTolerance)) {
    
    // If the robot's heading is off, fix it.
    if (abs(headingError) > angleTolerance) {
      velCommand.linear.x = 0.0;
      velCommand.angular.z = K_a  * headingError;
    }
    // Just fix the distance gap between current pose and nextpose.
    // The magnitude of the robot's velocity is directly
    // proportional to the distance the robot is from the 
    // goal.
    else {
      velCommand.linear.x = K_l * distanceTonextpose;
      velCommand.angular.z = 0.0;    
    }
  }
  else {
    cout << "Goal has been reached!" << endl << endl;
    velCommand.linear.x = 0.0;
    velCommand.angular.z = 0.0; 
    goTonextpose = false;
  }
}
 
// This callback function updates the current position and 
// orientation of the robot. 
void updatePose(const geometry_msgs::Pose2D &currentPose) {
  current.x = currentPose.x;
  current.y = currentPose.y;
  current.theta = currentPose.theta;
}
 
// This callback function updates the desired nextpose when a nextpose
// message is published to the /nextpose topic
void updatenextpose(const geometry_msgs::Pose2D &nextposePose) {
  nextposeGoal.x = nextposePose.x;
  nextposeGoal.y = nextposePose.y;
  goTonextpose = true;
}
 
int main(int argc, char **argv) {
 
  setup();  
 
  // Initiate ROS
  ros::init(argc, argv, "steer2nextpose");
     
  // Create the main access point to communicate with ROS
  ros::NodeHandle node;
 
  // Subscribe to the robot's pose
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  // Every time a new pose is received, update the robot's pose.
  ros::Subscriber currentPoseSub =
    node.subscribe("pedbot/pose", 0, updatePose);
     
  // Subscribe to the user's desired nextpose
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  // Every time a new nextpose is received, update the robot's 
  // desired nextpose.
  // The tcpNoDelay is to reduce latency between nodes and to make sure we are
  // not missing any critical nextpose messages.
  ros::Subscriber nextposePoseSub =
    node.subscribe("nextpose", 0, updatenextpose, 
    ros::TransportHints().tcpNoDelay());
 
  // Publish velocity commands to a topic.
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  velocityPub =
    node.advertise<geometry_msgs::Twist>("/pedbot/control/cmd_vel", 0);
 
  // Specify a frequency that want the while loop below to loop at
  // In this case, we want to loop 10 cycles per second
  ros::Rate loop_rate(10); 
 
  // Keep running the while loop below as long as the ROS Master is active. 
  while (ros::ok()) {
    // Here is where we call the callbacks that need to be called.
    ros::spinOnce();
 
    // After we call the callback function to update the robot's pose, we 
    // set the velocity values for the robot.
    if(goTonextpose == true) setVelocity();
 
    // Publish the velocity command to the ROS topic
    velocityPub.publish(velCommand);
 
    // Print the output to the console
    /*
    cout << "Current (x,y) = " << "(" << current.x << "," << current.y << ")"
         << endl
         << "nextpose (x,y) = " << "(" << nextposeGoal.x << ","
         << nextposeGoal.y << ")"
         << endl
         << "Distance to nextpose = " << getDistanceTonextpose() << " m"
         << endl
         << "Linear Velocity (x) = " << velCommand.linear.x << " m/s"
         << endl << endl;
    */
    // Sleep as long as we need to to make sure that we have a frequency of
    // 10Hz
    loop_rate.sleep();
  }
 
  return 0;
}
