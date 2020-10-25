#include <ros.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#include <WiFi.h>

// WiFi network name and password:
const char * networkName = "YOUR_NETWORK_HERE";
const char * networkPswd = "YOUR_PASSWORD_HERE";


#include <ros.h>
// RaspberryPi host address. Default ROS port is 11411
IPAddress server(192,168,0,ROS_HOST); // Set the rosserial socket server IP address
const uint16_t serverPort = ROS_PORT; // Set the rosserial socket server port
// Define the node handle to roscore
ros::NodeHandle nh;
bool nodeHandleCreated = false;

// Include the Pose message
#include <geometry_msgs/Pose.h>
// Define a variable to hold the Pose message
geometry_msgs::Pose pose_msg;
// Subscriber definition
ros::Subscriber<geometry_msgs::Pose> *sub_pose;
// Create matrix to hold the command input
// X position, Y position, Orientation
float refPose[3] = {0.0, 0.0, 0.0};
const byte X_POS = 0;
const byte Y_POS = 1;
const byte PSI = 2;
// Error matrix for feedback loop
float poseError[3] = {0.0, 0.0, 0.0};
// Set callback flag
bool commandCalled = false;


//--------------------------------------------//
// Setup
//--------------------------------------------//
void setup() {
  // Connect to WiFi
  connectWiFi();
  // Initialize the ROS nodehandle.
  nh.getHardware()->setConnection(server, serverPort);
  // Starting node
  nh.initNode();
  nodeHandleCreated = true;
  delay(1000);
  // Setup subscriber with callback function to receive the command
  sub_pose = new ros::Subscriber<geometry_msgs::Pose>("/pose_cmd", commandCb);
  nh.subscribe(*sub_pose);
  // Create a robot. Pass the ROS node handle.
  robot = new TwoWheeledRobot(&nh);

}

void loop() {
  

}
