#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

/* In order to connect to ROS you’ll need to specify the network address of the machine
on which the ROS server is hosted together with the port that it’s listening on */
// Jetson Nano host address. Default ROS port is 11411
IPAddress server(192,168,0,ROS_HOST); // Set the rosserial socket server IP address
const uint16_t serverPort = ROS_PORT; // Set the rosserial socket server port

/*You also have to create a node handle to manage the connection between your control program and the ROS server.*/
// Define the node handle to roscore
ros::NodeHandle nh;
bool nodeHandleCreated = false;


// Command callback
//--------------------------------------------//
void commandCb( const geometry_msgs::Twist& cmd_vel) {

  // Print the new pose command message to the ROS console
  ROSLOGString("Received: X_VELOCITY, Y_VELOCITY, Z_ANGULAR");
  ROSLOG3F(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  
  // Get the robot's current pose
  float *currentPose = robot->getLocalPose();

  // Print the robot's current pose to the ROS console
  robot->driveTrain.printLocalPose();

  // Assign new target pose. Current state + new pose.
  refPose[X_POS] = currentPose[X_POS] + cmd.position.x;
  refPose[Y_POS] = currentPose[Y_POS] + cmd.position.y;
  refPose[PSI] = currentPose[PSI] + cmd.orientation.y / (180/PI); // in radians 

  // Print target pose to the ROS console
  ROSLOGString("Target Pose X, Y, Psi");
  ROSLOG3F(refPose[X_POS], refPose[Y_POS], refPose[PSI]);
  
  // Set the commandCalled flag
  commandCalled = true;  
}
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

//-------------------------------------------------//
// Main command loop
//-------------------------------------------------//
void loop() {
    
  // Loop until command is sent.
  if (commandCalled) {

    // Position control loop
    robot->poseController(refPose);

    // Reset commandCalled flag
    commandCalled = false;
  }  

  // Wait for subscribed messages 
  if (nodeHandleCreated) {nh.spinOnce();}
  delay(1000);
}