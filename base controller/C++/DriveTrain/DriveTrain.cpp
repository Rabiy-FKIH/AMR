#include "DriveTrain.h"

// ------------------ Constructor ---------------------------------
DriveTrain::DriveTrain(ros::NodeHandle * nodeHandle) 
:nh_(nodeHandle) 
{ 
  // Attach robot to the global odometry frame
  state.header.frame_id = "/odom"; // Global odometry frame
  state.child_frame_id = "/base_link"; // Robot local frame

  // Setup publisher to report current state
  pub_odom = new ros::Publisher("/odom", &state);
  nh_->advertise(*pub_odom);

  // Start state update timer
  const esp_timer_create_args_t periodic_timer_args = {.callback = &updateStateISR};
  esp_timer_create(&periodic_timer_args, &stateUpdateTimer);
  esp_timer_start_periodic(stateUpdateTimer, updatePeriodMicros); // Time in milliseconds (50) 
  instances[0] = this; 
}

// -----------------------------------------------------------------
// Set the left and right wheels speeds 
// Input is a speed ratio between -1.0 (backwards) and 1.0 (forward) 
// Output is a wheel speed in pulses per/sec
// -----------------------------------------------------------------
void DriveTrain::setWheelSpeeds(float leftWheelsSpeed, float rightWheelsSpeed) {

  // Translate the speed ratio into pulses per/sec.
  // Where speed ratio of 1.0 equals 600 pulses per/sec
  int leftPulseSetpoint = int(maxPulsesPerSecond_ * leftWheelsSpeed);
  int rightPulseSetpoint = int(maxPulsesPerSecond_ * rightWheelsSpeed);
  
  leftFrontWheel.setSpeed(leftPulseSetpoint); // Pulses per/sec
  rightFrontWheel.setSpeed(rightPulseSetpoint); // Pulses per/sec 
  rightRearWheel.setSpeed(rightPulseSetpoint); // Pulses per/sec 
  leftRearWheel.setSpeed(leftPulseSetpoint); // Pulses per/sec
}

// ------------------------------------------------------
// Publish current robot state to ROS
// ------------------------------------------------------
void DriveTrain::publishState() { 
  
  // Add the timestamp
  state.header.stamp = nh_->now(); 

  state.pose.pose.position.x = local_pose[X_POS];
  state.pose.pose.position.y = local_pose[Y_POS]; // zero since cannot move sideways
  state.pose.pose.orientation.y = local_pose[PSI]; // theta

  state.twist.twist.linear.x = local_velocity[X_VELOCITY];
  state.twist.twist.linear.y = local_velocity[Y_VELOCITY];  // Can't move instantaniously sideways
  state.twist.twist.angular.y = local_velocity[Y_ANGULAR]; // angular yaw velocity in radians

  // Publish
  pub_odom->publish( &state );
}

// ------------------------------------------------------
// Update the current robot state 
// ------------------------------------------------------
void DriveTrain::updateState_() {
  
  // --- Get the distance delta since the last period ---  
  float leftPosition = leftWheel.currentPosition();
  float rightPosition = rightWheel.currentPosition();

  // --- Update the local position and orientation ---
  local_pose[X_POS] = (leftPosition + rightPosition) / 2.0; // distance in X direction 
  local_pose[Y_POS] = 0.0; // distance in Y direction
  local_pose[PSI] = (rightPosition - leftPosition) / wheelSeparation; // Change in orientation

  // --- Update the velocity ---
  float leftDistance = (leftPosition - leftPositionLast_); 
  float rightDistance = (rightPosition - rightPositionLast_);
  float delta_distance = (leftDistance + rightDistance) / 2.0; 
  float delta_theta = (rightDistance - leftDistance) / wheelSeparation; // in radians
 
  local_velocity[X_VELOCITY] = delta_distance / updatePeriodMicros; // Linear x velocity
  local_velocity[Y_VELOCITY] = 0.0; 
  local_velocity[Y_ANGULAR] = (delta_theta / updatePeriodMicros); // In radians per/sec

  // ---  Save the last position values ---
  leftPositionLast_ = leftPosition;    
  rightPositionLast_ = rightPosition; 
}  

// ------------------------------------------------------
// Returns the local robot state
// ------------------------------------------------------
float * DriveTrain::getLocalPose() { 
  return local_pose;
}

// ---------------------------------------------------
// Return local robot velocity of robot in meters/sec 
// ---------------------------------------------------
float * DriveTrain::getLocalVelocity() {
  return local_velocity;
}

// ------------------------------------------------------
// Returns the local robot state
// ------------------------------------------------------
void DriveTrain::printLocalPose() { 
  ROSLOGString("Local pose X, Y, Psi");
  ROSLOG3F(local_pose[X_POS], local_pose[Y_POS], local_pose[PSI]);
} 


// ----------------------------------------------------------------
// Calculate the ratio between the two wheels while driving in
// an arc. arcRadius is the Instantaneous Center of Rotation (ICR)
// ----------------------------------------------------------------
float DriveTrain::calculateArcWheelRatios(float arcRadius) {
  
  // Get the distance that each wheel has to travel
  float insideWheel = 2*PI * ( arcRadius - (wheelSeparation/2) );
  float outsideWheel = 2*PI * ( arcRadius + (wheelSeparation/2) );

  // Return the ratio between the two wheels 
  return (insideWheel / outsideWheel);
}