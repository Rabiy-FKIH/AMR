#ifndef _4_WHEELED_ROBOT_H
#define _4_WHEELED_ROBOT_H
#include "Wheel.h"

// Define the GPIO pins for the motors
static struct DRAM_ATTR MotorPins {
  const byte motorPWM; // motor PWM pin
  const byte motorDir; // motor Direction pin
  const byte encoderA; // encoder channel A
  const byte encoderB; // encoder channel B
} motorPinGroup[4] = {1, 2, 3, 4, 5, 
  6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

  class DriveTrain
{
  public:

    // Constructor
    DriveTrain(ros::NodeHandle * nodeHandle);

    uint8_t leftFrontWheelPinGroup = 0; // GPIO pin group config.h
    uint8_t rightFrontWheelPinGroup = 1; // GPIO pin group config.h
    uint8_t rightRearWheelPinGroup = 2; // GPIO pin group config.h
    uint8_t leftRearWheelPinGroup = 3; // GPIO pin group config.h

    const float wheelDiameter = 0.127; // wheel diameter in meters
    const uint8_t wheelType = STANDARD_FIXED;
    const float wheelHorizontalSeparation = 0.35; // left and right wheels separation  in meters
    const float wheelVerticalSeparation = 0.35; // front and rear wheels separation in meters

    Wheel leftFrontWheel (leftFrontWheelPinGroup, wheelDiameter, wheelType);
    Wheel rightFrontWheel = Wheel(rightFrontWheelPinGroup, wheelDiameter, wheelType);
    Wheel rigntRearWheel = Wheel(rightRearWheelPinGroup, wheelDiameter, wheelType);
    Wheel leftRearWheel = Wheel(leftRearWheelPinGroup, wheelDiameter, wheelType);

	// Pointer to the ROS node
	ros::NodeHandle * nh_; 

	// Robot state 
	nav_msgs::Odometry state;

	// Define publisher for ROS
	ros::Publisher* pub_odom;

	//--- Drive train functions ---
	void setWheelSpeeds(float leftWheelsSpeed, float rightWheelsSpeed);

	void publishState();

	float * getLocalPose();

	float * getLocalVelocity();

	float calculateArcWheelRatios(float arcRadius);

	void printLocalPose();

  private:
    // --- Odometry state variables and methods ---

    // Keeps track of encoder pulses from each wheel
    uint32_t leftFrontPositionLast_ = 0, rightFrontPositionLast_ = 0, rightRearPositionLast_ = 0, leftReartPositionLast_ = 0;
    const int maxPulsesPerSecond_ = 496; //for single channel RPS*PPR

    static DriveTrain * instances [1];

    // Static instance to update robot state
    static void updateStateISR(void *pArg)
    {
      if (DriveTrain::instances[0] != NULL)
        DriveTrain::instances[0]->updateState_();
    }

    // Instance member to update robot state. Called from updateStateISR
    void updateState_(); 
};


    #endif 