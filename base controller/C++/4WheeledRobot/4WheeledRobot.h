#ifndef _4_WHEELED_ROBOT_H
#define _4_WHEELED_ROBOT_H

class _4WheeledRobot
{
  public:

    // --- Constructor ---
    _4WheeledRobot(ros::NodeHandle * nodeHandle);

    // --- Robot configuration ---

    // Define the shape and size(in meter) of the robot
    struct Geometry {
      uint8_t type = cuboid;
      float length = 0.58; 
      float width = 0.49;
      float height = 0.5
    };

    // Drive train of robot
    DriveTrain driveTrain;

    // --- Kinematic functions ---------
    void moveAngular(float * refPose, float arcRadius);

    void moveLinear(float * refPose);

    void poseController(float * refPose);

    // Drive robot in an arc with the specified radius
    float calculateArcWheelRatios(float radius);
  private:
    // --- Variables used for kinematics 

    // Error matrix for feedback loop
    float poseError[3] = {0.0, 0.0, 0.0};

    double currentTime_;
    const double poseLoopPeriod_ = 0.05; // 50 milliseconds
    const int poseLoopPeriodMillis_ = poseLoopPeriod_ * 1000; // 50 milliseconds
    const int poseLoopPeriodsPerSecond_ = 1000/poseLoopPeriodMillis_;
    const int timeOut_ = poseLoopPeriodsPerSecond_ * 8; // 8 seconds

    // PI control for pose loop. Adjust gain constants as necessary
    const float startUpKi_ = 0.05;  // Ki value to startup robot
    const float Kp_ = 3.0; // Gets within 15% of target 
    const float Ki_ = 0.3; // Maintains a minimum speed of 0.3
    
    // Move phases
    const byte STARTUP = 1;
    const byte RUNNING = 2;
    const byte SHUTDOWN = 3;
    const byte STOPPED = 4; 

};

#endif 