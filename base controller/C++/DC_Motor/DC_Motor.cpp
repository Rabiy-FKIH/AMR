#include "DCMotor.h"


// -------------------Constructors -----------------------------------
DCMotor::DCMotor(uint8_t pinGroup) 
  :encoder(pinGroup), pinGroup_(pinGroup)
{   
  // Connect motor to GPIO pins
  pinMode(motorPinGroup[pinGroup].motorPWM, OUTPUT); // motor PWM pin
  pinMode(motorPinGroup[pinGroup].motorDir, OUTPUT); // motor direction pin


  // Setup PWM signal
  ledcSetup(pinGroup_, freq, resolution); // create a PWM channel 
  ledcAttachPin(motorPinGroup[pinGroup_].motorPWM, pinGroup_); // attach channel to pin

  // Start motor power timers 
  switch (pinGroup)
  {
  case 0: 
    {
      const esp_timer_create_args_t periodic_timer_args = {.callback = &motorISR0};
      esp_timer_create(&periodic_timer_args, &motorTimer0);
      esp_timer_start_periodic(motorTimer0, speedCtrlPeriodMicros); // Time in milliseconds (50)
      instances [0] = this; 	
    }
    break;
    
  case 1: 
    {
      const esp_timer_create_args_t periodic_timer_args = {.callback = &motorISR1};
      esp_timer_create(&periodic_timer_args, &motorTimer1);
      esp_timer_start_periodic(motorTimer1, speedCtrlPeriodMicros); // Time in milliseconds (50)
      instances [1] = this;
    }
    break;

      case 2: 
    {
      const esp_timer_create_args_t periodic_timer_args = {.callback = &motorISR2};
      esp_timer_create(&periodic_timer_args, &motorTimer2);
      esp_timer_start_periodic(motorTimer2, speedCtrlPeriodMicros); // Time in milliseconds (50)
      instances [2] = this; 
    }
    break;
    
  case 3: 
    {
      const esp_timer_create_args_t periodic_timer_args = {.callback = &motorISR3};
      esp_timer_create(&periodic_timer_args, &motorTimer3);
      esp_timer_start_periodic(motorTimer3, speedCtrlPeriodMicros); // Time in milliseconds (50)
      instances [3] = this;
    }
    break;
    
  } // end of switch
}

// -------------------------------------------------------- 
// Sets the local pulseSetpoint which is picked up in the
// setPower_() periodic timer. 
// --------------------------------------------------------
void DCMotor::setSpeed(float pulseSetpoint) {

  // PulseSetpoint drives the action of the setPower_() routine
  pulseSetpoint_ = pulseSetpoint;
  encoder.wheelDirection = sgn(pulseSetpoint);
} 

void IRAM_ATTR DCMotor::setPower_() {

  // Get the number of pulses since the last period
  int32_t pulses = encoder.pulses;  
  int32_t pulsesThisPeriod = abs(pulses - pulsesLast_);
     
  // Save the last pulses
  pulsesLast_ = pulses;

  // Compute the error between requested pulses/sec and actual pulses/sec
  pulsesPerSec_ = pulsesThisPeriod * periodsPerSec;
  error_ = abs(pulseSetpoint_) - pulsesPerSec_; 
  
  // --- PI control ---
  pPart_ = Kp * error_; // Proportional
  iPart_ += Ki * error_; // Integral

  // We've put the setpoint to zero, stopping the motors
  if (iPart_ < 0.0) { 
    iPart_ = 0.0; // Don't let integral part go negative
  }

  // Compute the PWM
  PWM_ = int(pPart_ + iPart_);

  // Motors are shutting down																						
  if (pulseSetpoint_ == 0 && iPart_ == 0.0) { 
    PWM_ = 0;
  } 
 
  // Apply the power with the direction and PWM signal
  applyPower_(sgn(pulseSetpoint_), PWM_);
} 

// ------------------------------------------------
// Apply power to motor 
// ------------------------------------------------
void IRAM_ATTR DCMotor::applyPower_(int dir, int PWM) {

  if(dir > 0) {
    dir = HIGH; 
  } else {
    dir = LOW; 
  }

  digitalWrite(motorPinGroup[pinGroup_].motorDir, dir); // Direction HIGH forward, LOW backward

  // See setupPWM(pinGroup) in Config.h
  ledcWrite(pinGroup_, abs(PWM));
} 