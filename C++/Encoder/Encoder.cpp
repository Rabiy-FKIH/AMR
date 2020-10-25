#include "Encoder.h"

// ---------------------  Constructors -------------------------------
Encoder::Encoder(uint8_t pinGroup) 
  :pinGroup_(pinGroup)
{   
  // Connect encoder to GPIO pins 
  pinMode(motorPinGroup[pinGroup].encoderA, INPUT_PULLUP); //   encoder, channel A
  pinMode(motorPinGroup[pinGroup].encoderB, INPUT_PULLUP); //   encoder, channel B


  // Attach interrupts
  switch (pinGroup)
  {
  case 0: 
    attachInterrupt (motorPinGroup[0].encoderB, encoderISR0, RISING);  // Left front encoder
    instances [0] = this; 
    break;
    
  case 1: 
    attachInterrupt (motorPinGroup[1].encoderA, encoderISR1, RISING); // Right front encoder
    instances [1] = this;
    break; 

  case 2: 
    attachInterrupt (motorPinGroup[2].encoderA, encoderISR2, FALLING);  // Right rear encoder
    instances [2] = this; 
    break;
    
  case 3: 
    attachInterrupt (motorPinGroup[3].encoderB, encoderISR3, FALLING); // Left rear encoder
    instances [3] = this;
    break;       
  } // end of switch

  // Switch on interrupts
  sei();  
  // Initialize pulses. 
  pulses = 0;
} 


void IRAM_ATTR Encoder::encoderFired_() {
  // pulses is 4 bytes so make sure that the write is not interupted
  portENTER_CRITICAL_ISR(&timerMux);
  
  if (digitalRead(motorPinGroup[pinGroup_].encoderA) != digitalRead(motorPinGroup[pinGroup_].encoderB)) {
    pulses++;
  } else {
    pulses--;  
  } 
  portEXIT_CRITICAL_ISR(&timerMux);
}