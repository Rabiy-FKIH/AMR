#ifndef ENCODER_H
#define ENCODER_H


 class Encoder
{  
   public:

    // Class variables
    static const int PPR = 120;  // Encoder Count per Revolutions for single channel 
    
    // Default constructor
    Encoder() {}

    // Constructor to connect encoder GPIO pins to microcontroller
    Encoder(uint8_t pinGroup);

    volatile int32_t pulses;
    int8_t wheelDirection = 0;

    static Encoder * instances [4];
   
  private:

    uint8_t pinGroup_;

    // Encoder interrupt routines
    static void encoderISR0 ()
    {
      if (Encoder::instances [0] != NULL)
        Encoder::instances [0]->encoderFired_();
    } 
    
    static void encoderISR1 ()
    {
      if (Encoder::instances [1] != NULL)
        Encoder::instances [1]->encoderFired_();
    }
    
    static void encoderISR2 ()
    {
      if (Encoder::instances [2] != NULL)
        Encoder::instances [2]->encoderFired_();
    } 
    
    static void encoderISR3 ()
    {
      if (Encoder::instances [3] != NULL)
        Encoder::instances [3]->encoderFired_();
    }

    // Checks encoder A
    void IRAM_ATTR encoderFired_();   
};

#endif