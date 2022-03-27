#ifndef _MiniTen_h_
#define _MiniTen_h_

#include <inttypes.h>
#include <Wire.h>
#include "Arduino.h"

#define FORWARD  1
#define BACKWARD 2
#define RELEASE  3

#define RIGHTM 1
#define LEFTM  2

#define LEFT_AXIS       1
#define RIGHT_AXIS      0
#define Y_TRACK         190 // sets how close the robot gets to the object before stopping

// Speeds for motors by size max value is 255
#define LG_MOTOR_MAX       150 
#define LG_MOTOR_MIN       -150
#define SM_MOTOR_MAX       50
#define SM_MOTOR_MIN       -50

#define MIN_ERR_THRESH  3
class tendril;

class DCMotor
{
   public:
	   int32_t count;
	   uint8_t dirInpin, dirOutpin;
     DCMotor(void);
     friend class tendril;
	   void setSpeed(uint8_t);
     void run(uint8_t);
	   void update(int32_t error);
     void setTenIntercept(float newInterc);
     int32_t getTension();

   private:
     uint8_t PWMpin, countPin, tenInPin;
     float tenScale, tenInterc;
	   uint32_t m_pgain, m_dgain;
     int32_t tenReading, tenValue;
     void updateTension();

     tendril *MC;
     uint8_t motornum;
	   int32_t m_prevError;
};

class tendril{
  public:
    DCMotor *getMotor(uint8_t num, uint8_t pwm, uint8_t dir, uint8_t trig, uint8_t dir1, uint32_t pgain, uint32_t dgain, uint8_t aIn, float tScale, float tInterc);
    void setPin(uint8_t pin, boolean val);
    friend class DCMotor;
  private:
    DCMotor dcmotors[9];
};

#endif
