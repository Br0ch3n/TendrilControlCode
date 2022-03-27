#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <Wire.h>

#include "MiniTen.h"

void DCMotor::run(uint8_t cmd){
  switch(cmd){
    case FORWARD:
    MC->setPin(dirOutpin, HIGH);  // take low first to avoid 'break
    break;
  case BACKWARD:  // take low first to avoid 'break'
    MC->setPin(dirOutpin, LOW);
    break;
  case RELEASE:
    MC->setPin(dirOutpin, LOW);
    break;
  }
}

void DCMotor::setTenIntercept(float newInterc) {
  tenInterc = newInterc;
}

DCMotor::DCMotor(void) {
  MC = NULL;
  motornum = 0;
  PWMpin = dirOutpin = dirInpin = tenInPin = countPin = m_pgain = m_dgain = 0;
  count = 0;
  tenScale = 0; tenInterc = 0;
  m_prevError = 0x80000000;
}

DCMotor *tendril::getMotor(uint8_t num, uint8_t pwm, uint8_t dir,
                            uint8_t trig, uint8_t dir1, uint32_t pgain,
                            uint32_t dgain, uint8_t aIn, float tScale, float tInterc)
  {
  if (num > 9) return NULL;

  num--;

  if (dcmotors[num].motornum == 0) {
    // not init'd yet!
    dcmotors[num].motornum = num;
    dcmotors[num].MC = this;
    dcmotors[num].PWMpin = pwm;
    dcmotors[num].dirOutpin = dir;
    dcmotors[num].countPin = trig;
    dcmotors[num].tenInPin = aIn;
	  dcmotors[num].dirInpin = dir1;
    dcmotors[num].tenScale = tScale;
    dcmotors[num].tenInterc = tInterc;
	  pinMode(dir,OUTPUT);
	  pinMode(dir1,INPUT);
    pinMode(aIn, INPUT);
	  pinMode(trig,INPUT_PULLUP);
	  dcmotors[num].m_pgain = pgain;
	  dcmotors[num].m_dgain = dgain;
  }
  return &dcmotors[num];
}

void tendril::setPin(uint8_t pin, boolean value) {
    digitalWrite(pin, value);
}

void DCMotor::setSpeed(uint8_t speed) {
  analogWrite(PWMpin, speed);
}

 int32_t DCMotor::getTension() {
  return tenValue;
}

void DCMotor::updateTension() {
  tenReading = analogRead(tenInPin);
  tenValue = (tenReading * tenScale) + tenInterc;
}

void DCMotor::update(int32_t errorL)
{
  long int vel;


  updateTension();

  if (m_prevError != 0x80000000)
  {
    vel = (long int)(errorL * m_pgain + (errorL - m_prevError) * m_dgain) >> 8;


    //m_pos += velocity;
    if (motornum < 3) {
      if (vel > LG_MOTOR_MAX) {
        vel = LG_MOTOR_MAX;
      }
      else if (vel < LG_MOTOR_MIN)
      {
        vel = LG_MOTOR_MIN;
      }
    } else {
      if (vel > SM_MOTOR_MAX)
      {
        vel = SM_MOTOR_MAX;
      }
      else if (vel < SM_MOTOR_MIN)
      {
        vel = SM_MOTOR_MIN;
      }
    }


  }

  if (abs(errorL) <= MIN_ERR_THRESH) vel = 0;
 // Serial.println(vel);
  //Serial.print("\t");
  if (vel >= 0) {
    run(BACKWARD);
    setSpeed((uint8_t)abs(vel));
  }
  else {
    run(FORWARD);
    setSpeed((uint8_t)abs(vel));
  }



  m_prevError = errorL;
}
