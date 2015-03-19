#include "Motor.h"
#include "Arduino.h"

#ifndef __MOTOR_PORT_INTERFACE__
#define __MOTOR_PORT_INTERFACE__

class MotorPortInterface
{
  protected:
      Motor* motor;

  public:
  
      MotorPortInterface()
      {
        
      }
      
      virtual void setMotorSpeed(int percent, bool direction);
};

#endif

