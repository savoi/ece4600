#include "AbstractDevice.h"

#ifndef __SERVO_INTERFACE__
#define __SERVO_INTERFACE__

class ServoInterface: public AbstractDevice
{
  public:
  
    virtual void setAngle(double angleInDegrees);
    virtual void setAngle(double angleInDegrees, double servoSpeed);
    
    virtual void RotateCW(double angleInDegrees, double servoSpeed);
    virtual void RotateCCW(double angleInDegrees, double servoSpeed);
    
    virtual double getAngle();
    
    virtual void setHoldingTorque(bool onOff);
};

#endif

