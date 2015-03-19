#include "ServoInterface.h"
#include "Servo_G15_Manufacturer_Drivers.h"

#ifndef __SERVO_G15__
#define __SERVO_G15__

class Servo_G15: public ServoInterface
{
  protected:
    USARTClass* serialBus;
    int address;
    int ctlPin;
    G15* g15;
    
    double latestAngle;
    
    byte CW_maxAngle;
    byte CCW_maxAngle;
        
  public:
    Servo_G15(int id, USARTClass& sBus, int addr, int cPin,  byte CW_max, byte CCW_max)
    {
      deviceID = id;
      serialBus = &sBus;
      address = addr;
      ctlPin = cPin;
      
      g15 = new G15(sBus, (byte)addr, cPin);
      
      latestAngle = 0;
      
      CW_maxAngle = CW_max;
      CCW_maxAngle = CCW_max;
    }
    
    //implemented interface functions from abstract parent "AbstractDevice"
    bool initializeInterface(bool deviceAlreadyInitialized);
    bool initializeDevice();
    void shutdownDevice();
    int getData(short* data);
    
    //implemented interface functions from parent "ServoInterface"
    void setAngle(double angleInDegrees);
    void setAngle(double angleInDegrees, double servoSpeed);
    void RotateCW(double angleInDegrees, double servoSpeed);
    void RotateCCW(double angleInDegrees, double servoSpeed);
    double getAngle();
    
    void setHoldingTorque(bool onOff);
};

#endif

