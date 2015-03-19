#include "ServoInterface.h"
#include "Arduino.h" 

#ifndef __MANIPULATOR_ARM__
#define __MANIPULATOR_ARM__

#define TxMode LOW
#define RxMode HIGH

const double MAX_SERVO_ANGLE = 80;

const double BOTTOM_SERVO_ZERO_ANGLE = 90;
const double MIDDLE_SERVO_ZERO_ANGLE = 270;
const double TOP_SERVO_ZERO_ANGLE = 180;

const double TOP_SERVO_UPRIGHT_ANGLE = 90;
const double TOP_SERVO_PASS_THROUGH_ANGLE = 5;

class ManipulatorArm: public AbstractDevice
{
  protected:
    USARTClass* serialBus;
    int ctlPin;
    
    ServoInterface* servoBottom;
    ServoInterface* servoMiddle;
    ServoInterface* servoTop;
    ServoInterface* servoPan;
    
    double lowerLength;
    double upperLength;
    
    double currentHeight = 0;
      
  public:
    
    ManipulatorArm(int id, USARTClass& sBus, ServoInterface& s1, ServoInterface& s2, ServoInterface& s3, ServoInterface& s4, double lengthA, double lengthB, int cPin)
    {
      deviceID = id;
      serialBus = &sBus;
      ctlPin = cPin;
      
      servoBottom = &s1;
      servoMiddle = &s2;
      servoTop = &s3;
      servoPan = &s4;
      
      //for later use in exact positioning
      lowerLength = lengthA;
      upperLength = lengthB;
    }
    
    //implemented interface functions from abstract parent "AbstractDevice"
    bool initializeInterface(bool deviceAlreadyInitialized);
    bool initializeDevice();
    void shutdownDevice();
    int getData(short* data);
  
    void setHeight(double percentHeight, double percentVelocity);
    void setPan(int panDegrees);
};

#endif

