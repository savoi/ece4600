#include "Arduino.h"
#include "MotorPortInterface.h"
#include "Motor.h"

#ifndef __MOTOR_PORT_SABERTOOTH_DUAL_25A__
#define __MOTOR_PORT_SABERTOOTH_DUAL_25A__

const byte SABERTOOTH_FULL_THROTTLE = 127;
const bool PORT_A = true;
const bool PORT_B = false;

const byte SABERTOOTH_FORWARD = 0;
const byte SABERTOOTH_REVERSE = 1;

class MotorPort_SaberToothDual25A: public MotorPortInterface
{
  protected:
    bool port;
    USARTClass* serialBus;
    byte address;
    double motorControllerInputVoltage = 30; //default to fully charged battery just in case voltage isnt read until motors have been driven already

  public:
    
    MotorPort_SaberToothDual25A(bool p, USARTClass& sBus, byte addr, Motor& m)
    {
      serialBus = &sBus;
      port = p;
      address = addr;
      motor = &m;
    }
    
    

    //implemented interface functions from interface parent "MotorPortInterface"
    void setMotorSpeed(int percent, bool direction);

    //SaberTooth specific methods
    void sendCommand(byte addr, byte dir, byte value);

    void setMotorControllerInputVoltage(double voltage);

};

#endif



