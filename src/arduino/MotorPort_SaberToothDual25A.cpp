#include "Arduino.h"
#include "MotorPort_SaberToothDual25A.h"
#include "Motor.h"

//cmd = 0 or 1 for motorA (forward, reverse)
//cmd = 4 or 5 for motorB (forward, reverse)
//constrain throttle (output voltage) below the motor maxVoltage
void MotorPort_SaberToothDual25A::setMotorSpeed(int percent, bool direction)
{
  byte motorOffset = 0;
  if(port==PORT_B)
  {
    motorOffset = 4;
  }

  double powerPercentage = (percent/100.0);
  if(powerPercentage>1)
  {
    powerPercentage = 1;
  }
  if(powerPercentage<0)
  {
    powerPercentage = 0;
  }

  double powerCorrectionConstant = (motor->getMaxVoltage() / motorControllerInputVoltage);
  if(powerCorrectionConstant>1)
  {
    powerCorrectionConstant = 1;
  }
  if(powerCorrectionConstant<0)
  {
    powerCorrectionConstant = 0;
  }

  byte speed = (byte)(powerPercentage * powerCorrectionConstant * SABERTOOTH_FULL_THROTTLE);

  if(direction == motor->getDirection())
  {
    sendCommand(address,SABERTOOTH_FORWARD + motorOffset,speed);
  }else
  {
    sendCommand(address,SABERTOOTH_REVERSE + motorOffset,speed);
  }
}

/*
expects addr between 128 and 135

expects value between 0 and 127
  0 is min throttle
  127 is full throttle
*/
void MotorPort_SaberToothDual25A::sendCommand(byte addr, byte cmd, byte value)
{
  unsigned int chksum = (addr + cmd + value)&0x7F;
  byte data[4];
  data[0] = addr;
  data[1] = cmd;
  data[2] = value;
  data[3] = chksum;
  serialBus->write(data,4);
}


void MotorPort_SaberToothDual25A::setMotorControllerInputVoltage(double voltage)
{
  motorControllerInputVoltage = voltage;
}

