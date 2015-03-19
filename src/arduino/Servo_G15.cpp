#include "Arduino.h"
#include "Servo_G15.h"

bool Servo_G15::initializeInterface(bool deviceAlreadyInitialized)
{
  //void setControlPin(ctlPin);
  return true;
}

bool Servo_G15::initializeDevice()
{
  g15->SetLED(ON,iWRITE_DATA);
  
  //g15->SetAngleLimit(CW_maxAngle,CCW_maxAngle);
  
  g15->SetTorqueOnOff(ON, iWRITE_DATA);
  
  deviceStatus = DEV_RUNNING;
  
  return true;  //TODO
}

void Servo_G15::shutdownDevice()
{
  
}

int Servo_G15::getData(short* data)
{
  getAngle(); //do this for now because we aren't calling it anywhere else
  
  data[0] = (short)(latestAngle*DECIMAL_PRECISION_FACTOR);
  return 1;
}

void Servo_G15::setAngle(double angleInDegrees)
{
  g15->SetPos((int)(angleInDegrees*1088.0/360.0), iWRITE_DATA);
}

void Servo_G15::setAngle(double angleInDegrees, double servoSpeed)
{
  //Serial.println(angleInDegrees);
  g15->SetPosSpeed((int)(angleInDegrees*1088.0/360.0), (int)servoSpeed, iWRITE_DATA);
}

void Servo_G15::RotateCW(double angleInDegrees, double servoSpeed)
{
  g15->SetSpeed((int)servoSpeed, iWRITE_DATA);
  g15->RotateCW((int)(angleInDegrees*1088.0/360.0), iWRITE_DATA);
}

void Servo_G15::RotateCCW(double angleInDegrees, double servoSpeed)
{
  g15->SetSpeed((int)servoSpeed, iWRITE_DATA);
  g15->RotateCCW((int)(angleInDegrees*1088.0/360.0), iWRITE_DATA);
}

double Servo_G15::getAngle()
{
  byte data[2];
  g15->GetPos(data);
  
  latestAngle =  (data[0] + (data[1]<<8))/1088.0*360.0;
  
  return latestAngle;
}

void Servo_G15::setHoldingTorque(bool onOff)
{
  if(onOff)
  {
    g15->SetTorqueOnOff(ON, iWRITE_DATA);
  }else
  {
    g15->SetTorqueOnOff(OFF, iWRITE_DATA);
  }
}

