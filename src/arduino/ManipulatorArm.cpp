
#include "ManipulatorArm.h"

bool ManipulatorArm::initializeInterface(bool deviceAlreadyInitialized)
{
  bool initSuccess = true;
  
  serialBus->begin(19200); 
  serialBus->setTimeout(10); 
  
  pinMode(ctlPin, OUTPUT);		//control pin setup		
  
  initSuccess &= servoBottom->initializeInterface(deviceAlreadyInitialized);
  initSuccess &= servoMiddle->initializeInterface(deviceAlreadyInitialized);
  initSuccess &= servoTop->initializeInterface(deviceAlreadyInitialized);
  initSuccess &= servoPan->initializeInterface(deviceAlreadyInitialized);
  
  return initSuccess;
}

bool ManipulatorArm::initializeDevice()
{
  bool initSuccess = true;
  
  digitalWrite(ctlPin,TxMode);
  
  initSuccess &= servoBottom->initializeDevice();
  initSuccess &= servoMiddle->initializeDevice();
  initSuccess &= servoTop->initializeDevice();
  initSuccess &= servoPan->initializeDevice();
  
  if(initSuccess)
  {
    deviceStatus = DEV_RUNNING;
  }
    
  setHeight(50,25);  
    
  return initSuccess;
}

void ManipulatorArm::shutdownDevice()
{
  setHeight(0,25);
  setPan(0);
}

int ManipulatorArm::getData(short* data)
{
  int i = servoBottom->getData(data);
  i += servoMiddle->getData(data + (i*2));
  i += servoTop->getData(data + (i*2));
  i += servoPan->getData(data + (i*2));
  
  return i;
}


/*
NOTE: percent height goes from -100 to 100. 0 to -100 when the robot is inverted
*/
void ManipulatorArm::setHeight(double percentHeight, double percentVelocity)
{
  
  if(percentHeight>100)
  {
    percentHeight = 100;
  }
  if(percentHeight<-100)
  {
    percentHeight = -100;
  }
  
  if(percentVelocity>100)
  {
    percentVelocity = 100;
  }
  if(percentVelocity<0)
  {
    percentVelocity = 0;
  }
  
  int servoSpeed = (int)((percentVelocity/100.0)*1024);
  
  //servoBottom->setAngle(BOTTOM_SERVO_ZERO_ANGLE - MAX_SERVO_ZERO_ANGLE*(percentHeight/100), servoSpeed);
  //servoMiddle->setAngle(((int)(MIDDLE_SERVO_ZERO_ANGLE - 2*MAX_SERVO_ZERO_ANGLE*(percentHeight/100)))%360, servoSpeed);
  
  int aboveBelow = -1;
  int outsideBody = TOP_SERVO_UPRIGHT_ANGLE;
  
  if(percentHeight >= 0)
  {
    aboveBelow = 1;
  }
  if(abs(percentHeight) < TOP_SERVO_PASS_THROUGH_ANGLE)
  {
    outsideBody = 0;
  }
  
  if(percentHeight > currentHeight)
  {
    servoBottom->RotateCW(BOTTOM_SERVO_ZERO_ANGLE - MAX_SERVO_ANGLE*(percentHeight/100), servoSpeed);
    servoMiddle->RotateCCW(((int)(MIDDLE_SERVO_ZERO_ANGLE - 2*MAX_SERVO_ANGLE*(percentHeight/100)))%360, servoSpeed);
    servoTop->RotateCW(TOP_SERVO_ZERO_ANGLE + aboveBelow*(outsideBody + MAX_SERVO_ANGLE*(percentHeight/100)), servoSpeed);
  }else
  {
    servoBottom->RotateCCW(BOTTOM_SERVO_ZERO_ANGLE - MAX_SERVO_ANGLE*(percentHeight/100), servoSpeed);
    servoMiddle->RotateCW(((int)(MIDDLE_SERVO_ZERO_ANGLE - 2*MAX_SERVO_ANGLE*(percentHeight/100)))%360, servoSpeed);
    servoTop->RotateCCW(TOP_SERVO_ZERO_ANGLE + aboveBelow*(outsideBody + MAX_SERVO_ANGLE*(percentHeight/100)), servoSpeed);
  }
  
//  const double TOP_SERVO_ZERO_ANGLE = 180;
//
//const double TOP_SERVO_UPRIGHT_ANGLE = 90;
//const double TOP_SERVO_PASS_THROUGH_ANGLE = 5;
  
  currentHeight = percentHeight;
  //servoTop->setAngle(360.0 - 90.0*(percentHeight/100), servoSpeed);
}

void ManipulatorArm::setPan(int panDegrees)
{
  const int servoSpeed = 500;
  servoPan->setAngle(panDegrees, servoSpeed);
}

