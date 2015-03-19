#include "Arduino.h"
#include "MotorController_SaberToothDual25A.h"

bool MotorController_SaberToothDual25A::initializeInterface(bool deviceAlreadyInitialized)
{
  pinMode(emergencyStopPin, OUTPUT);
  if(deviceAlreadyInitialized)
  {
    serialBus->begin(38400);
  }else
  {
    serialBus->begin(9600);
  }
  
  return true;
}

bool MotorController_SaberToothDual25A::initializeDevice()
{
  //wait for 2 seconds
  while(millis()<2000);
  
  //set baud rate here 38400
  setBaudRate(38400);
  
  serialBus->begin(38400);
  
  setRamp(40);
  
  setTimeOut(10);
  
  deviceStatus = DEV_RUNNING;
  
  return true;
}

void MotorController_SaberToothDual25A::shutdownDevice()
{
  portA->setMotorSpeed(0,true);
  portB->setMotorSpeed(0,true);
  emergencyStop();
}

int MotorController_SaberToothDual25A::getData(short* data)
{
  if(emergencyStopped)
  {
    data[0] = 1;
  }else
  {
    data[0] = 0;
  }
  
  data[1] = (short)(latestInputVoltage*DECIMAL_PRECISION_FACTOR);
  
  return 2;
}

void MotorController_SaberToothDual25A::setBaudRate(int rate)
{
  byte value;
  switch (rate)
  {
    case 2400:           
      value = 1; break;
    case 9600:           
      value = 2; break;
    case 19200:          
      value = 3; break;
    case 38400:         
      value = 4; break;
    case 115200:         
      value = 5; break;
    default:
      value = 2; break;
  }
  sendCommand(serialAddress,SABERTOOTH_SET_BAUDRATE_CMD,value);
  //delay for sabertooth to restart with new baud rate
  delay(2000);
}

void MotorController_SaberToothDual25A::setRamp(int rate)
{
  if(rate<0)
  {
    rate = 0;
  }
  if(rate>80)
  {
    rate = 80; 
  }
  
  sendCommand(serialAddress,SABERTOOTH_SET_RAMP_CMD,rate);
}

void MotorController_SaberToothDual25A::setTimeOut(int hundredsOfMilliSeconds)
{
  if(hundredsOfMilliSeconds>0)
  {
    sendCommand(serialAddress,SABERTOOTH_SET_TIMEOUT_CMD,hundredsOfMilliSeconds);
  }
}

void MotorController_SaberToothDual25A::sendCommand(byte addr, byte cmd, byte value)
{
  unsigned int chksum = (addr + cmd + value)&0x7F;
  byte data[4];
  data[0] = addr;
  data[1] = cmd;
  data[2] = value;
  data[3] = chksum;
  serialBus->write(data,4);
}

void MotorController_SaberToothDual25A::emergencyStop()
{
  digitalWrite(emergencyStopPin, HIGH);
}

void MotorController_SaberToothDual25A::releaseEmergencyStop()
{
  digitalWrite(emergencyStopPin, LOW);
}

void MotorController_SaberToothDual25A::setMotorControllerInputVoltage(double voltage)
{
  latestInputVoltage = voltage;
  
  portA->setMotorControllerInputVoltage(voltage);
  portB->setMotorControllerInputVoltage(voltage);
}

MotorPort_SaberToothDual25A* MotorController_SaberToothDual25A::getPortA()
{
  return portA;
}

MotorPort_SaberToothDual25A* MotorController_SaberToothDual25A::getPortB()
{
  return portB;
}

