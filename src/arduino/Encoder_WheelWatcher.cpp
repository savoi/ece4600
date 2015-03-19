
#include "Arduino.h"
#include "Encoder_WheelWatcher.h"

bool Encoder_WheelWatcher::initializeInterface(bool deviceAlreadyInitialized)
{
  //pinMode(clkPin, INPUT);
  pinMode(dirPin, INPUT);
  attachInterrupt(clkPin, fcnPtr, FALLING);
  return true;
}

bool Encoder_WheelWatcher::initializeDevice()
{
  pos = 0;
  deviceStatus = DEV_RUNNING;
  
  return true;
}

void Encoder_WheelWatcher::shutdownDevice()
{
  detachInterrupt(clkPin);
}

void Encoder_WheelWatcher::clockPulse()
{
  pinMode(dirPin, INPUT);
  if(digitalRead(dirPin)==HIGH && !reverse)
  {
    if(pos<oneRotation)
    {
      pos++;
    }else
    {
      pos = 0;
    }
  }else
  {
    if(pos>0)
    {
      pos--;
    }else
    {
      pos = oneRotation;
    }    
  }
}

//returns angle in degrees
double Encoder_WheelWatcher::getAngle()
{
  noInterrupts();
  double temp = (pos/oneRotation)*360.0;
  interrupts();
  
  return temp;
}

int Encoder_WheelWatcher::getData(short* data)
{
  data[0] = (short)(getAngle()*DECIMAL_PRECISION_FACTOR);
  
  return 1;
}

