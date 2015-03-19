#include "TemperatureSensor_Analog.h"

bool TemperatureSensor_Analog::initializeInterface(bool deviceAlreadyInitialized)
{
  return true;
}

bool TemperatureSensor_Analog::initializeDevice()
{
  deviceStatus = DEV_RUNNING;
  
  return true;
}

void TemperatureSensor_Analog::shutdownDevice()
{
  
}

int TemperatureSensor_Analog::getData(short* data)
{
  data[0] = (short)(latestTemperature*DECIMAL_PRECISION_FACTOR);
  
  return 1;
}

double TemperatureSensor_Analog::readTemperatureSensor()
{
  latestTemperature = analogRead(ADCPin) * BIT_TO_VOLTAGE_CONSTANT * manufacturerVoltageToTemperatureConstant;
  return latestTemperature;
}


