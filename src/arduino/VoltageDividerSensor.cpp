#include "VoltageDividerSensor.h"

bool VoltageDividerSensor::initializeInterface(bool deviceAlreadyInitialized)
{
  return true;
}

bool VoltageDividerSensor::initializeDevice()
{
  deviceStatus = DEV_RUNNING;
  
  return true;
}

void VoltageDividerSensor::shutdownDevice()
{
  
}

int VoltageDividerSensor::getData(short* data)
{
  data[0] = (short)(latestVoltage*DECIMAL_PRECISION_FACTOR);
  
  return 1;
}

double VoltageDividerSensor::readVoltageSensor()
{
  return analogRead(ADCPin) * BIT_TO_VOLTAGE_CONSTANT * measuredVoltageDividerConstant;
}

double VoltageDividerSensor::getLatestVoltage()
{
  return latestVoltage;
}

