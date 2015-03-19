
#include "AbstractDevice.h"
#include "Arduino.h"
#include "Robot_Arduino_main.h"

#ifndef __VOLTAGE_DIVIDER_SENSOR__
#define __VOLTAGE_DIVIDER_SENSOR__

class VoltageDividerSensor: public AbstractDevice
{
  protected:
      int ADCPin;
      double measuredVoltageDividerConstant;
      double latestVoltage = 3.0;
      
  public:
      
      VoltageDividerSensor(int id, int ADCp, double vDivConst)
      {
        deviceID = id;
        ADCPin = ADCp;
        measuredVoltageDividerConstant = vDivConst;
      }
      
      //implemented interface functions from abstract parent "AbstractDevice"
      bool initializeInterface(bool deviceAlreadyInitialized);
      bool initializeDevice();
      void shutdownDevice();
      int getData(short* data);
      
      double readVoltageSensor();
      
      double getLatestVoltage();
};

#endif

