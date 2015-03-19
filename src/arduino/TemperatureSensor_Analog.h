#include "TemperatureSensorInterface.h"
#include "Arduino.h"
#include "Robot_Arduino_main.h"

#ifndef __TEMPERATURE_SENSOR_ANALOG__
#define __TEMPERATURE_SENSOR_ANALOG__

class TemperatureSensor_Analog: public TemperatureSensorInterface
{
  protected:
      int ADCPin;
      double manufacturerVoltageToTemperatureConstant;
      
  public:
      
      TemperatureSensor_Analog(int id, int ADCp, double voltToTempConst)
      {
        deviceID = id;
        ADCPin = ADCp;
        manufacturerVoltageToTemperatureConstant = voltToTempConst;
        latestTemperature = 0;
      }
      
      //implemented interface functions from abstract parent "AbstractDevice"
      bool initializeInterface(bool deviceAlreadyInitialized);
      bool initializeDevice();
      void shutdownDevice();
      int getData(short* data);
      
      double readTemperatureSensor();
};

#endif

