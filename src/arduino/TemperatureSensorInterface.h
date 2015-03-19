#include "AbstractDevice.h"

#ifndef __TEMPERATURE_SENSOR_INTERFACE__
#define __TEMPERATURE_SENSOR_INTERFACE__

class TemperatureSensorInterface: public AbstractDevice
{
  protected:
      double latestTemperature;
      
  public:
      
      virtual double readTemperatureSensor();
      
      double getLatestTemperature();
};

#endif

