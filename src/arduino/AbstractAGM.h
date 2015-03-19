#include "AbstractDevice.h"

#ifndef __ABSTRACT_AGM__
#define __ABSTRACT_AGM__

class AbstractAGM: public AbstractDevice
{
  protected:
      double latestAxisAcceleration[3];
      double latestEulerAngles[3];
      
  public:
      
      //returns axisAcceleration: [0] = x, [1] = y, [2] = z
      virtual bool getLinearAcceleration(double* axisAcceleration);
      
      //returns eulerAngles: [0] = roll, [1] = pitch, [2] = yaw
      virtual bool getEulerAngles(double* eulerAngles);
      
      virtual bool getAngularVelocities(double* angularVelocities);
};

#endif

