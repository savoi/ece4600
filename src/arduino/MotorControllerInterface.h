#include "AbstractDevice.h"
#include "Arduino.h"
#include "MotorPortInterface.h"
#include <vector>

#ifndef __MOTOR_CONTROLLER_INTERFACE__
#define __MOTOR_CONTROLLER_INTERFACE__

class MotorControllerInterface: public AbstractDevice
{
  protected:
      
      bool emergencyStopped;
      
  public:
  
      MotorControllerInterface()
      {
        
      }
      
      //vector<MotorPortInterface> getPorts();
      virtual void emergencyStop();
      virtual void releaseEmergencyStop();
      
      bool getEmergencyStopped();
};

#endif

