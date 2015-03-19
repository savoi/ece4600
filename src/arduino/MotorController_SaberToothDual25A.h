#include "MotorControllerInterface.h"
#include "MotorPort_SaberToothDual25A.h"
#include "Motor.h"

#ifndef __MOTOR_CONTROLLER_SABERTOOH_DUAL_25A__
#define __MOTOR_CONTROLLER_SABERTOOH_DUAL_25A__

const byte AUTO_BAUD_DETECTION = 0xAA;
const byte SABERTOOTH_SET_TIMEOUT_CMD = 14;
const byte SABERTOOTH_SET_BAUDRATE_CMD = 15;
const byte SABERTOOTH_SET_RAMP_CMD = 16;

class MotorController_SaberToothDual25A: public MotorControllerInterface
{
  protected:
      int emergencyStopPin;
      USARTClass* serialBus;
      byte serialAddress;

      MotorPort_SaberToothDual25A* portA;
      MotorPort_SaberToothDual25A* portB;

      double latestInputVoltage = 30;
      
  public:
  
      MotorController_SaberToothDual25A(int id, USARTClass& sBus, byte serialAddr, Motor& motorA, Motor& motorB, int eStopPin)
      {
        deviceID = id;
        emergencyStopPin = eStopPin;
        serialBus = &sBus;
        serialAddress = serialAddr;

        portA = new MotorPort_SaberToothDual25A(PORT_A,sBus,serialAddr,motorA);
        portB = new MotorPort_SaberToothDual25A(PORT_B,sBus,serialAddr,motorB);
        
        emergencyStopped = false;
      }
      
      ~MotorController_SaberToothDual25A()
      {
        delete portA;
        delete portB;
      }
      
      //implemented interface functions from abstract parent "Device"
      bool initializeInterface(bool deviceAlreadyInitialized);
      bool initializeDevice();
      void shutdownDevice();
      int getData(short* data);

      //implemented from parent interface MotorControllerInterface
      void emergencyStop();
      void releaseEmergencyStop();

      void setRamp(int rate);
      void setTimeOut(int hundredsOfMilliSeconds);
      void setBaudRate(int rate);
      
      void sendCommand(byte addr, byte cmd, byte value);
      MotorPort_SaberToothDual25A* getPortA();
      MotorPort_SaberToothDual25A* getPortB();
      void setMotorControllerInputVoltage(double voltage);
};

#endif

