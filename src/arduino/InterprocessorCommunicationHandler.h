#include "Robot_Arduino_main.h"
#include "CommunicationPortInterface.h"
#include "CommunicationPort_Serial.h"
#include "AbstractDevice.h"
#include <vector>

#ifndef __INTERPROCESSOR_COMMUNICATION_HANDLER__
#define __INTERPROCESSOR_COMMUNICATION_HANDLER__

enum FlipperCommand {FLIPPER_STOP =0, FLIPPER_UP, FLIPPER_DOWN};
enum ManipulatorArmCommand {MANIPULATOR_ARM_STOP=0, MANIPULATOR_ARM_UP, MANIPULATOR_ARM_DOWN };
enum CameraPanCommand {PAN_STOP =0, PAN_LEFT, PAN_RIGHT };
enum CameraTiltCommand {TILT_STOP =0, TILT_UP, TILT_DOWN };

const int INTERPROCESSOR_COMMAND_PACKET_SIZE_IN_BYTES = 5;
const int INTERPROCESSOR_DATA_PACKET_MAX_SIZE_IN_BYTES = 200;

class InterprocessorCommunicationHandler: public AbstractDevice
{
  private:
      CommunicationPortInterface* communicationPort;
      int toOtherProcessorPin;
      int fromOtherProcessorPin;
      void (*fcnPtr)();
      std::vector<AbstractDevice*>* deviceList;
      
      long timeOfLastCommunication;
      long timeout;
      
      bool forward = false;
      double percentSpeed = 0;
      bool leftRight = false;
      double percentTurn = 0;
      FlipperCommand frontFlipperCmd = FLIPPER_STOP;
      FlipperCommand backFlipperCmd = FLIPPER_STOP;
      ManipulatorArmCommand armCmd = MANIPULATOR_ARM_STOP;
      CameraPanCommand panCmd = PAN_STOP;
      CameraTiltCommand tiltCmd = TILT_STOP;
      bool lightsOn = false;
      bool shutDownRobot = false;
      bool prevEmergencyStop = false;
      bool emergencyStop = false;
      
  public:
  
      InterprocessorCommunicationHandler(int id, CommunicationPortInterface& coms, int toPin, int fromPin, void (*ptr)(), std::vector<AbstractDevice*>& devices, long comsTimeout)
      {
        deviceID = id;
        communicationPort = &coms;
        toOtherProcessorPin = toPin;
        fromOtherProcessorPin = fromPin;
        fcnPtr = ptr;
        deviceList = &devices;
        timeout = comsTimeout;
      }
  
      //implemented interface functions from abstract parent "AbstractDevice"
      bool initializeInterface(bool deviceAlreadyInitialized);
      bool initializeDevice();
      void shutdownDevice();
      int getData(short* data);
      
      //returns false if coms timeout, updates private variables
      bool handleCommunication();
      void signalOtherProcessor();
      bool acknlowledgeOtherProcessor();
      bool checkTimer(long time);
      
      bool getForward();
      double getPercentSpeed();
      bool getLeftRight();
      double getPercentTurn();
      FlipperCommand getFrontFlipperCmd();
      FlipperCommand getBackFlipperCmd();
      ManipulatorArmCommand getArmCmd();
      CameraPanCommand getPanCmd();
      CameraTiltCommand getTiltCmd();
      bool getLightsOn();
      bool getShutDownRobot();
      bool getPrevEmergencyStop();
      bool getEmergencyStop();
};

#endif

