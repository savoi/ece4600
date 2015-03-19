#include "Arduino.h"
#include "InterprocessorCommunicationHandler.h"

//implemented interface functions from abstract parent "AbstractDevice"
bool InterprocessorCommunicationHandler::initializeInterface(bool deviceAlreadyInitialized)
{
  communicationPort->initializePort();
  pinMode(toOtherProcessorPin, OUTPUT);
  pinMode(fromOtherProcessorPin, INPUT);
  
  return true;
}



bool InterprocessorCommunicationHandler::initializeDevice()
{
  //wait for other processor to start up
  //do an initial handshake with other processor
  //Serial.println("InterprocessorCommunicationHandler::initializeDevice");
  

  
  digitalWrite(toOtherProcessorPin, HIGH);
  
  //Serial.println("toOtherProcessorPin, HIGH");
  
  while(digitalRead(fromOtherProcessorPin) != HIGH);
  
  //Serial.println("fromOtherProcessorPin) == HIGH");
  
  digitalWrite(toOtherProcessorPin, LOW);
  
  //Serial.println("toOtherProcessorPin, LOW");
  
  while(digitalRead(fromOtherProcessorPin) != LOW);
  
  //Serial.println("fromOtherProcessorPin) == LOW");
  
  attachInterrupt(fromOtherProcessorPin, fcnPtr, RISING);
  
  timeOfLastCommunication = millis();
  
  deviceStatus = DEV_RUNNING;
  
  return true;
}

void InterprocessorCommunicationHandler::shutdownDevice()
{
  detachInterrupt(fromOtherProcessorPin);
}

int InterprocessorCommunicationHandler::getData(short* data)
{
  return 0;
}

//returns false if coms timeout, updates private variables
bool InterprocessorCommunicationHandler::handleCommunication()
{
  
  digitalWrite(toOtherProcessorPin, HIGH);
  
//  #ifdef __USER_DEBUG_MODE__
//    Serial.println("while(digitalRead(fromOtherProcessorPin) != LOW)");
//  #endif
  
  while(digitalRead(fromOtherProcessorPin) != LOW);

//  #ifdef __USER_DEBUG_MODE__
//    Serial.println("communicationPort->recieveNBytes");
//  #endif
  
  byte dataFrom[INTERPROCESSOR_COMMAND_PACKET_SIZE_IN_BYTES];
  
  //Serial.println("                                             recieveNBytes");
  communicationPort->recieveNBytes(dataFrom,INTERPROCESSOR_COMMAND_PACKET_SIZE_IN_BYTES);
  //Serial.println("                                             recieveNBytes Complete");
  
  forward = (dataFrom[0] & B10000000) == B10000000;
  percentSpeed = (dataFrom[0] & B01111111);
  
  leftRight = (dataFrom[1] & B10000000) == B10000000;
  percentTurn = (dataFrom[1] & B01111111);
  
  frontFlipperCmd = (FlipperCommand)(dataFrom[2]>>6);
  backFlipperCmd = (FlipperCommand)((dataFrom[2]>>4) & B00000011);
  
  armCmd = (ManipulatorArmCommand)(dataFrom[3]>>6);
  panCmd = (CameraPanCommand)((dataFrom[3]>>4) & B00000011);
  tiltCmd = (CameraTiltCommand)((dataFrom[3]>>2) & B00000011);
  
  if((dataFrom[3] & B00000010) == B00000010)
  {
    lightsOn = !lightsOn;
  }
  
  prevEmergencyStop = emergencyStop;
  if(((dataFrom[4] >> 6) & B00000011) == B00000011)
  {
    emergencyStop = !emergencyStop;
  }
  
  shutDownRobot = (((dataFrom[4] >> 4) & B00000011) == B00000011);
  
  //delayMicroseconds(500);
  
  digitalWrite(toOtherProcessorPin, LOW);
  
  //delayMicroseconds(500);
  
  byte dataTo[INTERPROCESSOR_DATA_PACKET_MAX_SIZE_IN_BYTES];

  int positionInArray = 0;
  
  for (std::vector<AbstractDevice*>::iterator it = deviceList->begin() ; it != deviceList->end(); ++it)
  {
    if((*it) != this)
    {
      
      dataTo[positionInArray++] = (*it)->getDeviceID();
      dataTo[positionInArray++] = (*it)->getDeviceStatus();
      
      short tempData[20];
      int numData = (*it)->getData(tempData);
      
      for(int i=0;i<numData; i++)
      {
        dataTo[2*i + 1 + positionInArray] = (byte)(tempData[i]>>8);
        dataTo[2*i + positionInArray] = (byte)(tempData[i] & 0x00FF);
      }
      positionInArray += numData*2;
      
    }
  }
    
  communicationPort->sendNBytes(dataTo,positionInArray);
  
  timeOfLastCommunication = millis();
  
  return true; //TODO timeout
}

bool InterprocessorCommunicationHandler::checkTimer(long time)
{
  bool temp = false;
  
  if(((time - timeOfLastCommunication) > timeout)  && !emergencyStop)
  {
    emergencyStop = true;
    temp = true;
  }
  
  return temp;
}

void InterprocessorCommunicationHandler::signalOtherProcessor()
{
  //Serial.println("InterprocessorCommunicationHandler::signalOtherProcessor");
  //delayMicroseconds(500);
  //sendFlag = true;
  
  //if(digitalRead(fromOtherProcessorPin) == LOW)
  //{
    //Serial.println("fromOtherProcessorPin) == LOW");
  //  delayMicroseconds(500);
  //  digitalWrite(toOtherProcessorPin, HIGH);
  //}
}

bool InterprocessorCommunicationHandler::acknlowledgeOtherProcessor()
{
  //recieveFlag = true;
  
  //Serial.println("InterprocessorCommunicationHandler::acknlowledgeOtherProcessor");
  //delayMicroseconds(500);
  
  //digitalWrite(toOtherProcessorPin, LOW);
  
  return true;//!sendFlag;
}




/* usage in main system

//timer interrupt to periodically send statuses to other processor
void timerISR()
{
  
  //FOR NOW PUT THE SEND SATATUS HERE, LATER MAKE A BETTER TIMER INTERRUPT ROUTINE FOR ALL TIMERS
  interprocessorCommunicationHandler.signalOtherProcessor();
}

//interrupt triggered by other processor
void interprocessorComsISR()
{
  
  if(interprocessorCommunicationHandler.acknlowledgeOtherProcessor()) //if a task doesn't already exist in queue to perform coms
  {
    priorityQueue.push(RobotTask(handleCommunication,millis()+RECIEVE_COMMANDS_DEADLINE));
  }
}

*/








//ACCESSORS//////////////////////////////////////////////////////////

bool InterprocessorCommunicationHandler::getForward()
{
  return forward;
}

double InterprocessorCommunicationHandler::getPercentSpeed()
{
  return percentSpeed;
}

bool InterprocessorCommunicationHandler::getLeftRight()
{
  return leftRight;
}

double InterprocessorCommunicationHandler::getPercentTurn()
{
  return percentTurn;
}

FlipperCommand InterprocessorCommunicationHandler::getFrontFlipperCmd()
{
  return frontFlipperCmd;
}

FlipperCommand InterprocessorCommunicationHandler::getBackFlipperCmd()
{
  return backFlipperCmd;
}

ManipulatorArmCommand InterprocessorCommunicationHandler::getArmCmd()
{
  return armCmd;
}

CameraPanCommand InterprocessorCommunicationHandler::getPanCmd()
{
  return panCmd;
}

CameraTiltCommand InterprocessorCommunicationHandler::getTiltCmd()
{
  return tiltCmd;
}

bool InterprocessorCommunicationHandler::getLightsOn()
{
  return lightsOn;
}

bool InterprocessorCommunicationHandler::getShutDownRobot()
{
  return shutDownRobot;
}

bool InterprocessorCommunicationHandler::getEmergencyStop()
{
  return emergencyStop;
}

bool InterprocessorCommunicationHandler::getPrevEmergencyStop()
{
  return prevEmergencyStop;
}

