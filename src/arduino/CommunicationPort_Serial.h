#include "Arduino.h"
#include "CommunicationPortInterface.h"

#ifndef __COMMUNICATION_PORT_SERIAL__
#define __COMMUNICATION_PORT_SERIAL__

class CommunicationPort_Serial: public CommunicationPortInterface
{
  protected:
      UARTClass* serialBus;
      int baudRate;
      
  public:
  
      CommunicationPort_Serial(UARTClass& sBus, int baud)
      {
        serialBus = &sBus;
        baudRate = baud;
      }
  
      void initializePort();
  
      //data =  bytes to send
      void sendNBytes(byte* data, int n);

      //data = bytes received
      void recieveNBytes(byte* data, int n);
};

#endif

