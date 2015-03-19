#include "Arduino.h"

#ifndef __COMMUNICATION_PORT_INTERFACE__
#define __COMMUNICATION_PORT_INTERFACE__

class CommunicationPortInterface
{
  protected:
      
  public:
      virtual void initializePort();
  
      //data =  bytes to send
      virtual void sendNBytes(byte* data, int n);

      //data = bytes received
      virtual void recieveNBytes(byte* data, int n);
};

#endif

