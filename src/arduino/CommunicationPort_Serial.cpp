#include "Robot_Arduino_main.h"
#include "CommunicationPort_Serial.h"

//implemented interface functions from abstract parent "AbstractDevice"
void CommunicationPort_Serial::initializePort()
{
  serialBus->begin(baudRate);
  serialBus->setTimeout(10);
}

//data =  bytes to send
void CommunicationPort_Serial::sendNBytes(byte* data, int n)
{
  #ifdef __USER_DEBUG_MODE__
    for(int i=0;i<n;i++)
    {
      Serial.print(data[i],HEX);
      Serial.print(",");
    }
    Serial.println("");
  #endif
  
  serialBus->write(data,n);
}

//data = bytes received
void CommunicationPort_Serial::recieveNBytes(byte* data, int n)
{
  if(serialBus->readBytes(data,n)==0)
  {
    for(int i = 0; i<n; i++)
    {
      data[i] = 0;
    }
  }
  
  #ifdef __USER_DEBUG_MODE__
    for(int i=0;i<n;i++)
    {
      Serial.print(data[i],HEX);
      Serial.print(",");
    }
    Serial.println("");
  #endif
}

