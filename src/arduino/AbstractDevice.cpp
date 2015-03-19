#include "AbstractDevice.h"

byte AbstractDevice::getDeviceID()
{
  return deviceID;
}

DeviceStatus AbstractDevice::getDeviceStatus()
{
  return deviceStatus;
}

//char* AbstractDevice::getErrorMsg()
//{
//    return errorMsg;
//}
//char* AbstractDevice::getErrorLog()
//{
//    return errorLog;
//}

