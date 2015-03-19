#include "Robot_Arduino_main.h"
#include "Arduino.h"

#ifndef __ABSTRACT_DEVICE__
#define __ABSTRACT_DEVICE__

enum DeviceStatus { DEV_UNINITIALIZED = 1, DEV_RUNNING, DEV_INIT_ERROR, DEV_RUN_ERROR };



class AbstractDevice
{
    protected:
        //char* errorMsg;
        //char* errorLog;
        
        byte deviceID; //to be set once in the constructor of child classes
        
        DeviceStatus deviceStatus = DEV_UNINITIALIZED;

    public:

        virtual bool initializeInterface(bool deviceAlreadyInitialized);
        virtual bool initializeDevice();
        virtual void shutdownDevice();
        
        virtual int getData(short* data); //returns number of bytes stored in data
        
        byte getDeviceID();
        DeviceStatus getDeviceStatus();
        
        //char* getErrorMsg();
        //char* getErrorLog();
};

#endif

