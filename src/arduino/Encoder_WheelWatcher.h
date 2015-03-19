#include "EncoderInterface.h"

#ifndef __ENCODER__
#define __ENCODER__

class Encoder_WheelWatcher: public EncoderInterface
{
  protected:
    volatile double pos = 0;
    double gearRatio;
    double oneRotation;
    bool reverse;
    int clkPin;
    int dirPin;
    void (*fcnPtr)();
    
  public:
    
    Encoder_WheelWatcher(int id, double ratio, bool rev, void (*fcn)(), int clk, int dir)
    {
      deviceID = id;
      gearRatio = ratio;
      reverse = rev;
      oneRotation = 128*ratio - 1;
      fcnPtr = fcn;
      clkPin = clk;
      dirPin = dir;
    }
    
    //implemented interface functions from abstract parent "Device"
    bool initializeInterface(bool deviceAlreadyInitialized);
    bool initializeDevice();
    void shutdownDevice();
    int getData(short* data);
    
    //implemented interface functions from interface parent "EncoderInterface"
    double getAngle();
    
    //wheel watcher specific functions
    void clockPulse();
};

#endif

