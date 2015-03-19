#include "AbstractDevice.h"

#ifndef __ENCODER_INTERFACE__
#define __ENCODER_INTERFACE__

class EncoderInterface: public AbstractDevice
{
  virtual double getAngle();
};

#endif

