#include "AbstractAGM.h"

#ifndef __AGM_BNO055__
#define __AGM_BNO055__

const byte BNO055_ADDRESS_ALTERNATE = 0x28;//addr = 0x29, altAddr = 0x28, HIDAddr = 0x40
const byte BNO055_ADDRESS_PRIMARY = 0x29;

const byte BNO055_REG_CHIP_ID = 0x00;
const byte BNO055_CHIP_ID = 0xA0;

const byte BNO055_REG_SW_REV_ID_LSB = 0x04;

const byte BNO055_REG_SYS_STATUS = 0x39;
const byte BNO055_BIT_EXECUTING_SELF_TEST = 0x04;
const byte BNO055_BIT_SYSTEM_IDLE = 0x00;
const byte BNO055_REG_SYS_ERR = 0x3A;

const byte BNO055_REG_ST_RESULT = 0X36;
const byte BNO055_BITS_SELF_TEST_PASSED = 0x07; //0000 0111b G, M, A

const byte BNO055_REG_SYS_TRIGGER = 0X3F;
const byte BNO055_BIT_SELF_TEST = 0x01;

const byte BNO055_REG_UNIT_SEL = 0x80;

const byte BNO055_REG_OPR_MODE = 0x3D;
const byte BNO055_OPR_MODE_CONFIG = 0x00;
const byte BNO055_OPR_MODE_NDOF = 0x0C;

const byte BNO055_REG_CALIB_STAT = 0x35;
const byte BNO055_BITS_CALIB_STAT_SYS = B11000000;
const byte BNO055_BITS_CALIB_STAT_GYR = B00110000;
const byte BNO055_BITS_CALIB_STAT_ACC = B00001100;
const byte BNO055_BITS_CALIB_STAT_MAG = B00000011;

const byte BNO055_REG_EUL_HEADING_LSB = 0x1A;
const byte BNO055_REG_LIA_DATA_X_LSB = 0x28;
const byte BNO055_REG_GYR_RATE_X_LSB = 0x14;

const byte BNO055_REG_PAGE = 0x07;
const byte BNO055_BIT_PAGE_0 = 0x00;
const byte BNO055_BIT_PAGE_1 = 0x01;
  
const byte BNO055_REG_INT_EN = 0x10;
const byte BNO055_REG_INT_MSK = 0x0F;
const byte BNO055_REG_INT_STA  = 0x37;
const byte BNO055_BIT_GYRO_HIGH_RATE = B00001000;
  
const byte BNO055_REG_GYR_INT_SETTING = 0x17;
const byte BNO055_BIT_HR_X_AXIS = B00001000;
const byte BNO055_BIT_HR_Y_AXIS = B00010000;

const byte BNO055_REG_GYR_HR_X_SET = 0x18;
const byte BNO055_BITS_HR_X_THRESHOLD = 0x0F; //between 0x00 and 0x1F;
const byte BNO055_REG_GYR_HR_Y_SET = 0x1A;
const byte BNO055_BITS_HR_Y_THRESHOLD = 0x0F;
  

class AGM_BNO055: public AbstractAGM
{
  protected:
      byte slaveAddressBNO055 = BNO055_ADDRESS_PRIMARY;
      TwoWire* wire;
      int intPin;
      void (*fcnPtr)();
      bool flippingFlag = false;
      
  public:
      
      AGM_BNO055(int id, TwoWire& tw, void (*fcn)(), int iPin)
      {
        deviceID = id;
        wire = &tw;
        fcnPtr = fcn;
        intPin = iPin;
        
        latestAxisAcceleration[0] = 0.0;
        latestAxisAcceleration[1] = 0.0;
        latestAxisAcceleration[2] = 0.0;
        latestEulerAngles[0] = 0.0;  
        latestEulerAngles[1] = 0.0;  
        latestEulerAngles[2] = 0.0;     
      }
      
      //implemented interface functions from abstract parent "AbstractDevice"
      bool initializeInterface(bool deviceAlreadyInitialized);
      bool initializeDevice();
      void shutdownDevice();
      int getData(short* data);
      
      //implemented interface functions from abstract parent "AbstractAGM"
      bool getLinearAcceleration(double* axisAcceleration);
      bool getEulerAngles(double* eulerAngles);
      bool getAngularVelocities(double* angularVelocities);
      
      //BNO055 specific functions
      bool selectUnits();
      bool enableFlippingInterrupt();
      bool clearFlippingInterrupt();
      void setFlippingFlag();
      bool getFlippingFlag();
      bool readNIntDataReg(byte reg, short* data, int numData);
      bool initializeCalibration();
      bool performBIST();
      bool establishCommunications();
      bool verifyPOST();
      bool changeOperationMode(byte mode);
      bool readRegister(byte reg, byte* data, int numBytes);
      bool writeRegister(byte reg, byte data);

};

#endif

