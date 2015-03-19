#include <wire.h>
#include "Arduino.h"
#include "AGM_BNO055.h"
#include "Robot_Arduino_main.h"

bool AGM_BNO055::initializeInterface(bool deviceAlreadyInitialized) {
  pinMode(intPin, INPUT);
  attachInterrupt(intPin, fcnPtr, RISING);
  wire->setTimeout(10);
  wire->begin();
  return true;
}

bool AGM_BNO055::initializeDevice()
{
  
  #ifdef __USER_DEBUG_MODE__
    Serial.println("AGM_BNO055::initializeDevice()");
  #endif
  bool initSuccess = false;
  if(establishCommunications())
  {
    #ifdef __USER_DEBUG_MODE__
      Serial.print("established coms with AGM_BNO055 at address 0x");
      Serial.println(slaveAddressBNO055,HEX);
    #endif  
    if(verifyPOST())
    {
      #ifdef __USER_DEBUG_MODE__
        Serial.println("AGM_BNO055 passed POST");
      #endif  
      if(changeOperationMode(BNO055_OPR_MODE_CONFIG))
      {
        #ifdef __USER_DEBUG_MODE__
          Serial.println("AGM_BNO055 switched into OPR_MODE_CONFIG");
        #endif  
        if(selectUnits())
        {
          #ifdef __USER_DEBUG_MODE__
            Serial.println("AGM_BNO055 set selectUnits");
          #endif  
          //if(performBIST())
          //{
            //Serial.println("AGM_BNO055 passed BIST");
            //configure output data with UNIT_SEL register
            if(enableFlippingInterrupt())
            {
              #ifdef __USER_DEBUG_MODE__
                Serial.println("AGM_BNO055 enableFlippingInterrupt");
              #endif 
              //calibrate the sensors
              if(changeOperationMode(BNO055_OPR_MODE_NDOF))
              {
                initSuccess =  true;
                
                #ifdef __USER_DEBUG_MODE__
                  Serial.println("AGM_BNO055 switched into OPR_MODE_NDOF");
                #endif  
                
                
                /*if(initializeCalibration())
                {
                  initSuccess =  true;
                }else
                {
                  
                }*/
              }else
              {
                //Serial.println("AGM_BNO055 failed to enter OPR_MODE_NDOF");
              }
            }else
            {
              
            }
          //}else
          //{
            //Serial.println("AGM_BNO055 failed BIST");
          //}
        }else
        {
          
        }
      }else
      {
        //Serial.println("BNO055 Failed to enter OPR_MODE_CONFIG");
      }
    }else
    {
      //Serial.println("BNO055 failed POST");
    }
  }else
  {
    //Serial.println("Failed establish coms with BNO055");
  }
  
  if(initSuccess)
  {
    deviceStatus = DEV_RUNNING;
  }else
  {
    deviceStatus = DEV_INIT_ERROR;
  }
  
  return initSuccess;
}

void AGM_BNO055::shutdownDevice()
{
  detachInterrupt(intPin);
}

int AGM_BNO055::getData(short* data)
{
  data[0] = (short)(latestAxisAcceleration[0]*DECIMAL_PRECISION_FACTOR*10);
  data[1] = (short)(latestAxisAcceleration[1]*DECIMAL_PRECISION_FACTOR*10);
  data[2] = (short)(latestAxisAcceleration[2]*DECIMAL_PRECISION_FACTOR*10);
  
  data[3] = (short)(latestEulerAngles[0]*DECIMAL_PRECISION_FACTOR);
  data[4] = (short)(latestEulerAngles[1]*DECIMAL_PRECISION_FACTOR);
  data[5] = (short)(latestEulerAngles[2]*DECIMAL_PRECISION_FACTOR);  
  
  return 6;
}

/*
Params Value [Reg Addr]: Register Value 
Axis selection 
X-axis [GYR_INT_SETTING]: xxxx1xxxb 
Y-axis [GYR_INT_SETTING]: xxx1xxxxb 
Z-axis [GYR_INT_SETTING]: xx1xxxxxb 

High Rate Filter settings 
Filtered [GYR_INT_SETTING]: 0xxxxxxxb Unfiltered [GYR_INT_SETTING]: 1xxxxxxxb 

Interrupt Settings Xaxis 
Threshold [GYR_HR_X_SET]: bit4 : bit0 
Duration [GYR_DUR_X]: bit7 : bit0 
Hysteresis [GYR_HR_X_SET]: bit6 : bit5 

Interrupt Settings Yaxis 
Threshold [GYR_HR_Y_SET]: bit4 : bit0
Duration [GYR_DUR_Y]: bit7 : bit0
Hysteresis [GYR_HR_Y_SET]: bit6 : bit5 

Interrupt Settings Zaxis 
Threshold [GYR_HR_Z_SET]: bit4 : bit0 
Duration [GYR_DUR_Z]: bit7 : bit0 
Hysteresis [GYR_HR_Z_SET]: bit6 : bit5  
*/
bool AGM_BNO055::enableFlippingInterrupt()
{
  bool success = true;
  
  //select page one of memory
  //Serial.println("BNO055_BIT_PAGE_1");
  success &= writeRegister(BNO055_REG_PAGE,BNO055_BIT_PAGE_1);
  
  //enable gyroscope high rate interrupts
  //Serial.println("BNO055_REG_INT_EN");
  success &= writeRegister(BNO055_REG_INT_EN,BNO055_BIT_GYRO_HIGH_RATE);
  
  //enable gyroscope high rate interrupts to pull the INT pin high
  //Serial.println("BNO055_REG_INT_MSK");
  success &= writeRegister(BNO055_REG_INT_MSK,BNO055_BIT_GYRO_HIGH_RATE); 
  
  //enable the x and y axis interrupts
  //Serial.println("BNO055_REG_GYR_INT_SETTING  BNO055_BIT_HR_X_AXIS");
  success &= writeRegister(BNO055_REG_GYR_INT_SETTING,BNO055_BIT_HR_X_AXIS);
  //Serial.println("BNO055_REG_GYR_INT_SETTING  BNO055_BIT_HR_Y_AXIS");
  success &= writeRegister(BNO055_REG_GYR_INT_SETTING,BNO055_BIT_HR_Y_AXIS);
  
  //set the x and y axis interrupt thresholds
  //Serial.println("BNO055_REG_GYR_HR_X_SET");
  success &= writeRegister(BNO055_REG_GYR_HR_X_SET,BNO055_BITS_HR_X_THRESHOLD);
  //Serial.println("BNO055_REG_GYR_HR_Y_SET");
  success &= writeRegister(BNO055_REG_GYR_HR_Y_SET,BNO055_BITS_HR_Y_THRESHOLD);  
  
  //select page zero of memory
  //Serial.println("BNO055_BIT_PAGE_0");
  success &= writeRegister(BNO055_REG_PAGE,BNO055_BIT_PAGE_0);
  
  return success;
}

//expects page 0
//read clears register
bool AGM_BNO055::clearFlippingInterrupt()
{
  flippingFlag = false;
  byte d[1];
  bool readSuccess = readRegister(BNO055_REG_INT_STA, d, 1);
  return (readSuccess);  
}

void AGM_BNO055::setFlippingFlag()
{
  flippingFlag = true;
}

bool AGM_BNO055::getFlippingFlag()
{
  return flippingFlag;
}  

//expects page 0
//returns axisAcceleration: [0] = x, [1] = y, [2] = z
bool AGM_BNO055::getLinearAcceleration(double* axisAcceleration)
{
  bool readSuccess = false;
  if((deviceStatus != DEV_INIT_ERROR) ) //&& (deviceStatus != DEV_RUN_ERROR)
  {
    short temp[3];
    readSuccess = readNIntDataReg(BNO055_REG_LIA_DATA_X_LSB, temp, 3);
    
    //convert 2 byte signed word into an integer
    for(int i=0; i<3 && readSuccess; i++)
    {
      axisAcceleration[i] = temp[i]/100.0; //100LSB = 1m/s^2
      latestAxisAcceleration[i] = axisAcceleration[i];
    }
  }
  return (readSuccess);  
}

// expects page 0
//returns eulerAngles: [0] = roll, [1] = pitch, [2] = yaw
bool AGM_BNO055::getEulerAngles(double* eulerAngles)
{
  bool readSuccess = false;
  if((deviceStatus != DEV_INIT_ERROR) ) // && (deviceStatus != DEV_RUN_ERROR)
  {
    short temp[3];
    readSuccess = readNIntDataReg(BNO055_REG_EUL_HEADING_LSB, temp, 3);
    
    //convert 2 byte signed word into an integer
    for(int i=0; i<3 && readSuccess; i++)
    {
      eulerAngles[i] = temp[i]/16.0; //16LSB = 1 degree
      latestEulerAngles[i] = eulerAngles[i];
    } 
  }
  return (readSuccess);
}

// expects page 0
//returns angularVelocities: [0] = roll, [1] = pitch, [2] = yaw
bool AGM_BNO055::getAngularVelocities(double* angularVelocities)
{
  bool readSuccess = false;
  if((deviceStatus != DEV_INIT_ERROR) ) // && (deviceStatus != DEV_RUN_ERROR)
  {
    short temp[3];
    readSuccess = readNIntDataReg(BNO055_REG_GYR_RATE_X_LSB, temp, 3);
    
    //convert 2 byte signed word into an integer
    for(int i=0; i<3 && readSuccess; i++)
    {
      angularVelocities[i] = temp[i]/16.0; //16LSB = 1 degree
    } 
  }
  return (readSuccess);  
}

bool AGM_BNO055::readNIntDataReg(byte reg, short* data, int numData)
{
  byte byteData[numData*2];
  bool readSuccess = readRegister(reg,byteData,numData*2);
  for(int i=0; i<numData; i++)
  {
    data[i] = (byteData[2*i] + (byteData[2*i+1]<<8));
  }  
  
  return (readSuccess);  
}

//expects page 0
bool AGM_BNO055::initializeCalibration()
{
  bool calibrated = false;//readNonVolatileMem(ADDR_OF_CALIBRATED_BIT);
  if(calibrated)
  {
    //read calibration profile from non vol mem
    //store into BNO055 calibration profile registers
  }else
  {
    //calibrate the sensors
    //read calibration bits to determine when they are done calibration
    //store calibration profile into non vol mem
    //set CALIBRATED_BIT = 1;
  }
  return (true);
}

bool AGM_BNO055::selectUnits()
{
  bool writeSuccess = writeRegister(BNO055_REG_UNIT_SEL,0x00);
  
  return writeSuccess;
}

//expects page 0
bool AGM_BNO055::performBIST()
{
  bool writeSuccess = writeRegister(BNO055_REG_SYS_TRIGGER,BNO055_BIT_SELF_TEST);
  byte d[1];
  bool readSuccess = readRegister(BNO055_REG_SYS_STATUS,d,1);
  Serial.print("System status before: ");
  Serial.println(d[0],HEX);
  while(d[0]==BNO055_BIT_SYSTEM_IDLE)
  {
    delay(500);
    readSuccess &= readRegister(BNO055_REG_SYS_STATUS,d,1);
    Serial.print("System status before: ");
    Serial.println(d[0],HEX);
  }
  while(d[0]==BNO055_BIT_EXECUTING_SELF_TEST)
  {
    readSuccess &= readRegister(BNO055_REG_ST_RESULT,d,1);
    Serial.print("ST_RESULT during: ");
    Serial.println(d[0],HEX);
    delay(500);
    readSuccess &= readRegister(BNO055_REG_SYS_STATUS,d,1);
    Serial.print("System status during: ");
    Serial.println(d[0],HEX);
  }
  Serial.print("System status after: ");
  Serial.println(d[0],HEX);
  readSuccess &= readRegister(BNO055_REG_ST_RESULT,d,1);
  return (writeSuccess && readSuccess && (d[0]&BNO055_BITS_SELF_TEST_PASSED == BNO055_BITS_SELF_TEST_PASSED));
}

//expects page 0
bool AGM_BNO055::establishCommunications()
{
  slaveAddressBNO055 = BNO055_ADDRESS_PRIMARY;
  byte d[1];
  bool readSuccess = readRegister(BNO055_REG_CHIP_ID,d,1);
  if(!(readSuccess && (d[0]==BNO055_CHIP_ID)))
  {
    slaveAddressBNO055 = BNO055_ADDRESS_ALTERNATE;
    d[1];
    readSuccess = readRegister(BNO055_REG_CHIP_ID,d,1);  
  }
  return (readSuccess && (d[0]==BNO055_CHIP_ID));
}

/*
expects page 0
params n/a

returns true if BNO055 passed power on self testing
*/
bool AGM_BNO055::verifyPOST()//return true if success, false if error
{
  byte data[1];
  bool transmitSuccess = readRegister(BNO055_REG_ST_RESULT,data,1);
  if((data[0]&0xF)!=0x0F) //1111b = 1 bit for each system uC,G,M,A. bit = 1b if system passed POST
  {
    //errLog += "BNO POST ERROR: "+c&0xF+"\n";
    //Serial.println("(data[0]&0xF)!=0x0F");
  }
  return (transmitSuccess && ((data[0]&0xF)==0x0F));
}

/*
expects page 0
params mode is the operation mode you want to change into

it will wait until the mode should have switched and then read to
confirm the mode has been set to the variable mode

returns success of write & read & switch
*/
bool AGM_BNO055::changeOperationMode(byte mode)
{
  bool writeSuccess = writeRegister(BNO055_REG_OPR_MODE,mode);
  if(mode==BNO055_OPR_MODE_CONFIG)
  {
    delay(20);//it takes 19ms to go another mode into config mode
  }else
  {
    delay(8);//it takes 7ms to switch from config into any operation mode
  }
  byte d[1];
  bool readSuccess = readRegister(BNO055_REG_OPR_MODE,d,1);
  
  return (writeSuccess && readSuccess && (d[0] == mode));
}

/* MASTER READ
params
reg is the first register to read from
data is the array passed in to be filled
numBytes is the number of bytes to read
note: will read past the original register address

returns success of transmission 
*/
bool AGM_BNO055::readRegister(byte reg, byte* data, int numBytes)
{
  wire->beginTransmission(slaveAddressBNO055);
  wire->write(reg);
  int transResult = wire->endTransmission(false); //keep the conection alive
  
  int numBytesReceived = wire->requestFrom(slaveAddressBNO055,numBytes,true); //send stop bit after numBytes recieved
  byte i = 0;
  //while(!wire->available()); 
  //no need to wait for synchronization with i2c. i2c will have received bytes from the slave immediately
  //This while(!wire->available()); would also loop forever during establish connection if address is incorrect
  while(wire->available())
  {
    data[i++] = wire->read();
  }

  if(!(transResult==0) || (numBytesReceived==0))
  {
    deviceStatus = DEV_RUN_ERROR;
  }

  return (transResult==0);
}

/*MASTER WRITE
params
reg is the one register to be written to
data is the byte to be written to the slave

returns success of transmission 
*/
bool AGM_BNO055::writeRegister(byte reg, byte data)
{
  wire->beginTransmission(slaveAddressBNO055);
  wire->write(reg);
  wire->write(data);
  int transResult = wire->endTransmission(true);  
  //Serial.println(transResult,HEX);
  
  if(!(transResult==0))
  {
    deviceStatus = DEV_RUN_ERROR;
  }  
  
  return (transResult==0);
}

