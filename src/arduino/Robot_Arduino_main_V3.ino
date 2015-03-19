#include <DueFlashStorage.h>
#include <efc.h>
#include <flash_efc.h>
#include <DueTimer.h> //for timer interrupts
#include <Wire.h> //for i2c coms
#include <vector>

//my libs
#include "Robot_Arduino_main.h"

#include "RobotTask.h"
#include "TaskComparator.h"
#include "PriorityQueue.h"
#include "InterruptHandlerSafety.h"

#include "AbstractDevice.h"
#include "Encoder_WheelWatcher.h"
#include "AGM_BNO055.h"
#include "MotorController_SaberToothDual25A.h"
#include "MotorPort_SaberToothDual25A.h"
#include "Motor.h"
#include "InterprocessorCommunicationHandler.h"
#include "ManipulatorArm.h"
#include "Servo_G15.h"
#include "TemperatureSensor_Analog.h"
#include "VoltageDividerSensor.h"

#include "MotionControl.h"

int mainState = INITIALIZING;
PriorityQueue priorityQueue;
DueFlashStorage dueFlashStorage;
std::vector<AbstractDevice*> devices;


Encoder_WheelWatcher rightDriveEncoder(DEVICE_ID_RIGHT_DRIVE_ENCODER,ENCODER_DRIVE_GEAR_RATIO,false,encoderRightDriveClkISR,ENCODER_RIGHT_DRIVE_CLK_PIN,ENCODER_RIGHT_DRIVE_DIR_PIN);

#ifdef __ENCODERS_ENABLED__
  
  Encoder_WheelWatcher leftDriveEncoder(DEVICE_ID_LEFT_DRIVE_ENCODER,ENCODER_DRIVE_GEAR_RATIO,true,encoderLeftDriveClkISR,ENCODER_LEFT_DRIVE_CLK_PIN,ENCODER_LEFT_DRIVE_DIR_PIN);
  Encoder_WheelWatcher frontFlipperEncoder(DEVICE_ID_FRONT_FLIPPER_ENCODER,ENCODER_FLIPPER_GEAR_RATIO,false,encoderFrontFlipperClkISR,ENCODER_FRONT_FLIPPER_CLK_PIN,ENCODER_FRONT_FLIPPER_DIR_PIN); 
  Encoder_WheelWatcher rearFlipperEncoder(DEVICE_ID_BACK_FLIPPER_ENCODER,ENCODER_FLIPPER_GEAR_RATIO,true,encoderRearFlipperClkISR,ENCODER_REAR_FLIPPER_CLK_PIN,ENCODER_REAR_FLIPPER_DIR_PIN);
#endif

#ifdef __AGM_ENABLED__
  AGM_BNO055 agm_bno055(DEVICE_ID_AGM_BNO055,BNO055_I2C_BUS,flippingISR,BNO055_INTERRUPT_PIN);
#endif

#ifdef __MOTOR_CONTROLLERS_ENABLED__
  Motor leftDriveMotor(LEFT_DRIVE_MOTOR_DIRECTION, LEFT_DRIVE_MOTOR_MAX_VOLTAGE);
  Motor rightDriveMotor(RIGHT_DRIVE_MOTOR_DIRECTION, RIGHT_DRIVE_MOTOR_MAX_VOLTAGE);
  Motor frontFlipperMotor(FRONT_FLIPPER_MOTOR_DIRECTION, FRONT_FLIPPER_MOTOR_MAX_VOLTAGE);
  Motor backFlipperMotor(BACK_FLIPPER_MOTOR_DIRECTION, BACK_FLIPPER_MOTOR_MAX_VOLTAGE);
  
  MotorController_SaberToothDual25A driveMotorsController(DEVICE_ID_DRIVE_MOTOR_CONTROLLER,SABERTOOTH_DRIVE_SERIAL_BUS, SABERTOOTH_DRIVE_SERIAL_ADDRESS, leftDriveMotor, rightDriveMotor, DRIVER_MOTOR_CONTROLLER_EMERGENCY_STOP_PIN);
  MotorController_SaberToothDual25A flipperMotorsController(DEVICE_ID_FLIPPER_MOTOR_CONTROLLER,SABERTOOTH_FLIPPER_SERIAL_BUS, SABERTOOTH_FLIPPER_SERIAL_ADDRESS, frontFlipperMotor, backFlipperMotor, FLIPPER_MOTOR_CONTROLLER_EMERGENCY_STOP_PIN);
#endif

CommunicationPort_Serial serialPort(INTERPROCESSOR_SERIAL_BUS, INTERPROCESSOR_SERIAL_BUS_BAUD);
InterprocessorCommunicationHandler interprocessorCommunicationHandler(DEVICE_ID_INTERPROCESSOR_COM_HANDLER, serialPort, INTERPROCESSOR_COMS_TO_PIN, INTERPROCESSOR_COMS_FROM_PIN, interprocessorComsISR, devices, COMMUNICATION_TIMEOUT_MS);

#ifdef __SERVOS_ENABLED__
  Servo_G15 bottomServo(DEVICE_ID_G15_SERVO_BOTTOM,G15_SERVO_SERIAL_BUS, BOTTOM_G15_SERVO_ADDRESS, G15_SERVO_CONTROL_PIN, BOTTOM_G15_SERVO_CW_MAX_ANGLE, BOTTOM_G15_SERVO_CCW_MAX_ANGLE);
  Servo_G15 middleServo(DEVICE_ID_G15_SERVO_MIDDLE,G15_SERVO_SERIAL_BUS, MIDDLE_G15_SERVO_ADDRESS, G15_SERVO_CONTROL_PIN, MIDDLE_G15_SERVO_CW_MAX_ANGLE, MIDDLE_G15_SERVO_CCW_MAX_ANGLE);
  Servo_G15 topServo(DEVICE_ID_G15_SERVO_TOP,G15_SERVO_SERIAL_BUS, TOP_G15_SERVO_ADDRESS, G15_SERVO_CONTROL_PIN, TOP_G15_SERVO_CW_MAX_ANGLE, TOP_G15_SERVO_CCW_MAX_ANGLE);
  Servo_G15 panServo(DEVICE_ID_G15_SERVO_PAN,G15_SERVO_SERIAL_BUS, PAN_G15_SERVO_ADDRESS, G15_SERVO_CONTROL_PIN, PAN_G15_SERVO_CW_MAX_ANGLE, PAN_G15_SERVO_CCW_MAX_ANGLE);
  
  ManipulatorArm manipulatorArm(DEVICE_ID_MANIPULATOR_ARM,G15_SERVO_SERIAL_BUS, bottomServo, middleServo, topServo, panServo, MANIPULATOR_ARM_LOWER_LENGTH, MANIPULATOR_ARM_UPPER_LENGTH, G15_SERVO_CONTROL_PIN);
#endif

#ifdef __TEMPERATURE_SENSORS_ENABLED__
  TemperatureSensor_Analog temperatureSensorOnBattery(DEVICE_ID_TEMP_SENSOR_BATTERY,BATTERY_TEMP_SENSOR_PIN , VOLT_TO_TEMP_CONSTANT);
  TemperatureSensor_Analog temperatureSensorOnLeftDriveMotor(DEVICE_ID_TEMP_SENSOR_LEFT_DRIVE_MOTOR,LEFT_DRIVE_MOTOR_TEMP_SENSOR_PIN , VOLT_TO_TEMP_CONSTANT);
  TemperatureSensor_Analog temperatureSensorOnRightDriveMotor(DEVICE_ID_TEMP_SENSOR_RIGHT_DRIVE_MOTOR,RIGHT_DRIVE_MOTOR_TEMP_SENSOR_PIN , VOLT_TO_TEMP_CONSTANT);
  TemperatureSensor_Analog temperatureSensorOnFrontFlipperMotor(DEVICE_ID_TEMP_SENSOR_FRONT_FLIPPER_MOTOR,FRONT_FLIPPER_MOTOR_TEMP_SENSOR_PIN , VOLT_TO_TEMP_CONSTANT);
  TemperatureSensor_Analog temperatureSensorOnBackFlipperMotor(DEVICE_ID_TEMP_SENSOR_BACK_FLIPPER_MOTOR,BACK_FLIPPER_MOTOR_TEMP_SENSOR_PIN , VOLT_TO_TEMP_CONSTANT);
#endif

#ifdef __VOLTAGE_SENSOR_ENABLED__
  VoltageDividerSensor voltageSensorOnBattery(DEVICE_ID_VOLT_SENSOR_BATTERY,BATTERY_VOLT_SENSOR_PIN , PRIMARY_VOLT_DIVIDER_CONSTANT);
#endif

void setup() {
    
  pinMode(LED_COMS,OUTPUT);
  pinMode(LED_MOTION,OUTPUT);
  pinMode(LED_TEMPS,OUTPUT);
  pinMode(LED_VOLTS,OUTPUT);
  pinMode(LED_TIMEOUT_COMS,OUTPUT);
  
  pinMode(LED_ENCODER_ISR,OUTPUT);
  pinMode(LED_COMS_ISR,OUTPUT);
  pinMode(LED_WDT_ISR,OUTPUT);
  pinMode(LED_FLIPPING_ISR,OUTPUT);
  pinMode(LED_SENSORS_ISR,OUTPUT);
  pinMode(LED_MOTION_ISR,OUTPUT);
  
  pinMode(PWR_PRECHARGE_PIN,OUTPUT);
  
  devices.push_back(&rightDriveEncoder);
  
  #ifdef __ENCODERS_ENABLED__
    
    devices.push_back(&leftDriveEncoder);
    devices.push_back(&frontFlipperEncoder);
    devices.push_back(&rearFlipperEncoder);
  #endif
  
  #ifdef __AGM_ENABLED__
    devices.push_back(&agm_bno055);
  #endif
  
  #ifdef __MOTOR_CONTROLLERS_ENABLED__
    devices.push_back(&driveMotorsController);
    devices.push_back(&flipperMotorsController);
  #endif
  
  #ifdef __SERVOS_ENABLED__
    devices.push_back(&manipulatorArm);
  #endif
  
  #ifdef __TEMPERATURE_SENSORS_ENABLED__
    devices.push_back(&temperatureSensorOnBattery);
    devices.push_back(&temperatureSensorOnLeftDriveMotor);
    devices.push_back(&temperatureSensorOnRightDriveMotor);
    devices.push_back(&temperatureSensorOnFrontFlipperMotor);  
    devices.push_back(&temperatureSensorOnBackFlipperMotor); 
  #endif
  
  #ifdef __VOLTAGE_SENSOR_ENABLED__
    devices.push_back(&voltageSensorOnBattery); 
  #endif
  
  devices.push_back(&interprocessorCommunicationHandler);
}

void loop() {
  // state machine
  switch(mainState){
   case INITIALIZING:
   {
     bool normalStartUp = dueFlashStorage.read(PROCESSOR_FAILURE_FLAG_FLASH_ADDRESS) != PROCESSOR_FAILURE_FLAG;
     
     if(normalStartUp)
     {
       initializeSystem(false);
       initializeDevices();
     }else //processor failed and WDT restarted the system
     {
       initializeSystem(true);
     }
     
     //dueFlashStorage.write(PROCESSOR_FAILURE_FLAG_FLASH_ADDRESS,PROCESSOR_FAILURE_FLAG);
     initSystemTimers();
     
     mainState = RUNNING;
   }break;
   case RUNNING:
   {
     runningLoop();
   }break;
   case SHUTTING_DOWN:
   {
     dueFlashStorage.write(PROCESSOR_FAILURE_FLAG_FLASH_ADDRESS,EMPTY_BYTE);
     
     stopSystemTimers();
     shutdownDevices();
     
     while(true); //wait for power down
   }break;
   case DEBUGGING:
   {
     
   }break;
  }
}


/*
typedef struct {
  WoReg WDT_CR; //brief (Wdt Offset: 0x00) Control Register 
  RwReg WDT_MR //brief (Wdt Offset: 0x04) Mode Register 
  RoReg WDT_SR; //brief (Wdt Offset: 0x08) Status Register 
} Wdt;
*/

void initializeSystem(bool devicesAlreadyInitialized)
{
  
  delay(5000);
  
  digitalWrite(PWR_PRECHARGE_PIN,HIGH);
  
  //Serial.begin(115200);//coms to other processor
  
  #ifdef __USER_DEBUG_MODE__
    Serial.begin(115200);//coms to other processor
    Serial.println("INIT ********************************************************************************************************************");
  #endif
  
  //set low power wake up sources
  //pmc_set_fast_startup_input(PMC_FSMR_RTTAL|PMC_FSMR_RTCAL|0xFFFF);   //Enable Timer Wakeup
  //PMC->PMC_FSMR |= 0xFFFF;
  //pmc_enable_interrupt(0xFFFFFFFF);
  
  //for(int i=-20;i<256;i++)
  //{
  //  NVIC_ClearPendingIRQ((IRQn_Type)i);
  //}
  
  noInterrupts();
  
  for (std::vector<AbstractDevice*>::iterator it = devices.begin() ; it != devices.end(); ++it)
  {
    (*it)->initializeInterface(devicesAlreadyInitialized);
  }
  
  interrupts();
  
  #ifdef __USER_DEBUG_MODE__
    Serial.println("Interfaces Initialized");
  #endif
}

void initializeDevices()
{
  bool initSuccess = true;
  int count = 0;
  
  do
  {
    initSuccess = true;
    
    for (std::vector<AbstractDevice*>::iterator it = devices.begin() ; it != devices.end(); ++it)
    {
      if((*it)->getDeviceStatus() == DEV_UNINITIALIZED || (*it)->getDeviceStatus() == DEV_INIT_ERROR)
      {
        if((*it) != &interprocessorCommunicationHandler)
        {
          initSuccess &= (*it)->initializeDevice();
        }
      }
    }   
    
    count++;
    
  }while(!initSuccess && count < MAX_INIT_ATTEMPTS);

  #ifdef __USER_DEBUG_MODE__
    Serial.print("init attempts = ");
    Serial.println(count);  
    Serial.println("starting coms handshake");
  #endif
  
  interprocessorCommunicationHandler.initializeDevice();
  
  #ifdef __USER_DEBUG_MODE__
    Serial.println("Devices Initialized");
  #endif
}

void shutdownDevices()
{
  for (std::vector<AbstractDevice*>::iterator it = devices.begin() ; it != devices.end(); ++it)
  {
    (*it)->shutdownDevice();
  }
}

void initSystemTimers()
{
  //periodic timer interrupts
  Timer1.attachInterrupt(checkSensorsISR).start(100000);
  Timer2.attachInterrupt(runMotionControlISR).start(20000);
  Timer3.attachInterrupt(kickWatchDogISR).start(10000); 
  
  // Variable wdp_ms hold the period in 256 th of seconds of watchdog
  // It must be >3 && <= 4096
  // The following value set a periode of 4,5 seconds (256 x 4,5 = 1152)
  uint32_t wdp_ms = 1152;
  WDT_Enable(WDT, 0x2000 | wdp_ms | ( wdp_ms << 16 )); 
}

void stopSystemTimers()
{
  Timer1.stop();
  Timer2.stop();
  
  //leave the WDT timer running so the WDT doesn't restart the system
}

void runningLoop()
{
  noInterrupts();//mutex around the shared data structure
  
  RobotTask* nextTask; //const 
  //RobotTask lp(lowPowerMode,10);
  
  //semaphore here instead of checking size??
  //get next task from priority queue
  if(priorityQueue.size()>0)
  {
    //RobotTask t(priorityQueue.top().getFcn(),priorityQueue.top().getPriority());
    nextTask = new RobotTask(priorityQueue.top().getFcn(),priorityQueue.top().getPriority());//&t;
    priorityQueue.pop();
  }else
  {
    //enter low power mode
    nextTask = new RobotTask(lowPowerMode,10);// millis()+100
  }
  
  interrupts();//end mutex around the shared data structure
  
  //run task
  nextTask->runTask();
  
  delete nextTask;
}//END RUNNING LOOP

//Tasks/////////////////////////////////////////////////////////////////////////

void lowPowerMode()
{
  //TODO
  //Serial.println("LP");
  
  //while(priorityQueue.size()==0) //functional semaphore
  delay(10);
  
  //pmc_set_writeprotect(0);
  //pmc_enable_waitmode();
  
  //pmc_enable_sleepmode(0);
}

long currentTimeStamp_MC = 0;
long prevTimeStamp_MC = 0;

double angularVelocities[3] = {0,0,0};
double previousAngularVelocities[3] = {0,0,0};

void runMotionControl()
{
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_MOTION,HIGH);
  #endif
  
  #ifdef __USER_DEBUG_MODE__
    Serial.println("    MC");
  #endif
  
  prevTimeStamp_MC = currentTimeStamp_MC;
  currentTimeStamp_MC = millis();
  
    
  double axisAcceleration[3] = {0,0,0};
  //double gravityVector[3] = {0,0,0};
  
  double eulerAngles[3] = {0,0,0};

  double angularAccelerations[3] = {0,0,0};
  
  #ifdef __AGM_ENABLED__
    agm_bno055.getLinearAcceleration(axisAcceleration);
    agm_bno055.getEulerAngles(eulerAngles);
    
    previousAngularVelocities[0] = angularVelocities[0];
    previousAngularVelocities[1] = angularVelocities[1];
    previousAngularVelocities[2] = angularVelocities[2];
      
    agm_bno055.getAngularVelocities(angularVelocities);
    
    if(currentTimeStamp_MC - prevTimeStamp_MC > 0)
    {
      for(int i=0; i<3; i++)
      {
        angularAccelerations[i] = (angularVelocities[i] - previousAngularVelocities[i])/((currentTimeStamp_MC - prevTimeStamp_MC) / 1000.0);
      }
    }    
    
    //agm_bno055.getGravityVector(gravityVector);
    
    if(agm_bno055.getFlippingFlag())
    {
      #ifdef __USER_DEBUG_MODE__
        Serial.println(" runMotionControl -- agm_bno055 FlippingFlag set");
      #endif    
      agm_bno055.clearFlippingInterrupt();
    }
  #endif
  
  if(!interprocessorCommunicationHandler.getEmergencyStop())
  {  
    #ifdef __USER_DEBUG_MODE__
      Serial.println(" runMotionControl -- emergency stop flag NOT set");
    #endif     
    
    bool forward = interprocessorCommunicationHandler.getForward();
    double driveSpeed = interprocessorCommunicationHandler.getPercentSpeed();
    bool turnDirection = interprocessorCommunicationHandler.getLeftRight();
    double turnPercent = interprocessorCommunicationHandler.getPercentTurn();
    
    FlipperCommand frontFlipperCommand = interprocessorCommunicationHandler.getFrontFlipperCmd();
    FlipperCommand backFlipperCommand = interprocessorCommunicationHandler.getBackFlipperCmd();
    
    double frontFlipperAngle = 0;
    double backFlipperAngle = 0;
    
//    rightDriveEncoder.getAngle();
//    leftDriveEncoder.getAngle();
//    
//    frontFlipperEncoder.getAngle();
    
    #ifdef __ENCODERS_ENABLED__
      frontFlipperAngle = frontFlipperEncoder.getAngle();
      backFlipperAngle = rearFlipperEncoder.getAngle();   
    #endif
      
    #ifdef __MOTOR_CONTROLLERS_ENABLED__
      MotorPort_SaberToothDual25A* leftDriveMotor = driveMotorsController.getPortA();
      MotorPort_SaberToothDual25A* rightDriveMotor = driveMotorsController.getPortB();
      
      MotorPort_SaberToothDual25A* frontFlipperMotor = flipperMotorsController.getPortA();
      MotorPort_SaberToothDual25A* backFlipperMotor = flipperMotorsController.getPortB();     
      
      motionControlDriveAlgorithm(leftDriveMotor, rightDriveMotor, forward, driveSpeed, turnDirection, turnPercent);
      //motionControlDriveAlgorithm(driveMotorsController.getPortA(), driveMotorsController.getPortB(), interprocessorCommunicationHandler.getForward(), interprocessorCommunicationHandler.getPercentSpeed(), interprocessorCommunicationHandler.getLeftRight(), interprocessorCommunicationHandler.getPercentTurn());
      
      motionControlFlipperAlgorithm(frontFlipperMotor, backFlipperMotor, frontFlipperAngle, backFlipperAngle, frontFlipperCommand, backFlipperCommand);
      //motionControlFlipperAlgorithm(flipperMotorsController.getPortA(), flipperMotorsController.getPortA(), interprocessorCommunicationHandler.getFrontFlipperCmd(), interprocessorCommunicationHandler.getBackFlipperCmd());
    #endif
    
    #ifdef __AGM_ENABLED__
      #ifdef __SERVOS_ENABLED__

        CameraPanCommand panCmd = interprocessorCommunicationHandler.getPanCmd();
        CameraTiltCommand tiltCmd = interprocessorCommunicationHandler.getTiltCmd();
        ManipulatorArmCommand armCmd = interprocessorCommunicationHandler.getArmCmd();
        
        motionControlManipulatorArmAlgorithm(&manipulatorArm, eulerAngles, angularVelocities, angularAccelerations, panCmd, tiltCmd, armCmd); //gravityVector
        
      #endif
    #endif
  }
    
  #ifdef __USER_DEBUG_MODE__
    Serial.print("axisAcceleration ");
    Serial.print(axisAcceleration[0]);
    Serial.print(", ");
    Serial.print(axisAcceleration[1]);
    Serial.print(", ");
    Serial.println(axisAcceleration[2]);
    
    Serial.print("eulerAngles ");
    Serial.print(eulerAngles[0]);
    Serial.print(", ");
    Serial.print(eulerAngles[1]);
    Serial.print(", ");
    Serial.println(eulerAngles[2]);
  #endif

  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_MOTION,LOW);
  #endif

}


void handleCommunication()
{
  #ifdef __USER_DEBUG_MODE__
    Serial.println("         RC");
  #endif
  
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_COMS,HIGH);
  #endif
  
  //delayMicroseconds(500);
  
  interprocessorCommunicationHandler.handleCommunication();
  
  
  if(interprocessorCommunicationHandler.getEmergencyStop() && !interprocessorCommunicationHandler.getPrevEmergencyStop())
  {
    #ifdef __USER_DEBUG_MODE__
      Serial.println(" handleCommunication emergency Stop Detected");
    #endif    
    
    #ifdef __MOTOR_CONTROLLERS_ENABLED__
      driveMotorsController.emergencyStop();
      flipperMotorsController.emergencyStop();   
    #endif 
  }
 
  if(!interprocessorCommunicationHandler.getEmergencyStop() && interprocessorCommunicationHandler.getPrevEmergencyStop())
  {
    #ifdef __USER_DEBUG_MODE__
      Serial.println(" handleCommunication emergency Stop Released");
    #endif      
    
    #ifdef __MOTOR_CONTROLLERS_ENABLED__
      driveMotorsController.releaseEmergencyStop();
      flipperMotorsController.releaseEmergencyStop();
    #endif
  }
  
  if(interprocessorCommunicationHandler.getShutDownRobot())
  {
    #ifdef __USER_DEBUG_MODE__
      Serial.println(" handleCommunication shutdown Detected");
    #endif      
    
    pinMode(LEDpin, OUTPUT);
    digitalWrite(LEDpin, ON);
    
    mainState = SHUTTING_DOWN;
  }
  
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_COMS,LOW);
  #endif
  
}

void checkCommunication()
{
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_TIMEOUT_COMS,HIGH);
  #endif
  
  #ifdef __USER_DEBUG_MODE__
    Serial.println("                       CC");
  #endif   
  if(interprocessorCommunicationHandler.checkTimer(millis()))
  {
    #ifdef __USER_DEBUG_MODE__
      Serial.println(" checkCommunication emergency Stop due to loss of coms link");
    #endif      
    
    #ifdef __MOTOR_CONTROLLERS_ENABLED__
      driveMotorsController.emergencyStop();
      flipperMotorsController.emergencyStop();
    #endif    
  }
  
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_TIMEOUT_COMS,LOW);
  #endif
}

void checkTemperatures()
{
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_TEMPS,HIGH);
  #endif  
  
  #ifdef __USER_DEBUG_MODE__
    Serial.println("             CT");
  #endif
  
  #ifdef __TEMPERATURE_SENSORS_ENABLED__
    temperatureSensorOnBattery.readTemperatureSensor();
    
    temperatureSensorOnLeftDriveMotor.readTemperatureSensor();
    temperatureSensorOnRightDriveMotor.readTemperatureSensor();
    temperatureSensorOnFrontFlipperMotor.readTemperatureSensor();
    temperatureSensorOnBackFlipperMotor.readTemperatureSensor();
  #endif

  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_TEMPS,LOW);
  #endif    
}

bool toggleLED = false;

void checkVoltage()
{
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_VOLTS,HIGH);
  #endif    
  
  #ifdef __USER_DEBUG_MODE__
    Serial.println("                 CV");
  #endif
  
//  pinMode(LEDpin, OUTPUT);
//  digitalWrite(LEDpin, toggleLED);
//  toggleLED = !toggleLED;
  
  #ifdef __VOLTAGE_SENSOR_ENABLED__
    double voltToMotors = voltageSensorOnBattery.readVoltageSensor();
  #endif
  
  //driveMotorsController.setMotorControllerInputVoltage(voltToMotors);
  //flipperMotorsController.setMotorControllerInputVoltage(voltToMotors);
  
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_VOLTS,LOW);
  #endif  
}

//INTERRUPT ROUTINES////////////////////////////////////////////////////////////////////

int sensorCounter = 1;
void checkSensorsISR()
{
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_SENSORS_ISR,HIGH);
  #endif    
  //void* linkReg = enterISR();

  if(sensorCounter == 5)
  {
    //checkTemperatures_Flag = true;
    priorityQueue.push(RobotTask(checkTemperatures,millis()+CHECK_TEMPERATURES_DEADLINE));
    sensorCounter = 1;
  }else
  {
    sensorCounter++;
  }
  
  priorityQueue.push(RobotTask(checkCommunication,millis()+CHECK_VOLTAGE_DEADLINE));
  
  //checkVoltage_Flag = true;
  priorityQueue.push(RobotTask(checkVoltage,millis()+CHECK_VOLTAGE_DEADLINE));
  
  //exitISR(linkReg);
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_SENSORS_ISR,LOW);
  #endif    
}

bool attached_motionControlISR = false;

void runMotionControlISR()
{
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_MOTION_ISR,HIGH);
  #endif
  
  #ifdef __USER_DEBUG_MODE__
    Serial.println("***runMotionControlISR***");
  #endif
  if(attached_motionControlISR)
  {
    //void* linkReg = enterISR();
     
    priorityQueue.push(RobotTask(runMotionControl,millis()+RUN_MOTION_CONTROL_DEADLINE));
    
    //exitISR(linkReg);
  }else
  {
    attached_motionControlISR = true;
  }
  
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_MOTION_ISR,LOW);
  #endif  
}

void kickWatchDogISR()
{
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_WDT_ISR,HIGH);
  #endif  
  //kick watchdog directly in interrupt for fastest reponse
  //this is fine because it will be a very very short routine
  WDT_Restart(WDT);
  
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_WDT_ISR,LOW);
  #endif  
}

bool attached_interprocessorComsISR = false;

void interprocessorComsISR()
{
//  pinMode(LEDpin, OUTPUT);
//  digitalWrite(LEDpin, toggleLED);
//  toggleLED = !toggleLED;
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_COMS_ISR,HIGH);
  #endif   
  
  if(attached_interprocessorComsISR)
  {
    
    #ifdef __USER_DEBUG_MODE__
      Serial.println("interprocessorComsISR");
    #endif
    
    
    priorityQueue.push(RobotTask(handleCommunication,millis()+RECIEVE_COMMANDS_DEADLINE));
    
  }else
  {
    attached_interprocessorComsISR = true;
  }
  
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_COMS_ISR,LOW);
  #endif   
}

bool attached_flippingISR = false;

void flippingISR()
{
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_FLIPPING_ISR,HIGH);
  #endif     
  
  if(attached_flippingISR)
  {
    //void* linkReg = enterISR();
    
    #ifdef __USER_DEBUG_MODE__
      Serial.println("***flippingISR***");
    #endif
    
    #ifdef __AGM_ENABLED__
      agm_bno055.setFlippingFlag();
    #endif
    
    priorityQueue.push(RobotTask(runMotionControl,millis()+FLIPPING_DEADLINE));

    //exitISR(linkReg);
  }else
  {
    attached_flippingISR = true;
  }
  
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_FLIPPING_ISR,LOW);
  #endif     
}



bool attached_encoderRightDriveClkISR = false;

void encoderRightDriveClkISR()
{

  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_ENCODER_ISR,HIGH);
  #endif    
  
  if(attached_encoderRightDriveClkISR)
  {
    //void* linkReg = enterISR();
    #ifdef __USER_DEBUG_MODE__
      Serial.println("***encoderRightDriveClkISR***");
    #endif
    
    rightDriveEncoder.clockPulse();
    
    //exitISR(linkReg);
  }else
  {
    attached_encoderRightDriveClkISR = true;
  }
  
  #ifdef __LEDS_ENABLED__
    digitalWrite(LED_ENCODER_ISR,LOW);
  #endif     
}

#ifdef __ENCODERS_ENABLED__

bool attached_encoderLeftDriveClkISR = false;

void encoderLeftDriveClkISR()
{
  if(attached_encoderLeftDriveClkISR)
  {  
    //void* linkReg = enterISR();
    #ifdef __USER_DEBUG_MODE__
      Serial.println("***encoderLeftDriveClkISR***");
    #endif
    
    leftDriveEncoder.clockPulse();
    
    //exitISR(linkReg);
  }else
  {
    attached_encoderLeftDriveClkISR = true;
  }  
}



bool attached_encoderFrontFlipperClkISR = false;

void encoderFrontFlipperClkISR()
{
  if(attached_encoderFrontFlipperClkISR)
  {    
    //void* linkReg = enterISR();
    #ifdef __USER_DEBUG_MODE__
      Serial.println("***encoderFrontFlipperClkISR***");
    #endif
    
    frontFlipperEncoder.clockPulse();
    
    //exitISR(linkReg);
  }else
  {
    attached_encoderFrontFlipperClkISR = true;
  } 
}



bool attached_encoderRearFlipperClkISR = false;

void encoderRearFlipperClkISR()
{
  if(attached_encoderRearFlipperClkISR)
  {    
    //void* linkReg = enterISR();
    #ifdef __USER_DEBUG_MODE__
      Serial.println("***encoderRearFlipperClkISR***");
    #endif  
    
    rearFlipperEncoder.clockPulse();
    
    //exitISR(linkReg);
  }else
  {
    attached_encoderRearFlipperClkISR = true;
  }     
}

#endif
