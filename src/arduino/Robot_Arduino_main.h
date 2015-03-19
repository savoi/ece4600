#include "Arduino.h"
#include "Motor.h"

#ifndef __ROBOT_ARDUINO_MAIN__
#define __ROBOT_ARDUINO_MAIN__

//debugging

//#define __USER_DEBUG_MODE__

//#define __ENCODERS_ENABLED__
#define __AGM_ENABLED__
//#define __SERVOS_ENABLED__
#define __MOTOR_CONTROLLERS_ENABLED__
#define __TEMPERATURE_SENSORS_ENABLED__
#define __VOLTAGE_SENSOR_ENABLED__
#define __INTERPROCESSOR_COMS_ENABLED__
//#define __LEDS_ENABLED__

//Flash Memory
const uint32_t PROCESSOR_FAILURE_FLAG_FLASH_ADDRESS = 0;
const byte PROCESSOR_FAILURE_FLAG = 0xAA;
const byte EMPTY_BYTE = 0x00;

// DEVICE IDS //////////////////////////////////////////////////

const int DEVICE_ID_RIGHT_DRIVE_ENCODER = 01;
const int DEVICE_ID_LEFT_DRIVE_ENCODER = 02;
const int DEVICE_ID_FRONT_FLIPPER_ENCODER = 03;
const int DEVICE_ID_BACK_FLIPPER_ENCODER = 04;

const int DEVICE_ID_AGM_BNO055 = 11;

const int DEVICE_ID_DRIVE_MOTOR_CONTROLLER = 21;
const int DEVICE_ID_FLIPPER_MOTOR_CONTROLLER = 22;

const int DEVICE_ID_INTERPROCESSOR_COM_HANDLER = 31;

const int DEVICE_ID_G15_SERVO_BOTTOM = 41;
const int DEVICE_ID_G15_SERVO_MIDDLE = 42;
const int DEVICE_ID_G15_SERVO_TOP = 43;
const int DEVICE_ID_G15_SERVO_PAN = 44;

const int DEVICE_ID_MANIPULATOR_ARM = 51;

const int DEVICE_ID_TEMP_SENSOR_BATTERY = 61;
const int DEVICE_ID_TEMP_SENSOR_LEFT_DRIVE_MOTOR = 62;
const int DEVICE_ID_TEMP_SENSOR_RIGHT_DRIVE_MOTOR = 63;
const int DEVICE_ID_TEMP_SENSOR_FRONT_FLIPPER_MOTOR = 64;
const int DEVICE_ID_TEMP_SENSOR_BACK_FLIPPER_MOTOR = 65;

const int DEVICE_ID_VOLT_SENSOR_BATTERY = 71;

// PIN MANAGEMENT ///////////////////////////////////////

const int LED_COMS = 3;
const int LED_MOTION = 4;
const int LED_TEMPS = 5;
const int LED_VOLTS = 6;
const int LED_TIMEOUT_COMS = 7;

const int LED_ENCODER_ISR = 8;
const int LED_COMS_ISR = 9; //
const int LED_WDT_ISR = 10; //
const int LED_FLIPPING_ISR = 11;
const int LED_SENSORS_ISR = 12; 
const int LED_MOTION_ISR = 13; //
 
const int G15_SERVO_CONTROL_PIN =                        2;
const int LEDpin =                                       22;
//const int RESET_FROM_MX6_PIN =                         24;
//const int LIGHTS_MX6_PIN =                             26;
const int INTERPROCESSOR_COMS_TO_PIN =                   30;
const int INTERPROCESSOR_COMS_FROM_PIN =                 31;
const int BNO055_INTERRUPT_PIN =                         35;
const int PWR_PRECHARGE_PIN =                            36;
const int ENCODER_RIGHT_DRIVE_CLK_PIN =                  40;
const int ENCODER_RIGHT_DRIVE_DIR_PIN =                  41;
const int ENCODER_LEFT_DRIVE_CLK_PIN =                   42;
const int ENCODER_LEFT_DRIVE_DIR_PIN =                   43;
const int ENCODER_FRONT_FLIPPER_CLK_PIN =                44;
const int ENCODER_FRONT_FLIPPER_DIR_PIN =                45;
const int ENCODER_REAR_FLIPPER_CLK_PIN =                 46;
const int ENCODER_REAR_FLIPPER_DIR_PIN =                 47;
const int DRIVER_MOTOR_CONTROLLER_EMERGENCY_STOP_PIN =   50;
const int FLIPPER_MOTOR_CONTROLLER_EMERGENCY_STOP_PIN =  51;



const int BATTERY_TEMP_SENSOR_PIN = A0;
const int LEFT_DRIVE_MOTOR_TEMP_SENSOR_PIN = A1;
const int RIGHT_DRIVE_MOTOR_TEMP_SENSOR_PIN = A2;
const int FRONT_FLIPPER_MOTOR_TEMP_SENSOR_PIN = A3;
const int BACK_FLIPPER_MOTOR_TEMP_SENSOR_PIN = A4;
const int BATTERY_VOLT_SENSOR_PIN = A5;

//AGM//////////////////////////////////////////////////

#define BNO055_I2C_BUS Wire

//Interprocessor coms//////////////////////////////////////

#define INTERPROCESSOR_SERIAL_BUS Serial
const int INTERPROCESSOR_SERIAL_BUS_BAUD = 115200;
const long COMMUNICATION_TIMEOUT_MS = 1000;
const int DECIMAL_PRECISION_FACTOR = 10;

//Servos///////////////////////////////////////////////////////

#define G15_SERVO_SERIAL_BUS Serial2
const double MANIPULATOR_ARM_LOWER_LENGTH = 20;
const double MANIPULATOR_ARM_UPPER_LENGTH = 20;

#define ConvertAngle2Pos(Angle) word(word(Angle)*1088UL/360UL)

//to be set
const int BOTTOM_G15_SERVO_ADDRESS = 1;
const byte BOTTOM_G15_SERVO_CW_MAX_ANGLE = ConvertAngle2Pos(10);
const byte BOTTOM_G15_SERVO_CCW_MAX_ANGLE = ConvertAngle2Pos(170);

const int MIDDLE_G15_SERVO_ADDRESS = 2;
const byte MIDDLE_G15_SERVO_CW_MAX_ANGLE = ConvertAngle2Pos(80);
const byte MIDDLE_G15_SERVO_CCW_MAX_ANGLE = ConvertAngle2Pos(100);

const int TOP_G15_SERVO_ADDRESS = 3;
const byte TOP_G15_SERVO_CW_MAX_ANGLE = 0;
const byte TOP_G15_SERVO_CCW_MAX_ANGLE = 0;

const int PAN_G15_SERVO_ADDRESS = 4;
const byte PAN_G15_SERVO_CW_MAX_ANGLE = 0;
const byte PAN_G15_SERVO_CCW_MAX_ANGLE = 0;

//DC Brushless Motors//////////////////////////////////////////////////

const double AME_218_24V_MOTOR_MAX_VOLTAGE = 24.0;

const bool LEFT_DRIVE_MOTOR_DIRECTION =         MOTOR_DIRECTION_NORMAL;
const double LEFT_DRIVE_MOTOR_MAX_VOLTAGE =     AME_218_24V_MOTOR_MAX_VOLTAGE;
const bool RIGHT_DRIVE_MOTOR_DIRECTION =        MOTOR_DIRECTION_REVERSE;
const double RIGHT_DRIVE_MOTOR_MAX_VOLTAGE =    AME_218_24V_MOTOR_MAX_VOLTAGE;
const bool FRONT_FLIPPER_MOTOR_DIRECTION =      MOTOR_DIRECTION_NORMAL;
const double FRONT_FLIPPER_MOTOR_MAX_VOLTAGE =  AME_218_24V_MOTOR_MAX_VOLTAGE;
const bool BACK_FLIPPER_MOTOR_DIRECTION =       MOTOR_DIRECTION_REVERSE;
const double BACK_FLIPPER_MOTOR_MAX_VOLTAGE =   AME_218_24V_MOTOR_MAX_VOLTAGE;

#define SABERTOOTH_DRIVE_SERIAL_BUS Serial3
#define SABERTOOTH_FLIPPER_SERIAL_BUS Serial3

const int SABERTOOTH_DRIVE_SERIAL_ADDRESS = 128;
const int SABERTOOTH_FLIPPER_SERIAL_ADDRESS = 135;

//ENCODERS/////////////////////////////////////////////////

const double ENCODER_DRIVE_GEAR_RATIO = 2.0;

const double ENCODER_FLIPPER_GEAR_RATIO = 5.0;

//ADC constants/////////////////////////////////////////////////

const double BIT_TO_VOLTAGE_CONSTANT = 0.0049; //volts per bit

//TEMPERATURE SENSORS///////////////////////////////////////// ALL TO BE SET SOON

const double VOLT_TO_TEMP_CONSTANT = 100*3.0/2.0; // DEGREES PER VOLT

//VOLTAGE DIVDER SENSOR//////////////////////////////////////////////////////////////

const double PRIMARY_VOLT_DIVIDER_CONSTANT = 10;
const double SECONDARY_VOLT_DIVIDER_CONSTANT = 5;

//////////////////////////////////////////////////////////////////

//main states
const int INITIALIZING = 0;
const int RUNNING = 1;
const int SHUTTING_DOWN = 2;
const int DEBUGGING = 3;

const int MAX_INIT_ATTEMPTS = 3;

//scheduler task constants
const int SENSOR_TIMER_PERIOD_MS = 100; 

const int RUN_MOTION_CONTROL_PERIOD_MS = 100; 
const int CHECK_TEMPERATURES_PERIOD_MS = 500;
const int KICK_WATCHDOG_PERIOD_MS = 100;
const int CHECK_VOLTAGE_PERIOD_MS = 250;
const int SEND_STATUS_PERIOD_MS = 200;

/*const int RUN_MOTION_CONTROL_PRIORITY = 60; 
const int CHECK_TEMPERATURES_PRIORITY = 50;
const int KICK_WATCHDOG_PRIORITY = 0;
const int CHECK_VOLTAGE_PRIORITY = 40;
const int FLIPPING_PRIORITY = 10;
const int RECIEVE_COMMANDS_PRIORITY = 20;
const int SEND_STATUS_PRIORITY = 30;*/

const int RUN_MOTION_CONTROL_DEADLINE = 50; 
const int CHECK_TEMPERATURES_DEADLINE = 200;
const int CHECK_VOLTAGE_DEADLINE = 100;
const int FLIPPING_DEADLINE = 10;
const int RECIEVE_COMMANDS_DEADLINE = 50;
const int SEND_STATUS_DEADLINE = 100;

#endif

