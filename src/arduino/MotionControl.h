
#include "MotorPort_SaberToothDual25A.h"
#include "InterprocessorCommunicationHandler.h"
#include "ManipulatorArm.h"
#include "Arduino.h"

#ifndef __MOTION_CONTROL__
#define __MOTION_CONTROL__
//extern
void motionControlDriveAlgorithm(MotorPortInterface* leftDriveMotor, MotorPortInterface* rightDriveMotor, bool forward, double driveSpeed, bool turnDirection, double turnPercent);
void motionControlFlipperAlgorithm(MotorPortInterface* frontFlipperMotor, MotorPortInterface* backFlipperMotor, double frontFlipperAngle, double backFlipperAngle, FlipperCommand frontFlipperCommand, FlipperCommand backFlipperCommand);
void motionControlManipulatorArmAlgorithm(ManipulatorArm* manipulatorArm, double* eulerAngles, double* angularVelocities, double* angularAccelerations, CameraPanCommand panCmd, CameraTiltCommand tiltCmd, ManipulatorArmCommand armCmd);

#endif

