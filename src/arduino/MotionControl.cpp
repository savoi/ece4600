
#include "MotionControl.h"

void motionControlDriveAlgorithm(MotorPortInterface* leftDriveMotor, MotorPortInterface* rightDriveMotor, bool forward, double driveSpeed, bool turnDirection, double turnPercent)
{

  double leftDriveMotorSpeed = driveSpeed;
  double rigthDriveMotorSpeed = driveSpeed;
  bool leftDriveMotorDirection = forward;
  bool rightDriveMotorDirection = forward;
  
  if(turnDirection)
  {
    leftDriveMotorSpeed -= 2*(turnPercent/127.0)*driveSpeed;
  }else
  {
    rigthDriveMotorSpeed -= 2*(turnPercent/127.0)*driveSpeed;
  }
  
  if(forward)
  {
    if(leftDriveMotorSpeed>0)
    {
      leftDriveMotorDirection = true;
    }else
    {
      leftDriveMotorDirection = false;
    }
    
    if(rigthDriveMotorSpeed>0)
    {
      rightDriveMotorDirection = true;
    }else
    {
      rightDriveMotorDirection = false;
    }
  }else
  {
    if(leftDriveMotorSpeed>0)
    {
      leftDriveMotorDirection = false;
    }else
    {
      leftDriveMotorDirection = true;
    }
    
    if(rigthDriveMotorSpeed>0)
    {
      rightDriveMotorDirection = false;
    }else
    {
      rightDriveMotorDirection = true;
    }     
  }
  
  leftDriveMotor->setMotorSpeed(abs(leftDriveMotorSpeed),leftDriveMotorDirection);
  rightDriveMotor->setMotorSpeed(abs(rigthDriveMotorSpeed),rightDriveMotorDirection);
  
}


void motionControlFlipperAlgorithm(MotorPortInterface* frontFlipperMotor, MotorPortInterface* backFlipperMotor, double frontFlipperAngle, double backFlipperAngle, FlipperCommand frontFlipperCommand, FlipperCommand backFlipperCommand)
{
  
}


const int ROLL = 0;
const int PITCH = 1;
const int YAW = 2;

double manipulatorArmHeight = 0;

void motionControlManipulatorArmAlgorithm(ManipulatorArm* manipulatorArm, double* eulerAngles, double* angularVelocities, double* angularAccelerations, CameraPanCommand panCmd, CameraTiltCommand tiltCmd, ManipulatorArmCommand armCmd)
{
  
//  double PID_VELOCITY_CONTROL_P = 1.5;
//  double PID_VELOCITY_CONTROL_I = 0.5;
//  double PID_VELOCITY_CONTROL_D = 0.1;
//  
//  double percentPosition = 25;
//  double percentVelocity = PID_VELOCITY_CONTROL_P*abs(angularVelocities[PITCH]) + PID_VELOCITY_CONTROL_I*abs(eulerAngles[PITCH]) + PID_VELOCITY_CONTROL_D*abs(angularAccelerations[PITCH]); //PID
//  
//  
//  //double thetaNot =  180 - 
//  
//  if(eulerAngles[PITCH] > 0) //right side up
//  {
//    
//  }else //inverted operation
//  {
//    percentPosition = -percentPosition;
//  }
  
  switch(armCmd)
  {
    case MANIPULATOR_ARM_STOP:
    {
      
    }break;
    case MANIPULATOR_ARM_UP:
    {
      manipulatorArmHeight+=1;
    }break;
    case MANIPULATOR_ARM_DOWN:
    {
      manipulatorArmHeight-=1;
    }break;
  }
  
  if(manipulatorArmHeight>100)
  {
    manipulatorArmHeight = 100;
  }
  if(manipulatorArmHeight<-100)
  {
    manipulatorArmHeight = -100;
  }
  
  manipulatorArm->setHeight(manipulatorArmHeight, 25);
  
  //manipulatorArm->setHeight(percentPosition, percentVelocity);
  
}

















