#include "RobotTask.h"

void RobotTask::runTask() const
{
    fcnPtr();
}

unsigned long RobotTask::getPriority() const
{
  return priority;
}

void (*RobotTask::getFcn()const)() 
{
  return fcnPtr;
}


