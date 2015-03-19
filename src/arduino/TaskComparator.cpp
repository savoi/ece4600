#include "TaskComparator.h"

bool TaskComparator::operator() (const RobotTask& taskA, const RobotTask& taskB)
{
  bool ret = false;
  if(reverse)
  {
    ret = (taskA.getPriority() < taskB.getPriority());
    //ret = (taskA.priority > taskB.priority);
  }else
  {
    ret = (taskA.getPriority() > taskB.getPriority());
    //ret = (taskA.priority < taskB.priority);
  }
  return ret;
}


