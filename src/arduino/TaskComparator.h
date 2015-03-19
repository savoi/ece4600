
#ifndef __TASK_COMPARATOR__
#define __TASK_COMPARATOR__

#include "RobotTask.h"

class TaskComparator
{
    protected:
        bool reverse;

    public:
        TaskComparator(const bool& rev = false)
        {
          reverse = rev;
        }

        bool operator() (const RobotTask& taskA, const RobotTask& taskB);
};

#endif


