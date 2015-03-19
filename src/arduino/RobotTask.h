
#ifndef __ROBOT_TASK__
#define __ROBOT_TASK__

class RobotTask
{
    protected:
        void (*fcnPtr)();
        unsigned long priority; 

    public:
        
   
        RobotTask(void (*fcn)(), int prio)
        {
            fcnPtr = fcn;
            priority = prio;
        }

        void runTask() const;
        unsigned long getPriority() const;
        void (*getFcn()const)();
};

#endif


