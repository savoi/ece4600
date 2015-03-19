
#ifndef __PRIORITY_QUEUE__
#define __PRIORITY_QUEUE__

#include "RobotTask.h"
#include "TaskComparator.h"
#include <queue>
#include <vector>

//O(log(n)) insert time is suitable for ISR
typedef std::priority_queue<RobotTask,std::vector<RobotTask>,TaskComparator> PriorityQueue; //this need to be volatile and as such ill need to make my own version?

/*class Node
{
    public:
        Node* next; 
        Node* prev;
        RobotTask* t;
        
        Node(RobotTask& t, Node& next, Node& prev)
};*/

/*class PriorityQueue
{
    protected:
        int numTasks;
        RobotTask* tasks;

    public:
        
        PriorityQueue()
        {
            numTasks = 0;
        }

        void push(RobotTask t) volatile
        {
          tasks[numTasks++] = RobotTask(t.getFcn(),t.getPriority());
        }
        
        RobotTask pop() volatile
        {
          int maxT = 0;
          for(int i=1; i<numTasks; i++)
          {
            if(tasks[maxT].getPriority()<tasks[i].getPriority())
            {
              maxT = i;
            }
          }
          RobotTask ret(tasks[maxT].getFcn(),tasks[maxT].getPriority());
          for(int i=maxT+1; i<numTasks; i++)
          {
            tasks[i-1] = tasks[i];
          }
          numTasks--;
          return ret;
        }
        
        bool isEmpty() volatile
        {
          return numTasks==0;
        }
};*/

#endif


