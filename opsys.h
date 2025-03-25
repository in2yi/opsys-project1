#ifndef __OPSYS_H__
#define __OPSYS_H__

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <cstring>
#include "process.h"

#define TRUNCATE true 
#define TRUNC_TIME 10000

class CompArrivalTime
{
public:
  bool operator() (Process* a, Process* b)
  {
    if (a->arrival_time == b->arrival_time)
    {
      return std::strcmp(a->id, b->id) > 0;
    }

    return a->arrival_time > b->arrival_time;
  }
};

class CompPredBurstTime
{
public:
  bool operator() (Process* a, Process* b)
  {
    if (a->getTau() == b->getTau())
    {
      return std::strcmp(a->id, b->id) > 0;
    }
    return a->getTau() > b->getTau();
  }
};

class CompPredBurstRemTime
{
public:
  bool operator() (Process* a, Process* b)
  {
    if (a->tau_remaining == b->tau_remaining)
    {
      return std::strcmp(a->id, b->id) > 0;
    }
    return a->tau_remaining > b->tau_remaining;
  }
};

class CompBurstCompletionTime
{
public:
  bool operator() (Process* a, Process* b)
  {
    if (a->burstCompletionTime() == b->burstCompletionTime())
    {
      return std::strcmp(a->id, b->id) > 0;
    }

    return a->burstCompletionTime() > b->burstCompletionTime();
  }
};

void print_queue(const std::queue<Process*> &ready);
template<class T> void print_priority_queue(const std::priority_queue<Process*, std::vector<Process*>, T> &ready)
{
  std::priority_queue<Process*, std::vector<Process*>, T> q = ready;
  std::cout << "[Q";
  if (!q.empty())
  {
    while (!q.empty())
    {
      std::cout << " " << q.top()->id;
      q.pop();
    }
  } else
  {
    std::cout << " empty";
  }
  std::cout << "]\n";
};

class OpSys
{
public:
  Process* running = NULL;
  Process* switchingToRun = NULL;
  Process* switchingToReady = NULL;
  Process* switchingToIO = NULL;
  std::queue<Process*> readyFCFS;
  std::priority_queue<Process*, std::vector<Process*>, CompPredBurstTime> readySJF;
  std::priority_queue<Process*, std::vector<Process*>, CompPredBurstRemTime> readySRT;
  std::queue<Process*> readyRR; 
  std::priority_queue<Process*, std::vector<Process*>, CompBurstCompletionTime> waiting;
  std::priority_queue<Process*, std::vector<Process*>, CompArrivalTime> unarrived;
  std::unordered_set<Process*> unfinished;
  std::vector<Process*> finished;
  int time = 0;
  int contextSwitchTime;
  int timeSlice;
  
  void finishIOSwitchOut(int current_time) { switchingToIO = NULL; if (current_time == 0) return; };
  
  /* FCFS */
  void processArrivalFCFS(int currentTime);
  void switchOutCpuFCFS(int currentTime);
  void completeIOFCFS(int currentTime);
  void startCpuUsageFCFS(int currentTime);
  void startSwitchInFCFS(int currentTime);
  void runFCFSScheduler();
  
  /* SJF */
  void processArrivalSJF(int currentTime);
  void switchOutCpuSJF(int currentTime);
  void completeIOSJF(int currentTime);
  void startCpuUsageSJF(int currentTime);
  void startSwitchInSJF(int currentTime);
  void runSJFScheduler();
  
  /* SRT */
  void processArrivalSRT(int currentTime);
  void switchOutCpuSRT(int currentTime);
  void completeIOSRT(int currentTime);
  void startCpuUsageSRT(int currentTime);
  void startSwitchInSRT(int currentTime);
  void finishPreemptSwitchOutSRT(int currentTime);
  void preemptNowSRT(int currentTime);
  void runSRTScheduler();
  bool shouldPreempt(int currentTime, Process* cur, Process* top);
  
  /* RR */
  void processArrivalRR(int currentTime);
  void switchOutCpuRR(int currentTime);
  void completeIORR(int currentTime);
  void startCpuUsageRR(int currentTime);
  void timeSliceExpirationRR(int currentTime);
  void startSwitchInRR(int currentTime);
  void finishPreemptSwitchOutRR(int currentTime);
  void runRoundRobinScheduler();

  /* Statistics */
  void printStats(std::ofstream& simout);
  void printRRStats(std::ofstream& simout);
};

struct Action
{
  int time;
  void (OpSys::*func)(int);
  int priority;
};

class CompAction
{
public:
  bool operator() (Action a, Action b)
  {
    if (a.time == b.time)
    {
      return a.priority > b.priority;
    }
    
    return a.time > b.time;
  }
};

#endif
