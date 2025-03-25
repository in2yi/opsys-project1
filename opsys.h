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

/**
 * @brief Comparator for Process pointers based on arrival time
 *
 */
class CompArrivalTime
{
public:
/**
 * @brief Function to compare two Process pointers based on arrival time
 * 
 * @param a Process pointer
 * @param b Process pointer
 * @return true Returns true if a has a smaller arrival time than b
 * @return false Returns false if a has a larger arrival time than b
 */
  bool operator() (Process* a, Process* b)
  {
    if (a->arrival_time == b->arrival_time)
    {
      return std::strcmp(a->id, b->id) > 0;
    }

    return a->arrival_time > b->arrival_time;
  }
};

/**
 * @brief Comparator for Process pointers based on burst time
 * 
 */
class CompPredBurstTime
{
public:
/**
 * @brief Function to compare two Process pointers based on burst time
 * 
 * @param a Process pointer
 * @param b Process pointer
 * @return true Returns true if a has a smaller burst time than b
 * @return false Returns false if a has a larger burst time than b
 */
  bool operator() (Process* a, Process* b)
  {
    if (a->getTau() == b->getTau())
    {
      return std::strcmp(a->id, b->id) > 0;
    }
    return a->getTau() > b->getTau();
  }
};

/**
 * @brief Comparator for Process pointers based on remaining burst time
 * 
 */
class CompPredBurstRemTime
{
public:
 /**
  * @brief Function to compare two Process pointers based on remaining burst time
  * 
  * @param a Process pointer
  * @param b Process pointer
  * @return true Returns true if a has a smaller remaining burst time than b
  * @return false Returns false if a has a larger remaining burst time than b
  */
  bool operator() (Process* a, Process* b)
  {
    if (a->tau_remaining == b->tau_remaining)
    {
      return std::strcmp(a->id, b->id) > 0;
    }
    return a->tau_remaining > b->tau_remaining;
  }
};

/**
 * @brief Comparator for Process pointers based on burst completion time
 * 
 */
class CompBurstCompletionTime
{
public:
/**
 * @brief Function to compare two Process pointers based on burst completion time
 * 
 * @param a Process pointer
 * @param b Process pointer
 * @return true Returns true if a has a smaller burst completion time than b
 * @return false Returns false if a has a larger burst completion time than b
 */
  bool operator() (Process* a, Process* b)
  {
    if (a->burstCompletionTime() == b->burstCompletionTime())
    {
      return std::strcmp(a->id, b->id) > 0;
    }

    return a->burstCompletionTime() > b->burstCompletionTime();
  }
};

/**
 * @brief Function to print the contents of a queue of Process pointers
 * 
 * @param ready Queue of Process pointers
 */
void print_queue(const std::queue<Process*> &ready);

/**
 * @brief Template function to print a priority queue of Process pointers
 * 
 * @tparam T Comparator type
 * @param ready Priority queue of Process pointers
 */
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

/**
 * @brief Class to represent the Operating System
 * 
 */
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
  
  /**
   * @brief Function to finish the switch-out of a process to I/O
   * 
   * @param current_time 
   */
  void finishIOSwitchOut(int current_time) { switchingToIO = NULL; if (current_time == 0) return; };
  
  /* FCFS */
  /**
   * @brief Function to process the arrival of a process in the FCFS scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void processArrivalFCFS(int currentTime);

  /**
   * @brief Function to switch out the CPU in the FCFS scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void switchOutCpuFCFS(int currentTime);

  /**
   * @brief Function to complete I/O in the FCFS scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void completeIOFCFS(int currentTime);

  /**
   * @brief Function to start CPU usage in the FCFS scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void startCpuUsageFCFS(int currentTime);

  /**
   * @brief Function to start the switch-in in the FCFS scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void startSwitchInFCFS(int currentTime);

  /**
   * @brief Function to run the FCFS scheduler
   * 
   */
  void runFCFSScheduler();
  

  /* SJF */
  /**
   * @brief Function to process the arrival of a process in the SJF scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void processArrivalSJF(int currentTime);

  /**
   * @brief Function to switch out the CPU in the SJF scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void switchOutCpuSJF(int currentTime);

  /**
   * @brief Function to complete I/O in the SJF scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void completeIOSJF(int currentTime);

  /**
   * @brief Function to start CPU usage in the SJF scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void startCpuUsageSJF(int currentTime);

  /**
   * @brief Function to start the switch-in in the SJF scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void startSwitchInSJF(int currentTime);

  /**
   * @brief Function to run the SJF scheduler
   * 
   */
  void runSJFScheduler();
  
  /* SRT */
  /**
   * @brief Function to process the arrival of a process in the SRT scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void processArrivalSRT(int currentTime);

  /**
   * @brief Function to switch out the CPU in the SRT scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void switchOutCpuSRT(int currentTime);

  /**
   * @brief Function to complete I/O in the SRT scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void completeIOSRT(int currentTime);

  /**
   * @brief Function to start CPU usage in the SRT scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void startCpuUsageSRT(int currentTime);

  /**
   * @brief Function to start the switch-in in the SRT scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void startSwitchInSRT(int currentTime);

  /**
   * @brief Function to finish the preempt switch-out in the SRT scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void finishPreemptSwitchOutSRT(int currentTime);

  /**
   * @brief Function to preempt now in the SRT scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void preemptNowSRT(int currentTime);

  /**
   * @brief Function to run the SRT scheduler
   * 
   */
  void runSRTScheduler();

  /**
   * @brief Function to determine if a process should be preempted
   * 
   * @param currentTime Integer representing the current time
   * @param cur Process pointer representing the current process
   * @param top Process pointer representing the top process
   * @return true Returns true if the current process should be preempted
   * @return false Returns false if the current process should not be preempted
   */
  bool shouldPreempt(int currentTime, Process* cur, Process* top);
  
  /* RR */
  /**
   * @brief Function to process the arrival of a process in the RR scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void processArrivalRR(int currentTime);

  /**
   * @brief Function to switch out the CPU in the RR scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void switchOutCpuRR(int currentTime);

  /**
   * @brief Function to complete I/O in the RR scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void completeIORR(int currentTime);

  /**
   * @brief Function to start CPU usage in the RR scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void startCpuUsageRR(int currentTime);

  /**
   * @brief Function to determine if a time slice has expired in the RR scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void timeSliceExpirationRR(int currentTime);

  /**
   * @brief Function to start the switch-in in the RR scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void startSwitchInRR(int currentTime);

  /**
   * @brief Function to finish the preempt switch-out in the RR scheduler
   * 
   * @param currentTime Integer representing the current time
   */
  void finishPreemptSwitchOutRR(int currentTime);

  /**
   * @brief Function to run the RR scheduler
   * 
   */
  void runRoundRobinScheduler();

  /* Statistics */
  /**
   * @brief Function to print the statistics of the Operating System
   * 
   * @param simout Output file stream
   */
  void printStats(std::ofstream& simout);

  /**
   * @brief Function to print the statistics of the Operating System for RR
   * 
   * @param simout Output file stream
   */
  void printRRStats(std::ofstream& simout);
};

/**
 * @brief Struct to represent an action in the Operating System
 * 
 */
struct Action
{
  int time;
  void (OpSys::*func)(int);
  int priority;
};

/**
 * @brief Comparator for Action structs
 * 
 */
class CompAction
{
public:
/**
 * @brief Function to compare two Action structs
 * 
 * @param a Action struct
 * @param b Action struct
 * @return true Returns true if a has a smaller time than b
 * @return false Returns false if a has a larger time than b
 */
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
