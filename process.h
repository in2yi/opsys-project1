#ifndef __PROCESS_H__
#define __PROCESS_H__

#include <cmath>

/**
 * @brief Class to represent a process
 * 
 */
class Process {
public:
  int burst_completion_time = 0;
  int burst_index = 0;
  int prev_t;
  int prev_tau; 
  double alpha;
  int tau;
  int tau_0;
  int t;
  char * id;
  int * burst_times;
  int arrival_time;
  bool is_cpu_bound;
  int num_cpu_bursts;
  int num_total_bursts;
  int lastCpuBurstStart; 
  int time_remaining = 0;
  int tau_remaining = 0;
  int lastSwitchTime = 0;
  /* Statistics */
  int num_switches = 1;
  int num_preempts = 0;
  int total_cpu_time;
  int start_turnaround; 
  int total_turnaround; 

  /**
   * @brief Function to determine if the process is on a CPU burst
   * 
   * @return true Returns true if the process is on a CPU burst
   * @return false Returns false if the process is not on a CPU burst
   */
  bool onCPUBurst() { return burst_index % 2 == 0; };

  /**
   * @brief Function to determine if the process is on an I/O burst
   * 
   * @return true Returns true if the process is on an I/O burst
   * @return false Returns false if the process is not on an I/O burst
   */
  bool onIOBurst() { return burst_index % 2 != 0; };

  /**
   * @brief Function to get t (time) of the process
   * 
   * @return int Returns the time of the process
   */
  int getT() { return t; };

  /**
   * @brief Get the Tau object
   * 
   * @return int 
   */
  int getTau() { return tau; };

  /**
   * @brief Function to get the number of CPU bursts left
   * 
   * @return int Returns the number of CPU bursts left
   */
  int getCpuBurstsLeft() { return std::ceil( (num_total_bursts - burst_index ) / 2.0 ); };

  /**
   * @brief Get the Total Cpu Time object
   * 
   * @return int 
   */
  int getTotalCpuTime();

  /**
   * @brief Function to wait for a burst
   * 
   * @param current_time Integer representing the current time
   * @return int Returns the burst completion time
   */
  int waitBurst(int current_time) { return burst_completion_time = current_time + time_remaining; };

  /**
   * @brief Get the Burst Completion Time object
   * 
   * @return int 
   */
  int burstCompletionTime() { return burst_completion_time; };

  /**
   * @brief Function to finish a burst
   * 
   */
  void finishBurst() { burst_completion_time = 0; };

  /**
   * @brief Function to update the process
   * 
   * @param currentTime Integer representing the current time
   */
  void updateProcess(int currentTime); // Increments burst index

  /**
   * @brief Function to preempt the process
   * 
   * @param elapsedTime Integer representing the elapsed time
   */
  void preempt(int elapsedTime);

  /**
   * @brief Function to reset the process
   * 
   */
  void reset();
};

#endif
