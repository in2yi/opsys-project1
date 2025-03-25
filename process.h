#ifndef __PROCESS_H__
#define __PROCESS_H__

#include <cmath>

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

  bool onCPUBurst() { return burst_index % 2 == 0; };
  bool onIOBurst() { return burst_index % 2 != 0; };

  int getT() { return t; };
  int getTau() { return tau; };
  int getCpuBurstsLeft() { return std::ceil( (num_total_bursts - burst_index ) / 2.0 ); };
  int getTotalCpuTime();

  int waitBurst(int current_time) { return burst_completion_time = current_time + time_remaining; };
  int burstCompletionTime() { return burst_completion_time; };

  void finishBurst() { burst_completion_time = 0; };
  void updateProcess(int currentTime); // Increments burst index
  void preempt(int elapsedTime);

  void reset();
};

#endif
