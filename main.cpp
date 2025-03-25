// combined.cpp
#include "opsys.h"
#include "process.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <list>
#include <iomanip>
#include <queue>
#include <unordered_set>

// ----------------------------------------------------------------------
//                   FCFS (from fcfs.cpp)
// ----------------------------------------------------------------------
void OpSys::process_arrive_fcfs( int current_time )
{
  Process* p = unarrived.top();
  unarrived.pop();
  ready_fcfs.push(p);
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " arrived; added to ready queue ";
    print_queue(ready_fcfs);
  }
}

void OpSys::start_cpu_use_fcfs( int current_time )
{
  Process* p = switching_to_run;
  switching_to_run = NULL;
  running = p;
  p->waitBurst(current_time);
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " started using the CPU for " << p->getT() << "ms burst ";
    print_queue(ready_fcfs);
  }
}

void OpSys::switch_out_cpu_fcfs( int current_time )
{
  Process* p = running;
  running = NULL;
  p->update(current_time);
  int bursts_left = p->getCpuBurstsLeft();
  if (bursts_left == 0)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " terminated ";
    finished.push_back(p);
    unfinished.erase(p);
    print_queue(ready_fcfs);
  }
  else
  {
    p->waitBurst(current_time + t_cs/2);
    if (!TRUNCATE || current_time < TRUNC_TIME)
    {
      std::cout << "time " << current_time << "ms: Process " << p->id 
                << " completed a CPU burst; " << bursts_left 
                << " burst" << (bursts_left == 1 ? "" : "s") << " to go ";
      print_queue(ready_fcfs);
      std::cout << "time " << current_time << "ms: Process " << p->id 
                << " switching out of CPU; blocking on I/O until time " 
                << p->burstCompletionTime() << "ms ";
      print_queue(ready_fcfs);
    }
    waiting.push(p);
  }
  switching_to_io = p;
  p->last_switch_time = current_time;
}

void OpSys::complete_io_fcfs( int current_time )
{
  Process* p = waiting.top();
  waiting.pop();
  p->update(current_time);
  ready_fcfs.push(p);
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " completed I/O; added to ready queue ";
    print_queue(ready_fcfs);
  }
  p->finishBurst();
}

void OpSys::start_switch_in_fcfs( int current_time )
{
  switching_to_run = ready_fcfs.front();
  ready_fcfs.pop();
  switching_to_run->last_switch_time = current_time;
}

void OpSys::first_come_first_served()
{
  switching_to_run = NULL;
  switching_to_io = NULL;
  switching_to_ready = NULL;
  time = 0;
  std::cout << "time " << time << "ms: Simulator started for FCFS [Q empty]\n";
  while (!unfinished.empty())
  {
    std::priority_queue<Action, std::vector<Action>, CompAction> action_queue;

    if (switching_to_io != NULL)
      action_queue.push({ switching_to_io->last_switch_time + t_cs/2, &OpSys::finish_io_switch_out, 0 });
 
    if (running == NULL)
    {
      if (switching_to_run == NULL)
      {
        if (switching_to_io == NULL && !ready_fcfs.empty())
          action_queue.push({ time, &OpSys::start_switch_in_fcfs, 10 });
      }
      else
        action_queue.push({ switching_to_run->last_switch_time + t_cs/2, &OpSys::start_cpu_use_fcfs, 2 });
    }
    else
      action_queue.push({ running->burstCompletionTime(), &OpSys::switch_out_cpu_fcfs, 1 });

    if (!waiting.empty())
      action_queue.push({ waiting.top()->burstCompletionTime(), &OpSys::complete_io_fcfs, 3 });

    if (!unarrived.empty())
      action_queue.push({ unarrived.top()->arrival_time, &OpSys::process_arrive_fcfs, 4 });

    time = action_queue.top().time;
    (this->*(action_queue.top().func))(time);
  }
  time += t_cs/2;
  std::cout << "time " << time << "ms: Simulator ended for FCFS [Q empty]\n";    
  
  std::ofstream simout("simout.txt", std::ios_base::app);
  simout << "Algorithm FCFS\n";
  stats(simout);
  simout << "\n";
}

// ----------------------------------------------------------------------
//                   Round Robin (from rr.cpp)
// ----------------------------------------------------------------------
void OpSys::process_arrive_rr( int current_time )
{
  Process* p = unarrived.top();
  unarrived.pop();
  ready_rr.push(p);
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " arrived; added to ready queue ";
    print_queue(ready_rr);
  }
}

void OpSys::start_cpu_use_rr( int current_time )
{
  Process* p = switching_to_run;
  switching_to_run = NULL;
  running = p;
  p->last_cpu_burst_start = current_time;
  p->waitBurst(current_time);
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    if (p->time_remaining < p->getT())
      std::cout << "time " << current_time << "ms: Process " << p->id 
                << " started using the CPU for remaining " << p->time_remaining 
                << "ms of " << p->getT() << "ms burst ";
    else
      std::cout << "time " << current_time << "ms: Process " << p->id 
                << " started using the CPU for " << p->getT() << "ms burst ";
    print_queue(ready_rr);
  }
}

void OpSys::switch_out_cpu_rr( int current_time )
{
  Process* p = running;
  running = NULL;
  p->time_remaining = 0;
  p->update(current_time);
  int bursts_left = p->getCpuBurstsLeft();
  if (bursts_left == 0)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " terminated ";
    unfinished.erase(p);
    print_queue(ready_rr);
  }
  else
  {
    p->waitBurst(current_time + t_cs/2);
    if (!TRUNCATE || current_time < TRUNC_TIME)
    {
      std::cout << "time " << current_time << "ms: Process " << p->id 
                << " completed a CPU burst; " << bursts_left 
                << " burst" << (bursts_left == 1 ? "" : "s") << " to go ";
      print_queue(ready_rr);
      std::cout << "time " << current_time << "ms: Process " << p->id 
                << " switching out of CPU; blocking on I/O until time " 
                << p->burstCompletionTime() << "ms ";
      print_queue(ready_rr);
    }
    waiting.push(p);
  }
  switching_to_io = p;
  p->last_switch_time = current_time;
}

void OpSys::complete_io_rr( int current_time )
{
  Process* p = waiting.top();
  waiting.pop();
  p->update(current_time);
  ready_rr.push(p);
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " completed I/O; added to ready queue ";
    print_queue(ready_rr);
  }
  p->finishBurst();
}

void OpSys::ts_expiration_rr( int current_time )
{
  Process* p = running;
  if (ready_rr.empty())
  {
    p->time_remaining -= tslice;
    if (!TRUNCATE || current_time < TRUNC_TIME)
      std::cout << "time " << current_time 
                << "ms: Time slice expired; no preemption because ready queue is empty [Q empty]\n";
    p->last_cpu_burst_start = current_time;
    return;
  }

  p->preempt(tslice);
  running = NULL;
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    std::cout << "time " << current_time 
              << "ms: Time slice expired; preempting process " <<  p->id 
              << " with " << p->time_remaining << "ms remaining ";
    print_queue(ready_rr);
  }  
  switching_to_ready = p;
  p->last_switch_time = current_time;
}

void OpSys::start_switch_in_rr(int current_time)
{
  switching_to_run = ready_rr.front();
  ready_rr.pop();
  switching_to_run->last_switch_time = current_time;
}

void OpSys::finish_preempt_switch_out_rr(int current_time)
{
  ready_rr.push(switching_to_ready);
  switching_to_ready = NULL;
  if (current_time == 0) return;
}

void OpSys::round_robin()
{
  switching_to_run = NULL;
  switching_to_io = NULL;
  switching_to_ready = NULL;
  time = 0;
  std::cout << "time " << time << "ms: Simulator started for RR [Q empty]\n";
  while (!unfinished.empty())
  {
    std::priority_queue<Action, std::vector<Action>, CompAction> action_queue;

    if (switching_to_io != NULL)
      action_queue.push({ switching_to_io->last_switch_time + t_cs/2, &OpSys::finish_io_switch_out, 0 });
 
    if (running == NULL)
    {
      if (switching_to_ready != NULL)
        action_queue.push({ switching_to_ready->last_switch_time + t_cs/2, &OpSys::finish_preempt_switch_out_rr, 0 });
      else
      {
        if (switching_to_run == NULL)
        {
          if (switching_to_io == NULL && !ready_rr.empty())
            action_queue.push({ time, &OpSys::start_switch_in_rr, 10 });
        }
        else
          action_queue.push({ switching_to_run->last_switch_time + t_cs/2, &OpSys::start_cpu_use_rr, 2 });
      }
    }
    else
    {
      if ((running->last_cpu_burst_start + tslice) < running->burstCompletionTime())
        action_queue.push({ (running->last_cpu_burst_start + tslice), &OpSys::ts_expiration_rr, 1 });
      else
        action_queue.push({ running->burstCompletionTime(), &OpSys::switch_out_cpu_rr, 1 });
    }

    if (!waiting.empty())
      action_queue.push({ waiting.top()->burstCompletionTime(), &OpSys::complete_io_rr, 3 });

    if (!unarrived.empty())
      action_queue.push({ unarrived.top()->arrival_time, &OpSys::process_arrive_rr, 4 });

    time = action_queue.top().time;
    (this->*(action_queue.top().func))(time);
  }
  time += t_cs/2;
  std::cout << "time " << time << "ms: Simulator ended for RR [Q empty]\n";    
  
  std::ofstream simout("simout.txt", std::ios_base::app);
  simout << "Algorithm RR\n";
  stats(simout);
  stats_rr(simout);
}

// ----------------------------------------------------------------------
//                   Shortest Job First (from sjf.cpp)
// ----------------------------------------------------------------------
void OpSys::process_arrive_sjf( int current_time )
{
  Process* p = unarrived.top();
  unarrived.pop();
  ready_sjf.push(p);
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " (tau " << p->getTau() << "ms) arrived; added to ready queue ";
    print_priority_queue(ready_sjf);
  }
}

void OpSys::start_cpu_use_sjf( int current_time )
{
  Process* p = switching_to_run;
  switching_to_run = NULL;
  running = p;
  p->waitBurst(current_time);
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " (tau " << p->getTau() << "ms) started using the CPU for " 
              << p->getT() << "ms burst ";
    print_priority_queue(ready_sjf);
  }
}

void OpSys::switch_out_cpu_sjf( int current_time )
{
  Process* p = running;
  running = NULL;
  int old_tau = p->getTau();
  p->update(current_time);
  int bursts_left = p->getCpuBurstsLeft();
  if (bursts_left == 0)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id << " terminated ";
    unfinished.erase(p);
    print_priority_queue(ready_sjf);
  }
  else
  {
    p->waitBurst(current_time + t_cs/2);
    if (!TRUNCATE || current_time < TRUNC_TIME)
    {
      std::cout << "time " << current_time << "ms: Process " << p->id 
                << " (tau " << old_tau << "ms) completed a CPU burst; " << bursts_left 
                << " burst" << (bursts_left == 1 ? "" : "s") << " to go ";
      print_priority_queue(ready_sjf);
      std::cout << "time " << current_time << "ms: Recalculated tau for process " << p->id 
                << ": old tau " << old_tau << "ms ==> new tau " << p->getTau() << "ms ";
      print_priority_queue(ready_sjf);
      std::cout << "time " << current_time << "ms: Process " << p->id 
                << " switching out of CPU; blocking on I/O until time " 
                << p->burstCompletionTime() << "ms ";
      print_priority_queue(ready_sjf);
    }
    waiting.push(p);
  }
  switching_to_io = p;
  p->last_switch_time = current_time;
}

void OpSys::complete_io_sjf( int current_time )
{
  Process* p = waiting.top();
  waiting.pop();
  p->update(current_time);
  ready_sjf.push(p);
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " (tau " << p->getTau() << "ms) completed I/O; added to ready queue ";
    print_priority_queue(ready_sjf);
  }
  p->finishBurst();
}

void OpSys::start_switch_in_sjf(int current_time)
{
  switching_to_run = ready_sjf.top();
  ready_sjf.pop();
  switching_to_run->last_switch_time = current_time;
}

void OpSys::shortest_job_first()
{
  switching_to_run = NULL;
  switching_to_io = NULL;
  switching_to_ready = NULL;
  time = 0;
  std::cout << "time " << time << "ms: Simulator started for SJF [Q empty]\n";
  while (!unfinished.empty())
  {
    std::priority_queue<Action, std::vector<Action>, CompAction> action_queue;

    if (switching_to_io != NULL)
      action_queue.push({ switching_to_io->last_switch_time + t_cs/2, &OpSys::finish_io_switch_out, 0 });

    if (running == NULL)
    {
      if (switching_to_run == NULL)
      {
        if (switching_to_io == NULL && !ready_sjf.empty())
          action_queue.push({ time, &OpSys::start_switch_in_sjf, 10 });
      }
      else
        action_queue.push({ switching_to_run->last_switch_time + t_cs/2, &OpSys::start_cpu_use_sjf, 2 });
    }
    else
      action_queue.push({ running->burstCompletionTime(), &OpSys::switch_out_cpu_sjf, 1 });

    if (!waiting.empty())
      action_queue.push({ waiting.top()->burstCompletionTime(), &OpSys::complete_io_sjf, 3 });

    if (!unarrived.empty())
      action_queue.push({ unarrived.top()->arrival_time, &OpSys::process_arrive_sjf, 4 });

    time = action_queue.top().time;
    (this->*(action_queue.top().func))(time);
  }
  time += t_cs/2;
  std::cout << "time " << time << "ms: Simulator ended for SJF [Q empty]\n";

  std::ofstream simout("simout.txt", std::ios_base::app);
  simout << "Algorithm SJF\n";
  stats(simout);
  simout << "\n";  
}

// ----------------------------------------------------------------------
//                   Shortest Remaining Time (from srt.cpp)
// ----------------------------------------------------------------------
bool should_preempt(int current_time, Process* cur, Process* top)
{ 
  return (cur->tau_remaining - current_time + cur->last_cpu_burst_start) > top->tau_remaining;
}

void OpSys::process_arrive_srt( int current_time )
{
  Process* p = unarrived.top();
  unarrived.pop();
  ready_srt.push(p);
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " (tau " << p->getTau() << "ms) arrived; added to ready queue ";
    print_priority_queue(ready_srt);
  }
}

void OpSys::start_cpu_use_srt( int current_time )
{
  Process* p = switching_to_run;
  switching_to_run = NULL;
  running = p;
  p->last_cpu_burst_start = current_time;
  p->waitBurst(current_time);
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    if (p->time_remaining < p->getT())
      std::cout << "time " << current_time << "ms: Process " << p->id 
                << " (tau " << p->getTau() << "ms) started using the CPU for remaining " 
                << p->time_remaining << "ms of " << p->getT() << "ms burst ";
    else
      std::cout << "time " << current_time << "ms: Process " << p->id 
                << " (tau " << p->getTau() << "ms) started using the CPU for " 
                << p->getT() << "ms burst ";
    print_priority_queue(ready_srt);
  }
}

void OpSys::switch_out_cpu_srt( int current_time )
{
  Process* p = running;
  running = NULL;
  int old_tau = p->getTau();
  p->update(current_time);
  int bursts_left = p->getCpuBurstsLeft();
  if (bursts_left == 0)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id << " terminated ";
    unfinished.erase(p);
    print_priority_queue(ready_srt);
  }
  else
  {
    p->waitBurst(current_time + t_cs/2);
    if (!TRUNCATE || current_time < TRUNC_TIME)
    {
      std::cout << "time " << current_time << "ms: Process " << p->id 
                << " (tau " << old_tau << "ms) completed a CPU burst; " << bursts_left 
                << " burst" << (bursts_left == 1 ? "" : "s") << " to go ";
      print_priority_queue(ready_srt);
      std::cout << "time " << current_time << "ms: Recalculated tau for process " << p->id 
                << ": old tau " << old_tau << "ms ==> new tau " << p->getTau() << "ms ";
      print_priority_queue(ready_srt);
      std::cout << "time " << current_time << "ms: Process " << p->id 
                << " switching out of CPU; blocking on I/O until time " 
                << p->burstCompletionTime() << "ms ";
      print_priority_queue(ready_srt);
    }
    waiting.push(p);
  }
  switching_to_io = p;
  p->last_switch_time = current_time;
}

void OpSys::complete_io_srt( int current_time )
{
  Process* p = waiting.top();
  waiting.pop();
  p->update(current_time);
  ready_srt.push(p);
  if (running != NULL)
  {
    if (should_preempt(current_time, running, p))
    {
      running->preempt(current_time - running->last_cpu_burst_start);
      if (!TRUNCATE || current_time < TRUNC_TIME)
      {
        std::cout << "time " << current_time << "ms: Process " << p->id 
                  << " (tau " << p->getTau() << "ms) completed I/O; preempting " 
                  << running->id << " (predicted remaining time " << running->tau_remaining << "ms) ";
        print_priority_queue(ready_srt);
      }
      switching_to_ready = running;
      running->last_switch_time = current_time;
      running = NULL;
      p->finishBurst();
      return;
    }
  }
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " (tau " << p->getTau() << "ms) completed I/O; added to ready queue ";
    print_priority_queue(ready_srt);
  }
  p->finishBurst();
}

void OpSys::start_switch_in_srt(int current_time)
{
  switching_to_run = ready_srt.top();
  ready_srt.pop();
  switching_to_run->last_switch_time = current_time;
}

void OpSys::finish_preempt_switch_out_srt(int current_time)
{
  ready_srt.push(switching_to_ready);
  switching_to_ready = NULL;
  if (current_time == 0) return;
}

void OpSys::preempt_now_srt(int current_time)
{
  Process* p = ready_srt.top();
  running->preempt(current_time - running->last_cpu_burst_start);
  if (!TRUNCATE || current_time < TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id 
              << " (tau " << p->getTau() << "ms) will preempt " << running->id << " ";
    print_priority_queue(ready_srt);
  }
  switching_to_ready = running;
  running->last_switch_time = current_time;
  running = NULL;
}

void OpSys::shortest_remaining_time()
{
  switching_to_run = NULL;
  switching_to_io = NULL;
  switching_to_ready = NULL;
  time = 0;
  std::cout << "time " << time << "ms: Simulator started for SRT [Q empty]\n";
  while (!unfinished.empty())
  {
    std::priority_queue<Action, std::vector<Action>, CompAction> action_queue;

    if (switching_to_io != NULL)
      action_queue.push({ switching_to_io->last_switch_time + t_cs/2, &OpSys::finish_io_switch_out, 0 });
 
    if (running == NULL)
    {
      if (switching_to_ready != NULL)
        action_queue.push({ switching_to_ready->last_switch_time + t_cs/2, &OpSys::finish_preempt_switch_out_srt, 0 });
      else
      {
        if (switching_to_run == NULL)
        {
          if (switching_to_io == NULL && !ready_srt.empty())
            action_queue.push({ time, &OpSys::start_switch_in_srt, 10 });
        }
        else
          action_queue.push({ switching_to_run->last_switch_time + t_cs/2, &OpSys::start_cpu_use_srt, 2 });
      }
    }
    else
    {
      action_queue.push({ running->burstCompletionTime(), &OpSys::switch_out_cpu_srt, 1 });
      if (!ready_srt.empty() && should_preempt(time, running, ready_srt.top()))
        action_queue.push({ time, &OpSys::preempt_now_srt, -1 });
    }

    if (!waiting.empty())
      action_queue.push({ waiting.top()->burstCompletionTime(), &OpSys::complete_io_srt, 3 });

    if (!unarrived.empty())
      action_queue.push({ unarrived.top()->arrival_time, &OpSys::process_arrive_srt, 4 });

    time = action_queue.top().time;
    (this->*(action_queue.top().func))(time);
  }
  time += t_cs/2;
  std::cout << "time " << time << "ms: Simulator ended for SRT [Q empty]\n";   
  
  std::ofstream simout("simout.txt", std::ios_base::app);
  simout << "Algorithm SRT\n";
  stats(simout);
  simout << "\n";
}

// ----------------------------------------------------------------------
//                   Opsys (from opsys.cpp)
// ----------------------------------------------------------------------
void print_queue(const std::queue<Process*> &ready)
{
  std::queue<Process*> q = ready;
  std::cout << "[Q";
  if (!q.empty())
  {
    while (!q.empty())
    {
      std::cout << " " << q.front()->id;
      q.pop();
    }
  }
  else
    std::cout << " empty";
  std::cout << "]\n";
}

void OpSys::stats(std::ofstream& simout)
{
  int switches_cpu = 0, switches_io = 0;
  int preempts_cpu = 0, preempts_io = 0;
  int overall_cpu_time = 0;
  int cpu_turn = 0, io_turn = 0; 
  int cpu_cpu_bursts = 0, io_cpu_bursts = 0;
  int total_cpu_wait = 0, total_io_wait = 0;
  
  for (Process* p : finished)
  {
    overall_cpu_time += p->total_cpu_time;
    if (p->is_cpu_bound)
    {
      switches_cpu += p->num_switches;
      preempts_cpu += p->num_preempts;
      cpu_turn += p->total_turnaround;
      cpu_cpu_bursts += p->num_cpu_bursts;
      total_cpu_wait += p->total_turnaround - p->getTotalCpuTime() - (p->num_preempts * t_cs);
    }
    else
    {
      switches_io += p->num_switches;
      preempts_io += p->num_preempts;
      io_turn += p->total_turnaround;
      io_cpu_bursts += p->num_cpu_bursts;
      total_io_wait += p->total_turnaround - p->getTotalCpuTime() - (p->num_preempts * t_cs);
    }  
  }
  double utilization = std::ceil((overall_cpu_time / (double)time) * 100000) / 1000;
  double avg_cpu_turn = std::ceil(((double)cpu_turn / cpu_cpu_bursts) * 1000) / 1000 + (t_cs/2);
  double avg_io_turn = std::ceil(((double)io_turn / io_cpu_bursts) * 1000) / 1000 + (t_cs/2);
  double avg_turn = std::ceil(((double)(io_turn+cpu_turn) / (io_cpu_bursts+cpu_cpu_bursts)) * 1000) / 1000 + (t_cs/2);
  double avg_wait_cpu = std::ceil(((double)total_cpu_wait / cpu_cpu_bursts) * 1000) / 1000 - (t_cs/2);
  double avg_wait_io = std::ceil(((double)total_io_wait / io_cpu_bursts) * 1000) / 1000 - (t_cs/2);
  double avg_wait = std::ceil(((double)(total_cpu_wait+total_io_wait) / (io_cpu_bursts+cpu_cpu_bursts)) * 1000) / 1000 - (t_cs/2);
 
  simout << "-- CPU utilization: " << std::fixed << std::setprecision(3) << utilization << "%\n";
  simout << "-- CPU-bound average wait time: " << std::setprecision(3) << avg_wait_cpu << " ms\n";
  simout << "-- I/O-bound average wait time: " << std::setprecision(3) << avg_wait_io << " ms\n";
  simout << "-- overall average wait time: " << std::setprecision(3) << avg_wait << " ms\n";  
  simout << "-- CPU-bound average turnaround time: " << std::setprecision(3) << avg_cpu_turn << " ms\n";
  simout << "-- I/O-bound average turnaround time: " << std::setprecision(3) << avg_io_turn << " ms\n";
  simout << "-- overall average turnaround time: " << avg_turn << " ms\n";
  simout << "-- CPU-bound number of context switches: " << switches_cpu << "\n";
  simout << "-- I/O-bound number of context switches: " << switches_io << "\n";
  simout << "-- overall number of context switches: " << switches_io + switches_cpu << "\n";
  simout << "-- CPU-bound number of preemptions: " << preempts_cpu << "\n";
  simout << "-- I/O-bound number of preemptions: " << preempts_io << "\n";
  simout << "-- overall number of preemptions: " << preempts_io + preempts_cpu << "\n";
}

void OpSys::stats_rr(std::ofstream& simout)
{
  int cpu_within_ts = 0, io_within_ts = 0;
  int cpu_cpu_bursts = 0, io_cpu_bursts = 0;

  for (Process* p : finished)
  {
    int bursts_within = 0;
    for (int i = 0; i < p->num_total_bursts; i += 2)
    {
      if (p->burst_times[i] <= tslice) bursts_within++;
    }
    if (p->is_cpu_bound)
    { 
      cpu_within_ts += bursts_within; 
      cpu_cpu_bursts += p->num_cpu_bursts; 
    }
    else
    { 
      io_within_ts += bursts_within; 
      io_cpu_bursts += p->num_cpu_bursts;
    }
  }  

  double cpu = std::ceil(((double)cpu_within_ts / cpu_cpu_bursts) * 100000) / 1000;
  double io = std::ceil(((double)io_within_ts / io_cpu_bursts) * 100000) / 1000;
  double all = std::ceil(((double)(io_within_ts + cpu_within_ts) / (io_cpu_bursts + cpu_cpu_bursts)) * 100000) / 1000;

  simout << "-- CPU-bound percentage of CPU bursts completed within one time slice: " << std::setprecision(3) << cpu << "%\n";
  simout << "-- I/O-bound percentage of CPU bursts completed within one time slice: " << std::setprecision(3) << io << "%\n";
  simout << "-- overall percentage of CPU bursts completed within one time slice: " << std::setprecision(3) << all << "%\n";
}

// ----------------------------------------------------------------------
//                   Process (from process.cpp)
// ----------------------------------------------------------------------
void Process::update( int current_time ) 
{
  burst_index++;
  
  if (onCPUBurst())
  {
    prev_t = t;
    t = burst_times[burst_index];
    time_remaining = t;
    total_cpu_time += t;
    num_switches++;
    start_turnaround = current_time;
  }
  else // on I/O burst
  {
    prev_tau = tau;
    tau = std::ceil((alpha * burst_times[burst_index-1]) + ((1.0 - alpha) * prev_tau));
    tau_remaining = tau;
    if (burst_index < num_total_bursts)
      time_remaining = burst_times[burst_index];
    total_turnaround += (current_time - start_turnaround);
  }
}

void Process::preempt( int elapsed_time )
{
  num_preempts++;
  num_switches++;  
  time_remaining -= elapsed_time;
  tau_remaining -= elapsed_time;
}

void Process::reset()
{
  burst_index = 0;
  burst_completion_time = 0;
  t = burst_times[0];
  prev_t = t;
  tau = tau_0;
  prev_tau = tau;
  time_remaining = t;
  tau_remaining = tau;
  last_cpu_burst_start = 0;
  num_switches = 1;
  num_preempts = 0;
  total_cpu_time = t;
  start_turnaround = arrival_time;
  total_turnaround = 0;
}

int Process::getTotalCpuTime()
{
  int total_cpu = 0;
  for (int i = 0; i < num_total_bursts; i += 2)
    total_cpu += burst_times[i];
  return total_cpu;
}

// ----------------------------------------------------------------------
//                   Main (from main.cpp)
// ----------------------------------------------------------------------
int n;
int n_cpu;
int seed;
double lambda;
int ceiling;
int tcs;
double alpha;
int tslice;

double next_exp()
{
    int found = 0;
    double x;
    while (!found)
    {
        double r = drand48();
        x = -log(r) / lambda;
        if (x <= ceiling)
            found = 1;
    }
    return x;
}

int main(int argc, char* argv[])
{
    if (argc != 9)
    {
        std::cerr << "ERROR: Invalid arg count\n";
        exit(1);
    }

    n = atoi(argv[1]);          
    n_cpu = atoi(argv[2]);      
    seed = atoi(argv[3]);       
    lambda = atof(argv[4]);     
    ceiling = atoi(argv[5]);
    tcs = atoi(argv[6]);                 
    alpha = atof(argv[7]);
    tslice = atoi(argv[8]);

    if (n <= 0 || n > 260)
    {
        std::cerr << "ERROR: Invalid process simulation count\n";
        exit(1);
    }
    if (lambda == 0)
    {
        std::cerr << "ERROR: Lambda cannot be zero\n";
        exit(1);
    }
    if (tslice <= 0 || tcs <= 0)
    {
        std::cerr << "ERROR: Time values must be positive\n";
        exit(1);
    }

    std::cout << "<<< -- process set (n=" << argv[1] << ") with " << argv[2]
              << (n_cpu != 1 ? " CPU-bound processes\n" : " CPU-bound process\n")
              << "<<< -- seed=" << argv[3] << "; lambda=" << std::fixed << std::setprecision(6) 
              << lambda << "; bound=" << argv[5] << "\n";

    srand48(seed);

    std::list<Process*> processes;
    float cpu_cpu_total = 0, io_cpu_total = 0, cpu_io_total = 0, io_io_total = 0;
    float num_cpu_cpu = 0, num_io_cpu = 0, num_cpu_io = 0, num_io_io = 0;

    for (int i = 0; i < n; i++)
    {
        bool is_cpu_bound = (i < n_cpu);
        int p_arrival_time = floor(next_exp());
        int num_cpu = std::ceil(drand48() * 32);
        int num_io = num_cpu - 1;
        int num_total = num_cpu + num_io;
        int* burst_times = new int[num_total];

        for (int j = 0; j < num_total; j++)
        {
            int x = std::ceil(next_exp());
            if (is_cpu_bound)
            {
                if (j % 2 == 0)
                {
                    x = x * 4;
                    cpu_cpu_total += x; num_cpu_cpu++;
                }
                else
                {
                    cpu_io_total += x; num_cpu_io++;
                }
            }
            else
            {
                if (j % 2 == 1)
                {
                    x *= 8;
                    io_io_total += x; num_io_io++;
                }
                else
                {
                    io_cpu_total += x; num_io_cpu++;
                }
            }
            burst_times[j] = x;
        }

        Process* p = new Process();
        p->id = new char[4];
        std::snprintf(p->id, 4, "%c%d", 'A' + (i / 10), i % 10);
        p->tau_0 = std::ceil(1.0 / lambda);
        p->tau = p->tau_0;
        p->alpha = alpha;
        p->t = burst_times[0];
        p->is_cpu_bound = is_cpu_bound;
        p->num_cpu_bursts = num_cpu;
        p->num_total_bursts = num_total;
        p->burst_times = burst_times;
        p->arrival_time = p_arrival_time;
        p->start_turnaround = p_arrival_time;
        p->time_remaining = p->t;
        p->total_cpu_time = p->t;
        p->tau_remaining = p->tau;

        processes.push_back(p);
    }

    float cpu_cpu_avg = (num_cpu_cpu != 0) ? (cpu_cpu_total / num_cpu_cpu) : 0;
    float cpu_io_avg = (num_cpu_io != 0) ? (cpu_io_total / num_cpu_io) : 0;
    float cpu_avg = ((num_cpu_cpu + num_io_cpu) != 0) ? std::ceil(1000 * (cpu_cpu_total + io_cpu_total) / (num_cpu_cpu + num_io_cpu)) / 1000 : 0;
    float io_cpu_avg = (num_io_cpu != 0) ? (io_cpu_total / num_io_cpu) : 0;
    float io_io_avg = (num_io_io != 0) ? (io_io_total / num_io_io) : 0;
    float io_avg = ((num_cpu_io + num_io_io) != 0) ? std::ceil(1000 * (cpu_io_total + io_io_total) / (num_cpu_io + num_io_io)) / 1000 : 0;

    for (Process* p : processes)
    {
        if (p->is_cpu_bound)
            std::cout << "\nCPU-bound";
        else
            std::cout << "\nI/O-bound";
        std::cout << " process " << p->id << ": arrival time " << p->arrival_time 
                  << "ms; " << p->num_cpu_bursts 
                  << (p->num_cpu_bursts != 1 ? " CPU bursts\n" : " CPU burst\n");
        for (int j = 0; j < p->num_cpu_bursts * 2 - 1; ++j)
        {
            if (j % 2 == 0)
                std::cout << "==> CPU burst " << p->burst_times[j] << "ms";
            else
                std::cout << " ==> I/O burst " << p->burst_times[j] << "ms\n";
        }
        std::cout << "\n";
    }

    std::ofstream simout("simout.txt");
    simout << "-- number of processes: " << n << "\n";
    simout << "-- number of CPU-bound processes: " << n_cpu << "\n";
    simout << "-- number of I/O-bound processes: " << (n - n_cpu) << "\n";
    simout << "-- CPU-bound average CPU burst time: " 
           << std::fixed << std::setprecision(3) << std::ceil(1000 * cpu_cpu_avg) / 1000 << " ms\n";
    simout << "-- I/O-bound average CPU burst time: " 
           << std::fixed << std::setprecision(3) << std::ceil(1000 * io_cpu_avg) / 1000 << " ms\n";
    simout << "-- overall average CPU burst time: " 
           << std::fixed << std::setprecision(3) << cpu_avg << " ms\n";
    simout << "-- CPU-bound average I/O burst time: " 
           << std::fixed << std::setprecision(3) << std::ceil(1000 * cpu_io_avg) / 1000 << " ms\n";
    simout << "-- I/O-bound average I/O burst time: " 
           << std::fixed << std::setprecision(3) << std::ceil(1000 * io_io_avg) / 1000 << " ms\n";
    simout << "-- overall average I/O burst time: " 
           << std::fixed << std::setprecision(3) << io_avg << " ms\n\n";
    simout.close();

    std::cout << "<<< PROJECT SIMULATIONS\n<<< -- t_cs=" << tcs 
              << "ms; alpha=" << std::fixed << std::setprecision(2) << alpha 
              << "; t_slice=" << tslice << "ms\n";
    OpSys* simulation = new OpSys();
    simulation->t_cs = tcs;
    simulation->tslice = tslice;
    for (Process* p : processes)
      simulation->unfinished.insert(p); 
    for (Process* p : processes)
      simulation->unarrived.push(p); 
    simulation->first_come_first_served();

    std::cout << "\n";
    for (Process* p : processes)
    {
      p->reset();
      simulation->unfinished.insert(p);
      simulation->unarrived.push(p);
    }
    simulation->shortest_job_first();
    std::cout << "\n";
    for (Process* p : processes)
    {
      p->reset();
      simulation->unfinished.insert(p);
      simulation->unarrived.push(p);
    }
    simulation->shortest_remaining_time();
    std::cout << "\n";
    for (Process* p : processes)
    {
      p->reset();
      simulation->unfinished.insert(p);
      simulation->unarrived.push(p);
    }
    simulation->round_robin();

    delete simulation;
    for (Process* p : processes)
    {
      delete [] p->id;
      delete [] p->burst_times;
      delete p;
    }

    return 0;
}
