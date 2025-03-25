#include "opsys.h"

bool should_preempt(int current_time, Process* cur, Process* top )
{ 
  return (cur->tau_remaining - current_time + cur->last_cpu_burst_start) > top->tau_remaining;
}

void OpSys::process_arrive_srt( int current_time )
{
  Process* p = unarrived.top();
  unarrived.pop();
  ready_srt.push(p);
  if (!TRUNCATE || current_time<TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id << " (tau " << p->getTau() << "ms) arrived; added to ready queue ";
    print_priority_queue(ready_srt);
  }
}

void OpSys::start_cpu_use_srt( int current_time )
{
  Process* p = switching_to_run;
  this->switching_to_run = NULL;
  this->running = p;
  p->last_cpu_burst_start = current_time;
  p->waitBurst(current_time);
  if (!TRUNCATE || current_time<TRUNC_TIME)
  {
    if (p->time_remaining < p->getT())
    {
      std::cout << "time " << current_time << "ms: Process " << p->id << " (tau " << p->getTau() << "ms) started using the CPU for remaining " << p->time_remaining << "ms of " << p->getT() << "ms burst ";
    } else
    {
      std::cout << "time " << current_time << "ms: Process " << p->id << " (tau " << p->getTau() << "ms) started using the CPU for " << p->getT() << "ms burst ";
    }
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
  } else
  {
    p->waitBurst(current_time+t_cs/2);
    if (!TRUNCATE || current_time<TRUNC_TIME)
    {
      std::cout << "time " << current_time << "ms: Process " << p->id << " (tau " << old_tau << "ms) completed a CPU burst; " << bursts_left << " burst" << (bursts_left == 1 ? "" : "s")  << " to go ";
      print_priority_queue(ready_srt);
      /* maybe move below to another function? */
      std::cout << "time " << current_time << "ms: Recalculated tau for process " << p->id << ": old tau " << old_tau << "ms ==> new tau " << p->getTau() << "ms ";
      print_priority_queue(ready_srt);
      std::cout << "time " << current_time << "ms: Process " << p->id << " switching out of CPU; blocking on I/O until time " << p->burstCompletionTime() << "ms ";
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
      if (!TRUNCATE || current_time<TRUNC_TIME)
      {
        std::cout << "time " << current_time << "ms: Process " << p->id << " (tau " << p->getTau() << "ms) completed I/O; preempting " << running->id << " (predicted remaining time " << running->tau_remaining << "ms) ";
        print_priority_queue(ready_srt);
      }
      switching_to_ready = running;
      running->last_switch_time = current_time;
      running = NULL;
      p->finishBurst();
      return;
    }
  }
  if (!TRUNCATE || current_time<TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id << " (tau " << p->getTau() << "ms) completed I/O; added to ready queue ";
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
  if (!TRUNCATE || current_time<TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id << " (tau " << p->getTau() << "ms) will preempt " << running->id << " ";
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
  this->time = 0;
  std::cout << "time " << this->time << "ms: Simulator started for SRT [Q empty]\n";
  while (!this->unfinished.empty())
  {
    /* Search for next 'interesting' event. */
    std::priority_queue<Action, std::vector<Action>, CompAction> action_queue;

    if (switching_to_io != NULL)
    {
      action_queue.push( { switching_to_io->last_switch_time+t_cs/2, &OpSys::finish_io_switch_out, 0 } );
    }
 
    /* See if CPU isn't doing anything. */
    if (running == NULL)
    {
      if (switching_to_ready != NULL)
      {
        action_queue.push( { switching_to_ready->last_switch_time+t_cs/2, &OpSys::finish_preempt_switch_out_srt, 0 } );
      } else
      {
        if (switching_to_run == NULL)
        {
          if (switching_to_io == NULL && !ready_srt.empty()) action_queue.push( { this->time, &OpSys::start_switch_in_srt, 10 } );
        } else
        {
          action_queue.push( { switching_to_run->last_switch_time+t_cs/2, &OpSys::start_cpu_use_srt, 2 } );
        }
      }
    } else
    
    /* See if running process is done. */
    {
      action_queue.push( { running->burstCompletionTime(), &OpSys::switch_out_cpu_srt, 1 } );
      if (!ready_srt.empty() && should_preempt(this->time, running, ready_srt.top()))
      {
        action_queue.push( { this->time, &OpSys::preempt_now_srt, -1 } );
      }
    }

    /* Check if soonest IO burst is done. */
    if (!waiting.empty())
    {
      action_queue.push( { waiting.top()->burstCompletionTime(), &OpSys::complete_io_srt, 3 } );
    }

    /* Check for incoming processes. */
    if (!unarrived.empty())
    {
      action_queue.push( {unarrived.top()->arrival_time, &OpSys::process_arrive_srt, 4 } );
    }

    this->time = action_queue.top().time;
    (this->*(action_queue.top().func))(this->time);

  }
  time += t_cs/2;
  std::cout << "time " << (this->time) << "ms: Simulator ended for SRT [Q empty]\n";   
  
  std::ofstream simout;
  simout.open("simout.txt", std::ios_base::app);
  simout << "Algorithm SRT\n";
  stats(simout);
  simout << "\n";
}
