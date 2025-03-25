#include "opsys.h"

void OpSys::process_arrive_fcfs( int current_time )
{
  Process* p = unarrived.top();
  unarrived.pop();
  ready_fcfs.push(p);
  if (!TRUNCATE || current_time<TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id << " arrived; added to ready queue ";
    print_queue(ready_fcfs);
  }
}

void OpSys::start_cpu_use_fcfs( int current_time )
{
  Process* p = switching_to_run;
  this->switching_to_run = NULL;
  this->running = p;
  p->waitBurst(current_time);
  if (!TRUNCATE || current_time<TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id << " started using the CPU for " << p->getT() << "ms burst ";
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
    std::cout << "time " << current_time << "ms: Process " << p->id << " terminated ";
    finished.push_back(p);
    unfinished.erase(p);
    print_queue(ready_fcfs);
  } else
  {
    p->waitBurst(current_time+t_cs/2);
    if (!TRUNCATE || current_time<TRUNC_TIME)
    {
      std::cout << "time " << current_time << "ms: Process " << p->id << " completed a CPU burst; " << bursts_left << " burst" << (bursts_left == 1 ? "" : "s")  << " to go ";
      print_queue(ready_fcfs);
      /* maybe move below to another function? */
      std::cout << "time " << current_time << "ms: Process " << p->id << " switching out of CPU; blocking on I/O until time " << p->burstCompletionTime() << "ms ";
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
  if (!TRUNCATE || current_time<TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id << " completed I/O; added to ready queue ";
    print_queue(ready_fcfs);
  }
  p->finishBurst();
}

void OpSys::start_switch_in_fcfs( int current_time)
{
  switching_to_run = ready_fcfs.front();
  ready_fcfs.pop();
  switching_to_run->last_switch_time=current_time;
}

void OpSys::first_come_first_served()
{
  switching_to_run = NULL;
  switching_to_io = NULL;
  switching_to_ready = NULL;
  this->time = 0;
  std::cout << "time " << this->time << "ms: Simulator started for FCFS [Q empty]\n";
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
      if (switching_to_run == NULL)
      {
        if (switching_to_io == NULL && !ready_fcfs.empty()) action_queue.push( { this->time, &OpSys::start_switch_in_fcfs, 10 } );
      } else
      {
        action_queue.push( { switching_to_run->last_switch_time+t_cs/2, &OpSys::start_cpu_use_fcfs, 2 } );
      }
    } else
    
    /* See if running process is done. */
    {
      action_queue.push( { running->burstCompletionTime(), &OpSys::switch_out_cpu_fcfs, 1 } );
    }

    /* Check if soonest IO burst is done. */
    if (!waiting.empty())
    {
      action_queue.push( { waiting.top()->burstCompletionTime(), &OpSys::complete_io_fcfs, 3 } );
    }

    /* Check for incoming processes. */
    if (!unarrived.empty())
    {
      action_queue.push( {unarrived.top()->arrival_time, &OpSys::process_arrive_fcfs, 4 } );
    }

    this->time = action_queue.top().time;
    (this->*(action_queue.top().func))(this->time);

  }
  time += t_cs/2;
  std::cout << "time " << (this->time) << "ms: Simulator ended for FCFS [Q empty]\n";    
  
  std::ofstream simout;
  simout.open("simout.txt", std::ios_base::app);
  simout << "Algorithm FCFS\n";
  stats(simout);
  simout << "\n";
}
