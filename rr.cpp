#include "opsys.h"

void OpSys::process_arrive_rr( int current_time )
{
  Process* p = unarrived.top();
  unarrived.pop();
  ready_rr.push(p);
  if (!TRUNCATE || current_time<TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id << " arrived; added to ready queue ";
    print_queue(ready_rr);
  }
}

void OpSys::start_cpu_use_rr( int current_time )
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
      std::cout << "time " << current_time << "ms: Process " << p->id << " started using the CPU for remaining " << p->time_remaining << "ms of " << p->getT() << "ms burst ";
    } else 
    {
      std::cout << "time " << current_time << "ms: Process " << p->id << " started using the CPU for " << p->getT() << "ms burst ";
    }
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
    std::cout << "time " << current_time << "ms: Process " << p->id << " terminated ";
    unfinished.erase(p);
    print_queue(ready_rr);
  } else
  {
    p->waitBurst(current_time+t_cs/2);
    if (!TRUNCATE || current_time<TRUNC_TIME)
    {
      std::cout << "time " << current_time << "ms: Process " << p->id << " completed a CPU burst; " << bursts_left << " burst" << (bursts_left == 1 ? "" : "s")  << " to go ";
      print_queue(ready_rr);
      /* maybe move below to another function? */
      std::cout << "time " << current_time << "ms: Process " << p->id << " switching out of CPU; blocking on I/O until time " << p->burstCompletionTime() << "ms ";
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
  if (!TRUNCATE || current_time<TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Process " << p->id << " completed I/O; added to ready queue ";
    print_queue(ready_rr);
  }
  p->finishBurst();
}


void OpSys::ts_expiration_rr( int current_time)
{
	Process* p = running;
	/* If ready queue is empty, there is no preemption */
	if (ready_rr.empty())
	{
		p->time_remaining -= tslice;
    if (!TRUNCATE || current_time<TRUNC_TIME)
      std::cout << "time " << current_time << "ms: Time slice expired; no preemption because ready queue is empty [Q empty]\n";
		p->last_cpu_burst_start = current_time;
		return;
	}

	p->preempt(tslice);
	running = NULL;
	if (!TRUNCATE || current_time<TRUNC_TIME)
  {
    std::cout << "time " << current_time << "ms: Time slice expired; preempting process " <<  p->id << " with " << p->time_remaining << "ms remaining ";
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
  this->time = 0;
  std::cout << "time " << this->time << "ms: Simulator started for RR [Q empty]\n";
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
        action_queue.push( { switching_to_ready->last_switch_time+t_cs/2, &OpSys::finish_preempt_switch_out_rr, 0 } );
      } else
      {
        if (switching_to_run == NULL)
        {
          if (switching_to_io == NULL && !ready_rr.empty()) action_queue.push( { this->time, &OpSys::start_switch_in_rr, 10 } );
        } else
        {
          action_queue.push( { switching_to_run->last_switch_time+t_cs/2, &OpSys::start_cpu_use_rr, 2 } );
        }
      }
    } else
    
    /* See if running process is done OR check for preemption. */
    {
      if ((running->last_cpu_burst_start + tslice) < running->burstCompletionTime() )
			{
				action_queue.push( { (running->last_cpu_burst_start + tslice), &OpSys::ts_expiration_rr, 1 } );
			} else
      {
        action_queue.push( { running->burstCompletionTime(), &OpSys::switch_out_cpu_rr, 1 } );
      }
    }

    /* Check if soonest IO burst is done. */
    if (!waiting.empty())
    {
      action_queue.push( { waiting.top()->burstCompletionTime(), &OpSys::complete_io_rr, 3 } );
    }

    /* Check for incoming processes. */
    if (!unarrived.empty())
    {
      action_queue.push( {unarrived.top()->arrival_time, &OpSys::process_arrive_rr, 4 } );
    }

    this->time = action_queue.top().time;
    (this->*(action_queue.top().func))(this->time);
  }
  time += t_cs/2;
  std::cout << "time " << (this->time) << "ms: Simulator ended for RR [Q empty]\n";    
  
  std::ofstream simout;
  simout.open("simout.txt", std::ios_base::app);
  simout << "Algorithm RR\n";
  stats(simout);
  stats_rr(simout);
  //simout << "\n";
}
