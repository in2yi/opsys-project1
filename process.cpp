#include "process.h"

void Process::update( int current_time ) 
{
  this->burst_index++;
  
	if (onCPUBurst()) 
	{
		this->prev_t = t;
		this->t = burst_times[burst_index];
    this->time_remaining = t;
		this->total_cpu_time += t;
		this->num_switches++;	
		this->start_turnaround = current_time;
	}
	else //onIOBurst
	{
		this->prev_tau = tau;
		this->tau = std::ceil((alpha * burst_times[burst_index-1]) + ((1.0-alpha) * prev_tau));
	  this->tau_remaining = tau;
    if (burst_index < num_total_bursts) this->time_remaining = burst_times[burst_index];
		this->total_turnaround += (current_time - start_turnaround);
  }
}

void Process::preempt( int elapsed_time )
{
	this->num_preempts++;
	this->num_switches++;	
	this->time_remaining = this->time_remaining - elapsed_time;
	// burst_times[burst_index] = time_remaining;
	// this->t = time_remaining;
	this->tau_remaining -= elapsed_time;
} 

void Process::reset()
{
	this->burst_index = 0;
	this->burst_completion_time = 0;
	this->t = burst_times[0];
	this->prev_t = t;
	this->tau = tau_0;
	this->prev_tau = tau;
	this->time_remaining = t;
  this->tau_remaining = tau;
  this->last_cpu_burst_start = 0;
	this->num_switches = 1;
  this->num_preempts = 0;
	this->total_cpu_time = t;
	this->start_turnaround = arrival_time;
	this->total_turnaround = 0;
}

int Process::getTotalCpuTime()
{
	int total_cpu = 0;
	for (int i = 0; i < num_total_bursts; i += 2)
	{
		total_cpu += burst_times[i];
	}
	return total_cpu;
}
