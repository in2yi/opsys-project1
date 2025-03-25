#include "opsys.h"

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
  } else
  {
    std::cout << " empty";
  }
  std::cout << "]\n";
}

void OpSys::stats(std::ofstream& simout)
{
  int switches_cpu = 0; int switches_io = 0;
  int preempts_cpu = 0; int preempts_io = 0;
  int overall_cpu_time = 0;
  int cpu_turn = 0; int io_turn = 0; 
  int cpu_cpu_bursts = 0; int io_cpu_bursts = 0;
  int total_cpu_wait = 0; int total_io_wait = 0;
  
  
  for (Process* p : this->finished)
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
    else  // is_io_bound
    {
      switches_io += p->num_switches;
      preempts_io += p->num_preempts;
      io_turn += p->total_turnaround;
      io_cpu_bursts += p->num_cpu_bursts;
      total_io_wait += p->total_turnaround - p->getTotalCpuTime() - (p->num_preempts * t_cs);
    }  
  }
  double utilization = std::ceil((overall_cpu_time / (double)this->time) * 100000) / 1000;
  double avg_cpu_turn = std::ceil(((double)cpu_turn / (double)cpu_cpu_bursts) * 1000) / 1000 + (t_cs/2);
  double avg_io_turn = std::ceil(((double)io_turn / (double)io_cpu_bursts) * 1000) / 1000 + (t_cs/2);
  double avg_turn = std::ceil(((double)(io_turn+cpu_turn)/ (double)(io_cpu_bursts+cpu_cpu_bursts)) * 1000) / 1000 + (t_cs/2);
  double avg_wait_cpu = std::ceil(((double)total_cpu_wait / (double)cpu_cpu_bursts) * 1000) / 1000 - (t_cs/2);
  double avg_wait_io = std::ceil(((double)total_io_wait / (double)io_cpu_bursts) * 1000) / 1000 - (t_cs/2);
  double avg_wait = std::ceil(((double)(total_cpu_wait+total_io_wait) / (double)(io_cpu_bursts+cpu_cpu_bursts)) * 1000) / 1000 - (t_cs/2);
 
  simout << "-- CPU utilization: " << std::fixed << std::setprecision(3) <<  utilization << "%\n";

  simout << "-- CPU-bound average wait time: " << std::setprecision(3) << avg_wait_cpu << " ms\n";
  simout << "-- I/O-bound average wait time: " << std::setprecision(3) << avg_wait_io << " ms\n";
  simout << "-- overall average wait time: " << std::setprecision(3) << avg_wait << " ms\n";  

  simout <<"-- CPU-bound average turnaround time: " << std::setprecision(3) << avg_cpu_turn << " ms\n";
  simout <<"-- I/O-bound average turnaround time: " << std::setprecision(3) << avg_io_turn << " ms\n";
  simout <<"-- overall average turnaround time: " << avg_turn << " ms\n";

  simout << "-- CPU-bound number of context switches: " << switches_cpu << "\n";
  simout << "-- I/O-bound number of context switches: " << switches_io << "\n";
  simout << "-- overall number of context switches: " << switches_io + switches_cpu << "\n";

  simout << "-- CPU-bound number of preemptions: " << preempts_cpu << "\n";
  simout << "-- I/O-bound number of preemptions: " << preempts_io << "\n";
  simout << "-- overall number of preemptions: " << preempts_io + preempts_cpu << "\n";
}

void OpSys::stats_rr(std::ofstream& simout)
{
  int cpu_within_ts = 0; int io_within_ts = 0;
  int cpu_cpu_bursts = 0; int io_cpu_bursts = 0;


  for (Process* p : this->finished)
  {
    int bursts_within = 0;
    for (int i = 0; i < p->num_total_bursts; i += 2)
    {
      if (p->burst_times[i] <= this->tslice) bursts_within++;
    }

    if (p->is_cpu_bound) 
    { 
      cpu_within_ts += bursts_within; 
      cpu_cpu_bursts += p->num_cpu_bursts; 
    }
    else 
    { 
      io_within_ts += bursts_within; 
      io_cpu_bursts += p->num_cpu_bursts; ;
    }
  }  

double cpu = std::ceil(((double)cpu_within_ts / (double)cpu_cpu_bursts) * 100000) / 1000;
double io = std::ceil(((double)io_within_ts / (double)io_cpu_bursts) * 100000) / 1000;
double all = std::ceil(((double)(io_within_ts+cpu_within_ts) / (double)(io_cpu_bursts+cpu_cpu_bursts)) * 100000) / 1000;

simout << "-- CPU-bound percentage of CPU bursts completed within one time slice: " << std::setprecision(3) << cpu << "%\n";
simout << "-- I/O-bound percentage of CPU bursts completed within one time slice: " << std::setprecision(3) << io << "%\n";
simout << "-- overall percentage of CPU bursts completed within one time slice: " << std::setprecision(3) << all << "%\n";

}
