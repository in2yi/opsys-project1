#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <list>
#include <iomanip>
#include "opsys.h"

int n;
int n_cpu;
int seed;
double lambda;
int ceiling;
int tcs;
double alpha;
int tslice;

double nextExp()
{
    while (true)
    {
        double x = -log(drand48()) / lambda;
        if (x < ceiling) return x;
    }
}

int main(int argc, char* argv[])
{
    if (argc != 9) 
    {
		  std::cerr << "ERROR: Invalid arg count\n";
		  exit(1);
	  }

    n = atoi(*(argv+1));          /* # of processes to simulate */
    n_cpu = atoi(*(argv+2));      /* # of CPU-bound processes */
    seed = atoi(*(argv+3));       /* Rand num gen seed */
    lambda = atof(*(argv+4));     
    ceiling = atoi(*(argv+5));
    tcs = atoi(*(argv+6));                 /* Time (ms) to perform a context switch */
    alpha = atof(*(argv+7));
    tslice = atoi(*(argv+8));

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

    std::cout << "<<< -- process set (n=" << *(argv+1) << ") with " << *(argv+2)
              << (n_cpu != 1 ? "CPU-bound processes\n" : " CPU-bound process\n") << "<<< -- seed=" << *(argv+3) << "; "
              << "lambda=" << std::fixed << std::setprecision(6) << lambda << "; " 
              << "bound=" << *(argv+5) << "\n";

    srand48( seed );    // Seed rand num generator

    std::list<Process*> processes;

    float cpu_cpu_total = 0, io_cpu_total = 0, cpu_io_total = 0, io_io_total = 0;
    float num_cpu_cpu = 0, num_io_cpu = 0, num_cpu_io = 0, num_io_io = 0;

    
    // Process creation loop
    for (int i = 0; i < n; i++)
    {
        // Determine process type:
        bool is_cpu_bound = false;
        if (i < n_cpu) is_cpu_bound = true;
        
        // Process arrival time
        int p_arrival_time = floor(nextExp());
        
        // Burst times
        int num_cpu = std::ceil(drand48() * 32);      // # CPU bursts
        int num_io = num_cpu - 1;
        int num_total = num_cpu + num_io;
        int* burst_times = new int[num_total];

        // fill combined burst times list
        for (int j = 0; j < num_total; j++)
        {
            int x = std::ceil(nextExp());

            if (is_cpu_bound)
            {
                if (j % 2 == 0)
                {
                    x = x*4;
                    cpu_cpu_total += x; num_cpu_cpu++;
                } 
                else 
                { cpu_io_total += x; num_cpu_io++; }
            }      
            else {
                if (j % 2 == 1)
                {
                    x *= 8;
                    io_io_total += x; num_io_io++;
                }
                else
                { io_cpu_total += x; num_io_cpu++; }
            }
            
            burst_times[j] = x;
        }

        Process* p = new Process();
        p->id = new char[3];
        std::sprintf(p->id, "%c%d", 'A' + (i / 10), i % 10);
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

    float cpu_cpu_avg = num_cpu_cpu != 0 ? (cpu_cpu_total / num_cpu_cpu) : 0;
    float cpu_io_avg = num_cpu_io != 0 ? (cpu_io_total / num_cpu_io) : 0;
    float cpu_avg = (num_cpu_cpu + num_io_cpu) != 0 ? std::ceil(1000 * (cpu_cpu_total + io_cpu_total) / (num_cpu_cpu + num_io_cpu)) / 1000 : 0;
    float io_cpu_avg = num_io_cpu != 0 ? (io_cpu_total / num_io_cpu) : 0;
    float io_io_avg = num_io_io != 0 ? (io_io_total / num_io_io) : 0;
    float io_avg = (num_cpu_io + num_io_io) != 0 ? std::ceil(1000 * (cpu_io_total + io_io_total) / (num_cpu_io + num_io_io)) / 1000 : 0;

    for (Process* p : processes)
    {
        // Process name

        if (p->is_cpu_bound) std::cout << "\nCPU-bound";
        else std::cout << "\nI/O-bound";
        std::cout << " process " << p->id << ": arrival time " << p->arrival_time << "ms; "
             << p->num_cpu_bursts << (p->num_cpu_bursts != 1 ? " CPU bursts\n" : " CPU burst\n");
        for (int j = 0; j < p->num_cpu_bursts * 2 - 1; ++j)
        {
             if (j % 2 == 0)
                 std::cout << "==> CPU burst " << p->burst_times[j] << "ms";
             else std::cout << " ==> I/O burst " << p->burst_times[j] << "ms\n";
        }
         std::cout << "\n";
    }

    std::ofstream simout;
    simout.open("simout.txt");
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

    /* Initialize P2 Simulation */
    std::cout << "<<< PROJECT SIMULATIONS\n<<< -- t_cs=" << tcs << "ms; alpha=" << std::fixed << std::setprecision(2) << alpha << "; t_slice=" << tslice << "ms\n";
    OpSys* simulation = new OpSys();
    simulation->t_cs = tcs;
    simulation->tslice = tslice;
    for (Process* p : processes) simulation->unfinished.insert(p); 
    for (Process* p : processes) simulation->unarrived.push(p); 
    simulation->first_come_first_served();

    /* Reset and readd processes for next algorithm. */
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

    /* Clean up dynamic memory. */
    delete simulation;
    for (Process* p : processes)
    {
        delete [] p->id;
        delete [] p->burst_times;
        delete p;
    }

    return 0;
}


// num processes
