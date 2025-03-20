#ifndef PROJECT_H
#define PROJECT_H

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <cmath>
#include <sstream>
#include <fstream>

//---------------------------------------------------------------------
// Enumerations for scheduling algorithms and event types
//---------------------------------------------------------------------
enum class SchedulingAlgo {
    FCFS,
    SJF,
    SRT,
    RR
};

enum class EventType {
    PROCESS_ARRIVAL,
    CPU_BURST_COMPLETION,
    IO_BURST_COMPLETION,
    CPU_START,
    TIME_SLICE_EXPIRE  // (for RR preemptions)
    // You can add more event types as needed (e.g., CONTEXT_SWITCH events)
};

//---------------------------------------------------------------------
// Process structure – holds all info for each simulated process
//---------------------------------------------------------------------
struct Process {
    std::string pid;                // e.g., "A0"
    int arrivalTime;                // calculated using floor(next_exp())
    std::vector<int> cpuBursts;     // list of CPU burst times (last burst has no I/O)
    std::vector<int> ioBursts;      // list of I/O burst times (size = numBursts - 1)
    int currentBurst;               // index into cpuBursts (current CPU burst)
    int remainingBurstTime;         // remaining time of current CPU burst (for preemption)
    double tau;                     // estimated CPU burst time (for SJF/SRT)
    bool isCPUBound;                // true if CPU-bound; false if I/O-bound

    // Statistics (for final simout.txt)
    int totalCpuTime;
    int totalWaitTime;
    int totalTurnaroundTime;
    int contextSwitches;
    int preemptions;

    Process() : arrivalTime(0), currentBurst(0), remainingBurstTime(0), tau(0.0),
                isCPUBound(false), totalCpuTime(0), totalWaitTime(0),
                totalTurnaroundTime(0), contextSwitches(0), preemptions(0) {}
};

//---------------------------------------------------------------------
// Simulation parameters (command-line arguments)
//---------------------------------------------------------------------
struct SimulationParams {
    int n;          // total number of processes
    int ncpu;       // number of CPU-bound processes (first ncpu processes)
    long seed;      // seed for pseudo-random number generator
    double lambda;  // parameter for exponential distribution
    int bound;      // upper bound for valid pseudo-random numbers
    int tcs;        // context switch time (even positive integer)
    double alpha;   // constant for tau update (0<=alpha<=1)
    int tslice;     // time slice for Round Robin algorithm
};

//---------------------------------------------------------------------
// Event structure for simulation events
//---------------------------------------------------------------------
struct Event {
    int time;           // event time in ms
    EventType type;     // event type (arrival, CPU completion, etc.)
    Process* proc;      // pointer to the process associated with this event
    int remainingTime;  // for RR preemption events (if needed)

    Event(int t, EventType et, Process* p, int rem = 0)
        : time(t), type(et), proc(p), remainingTime(rem) {}
};

// Comparator for the event priority queue (earlier times first, then type, then pid)
struct EventComparator {
    bool operator()(const Event &a, const Event &b) const {
        if (a.time != b.time)
            return a.time > b.time;
        if (a.type != b.type)
            return a.type > b.type; // You may want a custom ordering here
        return a.proc->pid > b.proc->pid;
    }
};

//---------------------------------------------------------------------
// Simulator class encapsulates the simulation engine.
//---------------------------------------------------------------------
class Simulator {
public:
    Simulator(const SimulationParams& params, const std::vector<Process>& processes);
    void runSimulation(SchedulingAlgo algo);
    void printSimStatistics(SchedulingAlgo algo, std::ostream &out);

private:
    SimulationParams params;
    std::vector<Process> processes; // copy of the original process set
    std::priority_queue<Event, std::vector<Event>, EventComparator> eventQueue;
    int currentTime;  // current simulation time in ms

    // Ready queue – a vector of process pointers.
    std::vector<Process*> readyQueue;

    // Helper functions:
    void scheduleEvent(const Event &event);
    void processEvent(const Event &event, SchedulingAlgo algo);
    void addProcessToReadyQueue(Process* proc, SchedulingAlgo algo);
    std::string readyQueueToString();
    void logEvent(int time, const std::string &eventDetails);
    void resetSimulation();
};

//---------------------------------------------------------------------
// Custom pseudo-random number generator functions.
// These mimic srand48() and drand48() using a 48-bit linear congruential method.
//---------------------------------------------------------------------
void my_srand48(long seed);
double my_drand48();

#endif // PROJECT_H
