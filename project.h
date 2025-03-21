#ifndef PROJECT_H
#define PROJECT_H

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <cmath>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdexcept>

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
    CPU_START,
    CPU_BURST_COMPLETION,
    IO_BURST_COMPLETION,
    TIME_SLICE_EXPIRE
};

//---------------------------------------------------------------------
// Process structure – holds all info for each simulated process
//---------------------------------------------------------------------
struct Process {
    std::string pid;                // e.g., "A0"
    int arrivalTime;                // arrival time in ms
    std::vector<int> cpuBursts;     // list of CPU burst times
    std::vector<int> ioBursts;      // list of I/O burst times (one fewer than CPU bursts)
    int currentBurst;               // index of the current CPU burst
    int remainingBurstTime;         // remaining time in current CPU burst
    double tau;                     // estimated CPU burst time (for SJF/SRT)
    bool isCPUBound;                // true if CPU-bound, false if I/O-bound

    // Statistics per process (if needed individually)
    int totalCpuTime;
    int totalWaitTime;
    int totalTurnaroundTime;
    int contextSwitches;
    int preemptions;

    // NEW: For measuring wait and turnaround for the current burst.
    bool readyTimeSet;      // Indicates if firstReadyTime has been set for the current burst.
    int firstReadyTime;     // The time when the process first entered the ready queue for this burst.

    Process() : arrivalTime(0), currentBurst(0), remainingBurstTime(0), tau(0.0),
                isCPUBound(false), totalCpuTime(0), totalWaitTime(0),
                totalTurnaroundTime(0), contextSwitches(0), preemptions(0),
                readyTimeSet(false), firstReadyTime(0) {}
};

//---------------------------------------------------------------------
// Simulation parameters (from command-line arguments)
//---------------------------------------------------------------------
struct SimulationParams {
    int n;          // total number of processes
    int ncpu;       // number of CPU-bound processes
    long seed;      // seed for PRNG
    double lambda;  // exponential distribution parameter
    int bound;      // upper bound for random numbers
    int tcs;        // context switch time (ms, even positive integer)
    double alpha;   // constant for tau update (0 <= alpha <= 1)
    int tslice;     // time slice for RR (ms)
};

//---------------------------------------------------------------------
// Event structure for simulation events
//---------------------------------------------------------------------
struct Event {
    int time;           // event time in ms
    EventType type;     // type of event
    Process* proc;      // pointer to the associated process
    int remainingTime;  // (for RR preemption events)

    Event(int t, EventType type, Process* proc, int rem = 0)
        : time(t), type(type), proc(proc), remainingTime(rem) {}
};

//---------------------------------------------------------------------
// Comparator for event priority queue (earlier times first, then type, then PID)
//---------------------------------------------------------------------
struct EventComparator {
    bool operator()(const Event &a, const Event &b) const {
        if (a.time != b.time)
            return a.time > b.time;
        if (a.type != b.type)
            return static_cast<int>(a.type) > static_cast<int>(b.type);
        return a.proc->pid > b.proc->pid;
    }
};

//---------------------------------------------------------------------
// Simulator class: encapsulates the simulation engine and measurement tracking.
//---------------------------------------------------------------------
class Simulator {
public:
    Simulator(const SimulationParams& params, const std::vector<Process>& processes);
    void runSimulation(SchedulingAlgo algo);
    void printSimStatistics(SchedulingAlgo algo, std::ostream &out);

private:
    SimulationParams params;
    std::vector<Process> processes; // simulation process set
    std::priority_queue<Event, std::vector<Event>, EventComparator> eventQueue;
    std::vector<Process*> readyQueue;
    int currentTime;  // simulation time in ms
    Process* runningProcess;

    // Measurement fields (accumulated over the simulation):
    int totalCpuBusyTime;
    double sumCpuBurstTimeCpuBound, sumCpuBurstTimeIOBound;
    int countCpuBurstCpuBound, countCpuBurstIOBound;
    double sumIOBurstTimeCpuBound, sumIOBurstTimeIOBound;
    int countIOBurstCpuBound, countIOBurstIOBound;
    double sumWaitTimeCpuBound, sumWaitTimeIOBound;
    int countWaitCpuBound, countWaitIOBound;
    double sumTurnaroundCpuBound, sumTurnaroundIOBound;
    int countTurnaroundCpuBound, countTurnaroundIOBound;
    int contextSwitchesCpuBound, contextSwitchesIOBound;
    int preemptionsCpuBound, preemptionsIOBound;

    // Helper functions
    void scheduleEvent(const Event &event);
    void processEvent(const Event &event, SchedulingAlgo algo);
    void addProcessToReadyQueue(Process* proc, SchedulingAlgo algo);
    std::string readyQueueToString();
    void logEvent(int time, const std::string &eventDetails);
    void resetSimulation();
};

//---------------------------------------------------------------------
// Custom pseudo-random number generator functions (mimicking srand48/drand48)
//---------------------------------------------------------------------
void my_srand48(long seed);
double my_drand48();

#endif // PROJECT_H
