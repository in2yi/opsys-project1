#include "project.h"
#include <iomanip>
#include <algorithm>
#include <stdexcept>
#include <cstdlib>
#include <cmath>

//---------------------------------------------------------------------
// Global variables and constants for our drand48-like PRNG.
//---------------------------------------------------------------------
static unsigned long long drand_state = 0;
const unsigned long long A = 0x5DEECE66DLL;
const unsigned long long C = 0xBLL;
const unsigned long long MASK = (1ULL << 48) - 1;

void my_srand48(long seed) {
    // Set up an initial state (this is a typical initialization)
    drand_state = ((unsigned long long)seed << 16) | 0x330E;
}

double my_drand48() {
    drand_state = (A * drand_state + C) & MASK;
    return static_cast<double>(drand_state) / static_cast<double>(1ULL << 48);
}

//---------------------------------------------------------------------
// Simulator class member functions
//---------------------------------------------------------------------
Simulator::Simulator(const SimulationParams& params, const std::vector<Process>& processes)
    : params(params), processes(processes), currentTime(0) { }

// Schedule an event by inserting it into the priority queue.
void Simulator::scheduleEvent(const Event &event) {
    eventQueue.push(event);
}

// Convert the current ready queue into a string.
std::string Simulator::readyQueueToString() {
    std::ostringstream oss;
    if (readyQueue.empty()) {
        oss << "empty";
    } else {
        for (auto proc : readyQueue)
            oss << proc->pid << " ";
    }
    return oss.str();
}

// Log an event in the required format.
void Simulator::logEvent(int time, const std::string &eventDetails) {
    std::cout << "time " << time << "ms: " << eventDetails 
              << " [Q " << readyQueueToString() << "]" << std::endl;
}

// Insert a process into the ready queue; ordering depends on the algorithm.
void Simulator::addProcessToReadyQueue(Process* proc, SchedulingAlgo algo) {
    // For FCFS and RR: simply append.
    // For SJF and SRT: insert sorted by tau (and process id to break ties).
    if (algo == SchedulingAlgo::FCFS || algo == SchedulingAlgo::RR) {
        readyQueue.push_back(proc);
    } else {
        readyQueue.push_back(proc);
        std::sort(readyQueue.begin(), readyQueue.end(), [](Process* a, Process* b) {
            if (a->tau != b->tau)
                return a->tau < b->tau;
            return a->pid < b->pid;
        });
    }
}

// Process an event from the event queue.
void Simulator::processEvent(const Event &event, SchedulingAlgo algo) {
    currentTime = event.time;
    Process* proc = event.proc;

    switch (event.type) {
    case EventType::PROCESS_ARRIVAL: {
        std::ostringstream oss;
        if (algo == SchedulingAlgo::SJF || algo == SchedulingAlgo::SRT)
            oss << "Process " << proc->pid << " (tau " << std::fixed << std::setprecision(0)
                << proc->tau << "ms) arrived; added to ready queue";
        else
            oss << "Process " << proc->pid << " arrived; added to ready queue";
        logEvent(currentTime, oss.str());
        addProcessToReadyQueue(proc, algo);

        // If CPU is idle (i.e. the ready queue had no one before this), schedule its CPU start.
        if (readyQueue.size() == 1) {
            int cs_in = params.tcs / 2;
            scheduleEvent(Event(currentTime + cs_in, EventType::CPU_START, proc));
        }
        break;
    }
    case EventType::CPU_START: {
        // Pop the process from the ready queue.
        if (!readyQueue.empty() && readyQueue.front() == proc)
            readyQueue.erase(readyQueue.begin());

        int burstTime = proc->cpuBursts[proc->currentBurst];
        proc->remainingBurstTime = burstTime; // reset remaining burst time
        std::ostringstream oss;
        if (algo == SchedulingAlgo::SJF || algo == SchedulingAlgo::SRT)
            oss << "Process " << proc->pid << " (tau " << std::fixed << std::setprecision(0)
                << proc->tau << "ms) started using the CPU for " << burstTime << "ms burst";
        else
            oss << "Process " << proc->pid << " started using the CPU for " << burstTime << "ms burst";
        logEvent(currentTime, oss.str());

        // Schedule the CPU burst completion (include the second half of context switch).
        int completionTime = currentTime + burstTime + params.tcs / 2;
        scheduleEvent(Event(completionTime, EventType::CPU_BURST_COMPLETION, proc));
        break;
    }
    case EventType::CPU_BURST_COMPLETION: {
        // CPU burst has finished.
        int burstsLeft = proc->cpuBursts.size() - proc->currentBurst - 1;
        std::ostringstream oss;
        oss << "Process " << proc->pid << " completed a CPU burst; " << burstsLeft << " bursts to go";
        logEvent(currentTime, oss.str());

        // For SJF/SRT, recalculate tau using exponential averaging.
        if (algo == SchedulingAlgo::SJF || algo == SchedulingAlgo::SRT) {
            int actualBurst = proc->cpuBursts[proc->currentBurst];
            int oldTau = static_cast<int>(proc->tau);
            proc->tau = std::ceil(params.alpha * actualBurst + (1 - params.alpha) * proc->tau);
            std::ostringstream tauoss;
            tauoss << "Recalculated tau for process " << proc->pid << ": old tau " << oldTau
                   << "ms ==> new tau " << std::fixed << std::setprecision(0) << proc->tau << "ms";
            logEvent(currentTime, tauoss.str());
        }
        // Schedule context switch out and (if not terminated) an I/O burst.
        int cs_out = params.tcs / 2;
        int ioStartTime = currentTime + cs_out;
        if (proc->currentBurst < static_cast<int>(proc->cpuBursts.size()) - 1) {
            std::ostringstream oss2;
            int ioCompletionTime = ioStartTime + proc->ioBursts[proc->currentBurst];
            oss2 << "Process " << proc->pid << " switching out of CPU; blocking on I/O until time " << ioCompletionTime;
            logEvent(currentTime, oss2.str());
            scheduleEvent(Event(ioCompletionTime, EventType::IO_BURST_COMPLETION, proc));
        } else {
            std::ostringstream oss2;
            oss2 << "Process " << proc->pid << " terminated";
            logEvent(currentTime, oss2.str());
        }
        proc->currentBurst++;  // move on to the next burst (if any)
        break;
    }
    case EventType::IO_BURST_COMPLETION: {
        std::ostringstream oss;
        oss << "Process " << proc->pid << " completed I/O; added to ready queue";
        logEvent(currentTime, oss.str());
        addProcessToReadyQueue(proc, algo);
        // If CPU idle, schedule CPU start.
        if (readyQueue.size() == 1) {
            int cs_in = params.tcs / 2;
            scheduleEvent(Event(currentTime + cs_in, EventType::CPU_START, proc));
        }
        break;
    }
    case EventType::TIME_SLICE_EXPIRE: {
        // Only applicable for RR.
        std::ostringstream oss;
        oss << "Time slice expired; preempting process " << proc->pid << " with " 
            << event.remainingTime << "ms remaining";
        logEvent(currentTime, oss.str());
        proc->preemptions++;
        addProcessToReadyQueue(proc, algo);
        if (!readyQueue.empty()) {
            int cs_in = params.tcs / 2;
            Process* nextProc = readyQueue.front();
            scheduleEvent(Event(currentTime + cs_in, EventType::CPU_START, nextProc));
        }
        break;
    }
    default:
        break;
    }
}

// Reset simulation state between algorithm runs.
void Simulator::resetSimulation() {
    currentTime = 0;
    while (!eventQueue.empty()) eventQueue.pop();
    readyQueue.clear();
    for (auto &proc : processes) {
        proc.currentBurst = 0;
        proc.remainingBurstTime = 0;
        proc.totalCpuTime = 0;
        proc.totalWaitTime = 0;
        proc.totalTurnaroundTime = 0;
        proc.contextSwitches = 0;
        proc.preemptions = 0;
        // Reset tau to initial guess = ceil(1/lambda)
        proc.tau = std::ceil(1.0 / params.lambda);
    }
}

// Run a simulation pass using the specified scheduling algorithm.
void Simulator::runSimulation(SchedulingAlgo algo) {
    resetSimulation();
    // Schedule arrival events for every process.
    for (auto &proc : processes)
        scheduleEvent(Event(proc.arrivalTime, EventType::PROCESS_ARRIVAL, &proc));

    // Process events until either the event queue is empty or until time reaches 10000ms.
    while (!eventQueue.empty() && currentTime < 10000) {
        Event event = eventQueue.top();
        eventQueue.pop();
        processEvent(event, algo);
    }
    // For events at t >= 10000ms, you might choose to only process terminations.
    // (This skeleton simply drains the queue with minimal processing.)
    while (!eventQueue.empty()) {
        Event event = eventQueue.top();
        eventQueue.pop();
        if (event.type == EventType::CPU_BURST_COMPLETION)
            processEvent(event, algo);
    }
    logEvent(currentTime, "Simulator ended for algorithm");
}

// Print (dummy) simulation statistics for use in simout.txt.
void Simulator::printSimStatistics(SchedulingAlgo algo, std::ostream &out) {
    out << "Algorithm ";
    switch (algo) {
        case SchedulingAlgo::FCFS: out << "FCFS"; break;
        case SchedulingAlgo::SJF:  out << "SJF"; break;
        case SchedulingAlgo::SRT:  out << "SRT"; break;
        case SchedulingAlgo::RR:   out << "RR"; break;
    }
    out << std::endl;
    out << "-- CPU utilization: 0.000%" << std::endl;
    out << "-- CPU-bound average wait time: 0.000 ms" << std::endl;
    out << "-- I/O-bound average wait time: 0.000 ms" << std::endl;
    out << "-- overall average wait time: 0.000 ms" << std::endl;
    out << "-- CPU-bound average turnaround time: 0.000 ms" << std::endl;
    out << "-- I/O-bound average turnaround time: 0.000 ms" << std::endl;
    out << "-- overall average turnaround time: 0.000 ms" << std::endl;
    out << "-- CPU-bound number of context switches: 0" << std::endl;
    out << "-- I/O-bound number of context switches: 0" << std::endl;
    out << "-- overall number of context switches: 0" << std::endl;
    out << "-- CPU-bound number of preemptions: 0" << std::endl;
    out << "-- I/O-bound number of preemptions: 0" << std::endl;
    out << "-- overall number of preemptions: 0" << std::endl;
    if (algo == SchedulingAlgo::RR) {
        out << "-- CPU-bound percentage of CPU bursts completed within one time slice: 0.000%" << std::endl;
        out << "-- I/O-bound percentage of CPU bursts completed within one time slice: 0.000%" << std::endl;
        out << "-- overall percentage of CPU bursts completed within one time slice: 0.000%" << std::endl;
    }
}

//---------------------------------------------------------------------
// Process generation based on command-line parameters and pseudo-random values.
// (Remember: CPU-bound processes are generated first.)
//---------------------------------------------------------------------
std::vector<Process> generateProcesses(const SimulationParams &params) {
    std::vector<Process> processes;
    int totalProcesses = params.n;
    int cpuBoundCount = params.ncpu;
    std::vector<std::string> pids;
    for (char letter = 'A'; letter <= 'Z'; ++letter) {
        for (int num = 0; num < 10; ++num) {
            pids.push_back(std::string(1, letter) + std::to_string(num));
            if (pids.size() == static_cast<size_t>(totalProcesses))
                break;
        }
        if (pids.size() == static_cast<size_t>(totalProcesses))
            break;
    }

    my_srand48(params.seed);  // seed the generator

    for (int i = 0; i < totalProcesses; i++) {
        Process proc;
        proc.pid = pids[i];
        // Arrival time = floor(next_exp())
        double r = my_drand48();
        double exp_val = -std::log(r) / params.lambda;
        proc.arrivalTime = static_cast<int>(std::floor(exp_val));
        // Number of CPU bursts = ceiling(drand48() * 32) [range 1..32]
        double r2 = my_drand48();
        int numBursts = static_cast<int>(std::ceil(r2 * 32));
        // Set initial tau = ceil(1/lambda)
        proc.tau = std::ceil(1.0 / params.lambda);
        proc.isCPUBound = (i < cpuBoundCount);
        for (int b = 0; b < numBursts; b++) {
            double r_cpu = my_drand48();
            double burst_val = -std::log(r_cpu) / params.lambda;
            int cpuBurst = static_cast<int>(std::ceil(burst_val));
            if (proc.isCPUBound)
                cpuBurst *= 4;  // CPU-bound processes get longer CPU bursts
            proc.cpuBursts.push_back(cpuBurst);
            if (b < numBursts - 1) {  // if not the final burst, generate an I/O burst.
                double r_io = my_drand48();
                double io_val = -std::log(r_io) / params.lambda;
                int ioBurst = static_cast<int>(std::ceil(io_val));
                if (proc.isCPUBound) {
                    // For CPU-bound, make I/O bursts shorter.
                    ioBurst = (ioBurst > 8) ? ioBurst / 8 : 1;
                } else {
                    ioBurst *= 8;
                    if (ioBurst > params.bound)
                        ioBurst = params.bound;
                }
                proc.ioBursts.push_back(ioBurst);
            }
        }
        processes.push_back(proc);
    }
    return processes;
}

//---------------------------------------------------------------------
// Main: command-line parsing, process generation, simulation runs, and
// simout.txt statistics file output. (No sugar-coating here. You asked for it.)
//---------------------------------------------------------------------
int main(int argc, char* argv[]) {
    if (argc != 9) {
        std::cerr << "ERROR: Invalid number of arguments" << std::endl;
        return 1;
    }
    SimulationParams simParams;
    try {
        simParams.n = std::stoi(argv[1]);
        if (simParams.n <= 0)
            throw std::invalid_argument("n must be > 0");
        simParams.ncpu = std::stoi(argv[2]);
        simParams.seed = std::stol(argv[3]);
        simParams.lambda = std::stod(argv[4]);
        simParams.bound = std::stoi(argv[5]);
        simParams.tcs = std::stoi(argv[6]);
        if (simParams.tcs <= 0 || (simParams.tcs % 2 != 0))
            throw std::invalid_argument("tcs must be a positive even integer");
        simParams.alpha = std::stod(argv[7]);
        if (simParams.alpha < 0.0 || simParams.alpha > 1.0)
            throw std::invalid_argument("alpha must be between 0 and 1");
        simParams.tslice = std::stoi(argv[8]);
        if (simParams.tslice <= 0)
            throw std::invalid_argument("tslice must be a positive integer");
    } catch (std::exception &e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }

    // Generate the process set.
    std::vector<Process> processSet = generateProcesses(simParams);

    // Print the process set summary.
    std::cout << "<<< -- process set (n=" << simParams.n << ") with " << simParams.ncpu 
              << " CPU-bound process" << ((simParams.ncpu == 1) ? "" : "es") << std::endl;
    std::cout << "<<< -- seed=" << simParams.seed << "; lambda=" << std::fixed << std::setprecision(6)
              << simParams.lambda << "; bound=" << simParams.bound << std::endl;
    for (auto &proc : processSet) {
        std::cout << (proc.isCPUBound ? "CPU-bound" : "I/O-bound") << " process " << proc.pid 
                  << ": arrival time " << proc.arrivalTime << "ms; " << proc.cpuBursts.size()
                  << " CPU bursts:" << std::endl;
        for (size_t i = 0; i < proc.cpuBursts.size(); i++) {
            std::cout << "==> CPU burst " << proc.cpuBursts[i] << "ms";
            if (i < proc.ioBursts.size())
                std::cout << " ==> I/O burst " << proc.ioBursts[i] << "ms";
            std::cout << std::endl;
        }
    }

    std::cout << "<<< PROJECT SIMULATIONS" << std::endl;
    std::cout << "<<< -- t_cs=" << simParams.tcs << "ms; alpha=" << std::fixed << std::setprecision(2)
              << simParams.alpha << "; t_slice=" << simParams.tslice << "ms" << std::endl;

    // Create our simulator.
    Simulator simulator(simParams, processSet);

    // Run the simulations in succession.
    simulator.runSimulation(SchedulingAlgo::FCFS);
    simulator.runSimulation(SchedulingAlgo::SJF);
    simulator.runSimulation(SchedulingAlgo::SRT);
    simulator.runSimulation(SchedulingAlgo::RR);

    // Write summary statistics to simout.txt.
    std::ofstream simout("simout.txt");
    if (!simout) {
        std::cerr << "ERROR: Could not open simout.txt for writing" << std::endl;
        return 1;
    }
    simout << "-- number of processes: " << simParams.n << std::endl;
    simout << "-- number of CPU-bound processes: " << simParams.ncpu << std::endl;
    simout << "-- number of I/O-bound processes: " << (simParams.n - simParams.ncpu) << std::endl;
    simout << "-- CPU-bound average CPU burst time: 0.000 ms" << std::endl;
    simout << "-- I/O-bound average CPU burst time: 0.000 ms" << std::endl;
    simout << "-- overall average CPU burst time: 0.000 ms" << std::endl;
    simout << "-- CPU-bound average I/O burst time: 0.000 ms" << std::endl;
    simout << "-- I/O-bound average I/O burst time: 0.000 ms" << std::endl;
    simout << "-- overall average I/O burst time: 0.000 ms" << std::endl;
    simulator.printSimStatistics(SchedulingAlgo::FCFS, simout);
    simulator.printSimStatistics(SchedulingAlgo::SJF, simout);
    simulator.printSimStatistics(SchedulingAlgo::SRT, simout);
    simulator.printSimStatistics(SchedulingAlgo::RR, simout);
    simout.close();

    return 0;
}
