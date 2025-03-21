#include "project.h"
#include <cmath>
#include <sstream>
#include <algorithm>
#include <stdexcept>
#include <cstdlib>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iostream>

//---------------------------------------------------------------------
// Custom PRNG implementation
//---------------------------------------------------------------------
static unsigned long long drand_state = 0;
const unsigned long long A = 0x5DEECE66DLL;
const unsigned long long C = 0xBLL;
const unsigned long long MASK = (1ULL << 48) - 1;

void my_srand48(long seed) {
    drand_state = ((unsigned long long)seed << 16) | 0x330E;
}

double my_drand48() {
    drand_state = (A * drand_state + C) & MASK;
    return static_cast<double>(drand_state) / (1ULL << 48);
}

//---------------------------------------------------------------------
// Simulator class member functions
//---------------------------------------------------------------------
Simulator::Simulator(const SimulationParams& params, const std::vector<Process>& processes)
    : params(params), processes(processes), currentTime(0), runningProcess(nullptr) // NEW: initialize runningProcess
{
    resetSimulation();
}

void Simulator::scheduleEvent(const Event &event) {
    eventQueue.push(event);
}

std::string Simulator::readyQueueToString() {
    std::ostringstream oss;
    if (readyQueue.empty())
        oss << "empty";
    else {
        for (auto proc : readyQueue)
            oss << proc->pid << " ";
    }
    return oss.str();
}

void Simulator::logEvent(int time, const std::string &eventDetails) {
    std::cout << "time " << time << "ms: " << eventDetails 
              << " [Q " << readyQueueToString() << "]" << std::endl;
}

void Simulator::addProcessToReadyQueue(Process* proc, SchedulingAlgo algo) {
    // Record the time the process enters the ready queue.
    if (!proc->readyTimeSet) {
        proc->firstReadyTime = currentTime;
        proc->readyTimeSet = true;
    }
    readyQueue.push_back(proc);
    if (algo == SchedulingAlgo::SJF || algo == SchedulingAlgo::SRT) {
        std::sort(readyQueue.begin(), readyQueue.end(), [](Process* a, Process* b) {
            if (a->tau != b->tau)
                return a->tau < b->tau;
            return a->pid < b->pid;
        });
    }
}

void Simulator::processEvent(const Event &event, SchedulingAlgo algo) {
    currentTime = event.time;
    Process* proc = event.proc;
    int cs_in = params.tcs / 2;
    int cs_out = params.tcs / 2;

    switch (event.type) {
    case EventType::PROCESS_ARRIVAL: {
        proc->readyTimeSet = false;
        std::ostringstream oss;
        if (algo == SchedulingAlgo::SJF || algo == SchedulingAlgo::SRT)
            oss << "Process " << proc->pid << " (tau " << std::fixed << std::setprecision(0)
                << proc->tau << "ms) arrived; added to ready queue";
        else
            oss << "Process " << proc->pid << " arrived; added to ready queue";
        logEvent(currentTime, oss.str());
        addProcessToReadyQueue(proc, algo);

        // For SRT, if a process is running, check if the new arrival preempts it.
        if (algo == SchedulingAlgo::SRT && runningProcess != nullptr) {
            if (proc->cpuBursts[proc->currentBurst] < runningProcess->remainingBurstTime) {
                std::ostringstream preemptOSS;
                preemptOSS << "Process " << proc->pid << " preempts " << runningProcess->pid 
                           << " (remaining " << runningProcess->remainingBurstTime << "ms)";
                logEvent(currentTime, preemptOSS.str());
                scheduleEvent(Event(currentTime + cs_out, EventType::TIME_SLICE_EXPIRE, runningProcess, runningProcess->remainingBurstTime));
                runningProcess = nullptr;
            }
        }
        if (readyQueue.size() == 1 && runningProcess == nullptr) {
            scheduleEvent(Event(currentTime + cs_in, EventType::CPU_START, readyQueue.front()));
        }
        break;
    }
    case EventType::CPU_START: {
        if (!readyQueue.empty() && readyQueue.front() == proc)
            readyQueue.erase(readyQueue.begin());
        runningProcess = proc; // Mark as running.
        int burstTime = proc->cpuBursts[proc->currentBurst];
        if (proc->remainingBurstTime == 0)
            proc->remainingBurstTime = burstTime;
        int waitTime = currentTime - proc->firstReadyTime;
        if (waitTime < 0)
            waitTime = 0;
        if (proc->isCPUBound) {
            sumWaitTimeCpuBound += waitTime;
            countWaitCpuBound++;
        } else {
            sumWaitTimeIOBound += waitTime;
            countWaitIOBound++;
        }
        std::ostringstream oss;
        if (algo == SchedulingAlgo::SJF || algo == SchedulingAlgo::SRT)
            oss << "Process " << proc->pid << " (tau " << std::fixed << std::setprecision(0)
                << proc->tau << "ms) started using the CPU for " << proc->remainingBurstTime << "ms burst";
        else
            oss << "Process " << proc->pid << " started using the CPU for " << proc->remainingBurstTime << "ms burst";
        logEvent(currentTime, oss.str());
        
        if (algo == SchedulingAlgo::RR && proc->remainingBurstTime > params.tslice) {
            scheduleEvent(Event(currentTime + params.tslice, EventType::TIME_SLICE_EXPIRE, proc, proc->remainingBurstTime - params.tslice));
        } else {
            scheduleEvent(Event(currentTime + proc->remainingBurstTime + cs_out, EventType::CPU_BURST_COMPLETION, proc));
        }
        break;
    }
    case EventType::TIME_SLICE_EXPIRE: {
        std::ostringstream oss;
        oss << "Time slice expired; preempting process " << proc->pid << " with " 
            << event.remainingTime << "ms remaining";
        logEvent(currentTime, oss.str());
        if (proc->isCPUBound)
            preemptionsCpuBound++;
        else
            preemptionsIOBound++;
        if (!readyQueue.empty()) {
            proc->remainingBurstTime = event.remainingTime;
            scheduleEvent(Event(currentTime + cs_out, EventType::PROCESS_ARRIVAL, proc));
            runningProcess = nullptr;
            scheduleEvent(Event(currentTime + cs_in, EventType::CPU_START, readyQueue.front()));
        } else {
            scheduleEvent(Event(currentTime + event.remainingTime + cs_out, EventType::CPU_BURST_COMPLETION, proc));
        }
        break;
    }
    case EventType::CPU_BURST_COMPLETION: {
        int burstTime = proc->cpuBursts[proc->currentBurst];
        totalCpuBusyTime += burstTime;
        if (proc->isCPUBound) {
            sumCpuBurstTimeCpuBound += burstTime;
            countCpuBurstCpuBound++;
        } else {
            sumCpuBurstTimeIOBound += burstTime;
            countCpuBurstIOBound++;
        }
        std::ostringstream oss;
        int burstsLeft = static_cast<int>(proc->cpuBursts.size()) - proc->currentBurst - 1;
        oss << "Process " << proc->pid << " completed a CPU burst; " << burstsLeft << " bursts to go";
        logEvent(currentTime, oss.str());
        if (algo == SchedulingAlgo::SJF || algo == SchedulingAlgo::SRT) {
            int oldTau = static_cast<int>(proc->tau);
            proc->tau = std::ceil(params.alpha * burstTime + (1 - params.alpha) * proc->tau);
            std::ostringstream tauoss;
            tauoss << "Recalculated tau for process " << proc->pid << ": old tau " << oldTau
                   << "ms ==> new tau " << std::fixed << std::setprecision(0) << proc->tau << "ms";
            logEvent(currentTime, tauoss.str());
        }
        int turnaround = (currentTime + cs_out) - proc->firstReadyTime;
        if (proc->isCPUBound) {
            sumTurnaroundCpuBound += turnaround;
            countTurnaroundCpuBound++;
        } else {
            sumTurnaroundIOBound += turnaround;
            countTurnaroundIOBound++;
        }
        proc->readyTimeSet = false;
        runningProcess = nullptr;
        if (proc->currentBurst < static_cast<int>(proc->cpuBursts.size()) - 1) {
            int ioBurst = proc->ioBursts[proc->currentBurst];
            if (proc->isCPUBound) {
                sumIOBurstTimeCpuBound += ioBurst;
                countIOBurstCpuBound++;
            } else {
                sumIOBurstTimeIOBound += ioBurst;
                countIOBurstIOBound++;
            }
            std::ostringstream oss2;
            int ioCompletionTime = currentTime + cs_out + ioBurst;
            oss2 << "Process " << proc->pid << " switching out of CPU; blocking on I/O until time " << ioCompletionTime;
            logEvent(currentTime, oss2.str());
            scheduleEvent(Event(ioCompletionTime, EventType::IO_BURST_COMPLETION, proc));
        } else {
            std::ostringstream oss2;
            oss2 << "Process " << proc->pid << " terminated";
            logEvent(currentTime, oss2.str());
        }
        proc->currentBurst++;
        if (!readyQueue.empty()) {
            scheduleEvent(Event(currentTime + cs_in, EventType::CPU_START, readyQueue.front()));
        }
        break;
    }
    case EventType::IO_BURST_COMPLETION: {
        std::ostringstream oss;
        oss << "Process " << proc->pid << " completed I/O; added to ready queue";
        logEvent(currentTime, oss.str());
        proc->readyTimeSet = false;
        addProcessToReadyQueue(proc, algo);
        if (readyQueue.size() == 1 && runningProcess == nullptr) {
            scheduleEvent(Event(currentTime + cs_in, EventType::CPU_START, proc));
        }
        break;
    }
    default:
        break;
    }
}

void Simulator::resetSimulation() {
    currentTime = 0;
    runningProcess = nullptr; // NEW: reset runningProcess
    while (!eventQueue.empty())
        eventQueue.pop();
    readyQueue.clear();
    for (auto &proc : processes) {
        proc.currentBurst = 0;
        proc.remainingBurstTime = 0;
        proc.totalCpuTime = 0;
        proc.totalWaitTime = 0;
        proc.totalTurnaroundTime = 0;
        proc.contextSwitches = 0;
        proc.preemptions = 0;
        proc.readyTimeSet = false;
        proc.tau = std::ceil(1.0 / params.lambda);
    }
    totalCpuBusyTime = 0;
    sumCpuBurstTimeCpuBound = sumCpuBurstTimeIOBound = 0.0;
    countCpuBurstCpuBound = countCpuBurstIOBound = 0;
    sumIOBurstTimeCpuBound = sumIOBurstTimeIOBound = 0.0;
    countIOBurstCpuBound = countIOBurstIOBound = 0;
    sumWaitTimeCpuBound = sumWaitTimeIOBound = 0.0;
    countWaitCpuBound = countWaitIOBound = 0;
    sumTurnaroundCpuBound = sumTurnaroundIOBound = 0.0;
    countTurnaroundCpuBound = countTurnaroundIOBound = 0;
    contextSwitchesCpuBound = contextSwitchesIOBound = 0;
    preemptionsCpuBound = preemptionsIOBound = 0;
}

void Simulator::runSimulation(SchedulingAlgo algo) {
    resetSimulation();
    for (auto &proc : processes)
        scheduleEvent(Event(proc.arrivalTime, EventType::PROCESS_ARRIVAL, &proc));
    
    while (!eventQueue.empty()) {
        if (readyQueue.empty() && !eventQueue.empty() && eventQueue.top().time > currentTime)
            currentTime = eventQueue.top().time;
        Event event = eventQueue.top();
        eventQueue.pop();
        processEvent(event, algo);
    }
    logEvent(currentTime, "Simulator ended for " +
            std::string((algo == SchedulingAlgo::FCFS) ? "FCFS" :
                        (algo == SchedulingAlgo::SJF) ? "SJF" :
                        (algo == SchedulingAlgo::SRT) ? "SRT" : "RR"));
}

void Simulator::printSimStatistics(SchedulingAlgo algo, std::ostream &out) {
    out << "Algorithm ";
    switch (algo) {
        case SchedulingAlgo::FCFS: out << "FCFS"; break;
        case SchedulingAlgo::SJF:  out << "SJF";  break;
        case SchedulingAlgo::SRT:  out << "SRT";  break;
        case SchedulingAlgo::RR:   out << "RR";   break;
    }
    out << std::endl;
    
    int simulationTime = currentTime + (params.tcs / 2);
    double cpuUtilization = (simulationTime > 0) ? (totalCpuBusyTime / (double) simulationTime * 100.0) : 0.0;
    
    double avgWaitCpu = (countWaitCpuBound > 0) ? (sumWaitTimeCpuBound / countWaitCpuBound) : 0.0;
    double avgWaitIO  = (countWaitIOBound  > 0) ? (sumWaitTimeIOBound / countWaitIOBound)  : 0.0;
    double overallAvgWait = ((countWaitCpuBound + countWaitIOBound) > 0) ?
        ((sumWaitTimeCpuBound + sumWaitTimeIOBound) / (countWaitCpuBound + countWaitIOBound)) : 0.0;
    double avgTurnCpu = (countTurnaroundCpuBound > 0) ? (sumTurnaroundCpuBound / countTurnaroundCpuBound) : 0.0;
    double avgTurnIO  = (countTurnaroundIOBound > 0)  ? (sumTurnaroundIOBound / countTurnaroundIOBound)  : 0.0;
    double overallAvgTurn = ((countTurnaroundCpuBound + countTurnaroundIOBound) > 0) ?
        ((sumTurnaroundCpuBound + sumTurnaroundIOBound) / (countTurnaroundCpuBound + countTurnaroundIOBound)) : 0.0;
    int overallCS = contextSwitchesCpuBound + contextSwitchesIOBound;
    int overallPreemptions = preemptionsCpuBound + preemptionsIOBound;
    
    out << "-- CPU utilization: " << std::fixed << std::setprecision(3) << cpuUtilization << "%" << std::endl;
    out << "-- CPU-bound average wait time: " << avgWaitCpu << " ms" << std::endl;
    out << "-- I/O-bound average wait time: " << avgWaitIO << " ms" << std::endl;
    out << "-- overall average wait time: " << overallAvgWait << " ms" << std::endl;
    out << "-- CPU-bound average turnaround time: " << avgTurnCpu << " ms" << std::endl;
    out << "-- I/O-bound average turnaround time: " << avgTurnIO << " ms" << std::endl;
    out << "-- overall average turnaround time: " << overallAvgTurn << " ms" << std::endl;
    out << "-- CPU-bound number of context switches: " << contextSwitchesCpuBound << std::endl;
    out << "-- I/O-bound number of context switches: " << contextSwitchesIOBound << std::endl;
    out << "-- overall number of context switches: " << overallCS << std::endl;
    out << "-- CPU-bound number of preemptions: " << preemptionsCpuBound << std::endl;
    out << "-- I/O-bound number of preemptions: " << preemptionsIOBound << std::endl;
    out << "-- overall number of preemptions: " << overallPreemptions << std::endl;
    
    if (algo == SchedulingAlgo::RR) {
        double percentCpu = 0.0;
        double percentIO = 0.0;
        double overallPercent = 0.0;
        out << "-- CPU-bound percentage of CPU bursts completed within one time slice: " << percentCpu << "%" << std::endl;
        out << "-- I/O-bound percentage of CPU bursts completed within one time slice: " << percentIO << "%" << std::endl;
        out << "-- overall percentage of CPU bursts completed within one time slice: " << overallPercent << "%" << std::endl;
    }
}

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
    
    std::vector<Process> processSet;
    {
        int totalProcesses = simParams.n;
        int cpuBoundCount = simParams.ncpu;
        std::vector<std::string> pids;
        for (char letter = 'A'; letter <= 'Z'; letter++) {
            for (int num = 0; num < 10; num++) {
                pids.push_back(std::string(1, letter) + std::to_string(num));
                if (pids.size() == static_cast<size_t>(totalProcesses))
                    break;
            }
            if (pids.size() == static_cast<size_t>(totalProcesses))
                break;
        }
        my_srand48(simParams.seed);
        for (int i = 0; i < totalProcesses; i++) {
            Process proc;
            proc.pid = pids[i];
            double r = my_drand48();
            double exp_val = -std::log(r) / simParams.lambda;
            proc.arrivalTime = static_cast<int>(std::floor(exp_val));
            double r2 = my_drand48();
            int numBursts = static_cast<int>(std::ceil(r2 * 32));
            proc.tau = std::ceil(1.0 / simParams.lambda);
            proc.isCPUBound = (i < cpuBoundCount);
            for (int b = 0; b < numBursts; b++) {
                double r_cpu = my_drand48();
                double burst_val = -std::log(r_cpu) / simParams.lambda;
                int cpuBurst = static_cast<int>(std::ceil(burst_val));
                if (proc.isCPUBound)
                    cpuBurst *= 4;
                proc.cpuBursts.push_back(cpuBurst);
                if (b < numBursts - 1) {
                    double r_io = my_drand48();
                    double io_val = -std::log(r_io) / simParams.lambda;
                    int ioBurst = static_cast<int>(std::ceil(io_val));
                    if (proc.isCPUBound)
                        ioBurst = (ioBurst > 8) ? ioBurst / 8 : 1;
                    else {
                        ioBurst *= 8;
                        if (ioBurst > simParams.bound)
                            ioBurst = simParams.bound;
                    }
                    proc.ioBursts.push_back(ioBurst);
                }
            }
            processSet.push_back(proc);
        }
    }
    
    std::cout << "<<< -- process set (n=" << simParams.n << ") with " << simParams.ncpu 
              << " CPU-bound process" << ((simParams.ncpu == 1) ? "" : "es") << std::endl;
    
    Simulator simFCFS(simParams, processSet);
    simFCFS.runSimulation(SchedulingAlgo::FCFS);

    Simulator simSJF(simParams, processSet);
    simSJF.runSimulation(SchedulingAlgo::SJF);

    Simulator simSRT(simParams, processSet);
    simSRT.runSimulation(SchedulingAlgo::SRT);

    Simulator simRR(simParams, processSet);
    simRR.runSimulation(SchedulingAlgo::RR);
    
    std::ofstream simout("simout.txt");
    if (!simout) {
        std::cerr << "ERROR: Could not open simout.txt for writing" << std::endl;
        return 1;
    }
    simout << "-- number of processes: " << simParams.n << std::endl;
    simout << "-- number of CPU-bound processes: " << simParams.ncpu << std::endl;
    simout << "-- number of I/O-bound processes: " << (simParams.n - simParams.ncpu) << std::endl;
    
    simout << std::endl;
    simFCFS.printSimStatistics(SchedulingAlgo::FCFS, simout);
    simout << std::endl;
    simSJF.printSimStatistics(SchedulingAlgo::SJF, simout);
    simout << std::endl;
    simSRT.printSimStatistics(SchedulingAlgo::SRT, simout);
    simout << std::endl;
    simRR.printSimStatistics(SchedulingAlgo::RR, simout);
    simout.close();
    
    return 0;
}
