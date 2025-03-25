#include "opsys.h"
#include "process.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <list>
#include <iomanip>
#include <queue>
#include <unordered_set>

// -----------------------------------------------------------------------------
// Helper Functions for Printing Queues
// -----------------------------------------------------------------------------

/**
 * @brief Print a queue of Process pointers
 * 
 * @param readyQueue Queue of Process pointers
 */
void printQueue(const std::queue<Process*>& readyQueue) {
    std::queue<Process*> q = readyQueue;
    std::cout << "[Q";
    if (!q.empty()) {
        while (!q.empty()) {
            std::cout << " " << q.front()->id;
            q.pop();
        }
    } else {
        std::cout << " empty";
    }
    std::cout << "]\n";
}

/**
 * @brief Print a priority queue of Process pointers
 * 
 * @tparam T Type of the priority queue
 * @param readyQueue Queue of Process pointers
 */
template<typename T>
void printPriorityQueue(T readyQueue) {
    std::cout << "[Q";
    if (readyQueue.empty())
        std::cout << " empty";
    else {
        while (!readyQueue.empty()) {
            std::cout << " " << readyQueue.top()->id;
            readyQueue.pop();
        }
    }
    std::cout << "]\n";
}

// -----------------------------------------------------------------------------
// FCFS (First-Come First-Served) Functions
// -----------------------------------------------------------------------------

// Arrival function
void OpSys::processArrivalFCFS(int currentTime) {
    Process* proc = unarrived.top();
    unarrived.pop();
    readyFCFS.push(proc);
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " arrived; added to ready queue ";
        printQueue(readyFCFS);
    }
}

// Switch-in function
void OpSys::startSwitchInFCFS(int currentTime) {
    switchingToRun = readyFCFS.front();
    readyFCFS.pop();
    switchingToRun->lastSwitchTime = currentTime;
}

// Start CPU usage
void OpSys::startCpuUsageFCFS(int currentTime) {
    Process* proc = switchingToRun;
    switchingToRun = nullptr;
    running = proc;
    proc->waitBurst(currentTime);
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " started using the CPU for " << proc->getT() << "ms burst ";
        printQueue(readyFCFS);
    }
}

// Switch-out CPU function
void OpSys::switchOutCpuFCFS(int currentTime) {
    Process* proc = running;
    running = nullptr;
    proc->updateProcess(currentTime);
    int burstsLeft = proc->getCpuBurstsLeft();
    if (burstsLeft == 0) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " terminated ";
        finished.push_back(proc);
        unfinished.erase(proc);
        printQueue(readyFCFS);
    } else {
        proc->waitBurst(currentTime + contextSwitchTime / 2);
        if (!TRUNCATE || currentTime < TRUNC_TIME) {
            std::cout << "time " << currentTime << "ms: Process " << proc->id 
                      << " completed a CPU burst; " << burstsLeft 
                      << " burst" << (burstsLeft == 1 ? "" : "s") << " to go ";
            printQueue(readyFCFS);
            std::cout << "time " << currentTime << "ms: Process " << proc->id 
                      << " switching out of CPU; blocking on I/O until time " 
                      << proc->burstCompletionTime() << "ms ";
            printQueue(readyFCFS);
        }
        waiting.push(proc);
    }
    switchingToIO = proc;
    proc->lastSwitchTime = currentTime;
}

// Complete I/O function
void OpSys::completeIOFCFS(int currentTime) {
    Process* proc = waiting.top();
    waiting.pop();
    proc->updateProcess(currentTime);
    readyFCFS.push(proc);
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " completed I/O; added to ready queue ";
        printQueue(readyFCFS);
    }
    proc->finishBurst();
}

// Run scheduler
void OpSys::runFCFSScheduler() {
    switchingToRun = nullptr;
    switchingToIO = nullptr;
    switchingToReady = nullptr;
    time = 0;
    std::cout << "time " << time << "ms: Simulator started for FCFS [Q empty]\n";
    
    while (!unfinished.empty()) {
        std::priority_queue<Action, std::vector<Action>, CompAction> actionQueue;
        if (switchingToIO != nullptr)
            actionQueue.push({ switchingToIO->lastSwitchTime + contextSwitchTime / 2, &OpSys::finishIOSwitchOut, 0 });
 
        if (running == nullptr) {
            if (switchingToRun == nullptr) {
                if (switchingToIO == nullptr && !readyFCFS.empty())
                    actionQueue.push({ time, &OpSys::startSwitchInFCFS, 10 });
            } else {
                actionQueue.push({ switchingToRun->lastSwitchTime + contextSwitchTime / 2, &OpSys::startCpuUsageFCFS, 2 });
            }
        } else {
            actionQueue.push({ running->burstCompletionTime(), &OpSys::switchOutCpuFCFS, 1 });
        }
        if (!waiting.empty())
            actionQueue.push({ waiting.top()->burstCompletionTime(), &OpSys::completeIOFCFS, 3 });
        if (!unarrived.empty())
            actionQueue.push({ unarrived.top()->arrival_time, &OpSys::processArrivalFCFS, 4 });
        time = actionQueue.top().time;
        (this->*(actionQueue.top().func))(time);
    }
    time += contextSwitchTime / 2;
    std::cout << "time " << time << "ms: Simulator ended for FCFS [Q empty]\n";    
    
    std::ofstream simout("simout.txt", std::ios_base::app);
    simout << "Algorithm FCFS\n";
    printStats(simout);
    simout << "\n";
}

// -----------------------------------------------------------------------------
// Round Robin (RR) Functions
// -----------------------------------------------------------------------------

// Arrival function
void OpSys::processArrivalRR(int currentTime) {
    Process* proc = unarrived.top();
    unarrived.pop();
    readyRR.push(proc);
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " arrived; added to ready queue ";
        printQueue(readyRR);
    }
}

// Switch-in function
void OpSys::startSwitchInRR(int currentTime) {
    switchingToRun = readyRR.front();
    readyRR.pop();
    switchingToRun->lastSwitchTime = currentTime;
}

// Start CPU usage
void OpSys::startCpuUsageRR(int currentTime) {
    Process* proc = switchingToRun;
    switchingToRun = nullptr;
    running = proc;
    proc->lastCpuBurstStart = currentTime;
    proc->waitBurst(currentTime);
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        if (proc->time_remaining < proc->getT())
            std::cout << "time " << currentTime << "ms: Process " << proc->id 
                      << " started using the CPU for remaining " << proc->time_remaining 
                      << "ms of " << proc->getT() << "ms burst ";
        else
            std::cout << "time " << currentTime << "ms: Process " << proc->id 
                      << " started using the CPU for " << proc->getT() << "ms burst ";
        printQueue(readyRR);
    }
}

// Time-slice expiration
void OpSys::timeSliceExpirationRR(int currentTime) {
    Process* proc = running;
    if (readyRR.empty()) {
        proc->time_remaining -= timeSlice;
        if (!TRUNCATE || currentTime < TRUNC_TIME)
            std::cout << "time " << currentTime << "ms: Time slice expired; no preemption because ready queue is empty [Q empty]\n";
        proc->lastCpuBurstStart = currentTime;
        return;
    }
    proc->preempt(timeSlice);
    running = nullptr;
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        std::cout << "time " << currentTime << "ms: Time slice expired; preempting process " <<  proc->id 
                  << " with " << proc->time_remaining << "ms remaining ";
        printQueue(readyRR);
    }
    switchingToReady = proc;
    proc->lastSwitchTime = currentTime;
}

// Switch-out CPU function
void OpSys::switchOutCpuRR(int currentTime) {
    Process* proc = running;
    running = nullptr;
    proc->time_remaining = 0;
    proc->updateProcess(currentTime);
    int burstsLeft = proc->getCpuBurstsLeft();
    if (burstsLeft == 0) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " terminated ";
        unfinished.erase(proc);
        printQueue(readyRR);
    } else {
        proc->waitBurst(currentTime + contextSwitchTime / 2);
        if (!TRUNCATE || currentTime < TRUNC_TIME) {
            std::cout << "time " << currentTime << "ms: Process " << proc->id 
                      << " completed a CPU burst; " << burstsLeft 
                      << " burst" << (burstsLeft == 1 ? "" : "s") << " to go ";
            printQueue(readyRR);
            std::cout << "time " << currentTime << "ms: Process " << proc->id 
                      << " switching out of CPU; blocking on I/O until time " 
                      << proc->burstCompletionTime() << "ms ";
            printQueue(readyRR);
        }
        waiting.push(proc);
    }
    switchingToIO = proc;
    proc->lastSwitchTime = currentTime;
}

// Finish preempt switch-out
void OpSys::finishPreemptSwitchOutRR(int currentTime) {
    readyRR.push(switchingToReady);
    switchingToReady = nullptr;
    if (currentTime == 0) return;
}

// Complete I/O function
void OpSys::completeIORR(int currentTime) {
    Process* proc = waiting.top();
    waiting.pop();
    proc->updateProcess(currentTime);
    readyRR.push(proc);
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " completed I/O; added to ready queue ";
        printQueue(readyRR);
    }
    proc->finishBurst();
}

// Run scheduler
void OpSys::runRoundRobinScheduler() {
    switchingToRun = nullptr;
    switchingToIO = nullptr;
    switchingToReady = nullptr;
    time = 0;
    std::cout << "time " << time << "ms: Simulator started for RR [Q empty]\n";
    
    while (!unfinished.empty()) {
        std::priority_queue<Action, std::vector<Action>, CompAction> actionQueue;
        if (switchingToIO != nullptr)
            actionQueue.push({ switchingToIO->lastSwitchTime + contextSwitchTime / 2, &OpSys::finishIOSwitchOut, 0 });
 
        if (running == nullptr) {
            if (switchingToReady != nullptr)
                actionQueue.push({ switchingToReady->lastSwitchTime + contextSwitchTime / 2, &OpSys::finishPreemptSwitchOutRR, 0 });
            else {
                if (switchingToRun == nullptr) {
                    if (switchingToIO == nullptr && !readyRR.empty())
                        actionQueue.push({ time, &OpSys::startSwitchInRR, 10 });
                } else {
                    actionQueue.push({ switchingToRun->lastSwitchTime + contextSwitchTime / 2, &OpSys::startCpuUsageRR, 2 });
                }
            }
        } else {
            if ((running->lastCpuBurstStart + timeSlice) < running->burstCompletionTime())
                actionQueue.push({ (running->lastCpuBurstStart + timeSlice), &OpSys::timeSliceExpirationRR, 1 });
            else
                actionQueue.push({ running->burstCompletionTime(), &OpSys::switchOutCpuRR, 1 });
        }
        if (!waiting.empty())
            actionQueue.push({ waiting.top()->burstCompletionTime(), &OpSys::completeIORR, 3 });
        if (!unarrived.empty())
            actionQueue.push({ unarrived.top()->arrival_time, &OpSys::processArrivalRR, 4 });
        time = actionQueue.top().time;
        (this->*(actionQueue.top().func))(time);
    }
    time += contextSwitchTime / 2;
    std::cout << "time " << time << "ms: Simulator ended for RR [Q empty]\n";    
    
    std::ofstream simout("simout.txt", std::ios_base::app);
    simout << "Algorithm RR\n";
    printStats(simout);
    printRRStats(simout);
}

// -----------------------------------------------------------------------------
// Shortest Job First (SJF) Functions
// -----------------------------------------------------------------------------

// Arrival function
void OpSys::processArrivalSJF(int currentTime) {
    Process* proc = unarrived.top();
    unarrived.pop();
    readySJF.push(proc);
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " (tau " << proc->getTau() << "ms) arrived; added to ready queue ";
        printPriorityQueue(readySJF);
    }
}

// Switch-in function
void OpSys::startSwitchInSJF(int currentTime) {
    switchingToRun = readySJF.top();
    readySJF.pop();
    switchingToRun->lastSwitchTime = currentTime;
}

// Start CPU usage
void OpSys::startCpuUsageSJF(int currentTime) {
    Process* proc = switchingToRun;
    switchingToRun = nullptr;
    running = proc;
    proc->waitBurst(currentTime);
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " (tau " << proc->getTau() << "ms) started using the CPU for " 
                  << proc->getT() << "ms burst ";
        printPriorityQueue(readySJF);
    }
}

// Switch-out CPU function
void OpSys::switchOutCpuSJF(int currentTime) {
    Process* proc = running;
    running = nullptr;
    int oldTau = proc->getTau();
    proc->updateProcess(currentTime);
    int burstsLeft = proc->getCpuBurstsLeft();
    if (burstsLeft == 0) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id << " terminated ";
        unfinished.erase(proc);
        printPriorityQueue(readySJF);
    } else {
        proc->waitBurst(currentTime + contextSwitchTime / 2);
        if (!TRUNCATE || currentTime < TRUNC_TIME) {
            std::cout << "time " << currentTime << "ms: Process " << proc->id 
                      << " (tau " << oldTau << "ms) completed a CPU burst; " << burstsLeft 
                      << " burst" << (burstsLeft == 1 ? "" : "s") << " to go ";
            printPriorityQueue(readySJF);
            std::cout << "time " << currentTime << "ms: Recalculated tau for process " << proc->id 
                      << ": old tau " << oldTau << "ms ==> new tau " << proc->getTau() << "ms ";
            printPriorityQueue(readySJF);
            std::cout << "time " << currentTime << "ms: Process " << proc->id 
                      << " switching out of CPU; blocking on I/O until time " 
                      << proc->burstCompletionTime() << "ms ";
            printPriorityQueue(readySJF);
        }
        waiting.push(proc);
    }
    switchingToIO = proc;
    proc->lastSwitchTime = currentTime;
}

// Complete I/O function
void OpSys::completeIOSJF(int currentTime) {
    Process* proc = waiting.top();
    waiting.pop();
    proc->updateProcess(currentTime);
    readySJF.push(proc);
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " (tau " << proc->getTau() << "ms) completed I/O; added to ready queue ";
        printPriorityQueue(readySJF);
    }
    proc->finishBurst();
}

// Run scheduler
void OpSys::runSJFScheduler() {
    switchingToRun = nullptr;
    switchingToIO = nullptr;
    switchingToReady = nullptr;
    time = 0;
    std::cout << "time " << time << "ms: Simulator started for SJF [Q empty]\n";
    
    while (!unfinished.empty()) {
        std::priority_queue<Action, std::vector<Action>, CompAction> actionQueue;
        if (switchingToIO != nullptr)
            actionQueue.push({ switchingToIO->lastSwitchTime + contextSwitchTime / 2, &OpSys::finishIOSwitchOut, 0 });
 
        if (running == nullptr) {
            if (switchingToRun == nullptr) {
                if (switchingToIO == nullptr && !readySJF.empty())
                    actionQueue.push({ time, &OpSys::startSwitchInSJF, 10 });
            } else {
                actionQueue.push({ switchingToRun->lastSwitchTime + contextSwitchTime / 2, &OpSys::startCpuUsageSJF, 2 });
            }
        } else {
            actionQueue.push({ running->burstCompletionTime(), &OpSys::switchOutCpuSJF, 1 });
        }
        if (!waiting.empty())
            actionQueue.push({ waiting.top()->burstCompletionTime(), &OpSys::completeIOSJF, 3 });
        if (!unarrived.empty())
            actionQueue.push({ unarrived.top()->arrival_time, &OpSys::processArrivalSJF, 4 });
        time = actionQueue.top().time;
        (this->*(actionQueue.top().func))(time);
    }
    time += contextSwitchTime / 2;
    std::cout << "time " << time << "ms: Simulator ended for SJF [Q empty]\n";
    
    std::ofstream simout("simout.txt", std::ios_base::app);
    simout << "Algorithm SJF\n";
    printStats(simout);
    simout << "\n";  
}

// -----------------------------------------------------------------------------
// Shortest Remaining Time (SRT) Functions
// -----------------------------------------------------------------------------

bool OpSys::shouldPreempt(int currentTime, Process* cur, Process* top) { 
    // This function returns true if the current process should be preempted by the process on top of the SRT queue.
    return (cur->tau_remaining - (currentTime - cur->lastCpuBurstStart)) > top->tau_remaining;
}

// Arrival function
void OpSys::processArrivalSRT(int currentTime) {
    Process* proc = unarrived.top();
    unarrived.pop();
    readySRT.push(proc);
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " (tau " << proc->getTau() << "ms) arrived; added to ready queue ";
        printPriorityQueue(readySRT);
    }
}

// Switch-in function
void OpSys::startSwitchInSRT(int currentTime) {
    switchingToRun = readySRT.top();
    readySRT.pop();
    switchingToRun->lastSwitchTime = currentTime;
}

// Start CPU usage
void OpSys::startCpuUsageSRT(int currentTime) {
    Process* proc = switchingToRun;
    switchingToRun = nullptr;
    running = proc;
    proc->lastCpuBurstStart = currentTime;
    proc->waitBurst(currentTime);
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        if (proc->time_remaining < proc->getT())
            std::cout << "time " << currentTime << "ms: Process " << proc->id 
                      << " (tau " << proc->getTau() << "ms) started using the CPU for remaining " 
                      << proc->time_remaining << "ms of " << proc->getT() << "ms burst ";
        else
            std::cout << "time " << currentTime << "ms: Process " << proc->id 
                      << " (tau " << proc->getTau() << "ms) started using the CPU for " 
                      << proc->getT() << "ms burst ";
        printPriorityQueue(readySRT);
    }
}

// Switch-out CPU function
void OpSys::switchOutCpuSRT(int currentTime) {
    Process* proc = running;
    running = nullptr;
    int oldTau = proc->getTau();
    proc->updateProcess(currentTime);
    int burstsLeft = proc->getCpuBurstsLeft();
    if (burstsLeft == 0) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id << " terminated ";
        unfinished.erase(proc);
        printPriorityQueue(readySRT);
    } else {
        proc->waitBurst(currentTime + contextSwitchTime / 2);
        if (!TRUNCATE || currentTime < TRUNC_TIME) {
            std::cout << "time " << currentTime << "ms: Process " << proc->id 
                      << " (tau " << oldTau << "ms) completed a CPU burst; " << burstsLeft 
                      << " burst" << (burstsLeft == 1 ? "" : "s") << " to go ";
            printPriorityQueue(readySRT);
            std::cout << "time " << currentTime << "ms: Recalculated tau for process " << proc->id 
                      << ": old tau " << oldTau << "ms ==> new tau " << proc->getTau() << "ms ";
            printPriorityQueue(readySRT);
            std::cout << "time " << currentTime << "ms: Process " << proc->id 
                      << " switching out of CPU; blocking on I/O until time " 
                      << proc->burstCompletionTime() << "ms ";
            printPriorityQueue(readySRT);
        }
        waiting.push(proc);
    }
    switchingToIO = proc;
    proc->lastSwitchTime = currentTime;
}

// Complete I/O function
void OpSys::completeIOSRT(int currentTime) {
    Process* proc = waiting.top();
    waiting.pop();
    proc->updateProcess(currentTime);
    readySRT.push(proc);
    if (running != nullptr) {
        if (shouldPreempt(currentTime, running, readySRT.top())) {
            running->preempt(currentTime - running->lastCpuBurstStart);
            if (!TRUNCATE || currentTime < TRUNC_TIME) {
                std::cout << "time " << currentTime << "ms: Process " << proc->id 
                          << " (tau " << proc->getTau() << "ms) completed I/O; preempting " 
                          << running->id << " (predicted remaining time " << running->tau_remaining << "ms) ";
                printPriorityQueue(readySRT);
            }
            switchingToReady = running;
            running->lastSwitchTime = currentTime;
            running = nullptr;
            proc->finishBurst();
            return;
        }
    }
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " (tau " << proc->getTau() << "ms) completed I/O; added to ready queue ";
        printPriorityQueue(readySRT);
    }
    proc->finishBurst();
}

// Finish preempt switch-out
void OpSys::finishPreemptSwitchOutSRT(int currentTime) {
    readySRT.push(switchingToReady);
    switchingToReady = nullptr;
    if (currentTime == 0) return;
}

// Preempt now
void OpSys::preemptNowSRT(int currentTime) {
    Process* proc = readySRT.top();
    running->preempt(currentTime - running->lastCpuBurstStart);
    if (!TRUNCATE || currentTime < TRUNC_TIME) {
        std::cout << "time " << currentTime << "ms: Process " << proc->id 
                  << " (tau " << proc->getTau() << "ms) will preempt " << running->id << " ";
        printPriorityQueue(readySRT);
    }
    switchingToReady = running;
    running->lastSwitchTime = currentTime;
    running = nullptr;
}

// Run scheduler
void OpSys::runSRTScheduler() {
    switchingToRun = nullptr;
    switchingToIO = nullptr;
    switchingToReady = nullptr;
    time = 0;
    std::cout << "time " << time << "ms: Simulator started for SRT [Q empty]\n";
    
    while (!unfinished.empty()) {
        std::priority_queue<Action, std::vector<Action>, CompAction> actionQueue;
        if (switchingToIO != nullptr)
            actionQueue.push({ switchingToIO->lastSwitchTime + contextSwitchTime / 2, &OpSys::finishIOSwitchOut, 0 });
 
        if (running == nullptr) {
            if (switchingToReady != nullptr)
                actionQueue.push({ switchingToReady->lastSwitchTime + contextSwitchTime / 2, &OpSys::finishPreemptSwitchOutSRT, 0 });
            else {
                if (switchingToRun == nullptr) {
                    if (switchingToIO == nullptr && !readySRT.empty())
                        actionQueue.push({ time, &OpSys::startSwitchInSRT, 10 });
                } else {
                    actionQueue.push({ switchingToRun->lastSwitchTime + contextSwitchTime / 2, &OpSys::startCpuUsageSRT, 2 });
                }
            }
        } else {
            actionQueue.push({ running->burstCompletionTime(), &OpSys::switchOutCpuSRT, 1 });
            if (!readySRT.empty() && shouldPreempt(time, running, readySRT.top()))
                actionQueue.push({ time, &OpSys::preemptNowSRT, -1 });
        }
        if (!waiting.empty())
            actionQueue.push({ waiting.top()->burstCompletionTime(), &OpSys::completeIOSRT, 3 });
        if (!unarrived.empty())
            actionQueue.push({ unarrived.top()->arrival_time, &OpSys::processArrivalSRT, 4 });
        time = actionQueue.top().time;
        (this->*(actionQueue.top().func))(time);
    }
    time += contextSwitchTime / 2;
    std::cout << "time " << time << "ms: Simulator ended for SRT [Q empty]\n";   
    
    std::ofstream simout("simout.txt", std::ios_base::app);
    simout << "Algorithm SRT\n";
    printStats(simout);
    simout << "\n";
}

// -----------------------------------------------------------------------------
// Statistics Functions
// -----------------------------------------------------------------------------

void OpSys::printStats(std::ofstream& simout) {
    int cpuSwitches = 0, ioSwitches = 0;
    int cpuPreempts = 0, ioPreempts = 0;
    int overallCpuTime = 0;
    int cpuTurnaround = 0, ioTurnaround = 0; 
    int cpuBursts = 0, ioBursts = 0;
    int totalCpuWait = 0, totalIoWait = 0;
    
    for (Process* proc : finished) {
        overallCpuTime += proc->total_cpu_time;
        if (proc->is_cpu_bound) {
            cpuSwitches += proc->num_switches;
            cpuPreempts += proc->num_preempts;
            cpuTurnaround += proc->total_turnaround;
            cpuBursts += proc->num_cpu_bursts;
            totalCpuWait += proc->total_turnaround - proc->getTotalCpuTime() - (proc->num_preempts * contextSwitchTime);
        } else {
            ioSwitches += proc->num_switches;
            ioPreempts += proc->num_preempts;
            ioTurnaround += proc->total_turnaround;
            ioBursts += proc->num_cpu_bursts;
            totalIoWait += proc->total_turnaround - proc->getTotalCpuTime() - (proc->num_preempts * contextSwitchTime);
        }  
    }
    // Calculate statistics
    double utilization = std::ceil((overallCpuTime / (double)time) * 100000) / 1000;
    double avgCpuTurn = std::ceil(((double)cpuTurnaround / cpuBursts) * 1000) / 1000 + (contextSwitchTime / 2);
    double avgIoTurn = std::ceil(((double)ioTurnaround / ioBursts) * 1000) / 1000 + (contextSwitchTime / 2);
    double avgTurn = std::ceil(((double)(ioTurnaround + cpuTurnaround) / (ioBursts + cpuBursts)) * 1000) / 1000 + (contextSwitchTime / 2);
    double avgWaitCpu = std::ceil(((double)totalCpuWait / cpuBursts) * 1000) / 1000 - (contextSwitchTime / 2);
    double avgWaitIo = std::ceil(((double)totalIoWait / ioBursts) * 1000) / 1000 - (contextSwitchTime / 2);
    double avgWait = std::ceil(((double)(totalCpuWait + totalIoWait) / (ioBursts + cpuBursts)) * 1000) / 1000 - (contextSwitchTime / 2);

    // Print statistics
    simout << "-- CPU utilization: " << std::fixed << std::setprecision(3) << utilization << "%\n";
    simout << "-- CPU-bound average wait time: " << std::setprecision(3) << avgWaitCpu << " ms\n";
    simout << "-- I/O-bound average wait time: " << std::setprecision(3) << avgWaitIo << " ms\n";
    simout << "-- overall average wait time: " << std::setprecision(3) << avgWait << " ms\n";  
    simout << "-- CPU-bound average turnaround time: " << std::setprecision(3) << avgCpuTurn << " ms\n";
    simout << "-- I/O-bound average turnaround time: " << std::setprecision(3) << avgIoTurn << " ms\n";
    simout << "-- overall average turnaround time: " << avgTurn << " ms\n";
    simout << "-- CPU-bound number of context switches: " << cpuSwitches << "\n";
    simout << "-- I/O-bound number of context switches: " << ioSwitches << "\n";
    simout << "-- overall number of context switches: " << (ioSwitches + cpuSwitches) << "\n";
    simout << "-- CPU-bound number of preemptions: " << cpuPreempts << "\n";
    simout << "-- I/O-bound number of preemptions: " << ioPreempts << "\n";
    simout << "-- overall number of preemptions: " << (ioPreempts + cpuPreempts) << "\n";
}

void OpSys::printRRStats(std::ofstream& simout) {
    int cpuWithinTS = 0, ioWithinTS = 0;
    int cpuBursts = 0, ioBursts = 0;
    for (Process* proc : finished) {
        int burstsWithin = 0;
        for (int i = 0; i < proc->num_total_bursts; i += 2) {
            if (proc->burst_times[i] <= timeSlice)
                burstsWithin++;
        }
        if (proc->is_cpu_bound) { 
            cpuWithinTS += burstsWithin; 
            cpuBursts += proc->num_cpu_bursts; 
        } else { 
            ioWithinTS += burstsWithin; 
            ioBursts += proc->num_cpu_bursts;
        }
    }
    
    // Calculate statistics
    double cpuPercentage = std::ceil(((double)cpuWithinTS / cpuBursts) * 100000) / 1000;
    double ioPercentage = std::ceil(((double)ioWithinTS / ioBursts) * 100000) / 1000;
    double overallPercentage = std::ceil(((double)(ioWithinTS + cpuWithinTS) / (ioBursts + cpuBursts)) * 100000) / 1000;

    // Print statistics
    simout << "-- CPU-bound percentage of CPU bursts completed within one time slice: " << std::setprecision(3) << cpuPercentage << "%\n";
    simout << "-- I/O-bound percentage of CPU bursts completed within one time slice: " << std::setprecision(3) << ioPercentage << "%\n";
    simout << "-- overall percentage of CPU bursts completed within one time slice: " << std::setprecision(3) << overallPercentage << "%\n";
}

// -----------------------------------------------------------------------------
// Process Member Functions
// -----------------------------------------------------------------------------
// Update process
void Process::updateProcess(int currentTime) {
    burst_index++;
    if (onCPUBurst()) {
        prev_t = t;
        t = burst_times[burst_index];
        time_remaining = t;
        total_cpu_time += t;
        num_switches++;
        start_turnaround = currentTime;
    } else { // On I/O burst
        prev_tau = tau;
        tau = std::ceil((alpha * burst_times[burst_index - 1]) + ((1.0 - alpha) * prev_tau));
        tau_remaining = tau;
        if (burst_index < num_total_bursts)
            time_remaining = burst_times[burst_index];
        total_turnaround += (currentTime - start_turnaround);
    }
}

// Preempt process
void Process::preempt(int elapsedTime) {
    num_preempts++;
    num_switches++;  
    time_remaining -= elapsedTime;
    tau_remaining -= elapsedTime;
}

// Reset process
void Process::reset() {
    burst_index = 0;
    burst_completion_time = 0;
    t = burst_times[0];
    prev_t = t;
    tau = tau_0;
    prev_tau = tau;
    time_remaining = t;
    tau_remaining = tau;
    lastCpuBurstStart = 0;
    num_switches = 1;
    num_preempts = 0;
    total_cpu_time = t;
    start_turnaround = arrival_time;
    total_turnaround = 0;
}

// Get burst completion time
int Process::getTotalCpuTime() {
    int total_cpu = 0;
    for (int i = 0; i < num_total_bursts; i += 2)
        total_cpu += burst_times[i];
    return total_cpu;
}

// -----------------------------------------------------------------------------
// Random Exponential Function and Global Variables
// -----------------------------------------------------------------------------

int numProcesses;
int numCpuBound;
int seed;
double arrivalLambda;
int maxBurstCeiling;
int contextSwitchTime;
double alpha;
int timeSlice;

/**
 * @brief Function to get the next exponential random variable
 * 
 * @return double Double value of the next exponential random variable
 */
double nextExp() {
    int found = 0;
    double x;
    while (!found) {
        double r = drand48();
        x = -log(r) / arrivalLambda;
        if (x <= maxBurstCeiling)
            found = 1;
    }
    return x;
}

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------
/**
 * @brief Main function to run the simulation
 * 
 * @param argc Integer count of arguments
 * @param argv Character array of arguments
 * @return int Integer return value
 */
int main(int argc, char* argv[]) {
    // Check for valid arguments
    if (argc != 9) {
        std::cerr << "ERROR: Invalid arg count\n";
        exit(1);
    }

    // Set global variables
    numProcesses = atoi(argv[1]);          
    numCpuBound = atoi(argv[2]);      
    seed = atoi(argv[3]);       
    arrivalLambda = atof(argv[4]);     
    maxBurstCeiling = atoi(argv[5]);
    contextSwitchTime = atoi(argv[6]);                 
    alpha = atof(argv[7]);
    timeSlice = atoi(argv[8]);

    // Check for invalid arguments
    if (numProcesses <= 0 || numProcesses > 260) {
        std::cerr << "ERROR: Invalid process simulation count\n";
        exit(1);
    }
    if (arrivalLambda == 0) {
        std::cerr << "ERROR: Lambda cannot be zero\n";
        exit(1);
    }
    if (timeSlice <= 0 || contextSwitchTime <= 0) {
        std::cerr << "ERROR: Time values must be positive\n";
        exit(1);
    }

    // Print simulation parameters
    std::cout << "<<< -- process set (n=" << argv[1] << ") with " << argv[2]
              << (numCpuBound != 1 ? " CPU-bound processes\n" : " CPU-bound process\n")
              << "<<< -- seed=" << argv[3] << "; lambda=" << std::fixed << std::setprecision(6) 
              << arrivalLambda << "; bound=" << argv[5] << "\n";
    srand48(seed);
    // Create processes
    std::list<Process*> processes;
    float cpuCpuTotal = 0, ioCpuTotal = 0, cpuIoTotal = 0, ioIoTotal = 0;
    float numCpuCpu = 0, numIoCpu = 0, numCpuIo = 0, numIoIo = 0;
    for (int i = 0; i < numProcesses; i++) {
        // Determine if process is CPU-bound or I/O-bound
        bool isCpuBound = (i < numCpuBound);
        int processArrivalTime = floor(nextExp());
        int numCPUBursts = std::ceil(drand48() * 32);
        int numIOBursts = numCPUBursts - 1;
        int totalBursts = numCPUBursts + numIOBursts;
        int* burstTimes = new int[totalBursts];
        // Generate burst times
        for (int j = 0; j < totalBursts; j++) {
            int x = std::ceil(nextExp());
            if (isCpuBound) {
                if (j % 2 == 0) {
                    x = x * 4;
                    cpuCpuTotal += x; numCpuCpu++;
                } else {
                    cpuIoTotal += x; numCpuIo++;
                }
            } else {
                if (j % 2 == 1) {
                    x *= 8;
                    ioIoTotal += x; numIoIo++;
                } else {
                    ioCpuTotal += x; numIoCpu++;
                }
            }
            burstTimes[j] = x;
        }
        // Create process
        Process* proc = new Process();
        proc->id = new char[4];
        std::snprintf(proc->id, 4, "%c%d", 'A' + (i / 10), i % 10);
        proc->tau_0 = std::ceil(1.0 / arrivalLambda);
        proc->tau = proc->tau_0;
        proc->alpha = alpha;
        proc->t = burstTimes[0];
        proc->is_cpu_bound = isCpuBound;
        proc->num_cpu_bursts = numCPUBursts;
        proc->num_total_bursts = totalBursts;
        proc->burst_times = burstTimes;
        proc->arrival_time = processArrivalTime;
        proc->start_turnaround = processArrivalTime;
        proc->time_remaining = proc->t;
        proc->total_cpu_time = proc->t;
        proc->tau_remaining = proc->tau;

        // Add process to list
        processes.push_back(proc);
    }
    float cpuCpuAvg = (numCpuCpu != 0) ? (cpuCpuTotal / numCpuCpu) : 0;
    float cpuIoAvg = (numCpuIo != 0) ? (cpuIoTotal / numCpuIo) : 0;
    float cpuAvg = ((numCpuCpu + numIoCpu) != 0) ? std::ceil(1000 * (cpuCpuTotal + ioCpuTotal) / (numCpuCpu + numIoCpu)) / 1000 : 0;
    float ioCpuAvg = (numIoCpu != 0) ? (ioCpuTotal / numIoCpu) : 0;
    float ioIoAvg = (numIoIo != 0) ? (ioIoTotal / numIoIo) : 0;
    float ioAvg = ((numCpuIo + numIoIo) != 0) ? std::ceil(1000 * (cpuIoTotal + ioIoTotal) / (numCpuIo + numIoIo)) / 1000 : 0;

    // Print process information
    for (Process* proc : processes) {
        if (proc->is_cpu_bound)
            std::cout << "\nCPU-bound";
        else
            std::cout << "\nI/O-bound";
        std::cout << " process " << proc->id << ": arrival time " << proc->arrival_time 
                  << "ms; " << proc->num_cpu_bursts
                  << (proc->num_cpu_bursts != 1 ? " CPU bursts\n" : " CPU burst\n");
        for (int j = 0; j < proc->num_cpu_bursts * 2 - 1; ++j) {
            if (j % 2 == 0)
                std::cout << "==> CPU burst " << proc->burst_times[j] << "ms";
            else
                std::cout << " ==> I/O burst " << proc->burst_times[j] << "ms\n";
        }
        std::cout << "\n";
    }

    // Print simulation information
    std::ofstream simout("simout.txt");
    simout << "-- number of processes: " << numProcesses << "\n";
    simout << "-- number of CPU-bound processes: " << numCpuBound << "\n";
    simout << "-- number of I/O-bound processes: " << (numProcesses - numCpuBound) << "\n";
    simout << "-- CPU-bound average CPU burst time: " 
           << std::fixed << std::setprecision(3) << std::ceil(1000 * cpuCpuAvg) / 1000 << " ms\n";
    simout << "-- I/O-bound average CPU burst time: " 
           << std::fixed << std::setprecision(3) << std::ceil(1000 * ioCpuAvg) / 1000 << " ms\n";
    simout << "-- overall average CPU burst time: " 
           << std::fixed << std::setprecision(3) << cpuAvg << " ms\n";
    simout << "-- CPU-bound average I/O burst time: " 
           << std::fixed << std::setprecision(3) << std::ceil(1000 * cpuIoAvg) / 1000 << " ms\n";
    simout << "-- I/O-bound average I/O burst time: " 
           << std::fixed << std::setprecision(3) << std::ceil(1000 * ioIoAvg) / 1000 << " ms\n";
    simout << "-- overall average I/O burst time: " 
           << std::fixed << std::setprecision(3) << ioAvg << " ms\n\n";
    simout.close();
    std::cout << "<<< PROJECT SIMULATIONS\n<<< -- t_cs=" << contextSwitchTime 
              << "ms; alpha=" << std::fixed << std::setprecision(2) << alpha 
              << "; t_slice=" << timeSlice << "ms\n";

    // Run simulations
    OpSys* simulation = new OpSys();
    simulation->contextSwitchTime = contextSwitchTime;
    simulation->timeSlice = timeSlice;
    for (Process* proc : processes)
        simulation->unfinished.insert(proc); 
    for (Process* proc : processes)
        simulation->unarrived.push(proc); 
    simulation->runFCFSScheduler();
    std::cout << "\n";
    for (Process* proc : processes) {
        proc->reset();
        simulation->unfinished.insert(proc);
        simulation->unarrived.push(proc);
    }
    simulation->runSJFScheduler();
    std::cout << "\n";
    for (Process* proc : processes) {
        proc->reset();
        simulation->unfinished.insert(proc);
        simulation->unarrived.push(proc);
    }
    simulation->runSRTScheduler();
    std::cout << "\n";
    for (Process* proc : processes) {
        proc->reset();
        simulation->unfinished.insert(proc);
        simulation->unarrived.push(proc);
    }
    simulation->runRoundRobinScheduler();
    delete simulation;
    for (Process* proc : processes) {
        delete [] proc->id;
        delete [] proc->burst_times;
        delete proc;
    }
    return 0;
}
