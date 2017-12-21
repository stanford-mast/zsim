#include <cstring>
#include <signal.h>
#include "interval_timer.h"
#include "process_tree.h"
#include "scheduler.h"
#include "virt/time_conv.h"

void IntervalTimer::signalOrderedInsert(SignalInfo* si) {
    if (signalQueue.empty() || signalQueue.front()->phase > si->phase) {
        signalQueue.push_front(si);
    } else {
        SignalInfo* cur = signalQueue.front();
        while (cur->next && cur->next->phase <= si->phase) {
            cur = cur->next;
        }
        signalQueue.insertAfter(cur, si);
    }
}

//The only reason this is private is because it assumes the lock is already held
void IntervalTimer::scheduleSignal(pid_t osPid, int sig, uint64_t phase, uint64_t period) {
    SignalInfo* si = new SignalInfo(); //delete when dequeued
    si->osPid = osPid;
    si->sig = sig;
    si->phase = phase;
    si->period = period;
    ZSIM_TRACE(Sched, "Queueing signal %d for pid %d phase %lu (cur %lu, delta %lu)", sig, osPid, phase, curPhase, phase - curPhase);
    signalOrderedInsert(si);
}

IntervalTimer::IntervalTimer(uint32_t maxProcesses)
    : timerLock(0), curPhase(0), signalQueue(), processVTimers(), processCycles(), lastCoreCycles() {
    processCycles.resize(maxProcesses, 0);
    lastCoreCycles.resize(zinfo->numCores, 0);
}

void IntervalTimer::phaseTick() {
    futex_lock(&timerLock);
    curPhase++;

    //Update per-process cycles if there is at least one running countdown
    //TODO: Dedup this work from process_stats
    if (processVTimers.size() > 0) {
        for (uint32_t cid = 0; cid < zinfo->numCores; cid++) {
            uint32_t pid = zinfo->sched->getScheduledPid(cid);
            if (pid != std::numeric_limits<uint32_t>::max()) {
                pid = zinfo->procArray[pid]->getGroupIdx();
                uint64_t coreCycles = zinfo->cores[cid]->getCycles();
                processCycles[pid] += coreCycles - lastCoreCycles[cid];
                lastCoreCycles[cid] = coreCycles;
            }
        }
        for (auto it = processVTimers.begin(); it != processVTimers.end(); ++it) {
            VTimeInfo* vi = *it;
            if (vi->cyclesTrigger <= processCycles[vi->pidGrpIdx]) {
                switch (vi->type) {
                    case ITIMER_VIRTUAL:
                        scheduleSignal(vi->osPid, SIGVTALRM, curPhase, 0);
                        break;
                    case ITIMER_PROF:
                        scheduleSignal(vi->osPid, SIGPROF, curPhase, 0);
                        break;
                    default:
                        warn("Invalid VTimeInfo type %d", vi->type);
                        break;
                }
                // Reenable if periodic, otherwise delete
                if (vi->cyclesPeriod > 0) {
                    vi->cyclesTrigger += vi->cyclesPeriod;
                } else {
                    processVTimers.erase(it);
                    delete vi;
                }
            }
        }
    }

    //Check for pending signals that are ready to be sent
    if (!signalQueue.empty()) {
        SignalInfo* si = signalQueue.front();
        while (si && si->phase <= curPhase) {
            assert(si->phase == curPhase);
            ZSIM_TRACE(Sched, "Sending delayed signal %d to pid %d at phase %lu", si->sig, si->osPid, curPhase);

            //Note that SYS_tgkill could be used instead to target a specific thread
            syscall(SYS_kill, si->osPid, si->sig);  // XXX errno is not thread-safe for pin tools. Check

            signalQueue.pop_front();

            //Reinsert if the signal is realtime-periodic, otherwise delete
            if (si->period != 0) {
                si->phase = curPhase + si->period;
                signalOrderedInsert(si);
            } else {
                delete si;
            }
            si = signalQueue.front();
        }
    }
    futex_unlock(&timerLock);
}

unsigned int IntervalTimer::setAlarm(pid_t osPid, unsigned int seconds) {
  futex_lock(&timerLock);
    //Check if there was a prior alarm countdown and how much time it had left
    uint64_t priorPhaseRemain = 0;
    unsigned int priorSecRemain = 0;
    SignalInfo* si = signalQueue.front();
    while (si) {
        if (si->osPid == osPid && si->sig == SIGALRM) {
            priorPhaseRemain = si->phase - curPhase;
            uint64_t priorCyclesRemain = priorPhaseRemain * zinfo->phaseLength;
            priorSecRemain = (unsigned int) (cyclesToNs(priorCyclesRemain) / NSPS);
            signalQueue.remove(si);
            delete si;
            break;
        }
        si = si->next;
    }
    ZSIM_TRACE(Sched, "Last alarm for pid %d had %u seconds remaining", osPid, priorSecRemain);

    //Convert seconds into target phase then schedule
    if (seconds > 0) {
        uint64_t waitNsecs = NSPS * (uint64_t) seconds;
        uint64_t waitCycles = nsToCycles(waitNsecs);
        uint64_t waitPhases = waitCycles / zinfo->phaseLength + 1; //wait at least 1 phase
        uint64_t alarmPhase = zinfo->numPhases + waitPhases;
        scheduleSignal(osPid, SIGALRM, alarmPhase, 0);
    } else {
        ZSIM_TRACE(Sched, "Alarm blocked for pid %d", osPid);
    }

    futex_unlock(&timerLock);
    return priorSecRemain;
}

int IntervalTimer::getIntervalTimer(pid_t osPid, int type, struct itimerval* val) {
    futex_lock(&timerLock);
    int retval = 0;
    memset(val, 0, sizeof(struct itimerval));
    if (type == ITIMER_REAL) {
        SignalInfo* si = signalQueue.front();
        while (si) {
            if (si->osPid == osPid && si->sig == SIGALRM) {
                uint64_t remainCycles = (si->phase - curPhase) * zinfo->phaseLength;
                uint64_t remainNsecs = cyclesToNs(remainCycles);
                val->it_value.tv_sec = remainNsecs / NSPS;
                val->it_value.tv_usec = (remainNsecs % NSPS) / 1000;
                uint64_t periodCycles = si->period * zinfo->phaseLength;
                uint64_t periodNsecs = cyclesToNs(periodCycles);
                val->it_interval.tv_sec = periodNsecs / NSPS;
                val->it_interval.tv_usec = (periodNsecs % NSPS) / 1000;
                break;
            }
        }
    } else if (type == ITIMER_VIRTUAL || type == ITIMER_PROF) {
        uint32_t pidGrpIdx = zinfo->procArray[procIdx]->getGroupIdx();
        for (VTimeInfo *vi : processVTimers) {
            if (vi->pidGrpIdx == pidGrpIdx && vi->type == type) {
                uint64_t remainNsecs = cyclesToNs(vi->cyclesTrigger - processCycles[pidGrpIdx]);
                val->it_value.tv_sec = remainNsecs / NSPS;
                val->it_value.tv_usec = (remainNsecs % NSPS) / 1000;
                uint64_t periodNsecs = cyclesToNs(vi->cyclesPeriod);
                val->it_interval.tv_sec = periodNsecs / NSPS;
                val->it_interval.tv_usec = (periodNsecs % NSPS) / 1000;
                break;
            }
        }
    } else {
        warn("Invalid interval time type %d", type);
        retval = -1;
    }
    futex_unlock(&timerLock);
    return retval;
}

int IntervalTimer::setIntervalTimer(pid_t osPid, int type, const struct itimerval* newVal, struct itimerval* oldVal) {
    futex_lock(&timerLock);
    int retval = 0;
    uint64_t countdownNsecs = NSPS * (uint64_t) newVal->it_value.tv_sec;
    countdownNsecs += 1000UL * (uint64_t) newVal->it_value.tv_usec;
    uint64_t countdownCycles = nsToCycles(countdownNsecs);
    uint64_t periodNsecs = NSPS * (uint64_t) newVal->it_interval.tv_sec;
    periodNsecs += 1000UL * (uint64_t) newVal->it_interval.tv_usec;
    uint64_t periodCycles = nsToCycles(periodNsecs);

    //Wall clock timer similar to alarm() but with optional restart
    if (type == ITIMER_REAL) {
        uint64_t countdownPhases = countdownCycles / zinfo->phaseLength + 1; //wait at least 1 phase
        uint64_t alarmPhase = zinfo->numPhases + countdownPhases;
        uint64_t periodPhases = (periodNsecs > 0)? (periodCycles / zinfo->phaseLength + 1) : 0;
        memset(oldVal, 0, sizeof(struct itimerval));

        //Remove any queued SIGALRMs (there should only be 0 or 1 entries)
        SignalInfo* si = signalQueue.front();
        while (si) {
            if (si->osPid == osPid && si->sig == SIGALRM) {
                //Fill out the oldVal info
                uint64_t oldRemainCycles = (si->phase - curPhase) * zinfo->phaseLength;
                uint64_t oldRemainNsecs = cyclesToNs(oldRemainCycles);
                oldVal->it_value.tv_sec = oldRemainNsecs / NSPS;
                oldVal->it_value.tv_usec = (oldRemainNsecs % NSPS) / 1000;
                uint64_t oldPeriodCycles = si->period * zinfo->phaseLength;
                uint64_t oldPeriodNsecs = cyclesToNs(oldPeriodCycles);
                oldVal->it_interval.tv_sec = oldPeriodNsecs / NSPS;
                oldVal->it_interval.tv_usec = (oldPeriodNsecs % NSPS) / 1000;

                //Remove the old queued signal
                signalQueue.remove(si);
                delete si;
                break;
            }
            si = si->next;
        }
        // Nothing happens unless it_value was nonzero
        if (countdownNsecs > 0) {
            scheduleSignal(osPid, SIGALRM, alarmPhase, periodPhases);
        }
    //Virtual time counting down CPU time used by all threads of a process
    } else if (type == ITIMER_VIRTUAL || type == ITIMER_PROF) {
        //TODO: Distinguish between ITIMER_VIRTUAL (no system call time) and ITIMER_PROF (with system calls)

        // Check for an existing interval and fill out the old value (or delete if new time is 0)
        uint32_t pidGrpIdx = zinfo->procArray[procIdx]->getGroupIdx();
        bool exists = false;
        for (auto it = processVTimers.begin(); it != processVTimers.end(); ++it) {
            VTimeInfo* vi = *it;
            if (vi->pidGrpIdx == pidGrpIdx && vi->type == type) {
                exists = true;
                uint64_t oldRemainNsecs = cyclesToNs(vi->cyclesTrigger - processCycles[pidGrpIdx]);
                oldVal->it_value.tv_sec = oldRemainNsecs / NSPS;
                oldVal->it_value.tv_usec = (oldRemainNsecs % NSPS) / 1000;
                uint64_t oldPeriodNsecs = cyclesToNs(vi->cyclesPeriod);
                oldVal->it_interval.tv_sec = oldPeriodNsecs / NSPS;
                oldVal->it_interval.tv_usec = (oldPeriodNsecs % NSPS) / 1000;
                assert(vi->osPid == osPid);
                if (countdownCycles > 0) {
                    vi->cyclesTrigger = countdownCycles + processCycles[pidGrpIdx];
                    vi->cyclesPeriod = periodCycles;
                } else {
                    processVTimers.erase(it);
                    delete vi;
                }
                break;
            }
        }
        if (!exists) {
            memset(oldVal, 0, sizeof(struct itimerval));
            if (countdownCycles > 0) {
                VTimeInfo* vi = new VTimeInfo(); //deleted when removed from processVTimers
                vi->osPid = osPid;
                vi->pidGrpIdx = pidGrpIdx;
                vi->type = type;
                vi->cyclesTrigger = countdownCycles + processCycles[pidGrpIdx];
                vi->cyclesPeriod = periodCycles;
                processVTimers.push_back(vi);
            }
        }
    } else {
        warn("Invalid interval timer type %d", type);
        retval = -1;
    }
    futex_unlock(&timerLock);
    return retval;
}
