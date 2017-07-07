#ifndef INTERVAL_TIMER_H_
#define INTERVAL_TIMER_H_

#include "galloc.h"
#include "g_std/g_vector.h"
#include "intrusive_list.h"
#include "locks.h"
#include "pad.h"
#include "zsim.h"

//TODO: Figure out best locking policy then implement
//TODO: Add a trace macro for 'ITimer'
//TODO: Modify InList to take const pointers for insertion

class IntervalTimer : public GlobAlloc {
    private:
        PAD();
        lock_t timerLock;
        PAD();

        uint64_t curPhase;

        struct SignalInfo : InListNode<SignalInfo> {
            uint64_t phase; //phase at which to send a signal to a process
            int sig; //signal number
            pid_t osPid; //signal recipient process (OS PID)
            uint64_t period; //repeat phase period, or 0 for one-shot
        };
        InList<SignalInfo> signalQueue;

        struct VTimeInfo : InListNode<VTimeInfo> {
            pid_t osPid;
            uint32_t pidGrpIdx;
            int type; //Either ITIMER_VIRTUAL or ITIMER_PROF
            uint64_t cyclesTrigger;
            uint64_t cyclesPeriod;
        };
        g_vector<VTimeInfo*> processVTimers;
        g_vector<uint64_t> processCycles, lastCoreCycles;

        void signalOrderedInsert(struct SignalInfo* si);

        //Schedule a signal for future delivery to a process
        void scheduleSignal(pid_t osPid, int sig, uint64_t phase, uint64_t period);

    public:
        IntervalTimer(uint32_t maxProcesses);
        void phaseTick();

        //A wrapper around scheduleSignal() to support the virtualization of SYS_alarm:
        //The return value is the number of seconds remaining for a prior alarm (if any)
        unsigned int setAlarm(pid_t osPid, unsigned int seconds);

        //Retrive the current interval timer of type ITIMER_{REAL,VIRTUAL,PROF} similar to getitimer()
        int getIntervalTimer(pid_t osPid, int type, struct itimerval* val);

        //A wrapper around scheduleSignal() to support setting interval timers, e.g., via setitimer()
        int setIntervalTimer(pid_t osPid, int type, const struct itimerval* newVal, struct itimerval* oldVal);

};

#endif  // INTERVAL_TIMER_H_
