/** $glic$
 * Copyright (C) 2017 by Google Inc.
 * Copyright (C) 2012-2015 by Massachusetts Institute of Technology
 * Copyright (C) 2010-2013 by The Board of Trustees of Stanford University
 * Copyright (C) 2011 Google Inc.
 *
 * This file is part of zsim.
 *
 * zsim is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, version 2.
 *
 * If you use this software in your research, we request that you reference
 * the zsim paper ("ZSim: Fast and Accurate Microarchitectural Simulation of
 * Thousand-Core Systems", Sanchez and Kozyrakis, ISCA-40, June 2013) as the
 * source of the simulator in any publications that use this software, and that
 * you send us a citation of your work.
 *
 * zsim is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "zsim.h"

#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>

#include "access_tracing.h"
#include "config.h"
#include "contention_sim.h"
#include "core.h"
#include "decoder.h"
#include "event_queue.h"
#include "init.h"

#ifdef ZSIM_USE_YT
#include "trace_reader_yt.h"
#include "yt_tracer.h"
/* Both yt_tracer and drmemtrace define int64. Include trace_reader_memtrace.h
 * after yt_tracer.h and the do not define statement
 */
#define DR_DO_NOT_DEFINE_int64
#define DR_DO_NOT_DEFINE_uint64
#endif  // ZSIM_USE_YT
#include "trace_reader_memtrace.h"
#include "trace_reader_pt.h"

const uint32_t END_TRACE_SIM = ~0x0;
const uint32_t CONTENTION_THREAD = ~0x0 - 1;

/* Global Variables */
GlobSimInfo* zinfo;
pthread_t contention_thread;

/* Per-process variables */
uint32_t procIdx = 0;
uint32_t lineBits;
Address procMask;
InstrFuncPtrs fPtrs[MAX_THREADS] ATTR_LINE_ALIGNED; //minimize false sharing
pthread_barrier_t barrier;
std::atomic<int> threads_active;
std::chrono::steady_clock::time_point last;

/* Per-thread variables */
thread_local bool finished = false;

uint32_t getCid(uint32_t tid) {
    /* Fixed tid to cid mapping */
    return tid;
}

void CheckForTermination() {
    assert(zinfo->terminationConditionMet == false);
    if (zinfo->maxPhases && zinfo->numPhases >= zinfo->maxPhases) {
        zinfo->terminationConditionMet = true;
        info("Max phases reached (%ld)", zinfo->numPhases);
        return;
    }

    if (zinfo->maxMinInstrs) {
        uint64_t minInstrs = zinfo->cores[0]->getInstrs();
        for (uint32_t i = 1; i < zinfo->numCores; i++) {
            uint64_t coreInstrs = zinfo->cores[i]->getInstrs();
            if (coreInstrs < minInstrs && coreInstrs > 0) {
                minInstrs = coreInstrs;
            }
        }

        if (minInstrs >= zinfo->maxMinInstrs) {
            zinfo->terminationConditionMet = true;
            info("Max min instructions reached (%ld)", minInstrs);
            return;
        }
    }

    if (zinfo->maxTotalInstrs) {
        uint64_t totalInstrs = 0;
        for (uint32_t i = 0; i < zinfo->numCores; i++) {
            totalInstrs += zinfo->cores[i]->getInstrs();
        }

        if (totalInstrs >= zinfo->maxTotalInstrs) {
            zinfo->terminationConditionMet = true;
            info("Max total (aggregate) instructions reached (%ld)", totalInstrs);
            return;
        }
    }

    if (zinfo->maxSimTimeNs) {
        uint64_t simNs = zinfo->profSimTime->count(PROF_BOUND) + zinfo->profSimTime->count(PROF_WEAVE);
        if (simNs >= zinfo->maxSimTimeNs) {
            zinfo->terminationConditionMet = true;
            info("Max simulation time reached (%ld ns)", simNs);
            return;
        }
    }

    if (zinfo->externalTermPending) {
        zinfo->terminationConditionMet = true;
        info("Terminating due to external notification");
        return;
    }
}

void EndOfPhaseActions() {
    zinfo->profSimTime->transition(PROF_WEAVE);
    zinfo->contentionSim->simulatePhase(zinfo->globPhaseCycles + zinfo->phaseLength);
    zinfo->eventQueue->tick();
    zinfo->profSimTime->transition(PROF_BOUND);
    CheckForTermination();
}

uint32_t TakeBarrier(uint32_t tid, uint32_t cid) {

    pthread_barrier_wait(&barrier);

    /* This needs to be in between 2 barriers to avoid races */
    if (threads_active.load() == 0) {
        tid = END_TRACE_SIM;
    }

    if (tid == CONTENTION_THREAD) {
        EndOfPhaseActions();
        zinfo->numPhases++;
        zinfo->globPhaseCycles += zinfo->phaseLength;
        int interval = 10000;
        if ((zinfo->numPhases % interval) == 0) {
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count();
            info("Simulate Phase %lu Simulation Speed: %lu KIPS", zinfo->numPhases,
                 static_cast<uint64_t>((zinfo->phaseLength * interval)/duration));
            last = now;
        }
    }

    pthread_barrier_wait(&barrier);

    finished = zinfo->terminationConditionMet;

    return tid;
}

Core* cores[MAX_THREADS];

struct bbl_cache_entry {
    BblInfo *bbl;
    uint32_t size_bytes;
    uint32_t nr_inst;
    uint32_t mem_ops;

    bbl_cache_entry(BblInfo *_bbl, uint32_t _size_bytes, uint32_t _nr_inst,  uint32_t _mem_ops) :
        bbl(_bbl), size_bytes(_size_bytes), nr_inst(_nr_inst), mem_ops(_mem_ops) {};
};

struct threadInfo {
    int tid;
    std::string tracefile;
    std::string binaries;
    std::string type;

    threadInfo() {};

    threadInfo(int _tid, std::string _tracefile, std::string _binaries, std::string _type) :
        tid(_tid), tracefile(_tracefile), binaries(_binaries), type(_type) {};
};

void sim_mem_op(uint32_t tid, const InstInfo *insi) {
    if (!INS_Nop(insi->ins) && !INS_LEA(insi->ins)) { //NOPs and LEA never access memory
        for (uint32_t op = 0; op < xed_decoded_inst_number_of_memory_operands(insi->ins); op++) {
            //predicated true ld/st are handled just as regular ld/st
            if(xed_decoded_inst_mem_read(insi->ins, op) && !insi->mem_used[op]) {
                fPtrs[tid].predLoadPtr(tid, insi->mem_addr[op], insi->pc, false);
            }
            else if(xed_decoded_inst_mem_read(insi->ins, op)) {
                fPtrs[tid].loadPtr(tid, insi->mem_addr[op], insi->pc);
            }
            if(xed_decoded_inst_mem_written(insi->ins, op) && !insi->mem_used[op]) {
                fPtrs[tid].predStorePtr(tid, insi->mem_addr[op], insi->pc, false);
            }
            else if(xed_decoded_inst_mem_written(insi->ins, op)) {
                fPtrs[tid].storePtr(tid, insi->mem_addr[op], insi->pc);
            }
        }
    }
}

void *wait_for_threads_and_exit(int tid) {
    cores[tid]->leave();
    threads_active--;

    while (TakeBarrier(tid, getCid(tid)) != END_TRACE_SIM) {}

    return NULL;
}

void *simtrace(void *arg) {
    uint64_t skipped_instructions = 0;
    threadInfo *ti = (threadInfo *)arg;
    int tid = ti->tid;
    std::unordered_map<uint64_t, bbl_cache_entry> bbl_cache;
    TraceReader *reader;
    //TODO heinerl: Only enable buffer for dataflow prefetcher
    uint32_t bufsize = 10000;

    if(ti->type.compare("MEMTRACE") == 0) {
        reader = new TraceReaderMemtrace(ti->tracefile, ti->binaries, bufsize);
    }
#ifdef ZSIM_USE_YT
    else if(ti->type.compare("YT") == 0) {
        reader = new TraceReaderYT(ti->tracefile, ti->binaries, bufsize);
    }
#endif  // ZSIM_USE_YT
    else if(ti->type.compare("PT") == 0) {
        if(zinfo->enable_code_bloat_effect && zinfo->prev_to_new_bbl_address_map.size() > 0)
        {
            reader = new TraceReaderPT(ti->tracefile, zinfo->enable_code_bloat_effect, &(zinfo->prev_to_new_bbl_address_map));
        }
        else
        {
            reader = new TraceReaderPT(ti->tracefile);
        }
    }
    else {
        panic("Tid %i: Unsupported trace format", tid);
    }

    if (!reader) {
        panic("Tid %i: Could not open input files", tid);
    }
    zinfo->readers[tid] = reader;

    std::vector<InstInfo> bbl;
    uint64_t sim_inst = 1;
    uint64_t unknown_instrs = 0;
    uint64_t simulated_bbls = 0;
    uint64_t interrupted_bbls = 0;

    cores[tid]->join();

    const InstInfo *insi = reader->nextInstruction();
    uint64_t last_pid = insi->pid;
    uint64_t last_tid = insi->tid;

    if (!insi->valid) {
        info("Tid %i: Empty trace - exiting", tid);
        return wait_for_threads_and_exit(tid);
    }

    while (!finished) {
        /* Skip until we find valid bbl */
        while (insi->ins == NULL && insi->valid) {
            skipped_instructions++;
            insi = reader->nextInstruction();
            last_tid = insi->tid;
            last_pid = insi->pid;
        }

        if (!insi->valid) {
            break;
        }

        uint64_t bbl_pc = insi->pc;
        auto bbl_to_sim = bbl_cache.find(bbl_pc);

        /* Decode the bbl if we haven't done so yet */
        if (bbl_to_sim == bbl_cache.end()) {
            bbl.clear();
            int mem_ops = 0;
            int bbl_size = 0;
            int nr_inst = 0;
            bool bbl_interrupted = false;
            /* In Memtrace, basic blocks can be interrupted by signal handlers and potentially for
             * other reasons which is why we cannot cache basic blocks and need to decode and
             * simulate each instruction individually */
            bool cache_bbl = (ti->type != "MEMTRACE") || (ti->type != "PT");

            while (1) {
                bbl.push_back(*insi);
                nr_inst++;
                if (insi->mem_used[0] || insi->mem_used[1]) {
                    mem_ops++;
                }

                /* This Bbl contains unknown instructions (NOPs). We can simulate but not cache */
                if(insi->unknown_type) {
                    unknown_instrs++;
                    cache_bbl = false;
                }

                /* This Bbl was interrrupted by a context switch. Do not simulate and do not cache */
                if (insi->tid != last_tid || insi->pid != last_pid) {
                    zinfo->cores[tid]->contextSwitch(-1);
                    last_tid = insi->tid;
                    last_pid = insi->pid;
                    insi = reader->nextInstruction();
                    bbl_interrupted = true;
                    interrupted_bbls++;
                    break;
                }

                /* End the current BBL */
                if (likely(insi->custom_op == CustomOp::NONE)) {
                    // NOTE: Assuming custom ops are 0-sized and not branches
                    bbl_size += INS_Size(insi->ins);
                    if (INS_ChangeControlFlow(insi->ins)) {
                      insi = reader->nextInstruction();
                      sim_inst++;
                      break;
                    }
                }
                insi = reader->nextInstruction();
                sim_inst++;
                if (!insi->valid) {
                    finished = true;
                    break;
                }
            }

            if (unlikely(bbl_interrupted)) {
                continue;
            }

            bbl_cache_entry ce(Decoder::decodeBbl(&bbl, zinfo->oooDecode), bbl_size, nr_inst, mem_ops);
            ce.bbl->preserve = cache_bbl;
            if (cache_bbl) {
                bbl_cache.insert(std::pair <uint64_t, bbl_cache_entry>(bbl_pc, ce));
            }

            /* Simulate BBL */
            fPtrs[tid].bblPtr(tid, bbl_pc, ce.bbl);
            simulated_bbls++;

            /* Simulate Ld/St & and branch pred */
            int cur_ins = 1;
            for (auto it = bbl.begin(); it != bbl.end(); it++, cur_ins++) {
                const InstInfo &info = *it;
                if (info.mem_used[0] || info.mem_used[1]) {  // Expect idx 1 => idx 0, but short circuit is fast
                    sim_mem_op(tid, &info);
                }
                if (likely(info.custom_op == CustomOp::NONE)) {
                    if (INS_Category(info.ins) == XED_CATEGORY_COND_BR) {
                        uint64_t fall_through_addr = bbl_pc + ce.size_bytes;
                        fPtrs[tid].branchPtr(tid, it->pc, it->taken, it->target, fall_through_addr);
                    }
                }
            }
        }
        else { //Bbl is cached
            uint32_t size_bytes = bbl_to_sim->second.size_bytes;
            uint32_t size = bbl_to_sim->second.nr_inst;

            fPtrs[tid].bblPtr(tid, bbl_pc, bbl_to_sim->second.bbl);
            simulated_bbls++;

            for (uint32_t i = 0; i < size; i++) {
                if (insi->mem_used[0] || insi->mem_used[1]) {
                    sim_mem_op(tid, insi);
                }

                if (insi->tid != last_tid || insi->pid != last_pid) {
                    last_tid = insi->tid;
                    last_pid = insi->pid;
                    insi = reader->nextInstruction();
                    zinfo->cores[tid]->contextSwitch(-1);
                    interrupted_bbls++;
                    break;
                }

                if (likely(insi->custom_op == CustomOp::NONE)) {
                    if (INS_Category(insi->ins) == XED_CATEGORY_COND_BR) {
                        uint64_t fall_through_addr = bbl_to_sim->first + size_bytes;
                        //Check that the branch is the last instruction of the BBL
                        assert(i + 1 == bbl_to_sim->second.nr_inst);
                        fPtrs[tid].branchPtr(tid, insi->pc, insi->taken, insi->target, fall_through_addr);
                    }
                }

                insi = reader->nextInstruction();
                sim_inst++;
                if (!insi->valid) {
                    finished = true;
                    break;
                }
            }
        }
    }

    info("Tid: %i finished. Instruction trace size: %lu. Skipped instructions due to unavailable symbols: %lu. BBLs in trace: %lu, dropped BBLs due to context switches: %lu",
         tid, sim_inst, unknown_instrs, simulated_bbls, interrupted_bbls);

    return wait_for_threads_and_exit(tid);
}

int main(int argc, char *argv[]) {
    /* zsim_harness.cpp */
    InitLog("[H] ", nullptr /*log to stdout/err*/);
    info("Starting trace zsim..");

    /* Canonicalize paths --- because we change dirs, we deal in absolute paths */
    const char* configFile = realpath(argv[1], nullptr);
    const char* cwd = getcwd(nullptr, 0); //already absolute
    Config conf(configFile);

    /* Remove all zsim.log.* files (we append to them, and want to avoid outputs from multiple simulations) */
    uint32_t removedLogfiles = 0;
    while (true) {
        std::stringstream ss;
        ss << "zsim.log." << removedLogfiles;
        if (remove(ss.str().c_str()) != 0) break;
        removedLogfiles++;
    }
    if (removedLogfiles) info("Removed %d old logfiles", removedLogfiles);

    uint32_t gmSize = conf.get<uint32_t>("sim.gmMBytes", (1<<10) /*default 1024MB*/);
    info("Creating global segment, %d MBs", gmSize);
    int shmid = gm_init(((size_t)gmSize) << 20 /*MB to Bytes*/);
    info("Global segment shmid = %d", shmid);
    ZSIM_TRACE(Harness, "Created global segment, starting pin processes, shmid = %d", shmid);

    /* zsim.cpp */
    SimInit(configFile, std::string("./").c_str(), 0);
    lineBits = ilog2(zinfo->lineSize);
    procMask = ((uint64_t)procIdx) << (64-lineBits);

    /* Read the per-thread traces */
    pthread_t simthread[zinfo->numCores];
    threadInfo ti[zinfo->numCores];

    /* Setup barrier before spawning the threads */
    int spawn_threads = 1; //+1 because of contention thread
    for (uint32_t i = 0; i < zinfo->numCores; i++) {
        std::stringstream p_ss;
        p_ss << "trace" << i;

        if (conf.exists(p_ss.str().c_str())) {
            spawn_threads++;
        }
    }

    pthread_barrier_init(&barrier, NULL, spawn_threads);

    for (uint32_t i = 0; i < zinfo->numCores; i++) {
        std::stringstream p_ss;
        p_ss << "trace" << i;

        if (conf.exists(p_ss.str().c_str())) {
            const char* trace = realpath(conf.get<const char*>(p_ss.str(), cwd), nullptr);
            const char* binaries = realpath(conf.get<const char*>("trace_binaries", cwd), nullptr);
            const char* type = conf.get<const char*>("trace_type", cwd);

            ti[i] = {(int32_t)i, std::string(trace), std::string(binaries), std::string(type)};
            fPtrs[i] = zinfo->cores[i]->GetFuncPtrs();
            cores[i] = zinfo->cores[i];
            threads_active++;
            pthread_create(&simthread[i], NULL, simtrace, &ti[i]);
        }
    }

    last = std::chrono::steady_clock::now();

    /* Start Contention simulation thread */
    while (TakeBarrier(CONTENTION_THREAD, CONTENTION_THREAD) != END_TRACE_SIM) {}

    void *retval[zinfo->numCores];
    for (uint32_t i = 0; i < zinfo->numCores; i++) {
        std::stringstream p_ss;
        p_ss << "trace" << i;

        if (conf.exists(p_ss.str().c_str())) {
            pthread_join(simthread[i], &retval[i]);
        }
    }

    zinfo->trigger = 20000;
    for (StatsBackend* backend : *(zinfo->statsBackends)) {
        backend->dump(false /*unbuffered, write out*/);
    }
    for (AccessTraceWriter* t : *(zinfo->traceWriters)) t->dump(false);  // flushes trace writer
#ifdef ZSIM_USE_YT
    for (YTTracer* t : *(zinfo->YTtraceWriters)) t->dump(false);
#endif  // ZSIM_USE_YT
}
