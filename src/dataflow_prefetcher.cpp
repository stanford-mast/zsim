/** $lic$
 * Copyright (C) 2019      by Google
 * Copyright (C) 2012-2015 by Massachusetts Institute of Technology
 * Copyright (C) 2010-2013 by The Board of Trustees of Stanford University
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

/* The dataflow prefetcher prefetches memory addresses based on dataflow kernels
 * (or code slices) extracted via offline profiling. The prefetcher needs to be
 * provided with a file (prefetch-kernels.txt) which contains the kernels using
 * a trigger PC as key. For each memory access that triggers, the prefetcher
 * determines the corresponding dataflow kernel an executes them, prefetching
 * memory addresses in the process.
 */

#include "dataflow_prefetcher.h"
#include "ooo_core.h"
#include "hash.h"
#include "ooo_filter_cache.h"
#include <limits>
#include <string>
#include <fstream>

const uint32_t MAX_KERNEL_OPS = 10000; //Maximum (ops) in a kernel
const uint32_t MIN_EXEC_CNT = 50; //Minimum execution count in trace
const double MIN_PERCENTAGE = 0.0005; //Kernel frequency of occurance / all kernels
const uint32_t MAX_KERNELS_TO_PREF = 8; //Maximum allowed kernels for a PC
const uint32_t MAX_DEGREE = 32; //Maximum variable prefetch degree
const double DRAM_LAT = 264.0; //TODO: Derive latency from cfg file
const uint64_t LD_BARRIER = 0UL;

void DataflowPrefetcher::initKernels() {
    std::ifstream inFile;
    inFile.open(pref_kernels_file.c_str());

    std::string line;
    int64_t linenr = 0;
    double last_cumulative_percent;
    Kernel kern;
    kern.cumulative_percent = 0.0;
    while (std::getline(inFile, line)) {
        std::istringstream is(line);
        uint64_t trigger_pc;
        std::string field;
        uint8_t state = LD_PCS;

        kern.ld_pcs.clear();
        last_cumulative_percent = kern.cumulative_percent;
        linenr++;
        if (line.find("#", 0, 1) != std::string::npos) {
            continue;
        }
        std::getline(is, field, ',');
        trigger_pc = std::stol(field, nullptr, 0);
        /* FSM to parse load-chain file */
        while (std::getline(is, field, ',')) {
            switch (state) {
            case LD_PCS:
                if ((field == "<") || (field == ">")) {
                    continue;
                }
                else if(field == "@") {
                    state = P_OPS;
                }
                else if(field == "|") {
                    kern.ld_pcs.push_back(LD_BARRIER);
                }
                else {
                    kern.ld_pcs.push_back(std::stol(field, nullptr, 0));
                }
                break;
            case P_OPS:
                kern.program_ops = std::stol(field, nullptr, 0);
                state = K_OPS;
                break;
            case K_OPS:
                kern.kernel_ops = std::stol(field, nullptr, 0);
                state = AVG_MISSES;
                break;
            case AVG_MISSES:
                kern.avg_load_misses = std::stof(field, nullptr);
                state = EX_CNT;
                break;
            case EX_CNT:
                kern.execution_count = std::stol(field, nullptr, 0);
                state = PERCENT;
                break;
            case PERCENT:
                kern.cumulative_percent = std::stof(field, nullptr);
                state = LD_PCS;
                break;
            }
        }
        /* Add kernel to list of simple kernels, if executed
         * sufficiently often and number of kernel ops lower than threshold */
        double percent = kern.cumulative_percent - last_cumulative_percent;
        if (kern.ld_pcs.size() == 1) {
            auto kernel_group = simple_kernels.find(trigger_pc);
            if (kernel_group != simple_kernels.end()) {
                if(kern.program_ops < MAX_KERNEL_OPS
                   && kern.execution_count > MIN_EXEC_CNT) {
                    kernel_group->second.back().num_kernels += 1;
                    kernel_group->second.back().weighted_ops +=
                        kern.program_ops * kern.execution_count;
                    kernel_group->second.back().all_execs +=
                        kern.execution_count;
                }
            } else if(kern.program_ops < MAX_KERNEL_OPS
                      && kern.execution_count > MIN_EXEC_CNT) {
                std::vector<Kernel> new_kernel_group;
                kern.linenr = linenr;
                kern.num_kernels = 1;
                kern.all_execs = kern.execution_count;
                kern.weighted_ops = kern.program_ops * kern.execution_count;
                new_kernel_group.push_back(kern);
                simple_kernels.insert(std::pair<uint64_t,
                                    std::vector<Kernel>>(trigger_pc,
                                                         new_kernel_group));
            }
        }
        /* Add kernel to list of complex kernels, if executed
         * sufficiently often and number of kernel ops lower than threshold */
        else {
            auto kernel_group = complex_kernels.find(trigger_pc);
            if (kernel_group != complex_kernels.end()) {
                if(kern.program_ops < MAX_KERNEL_OPS &&
                   percent > MIN_PERCENTAGE) {
                    kernel_group->second.front().num_kernels += 1;
                    if (kernel_group->second.size() < 4) {
                      kernel_group->second.front().num_kernels += 1;
                      kernel_group->second.front().weighted_ops +=
                          kern.program_ops * kern.execution_count;
                      kernel_group->second.front().all_execs +=
                          kern.execution_count;
                      kernel_group->second.push_back(kern);
                    }
                }
            } else if(kern.program_ops < MAX_KERNEL_OPS &&
                      percent > MIN_PERCENTAGE) {
                std::vector<Kernel> new_kernel_group;
                kern.linenr = linenr;
                kern.num_kernels = 1;
                kern.all_execs = kern.execution_count;
                kern.weighted_ops = kern.program_ops * kern.execution_count;
                new_kernel_group.push_back(kern);
                complex_kernels.insert(std::pair<uint64_t,
                                       std::vector<Kernel>>(trigger_pc,
                                                            new_kernel_group));
            }
        }
    }
    /* Print stats about complex kernels */
    int const_few = 0, compl_few = 0;
    for (auto it = complex_kernels.begin(); it != complex_kernels.end(); it++){
        if (it->second.back().num_kernels < 8)
            compl_few++;
        double avgo = ((double)it->second.back().weighted_ops /
                       it->second.back().all_execs);
        it->second.back().avg_ops = (uint64_t)((264.0 * 0.8/avgo) + 1);
        std::cout << "Complex kernel " << std::hex << it->first << " num "
                  << std::dec << it->second.back().num_kernels << " degree "
                  << it->second.back().avg_ops << " line " << std::dec
                  << it->second.back().linenr << std::endl;
        /* If for the same load PC there exist simple and complex kernels treat
         * it as complex */
        auto search = simple_kernels.find(it->first);
        if (search != simple_kernels.end()) {
            simple_kernels.erase(search);
        }
    }
    /* Print stats about simple kernels */
    for (auto it = simple_kernels.begin(); it != simple_kernels.end(); it++){
        if (it->second.back().num_kernels < 8)
            const_few++;
        double avgo = ((double)it->second.back().weighted_ops /
                       it->second.back().all_execs);
        it->second.back().avg_ops = (uint64_t)((264.0 * 0.8/avgo) + 1);
        std::cout << "Simple kernel " << std::hex << it->first << " num " << std::dec
                  << it->second.back().num_kernels << " degree "
                  << it->second.back().avg_ops << " line " << std::dec
                  << it->second.back().linenr << std::endl;
    }
    std::cout << "Loaded " << simple_kernels.size() << " simple prefetch kernels and "
              << complex_kernels.size() << " complex prefetch kernels." << std::endl;
    std::cout << "Loaded " << const_few << " simple PCs with <3 prefetch kernels and "
              << compl_few << " complex PCs with <3 prefetch kernels." << std::endl;

}

void DataflowPrefetcher::postInit() {
    /*If the prefetcher is part of the filter cache, filter cache is known at
      build time. Otherwise determine memory hierarchy by executing postInit();
    */
    if (!filter_cache) {
        CachePrefetcher::postInit();
        filter_cache = dynamic_cast<OOOFilterCache*>(d_caches_[0].first);
        assert(filter_cache);
    }
    BaseCache *cache = dynamic_cast<BaseCache *>(filter_cache);
    g_vector<MemObject*> *parents = NULL;
    while (true) {
        parents = cache->getParents();
        mem_ctrl_ = dynamic_cast<MD1Memory*>((*parents)[0]);
        if (mem_ctrl_) {
            break;
        }
        cache = dynamic_cast<BaseCache*>((*parents)[0]);
        if (!cache) {
            break;
        }
    }
}

void DataflowPrefetcher::initStats(AggregateStat* parentStat) {
    AggregateStat* s = new AggregateStat();

    s->init(name_.c_str(), "REF Prefetcher stats");
    prof_emitted_prefetches.init("pf", "Prefetches emitted, might hit cache");
    s->append(&prof_emitted_prefetches);
    prof_emitted_actual_prefetches.init("pfActual",
                                        "Prefetches emitted that miss cache");
    s->append(&prof_emitted_actual_prefetches);
    prof_prefetcher_accessed.init("accessed", "Prefetcher accessed");
    s->append(&prof_prefetcher_accessed);
    prof_kernels_found.init("kernels", "Number of prefetch kernels found");
    s->append(&prof_kernels_found);
    prof_wrong_path_prefetches.init("wrong", "Wrong patch prefetches");
    s->append(&prof_wrong_path_prefetches);
    prof_pc_not_found.init("noPc", "End of buffer reached - pc not found");
    s->append(&prof_pc_not_found);
    prof_prefetcher_throttled.init("throttled", "prefetch skipped due to DRAM throttling");
    s->append(&prof_prefetcher_throttled);
    parentStat->append(s);
}

void DataflowPrefetcher::schedReq(uint32_t srcId, uint64_t lineAddr,
                                      uint64_t pc, uint64_t dispatchCycle,
                                      bool serialize) {
    OOOCore *core = static_cast<OOOCore *>(zinfo->cores[srcId]);
    uint32_t distance = 0; //TODO: Perform PostInit to determine target distance

    if (!core) {
        panic("Reference Prefetcher requires OOO cores\n");
    }

    if (mem_ctrl_ && mem_ctrl_->getLoad() > 1.1) {
        prof_prefetcher_throttled.inc();
        return;
    }

    prof_emitted_prefetches.inc();
    filter_cache->schedulePrefetch(lineAddr, distance, pc, false,
                                   serialize);
}

void DataflowPrefetcher::prefetch(MemReq& _req) {
    if (pref_simple) {
        prefetchKernelGroup(_req, &simple_kernels);
    }
    if (pref_complex) {
        prefetchKernelGroup(_req, &complex_kernels);
    }
}

void DataflowPrefetcher::prefetchKernelGroup(MemReq& _req,
                                 std::map<uint64_t, std::vector<Kernel>> *kgs) {
    uint32_t coreId = _req.srcId;
    uint64_t dispatchCycle = _req.cycle;
    uint64_t trigger_pc = _req.pc;
    assert(coreId < zinfo->numCores);

    auto kernel_group = kgs->find(trigger_pc);
    if (kernel_group == kgs->end())
        return;

    prof_kernels_found.inc(kernel_group->second.size());
    prof_prefetcher_accessed.inc();

    TraceReader *reader;
    reader = static_cast<TraceReader*>(zinfo->readers[coreId]);
    for (auto kernel : kernel_group->second) {
        if (kernel.num_kernels > MAX_KERNELS_TO_PREF) {
            continue;
        }
        auto entry = reader->bufferStart();
        TraceReader::returnValue res;
        bool wrong_path_prefetch = false;
        int degree;
        if (!variable_degree) {
            degree = degree_;
        }
        else {
            OOOCore *core = static_cast<OOOCore *>(zinfo->cores[coreId]);
            double ipc = (double)core->getInstrs() / (double)core->getCycles();
            double avgo = ((double)kernel.weighted_ops /
                           kernel.all_execs);
            degree = MAX(1U, MIN(MAX_DEGREE, (uint64_t)(DRAM_LAT * ipc / avgo)));
        }

        bool serialize = false;
        for (int deg = 0; deg < degree; deg++) {
            bool kernel_finished = false;
            for (auto pc = kernel.ld_pcs.begin(); pc != kernel.ld_pcs.end();
                 pc++) {
                assert(!kernel_finished);

                if (*pc == LD_BARRIER) {
                    serialize = true;
                    continue;
                }
                if(wrong_path_prefetch) {
                    prof_wrong_path_prefetches.inc();
                    continue;
                }
                res = reader->findPCInSegment(entry, *pc, trigger_pc);
                if (res == TraceReader::ENTRY_NOT_FOUND) {
                    prof_pc_not_found.inc();

                    if (!limit_prefetching) {
                        //do random load
                        for (int i = deg; i < degree; i++){
                            uint64_t add = rand();
                            schedReq(coreId, add, trigger_pc,
                                     dispatchCycle, serialize);
                            serialize = false;
                        }
                    }
                    //Do not try next degree
                    deg = degree;
                    break;
                }
                else if (res == TraceReader::ENTRY_OUT_OF_SEGMENT
                         && trigger_pc != *pc) {
                    wrong_path_prefetch = true;
                    prof_wrong_path_prefetches.inc();
                }
                else {
                    int mem_load_per_ins = 0;
                    if (res == TraceReader::ENTRY_OUT_OF_SEGMENT)
                        kernel_finished = true;

                    uint32_t mem_ops = xed_decoded_inst_number_of_memory_operands
                        (entry->ins);
                    for (uint32_t op = 0; op < mem_ops; op++) {
                        if(xed_decoded_inst_mem_read(entry->ins, op)) {
                            uint64_t laddr = entry->mem_addr[op] >> lineBits;
                            mem_load_per_ins++;
                            schedReq(coreId, laddr, trigger_pc,
                                     dispatchCycle, serialize);
                            serialize = false;
                        }
                    }

                    if (mem_load_per_ins != 1) {
                        int SIZEBUF = 1024;
                        char buf[SIZEBUF];
                        xed_decoded_inst_dump(entry->ins, buf, SIZEBUF);
                        std::cout << "WARN: Executed " << mem_load_per_ins
                                  << " != 1 memory reads per load ins. ASM: "
                                  << std::string(buf) << std::endl;
                        assert(xed_decoded_inst_number_of_memory_operands(
                                   entry->ins) > 0);
                    }
                }
                //entry++;
            }
        }
    }
}

uint64_t DataflowPrefetcher::access(MemReq& _req) {
    MemReq req = _req;
    req.childId = childId_;
    uint32_t bank = CacheBankHash::hash(req.lineAddr, parents_.size());
    uint64_t resp_cycle = parents_[bank]->access(req);

    bool monitored = (monitor_GETS && _req.type == GETS)
        || (monitor_GETX && _req.type == GETX);
    if (!monitored) {
        return resp_cycle;
    }

    prefetch(_req);
    return resp_cycle;
}
