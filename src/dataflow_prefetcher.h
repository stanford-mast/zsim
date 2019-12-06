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

#ifndef DATAFLOW_PREFETCHER_H_
#define DATAFLOW_PREFETCHER_H_

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <map>

#include "trace_reader.h"
#include "cache.h"
#include "cache_prefetcher.h"
#include "g_std/g_string.h"
#include "memory_hierarchy.h"
#include "stats.h"
#include "zsim.h"
#include "mem_ctrls.h"

class OOOFilterCache;

class DataflowPrefetcher : public CachePrefetcher {

  struct Kernel {
      std::vector<uint64_t> ld_pcs;
      uint32_t program_ops;
      uint32_t kernel_ops;
      float avg_load_misses;
      uint32_t execution_count;
      float cumulative_percent;
      uint32_t num_kernels;
      int64_t linenr;
      uint64_t all_execs;
      uint64_t weighted_ops;
      uint64_t avg_ops;
  };

    enum STATES {
        LD_PCS,
        P_OPS,
        K_OPS,
        AVG_MISSES,
        EX_CNT,
        PERCENT,
    };

private:
    int degree_;
    Counter prof_emitted_prefetches, prof_emitted_actual_prefetches,
        prof_prefetcher_accessed, prof_kernels_found,
        prof_wrong_path_prefetches, prof_pc_not_found, prof_prefetcher_throttled;
    //key: trigger PC, value: prefetch kernels for this PC
    std::map<uint64_t, std::vector<Kernel>> simple_kernels;
    std::map<uint64_t, std::vector<Kernel>> complex_kernels;
    bool monitor_GETS; //trigger prefetcher on read misses
    bool monitor_GETX; //trigger prefetcher on write misses
    bool pref_simple; //enable prefetching simple kernels
    bool pref_complex; //enable prefetching complex kernels
    bool limit_prefetching; //stop prefetching when end of segment is reached
    bool variable_degree; //prefetch with variable degree
    OOOFilterCache *filter_cache;
    g_string pref_kernels_file;
    MD1Memory* mem_ctrl_;

    void schedReq(uint32_t srcId, uint64_t lineAddr, uint64_t pc,
                      uint64_t dispatchCycle, bool serialize);

    void initKernels();
    void prefetchKernelGroup(MemReq& _req,
                             std::map<uint64_t, std::vector<Kernel>> *kgs);

  public:
    explicit DataflowPrefetcher(const g_string& _name,
                                uint32_t _degree,
                                const g_string &target_cache,
                                bool _monitor_GETS, bool _monitor_GETX,
                                bool _pref_simple, bool _pref_complex,
                                bool _limit_prefetching, bool _var_degree,
                                OOOFilterCache *_filter_cache,
                                g_string _pref_kernels_file) :
        CachePrefetcher(_name, target_cache), degree_(_degree),
        monitor_GETS(_monitor_GETS), monitor_GETX(_monitor_GETX),
        pref_simple(_pref_simple), pref_complex(_pref_complex),
        limit_prefetching(_limit_prefetching), variable_degree(_var_degree),
        filter_cache(_filter_cache), pref_kernels_file(_pref_kernels_file)
    {
        if (_degree) {
            initKernels();
        }
    };

    void postInit();
    void initStats(AggregateStat* parentStat) override;
    uint64_t access(MemReq& _req) override;
    void prefetch(MemReq& _req) override;
};

#endif
