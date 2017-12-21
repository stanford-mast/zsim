/** $lic$
 * Copyright (C) 2017      by Google
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

#ifndef GHB_PREFETCHER_H_
#define GHB_PREFETCHER_H_

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cache.h"
#include "cache_prefetcher.h"
#include "g_std/g_string.h"
#include "memory_hierarchy.h"
#include "stats.h"
#include "zsim.h"

class GhbPrefetcher : public CachePrefetcher {

private:
    struct IndexEntry {
        uint32_t ghb_entry;
        uint64_t last_access;

        IndexEntry(uint32_t _ghb_entry, uint64_t _last_access)
            : ghb_entry(_ghb_entry), last_access(_last_access) {}
    };

    struct GhbEntry {
        uint64_t line_addr;
        uint32_t next;

        GhbEntry() {};
        GhbEntry(uint64_t _line_addr, uint32_t _next) :
            line_addr(_line_addr), next(_next) {};
    };

    uint32_t ghb_size; //entries in ghb ring buffer
    uint32_t index_size; //index table size (streams)
    uint32_t log_distance; //memory area monitored per stream
    uint32_t degree; //number of lines to prefetch
    uint32_t ghb_head_hidden;
    bool delta_correlation; //delta correlation prefetch in addition to stride
    bool monitor_GETS; //trigger prefetcher on read misses
    bool monitor_GETX; //trigger prefetcher on write misses

    //One Index and Ghb per core
    std::vector<std::unordered_map<uint64_t, IndexEntry>> index;
    std::vector<std::vector<GhbEntry>> ghb;

    Counter prof_emitted_prefetches, prof_stride_prefetches;

    bool ghb_ptr_valid(uint32_t ghb_ptr, uint32_t coreId);
    uint32_t ghb_head_idx(uint32_t ghb_ptr);
    void schedReq(uint32_t srcId, uint64_t lineAddr);

public:
    explicit GhbPrefetcher(const g_string& _name,
                           uint32_t _ghb_size, uint32_t _index_size,
                           uint32_t _log_distance, uint32_t _degree,
                           const g_string &target_cache, bool _delta_correlation,
                           bool _monitor_GETS, bool _monitor_GETX) :
        CachePrefetcher(_name, target_cache),
        ghb_size(_ghb_size), index_size(_index_size),
        log_distance(_log_distance), degree(_degree), ghb_head_hidden(0),
        delta_correlation(_delta_correlation), monitor_GETS(_monitor_GETS),
        monitor_GETX(_monitor_GETX), index(zinfo->numCores),
        ghb(zinfo->numCores, std::vector<GhbEntry>(ghb_size)) {};

    void initStats(AggregateStat* parentStat) override;
    uint64_t access(MemReq& _req) override;
};

#endif
