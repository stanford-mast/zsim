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

/* Nesbit & Smith: Data Cache Prefetching Using a Global History Buffer
 * Address space is partitioned into zones (e.g. page sized). Index table caches
 * the most frequency used zones. Each miss address is inserted at the head of
 * the GHB and a pointer to that last entry is installed in the index. Each GHB
 * entry holds a pointer to the previous miss in the same zone. On a miss, the
 * prefetcher traverses the GHB to find miss patterns (regular strides or any
 * other periodic signal). Prefetches are emitted to the parent cache level
 * according to that pattern. The cache looks up the line in the cache-array,
 * and, if not present emits the prefetch. The prefetch access will be received
 * by the prefetcher (as the prefetcher sits above the cache) and will be passed
 * on to the next level of the memory hierarchy without further action.
 */

#include "ghb_prefetcher.h"
#include "hash.h"
#include <limits>

constexpr uint32_t GHB_INVALID_PTR = std::numeric_limits<uint32_t>::max();

bool GhbPrefetcher::ghb_ptr_valid(uint32_t ghb_ptr, uint32_t coreId) {
    //ghb_ptr points to an entry that has already been evicted from the ghb
    if (ghb_ptr + ghb_size <= ghb_head_hidden) {
        return false;
    }

    return ghb[coreId][ghb_head_idx(ghb_ptr)].next != GHB_INVALID_PTR;
}

//converts a ghb_head_hidden into an actual ghb index
uint32_t GhbPrefetcher::ghb_head_idx(uint32_t ghb_ptr) {
    return ghb_ptr % ghb_size;
}

void GhbPrefetcher::initStats(AggregateStat* parentStat) {
    AggregateStat* s = new AggregateStat();
    s->init(name_.c_str(), "GHB Prefetcher stats");
    prof_emitted_prefetches.init("pf", "Actual prefetches emitted");
    s->append(&prof_emitted_prefetches);
    prof_stride_prefetches.init("spf", "Stride prefetches emitted (subset of emitted prefetches)");
    s->append(&prof_stride_prefetches);
    prof_prefetcher_accessed.init("accessed", "Prefetcher accessed");
    s->append(&prof_prefetcher_accessed);
    parentStat->append(s);
}

void GhbPrefetcher::schedReq(uint32_t srcId, uint64_t lineAddr) {
    auto filterCache = d_caches_[srcId].first;
    auto distance = d_caches_[srcId].second;

    prof_emitted_prefetches.inc();
    filterCache->schedulePrefetch(lineAddr, distance);
}

void GhbPrefetcher::prefetch(MemReq& _req) {
    prof_prefetcher_accessed.inc();

    uint32_t coreId = _req.srcId;
    assert(coreId < zinfo->numCores);
    uint64_t line_addr = _req.lineAddr;
    //Address space is partitioned into zones (i.e. pages).
    //Index table is CAM addressed with the czone.
    uint64_t czone_tag = line_addr >> log_distance;
    //ghb_head is wrapped at ghb_size and represents the actual
    //index for the ghb. ghb_head_hidden is a 32 bit var that
    //overflows much later and is used to determine whether a
    //gbh ptr is valid or already evicted from the ghb (see ghb_ptr_valid)
    uint32_t ghb_head = ghb_head_idx(ghb_head_hidden);
    assert(ghb_head < ghb_size);

    auto it_entry = index[coreId].find(czone_tag);

    if (it_entry == index[coreId].end()) {
        //Not present in the index table, if full, evict oldest entry
        if (index[coreId].size() >= index_size) {
            uint64_t oldest_ts = std::numeric_limits<uint64_t>::max();
            auto deleteme = index[coreId].begin();
            for (auto it = index[coreId].begin(); it != index[coreId].end();
                 it++) {
                if(it->second.last_access < oldest_ts) {
                    oldest_ts = it->second.last_access;
                    deleteme = it;
                }
            }
            index[coreId].erase(deleteme);
        }
        index[coreId].emplace(czone_tag, IndexEntry(ghb_head_hidden,
                                                     _req.cycle));
        //Add ghb entry to head without chain ptr
        ghb[coreId][ghb_head].line_addr = _req.lineAddr;
        ghb[coreId][ghb_head].next = GHB_INVALID_PTR;
    }
    else {
        //Index table hit
        IndexEntry *idx_entry = &it_entry->second;
        //Add ghb entry to head
        ghb[coreId][ghb_head].line_addr = _req.lineAddr;
        ghb[coreId][ghb_head].next = idx_entry->ghb_entry;

        //Update index
        idx_entry->ghb_entry = ghb_head_hidden;
        idx_entry->last_access = _req.cycle;

        //Now traverse the GHB and emit prefetches:
        //1. Find all deltas by pointer chasing through the GHB and computing
        //   diff  between 2 consecutive miss addresses
        std::vector<int64_t> delta_buffer;

        uint32_t ghb_ptr = ghb_head;
        uint64_t last_line_addr = line_addr;
        int64_t delta_1st, delta_2nd;

        assert(ghb_ptr < ghb_size);
        while (ghb_ptr_valid(ghb[coreId][ghb_ptr].next, coreId)){
            ghb_ptr = ghb_head_idx(ghb[coreId][ghb_ptr].next);
            assert(ghb_ptr < ghb_size);

            if (last_line_addr > ghb[coreId][ghb_ptr].line_addr) {
                int64_t delta = last_line_addr - ghb[coreId][ghb_ptr].line_addr;
                delta_buffer.push_back(delta);
            }
            else {
                int64_t delta = ghb[coreId][ghb_ptr].line_addr - last_line_addr;
                delta_buffer.push_back(-delta);
            }

            last_line_addr = ghb[coreId][ghb_ptr].line_addr;
        }

        //Determining prefetch pattern _requires at least two misses in the history
        if (delta_buffer.size() < 2) {
            ghb_head_hidden++;

            return;
        }

        auto it = delta_buffer.begin();
        delta_1st = *it++;
        auto last_it = it;
        delta_2nd = *it++;

        if (delta_1st == 0 || delta_2nd == 0) {}
        //2. Find regular strides
        else if (delta_1st == delta_2nd) {
            for (uint32_t i = 0; i < degree; i++) {
                line_addr += delta_1st;
                prof_stride_prefetches.inc();
                schedReq(_req.srcId, line_addr);
            }
        }
        //3. Find periodic signal
        else if (delta_correlation) {
            for (int period = 2; it != delta_buffer.end();
                 last_it = it, it++, period++) {
                if (*last_it == delta_1st && *it == delta_2nd) {
                    //First double match, determines the period of the prefetch
                    //pattern. Example history: delta_1st, delta_2nd, delta_x,
                    //delta_y, delta_1st, delta_2nd. In this case, period is 4
                    //and we would prefetch that sequence
                    for (uint32_t i = 0; i < degree && i < delta_buffer.size(); i++) {
                        line_addr += delta_buffer[i % period];
                        schedReq(_req.srcId, line_addr);
                    }
                    break;
                }
                last_it = it;
            }
        }
    }

    ghb_head_hidden++;
}

uint64_t GhbPrefetcher::access(MemReq& _req) {
    MemReq req = _req;
    req.childId = childId_;
    uint32_t bank = CacheBankHash::hash(req.lineAddr, parents_.size());
    uint64_t resp_cycle = parents_[bank]->access(req);

    if (req.flags & MemReq::SPECULATIVE) {
        return resp_cycle;
    }
    //Only do data prefetches for now
    if (req.flags & MemReq::IFETCH) {
        return resp_cycle;
    }
    bool monitored = (monitor_GETS && req.type == GETS)
        || (monitor_GETX && req.type == GETX);
    if (!monitored) {
        return resp_cycle;
    }

    prefetch(req);

    return resp_cycle;
}
