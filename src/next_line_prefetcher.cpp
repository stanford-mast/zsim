/** $lic$
 * Copyright (C) 2012-2015 by Massachusetts Institute of Technology
 * Copyright (C) 2010-2013 by The Board of Trustees of Stanford University
 * Copyright (C) 2017 by Google (implemented by Grant Ayers)
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

#include "hash.h"
#include "next_line_prefetcher.h"

NextLinePrefetcher::NextLinePrefetcher(const g_string& _name,
                                       const g_string& _target,
                                       bool _monitor_GETS, bool _monitor_GETX,
                                       uint32_t _degree) :
    CachePrefetcher(_name, _target),  monitor_GETS_(_monitor_GETS),
    monitor_GETX_(_monitor_GETX), degree_(_degree) {}

void NextLinePrefetcher::initStats(AggregateStat* _parentStat) {
    AggregateStat* s = new AggregateStat();
    s->init(name_.c_str(), "Next line prefetcher stats");
    prof_emitted_prefetches_.init("pf", "Emitted prefetches");
    s->append(&prof_emitted_prefetches_);
    _parentStat->append(s);
}

uint64_t NextLinePrefetcher::access(MemReq& _req) {
    MemReq req = _req;
    req.childId = childId_;
    uint32_t bank = CacheBankHash::hash(req.lineAddr, parents_.size());
    uint64_t resp_cycle = parents_[bank]->access(req);

    if (_req.flags & MemReq::SPECULATIVE) {
        return resp_cycle;
    }
    //Only do data prefetches for now
    if (req.flags & MemReq::IFETCH) {
        return resp_cycle;
    }
    bool monitored = (monitor_GETS_ && _req.type == GETS) ||
        (monitor_GETX_ && _req.type == GETX);
    if (!monitored) {
        return resp_cycle;  //Ignore other requests
    }

    prefetch(req);

    return resp_cycle;
}

void NextLinePrefetcher::prefetch(MemReq& _req) {
    auto &queue_info = d_caches_[_req.srcId];  //This is pair<cache-ptr, skip>
    uint64_t line_addr = _req.lineAddr;
    for (uint32_t i = 0; i < degree_; i++) {
        line_addr++;
        queue_info.first->schedulePrefetch(line_addr, queue_info.second);
        prof_emitted_prefetches_.inc();
    }
    return;
}
