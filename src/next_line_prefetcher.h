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

#ifndef NEXT_LINE_PREFETCHER_H_
#define NEXT_LINE_PREFETCHER_H_

#include "cache_prefetcher.h"
#include "filter_cache.h"
#include "g_std/g_string.h"
#include "memory_hierarchy.h"
#include "stats.h"


class NextLinePrefetcher : public CachePrefetcher {
public:
    explicit NextLinePrefetcher(const g_string& _name, const g_string& _target,
                                bool _monitor_GETS, bool _monitor_GETX,
                                uint32_t _degree);
    void initStats(AggregateStat* _parentStat) override;
    uint64_t access(MemReq& _req) override;
    void prefetch(MemReq& _req) override;

private:
    bool monitor_GETS_;
    bool monitor_GETX_;
    uint32_t degree_;
    Counter prof_emitted_prefetches_;
};

#endif
