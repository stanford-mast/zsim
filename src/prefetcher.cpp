/** $lic$
 * Copyright (C) 2017 by Google
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

#include "prefetcher.h"
#include "bithacks.h"
#include "hash.h"
#include <limits>
#include "g_std/g_list.h"
#include <unordered_set>

//#define DBG(args...) info(args)
#define DBG(args...)

void StreamPrefetcher::initStats(AggregateStat* parentStat) {
    AggregateStat* s = new AggregateStat();
    s->init(name_.c_str(), "Prefetcher stats");
    profAccesses.init("acc", "Accesses"); s->append(&profAccesses);
    profPrefetches.init("pf", "Issued prefetches"); s->append(&profPrefetches);
    profDoublePrefetches.init("dpf", "Issued double prefetches"); s->append(&profDoublePrefetches);
    profTriplePrefetches.init("tpf", "Issued triple prefetches");
    s->append(&profTriplePrefetches);
    profQuadPrefetches.init("qpf", "Issued quad prefetches");
    s->append(&profQuadPrefetches);
    profPageHits.init("pghit", "Page/entry hit"); s->append(&profPageHits);
    profHits.init("hit", "Prefetch buffer hits, short and full"); s->append(&profHits);
    profShortHits.init("shortHit", "Prefetch buffer short hits"); s->append(&profShortHits);
    profStrideSwitches.init("strideSwitches", "Predicted stride switches"); s->append(&profStrideSwitches);
    profLowConfAccs.init("lcAccs", "Low-confidence accesses with no prefetches"); s->append(&profLowConfAccs);
    parentStat->append(s);
     prof_emitted_prefetches_.init("pff", "Total prefetches emitted");
    s->append(&prof_emitted_prefetches_);
}

void StreamPrefetcher::schedReq(uint32_t srcId, uint64_t lineAddr) {
    auto filterCache_ = d_caches_[srcId].first;
    auto distance_ = d_caches_[srcId].second;

    prof_emitted_prefetches_.inc();
    filterCache_->schedulePrefetch(lineAddr, distance_);
}

uint64_t StreamPrefetcher::access(MemReq& req) {
    req.childId = childId_;
    uint32_t bank = CacheBankHash::hash(req.lineAddr, parents_.size());
    uint64_t respCycle = parents_[bank]->access(req);

  if (req.flags & MemReq::SPECULATIVE) {
        return respCycle;
    }
    //Only do data prefetches for now
  if (req.flags & MemReq::IFETCH) {
        return respCycle;
    }

  bool monitored = ( req.type == GETS) || ( req.type == GETX);
  if (!monitored) {
        return respCycle;
    }
   prefetch(req);
   return respCycle;
}

void StreamPrefetcher::prefetch(MemReq& req) {
    profAccesses.inc();
    uint32_t coreId = req.srcId;
    assert(coreId < zinfo->numCores);
    Address pageAddr = req.lineAddr >> log_distance;
    uint32_t pos = req.lineAddr & (distance - 1);
    uint32_t idx = streams;
    // This loop gets unrolled and there are no control dependences. Way faster than a break (but should watch for the avoidable loop-carried dep)
    for (uint32_t i = 0; i < streams; i++) {
        bool match = (pageAddr == tag[i]);
        idx = match?  i : idx;  // ccmov, no branch
    }

    DBG("%s: 0x%lx page %lx pos %d", name.c_str(), req.lineAddr, pageAddr, pos);

    if (idx == streams) {  // entry miss
        uint32_t cand = streams;
        uint64_t candScore = -1;
        //uint64_t candScore = 0;
        for (uint32_t i = 0; i < streams; i++) {
            if (array[i].lastCycle > req.cycle + 500) continue;  // warm prefetches, not even a candidate
            if (array[i].ts < candScore) {  // just LRU
                cand = i;
                candScore = array[i].ts;
            }
        }
        if (cand < streams) {
            idx = cand;
            array[idx].alloc(req.cycle);
            array[idx].lastPos = pos;
            array[idx].ts = timestamp++;
            tag[idx] = pageAddr;
        }
        DBG("%s: MISS alloc idx %d cand %d", name.c_str(), idx, cand);
    } else {  // entry hit
        profPageHits.inc();
        Entry& e = array[idx];
        array[idx].ts = timestamp++;
        DBG("%s: PAGE HIT idx %d", name.c_str(), idx);
        bool shortPrefetch = false;
	// 1. Did we prefetch-hit?
	 if (e.valid[pos]) {
		 e.valid[pos] = false;  // close, will help with long-lived transactions
		  profHits.inc();
	 }
	 // 2. Update predictors, issue prefetches
        int32_t stride = pos - e.lastPos;
        DBG("%s: pos %d lastPos %d lastLastPost %d e.stride %d stride %d", name.c_str(), pos, e.lastPos, e.lastLastPos, e.stride, stride);
        if (e.stride == stride) {
            e.conf.inc();
            if (e.conf.pred()) {  // do prefetches
                int32_t fetchDepth = (e.lastPrefetchPos - e.lastPos)/stride;
                uint32_t prefetchPos = e.lastPrefetchPos + stride;
                if (fetchDepth < 1) {
                    prefetchPos = pos + stride;
                    fetchDepth = 1;
                }
                DBG("%s: pos %d stride %d conf %d lastPrefetchPos %d prefetchPos %d fetchDepth %d", name.c_str(), pos, stride, e.conf.counter(), e.lastPrefetchPos, prefetchPos, fetchDepth);

                if (prefetchPos < distance && !e.valid[prefetchPos]) {
                    MESIState state = I;
                    MemReq pfReq = {0 /*no PC*/, req.lineAddr + prefetchPos - pos, GETS, req.childId, &state, req.cycle, req.childLock, state, req.srcId, MemReq::PREFETCH, 0 /*not an in-cache prefetch*/};
                    schedReq(pfReq.srcId, pfReq.lineAddr);
                    e.valid[prefetchPos] = true;
                    profPrefetches.inc();
                    if (degree > 1 && shortPrefetch && fetchDepth < 8 && prefetchPos + stride < distance && !e.valid[prefetchPos + stride]) {
                        prefetchPos += stride;
                        pfReq.lineAddr += stride;
			schedReq(pfReq.srcId, pfReq.lineAddr);
                        e.valid[prefetchPos] = true;
                        profPrefetches.inc();
                        profDoublePrefetches.inc();
                    }
                    if (degree > 2 && shortPrefetch && fetchDepth < 8 && prefetchPos + stride < distance && !e.valid[prefetchPos + stride]) {
                        prefetchPos += stride;
                        pfReq.lineAddr += stride;
			schedReq(pfReq.srcId, pfReq.lineAddr);
                        e.valid[prefetchPos] = true;
                        profPrefetches.inc();
                        profTriplePrefetches.inc();
                    }
                    if (degree > 3 && shortPrefetch && fetchDepth < 8 && prefetchPos + stride < distance && !e.valid[prefetchPos + stride]) {
                        prefetchPos += stride;
                        pfReq.lineAddr += stride;
		        schedReq(pfReq.srcId, pfReq.lineAddr);
                        e.valid[prefetchPos] = true;
                        profPrefetches.inc();
                        profQuadPrefetches.inc();
                    }
                    e.lastPrefetchPos = prefetchPos;
                    assert(state == I);  // prefetch access should not give us any permissions
                }
            } else {
                profLowConfAccs.inc();
            }
        } else {
            e.conf.dec();
            // See if we need to switch strides
            if (!e.conf.pred()) {
                int32_t lastStride = e.lastPos - e.lastLastPos;
                if (stride && stride != e.stride && stride == lastStride) {
                    e.conf.reset();
                    e.stride = stride;
                    profStrideSwitches.inc();
                }
            }
            e.lastPrefetchPos = pos;
        }
        e.lastLastPos = e.lastPos;
        e.lastPos = pos;
    }
}

// nop for now; do we need to invalidate our own state?
uint64_t StreamPrefetcher::invalidate(const InvReq& req) {
    uint64_t respCycle = 0;
    for (auto child : children_) {
        respCycle = MAX(respCycle, child->invalidate(req));
    }
    return respCycle;
}
