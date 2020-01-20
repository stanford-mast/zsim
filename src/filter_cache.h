/** $lic$
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

#ifndef FILTER_CACHE_H_
#define FILTER_CACHE_H_

#include "bithacks.h"
#include "cache.h"
#include "galloc.h"
#include "zsim.h"
#include "ooo_core_recorder.h"
#include "lbr.h"

/* Extends Cache with an L0 direct-mapped cache, optimized to hell for hits
 *
 * L1 lookups are dominated by several kinds of overhead (grab the cache locks,
 * several virtual functions for the replacement policy, etc.). This
 * specialization of Cache solves these issues by having a filter array that
 * holds the most recently used line in each set. Accesses check the filter array,
 * and then go through the normal access path. Because there is one line per set,
 * it is fine to do this without grabbing a lock.
 */

class FilterCache : public Cache {
    protected:
        struct FilterEntry {
            volatile Address rdAddr;
            volatile Address wrAddr;
            volatile uint64_t availCycle;

            void clear() {wrAddr = 0; rdAddr = 0; availCycle = 0;}
        };

        //Replicates the most accessed line of each set in the cache
        FilterEntry* filterArray;
        Address setMask;
        uint32_t numSets;
        uint32_t srcId; //should match the core
        uint32_t reqFlags;

        lock_t filterLock;
        uint64_t fGETSHit, fGETXHit;

        struct PrefetchInfo {
            Address addr;
            uint32_t skip;
            uint64_t pc;
            bool isSW;
            bool serialize; //Serializes this prefetch after the previous one (dispatchCycle)

            PrefetchInfo(Address _addr, uint32_t _skip, uint64_t _pc, bool _isSW, bool _serialize)
                : addr(_addr), skip(_skip), pc(_pc), isSW(_isSW), serialize(_serialize) {};
        };
        g_vector<PrefetchInfo> prefetchQueue;

    public:
        FilterCache(uint32_t _numSets, uint32_t _numLines, CC* _cc, CacheArray* _array,
                ReplPolicy* _rp, uint32_t _accLat, uint32_t _invLat, g_string& _name)
            : Cache(_numLines, _cc, _array, _rp, _accLat, _invLat, _name)
        {
            numSets = _numSets;
            setMask = numSets - 1;
            filterArray = gm_memalign<FilterEntry>(CACHE_LINE_BYTES, numSets);
            for (uint32_t i = 0; i < numSets; i++) filterArray[i].clear();
            futex_init(&filterLock);
            fGETSHit = fGETXHit = 0;
            srcId = -1;
            reqFlags = 0;
        }

        void setSourceId(uint32_t id) {
            srcId = id;
        }

        uint32_t getSourceId() const {
            return srcId;
        }

        void setFlags(uint32_t flags) {
            reqFlags = flags;
        }

        enum class Type {
            D, I
        };

        void setType(Type _type) {
            type = _type;
        }

        Type getType() const {
            return type;
        }

        void initStats(AggregateStat* parentStat) {
            AggregateStat* cacheStat = new AggregateStat();
            cacheStat->init(name.c_str(), "Filter cache stats");

            ProxyStat* fgetsStat = new ProxyStat();
            fgetsStat->init("fhGETS", "Filtered GETS hits", &fGETSHit);
            ProxyStat* fgetxStat = new ProxyStat();
            fgetxStat->init("fhGETX", "Filtered GETX hits", &fGETXHit);
            cacheStat->append(fgetsStat);
            cacheStat->append(fgetxStat);

            initCacheStats(cacheStat);
            parentStat->append(cacheStat);
        }

        inline uint64_t load(Address vAddr, uint64_t curCycle, Address pc, LBR_Stack *lbr=nullptr, bool no_update_timestamp=false) {
            Address vLineAddr = vAddr >> lineBits;
            uint32_t idx = vLineAddr & setMask;
            uint64_t availCycle = filterArray[idx].availCycle; //read before, careful with ordering to avoid timing races

            if (vLineAddr == filterArray[idx].rdAddr) {
                fGETSHit++;
                return MAX(curCycle, availCycle);
            } else {
                return replace(vLineAddr, idx, true, curCycle, pc, lbr, no_update_timestamp);
            }
        }

        inline uint64_t store(Address vAddr, uint64_t curCycle, Address pc) {
            Address vLineAddr = vAddr >> lineBits;
            uint32_t idx = vLineAddr & setMask;
            uint64_t availCycle = filterArray[idx].availCycle; //read before, careful with ordering to avoid timing races
            if (vLineAddr == filterArray[idx].wrAddr) {
                fGETXHit++;
                //NOTE: Stores don't modify availCycle; we'll catch matches in the core
                //filterArray[idx].availCycle = curCycle; //do optimistic store-load forwarding
                return MAX(curCycle, availCycle);
            } else {
                return replace(vLineAddr, idx, false, curCycle, pc);
            }
        }

        uint64_t replace(Address vLineAddr, uint32_t idx, bool isLoad, uint64_t curCycle, Address pc, LBR_Stack *lbr=nullptr, bool no_update_timestamp=false) {
          //assert(prefetchQueue.empty());
            Address pLineAddr = procMask | vLineAddr;
            MESIState dummyState = MESIState::I;
            futex_lock(&filterLock);
            MemReq req = {pc, pLineAddr, isLoad? GETS : GETX, 0, &dummyState, curCycle, &filterLock, dummyState, srcId, reqFlags};
            if(lbr)
            {
                req.core_lbr = lbr;
            }
            else
            {
                req.core_lbr = nullptr;
            }
            req.no_update_timestamp = no_update_timestamp;
            uint64_t respCycle  = access(req);

            //Due to the way we do the locking, at this point the old address might be invalidated, but we have the new address guaranteed until we release the lock

            //Careful with this order
            Address oldAddr = filterArray[idx].rdAddr;
            filterArray[idx].wrAddr = isLoad? -1L : vLineAddr;
            filterArray[idx].rdAddr = vLineAddr;

            //For LSU simulation purposes, loads bypass stores even to the same line if there is no conflict,
            //(e.g., st to x, ld from x+8) and we implement store-load forwarding at the core.
            //So if this is a load, it always sets availCycle; if it is a store hit, it doesn't

            if (oldAddr != vLineAddr) filterArray[idx].availCycle = respCycle;

            futex_unlock(&filterLock);
            return respCycle;
        }

        uint64_t invalidate(const InvReq& req) {
            Cache::startInvalidate();  // grabs cache's downLock
            futex_lock(&filterLock);
            uint32_t idx = req.lineAddr & setMask; //works because of how virtual<->physical is done...
            if ((filterArray[idx].rdAddr | procMask) == req.lineAddr) { //FIXME: If another process calls invalidate(), procMask will not match even though we may be doing a capacity-induced invalidation!
                filterArray[idx].wrAddr = -1L;
                filterArray[idx].rdAddr = -1L;
            }
            uint64_t respCycle = Cache::finishInvalidate(req); // releases cache's downLock
            futex_unlock(&filterLock);
            return respCycle;
        }

        void schedulePrefetch(Address lineAddr, uint32_t skip, uint64_t pc, bool isSW, bool serialize) {
            // A prefetch from any level of the memory hierarchy is inserted in the filter cache queue of the core
            // whose demand access triggered the prefetch. By originating all prefetch requests from the filter caches,
            // we avoid races and complexities that come with injecting new accesses into an in-progress request.
            // The 'skip' parameter is decremented at each cache level and the prefetch block is allocated starting at
            // the cache for which the value is zero.
            prefetchQueue.emplace_back(lineAddr, skip, pc, isSW, serialize);
        }

        void schedulePrefetch(Address lineAddr, uint32_t skip) {
            prefetchQueue.emplace_back(lineAddr, skip, 0, false, false);
        }

        void executePrefetch(uint64_t curCycle, uint64_t dispatchCycle, uint64_t reqSatisfiedCycle, OOOCoreRecorder *cRec) {
            futex_lock(&filterLock);
            //Send out any prefetch requests that were created during the prior access
            if (unlikely(!prefetchQueue.empty())) {
                uint64_t respCycle = dispatchCycle;
                for (auto& prefetch : prefetchQueue) {
                    MESIState dummyState = MESIState::I;
                    if (prefetch.serialize) {
                        dispatchCycle = respCycle;
                    }
                    MemReq req = {prefetch.pc, prefetch.addr, GETS, 0, &dummyState, dispatchCycle, &filterLock, dummyState,
                                  srcId, MemReq::PREFETCH | MemReq::SPECULATIVE | (prefetch.isSW ? MemReq::SW_SPECULATIVE : 0U),
                                  prefetch.skip};
                    respCycle = access(req);
                    cRec->record(curCycle, dispatchCycle, respCycle);
                }
                prefetchQueue.clear();
            }
            futex_unlock(&filterLock);
        }

        void contextSwitch() {
            futex_lock(&filterLock);
            for (uint32_t i = 0; i < numSets; i++) filterArray[i].clear();
            futex_unlock(&filterLock);
        }

    private:
        Type type;
};

#endif  // FILTER_CACHE_H_
