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

#ifndef OOO_FILTER_CACHE_H_
#define OOO_FILTER_CACHE_H_

#include "filter_cache.h"
#ifdef TRACE_BASED
#include "dataflow_prefetcher.h"
#endif

class OOOFilterCache : public FilterCache {
private:
    //Number of lines to prefetch for a Next line prefetcher
    uint32_t numLinesNLP;
    bool zeroLatencyCache;
    uint32_t pref_degree;
#ifdef TRACE_BASED
    DataflowPrefetcher *dataflow_prefetcher;
#endif

public:
    OOOFilterCache(uint32_t _numSets, uint32_t _numLines, CC* _cc,
                   CacheArray* _array, ReplPolicy* _rp, uint32_t _accLat,
                   uint32_t _invLat, g_string& _name)
        : FilterCache(_numSets, _numLines, _cc, _array, _rp, _accLat,
                      _invLat, _name)
        {
            numLinesNLP = 0;
            zeroLatencyCache = false;
            pref_degree = 0;
        }

    explicit OOOFilterCache(uint32_t _numSets, uint32_t _numLines, CC* _cc,
                            CacheArray* _array, ReplPolicy* _rp,
                            uint32_t _accLat, uint32_t _invLat, g_string& _name,
                            uint32_t _numLinesNLP, bool _zeroLatencyCache)
        : OOOFilterCache(_numSets, _numLines, _cc, _array, _rp, _accLat,
                         _invLat, _name)
        {
            numLinesNLP = _numLinesNLP;
            zeroLatencyCache = _zeroLatencyCache;
        }

#ifdef TRACE_BASED
    explicit OOOFilterCache(uint32_t _numSets, uint32_t _numLines, CC* _cc,
                            CacheArray* _array, ReplPolicy* _rp,
                            uint32_t _accLat, uint32_t _invLat, g_string& _name,
                            uint32_t _numLinesNLP, bool _zeroLatencyCache,
                            uint32_t _pref_degree, bool _pref_constant,
                            bool _pref_complex, bool _limit_prefetching,
                            bool _var_degree, g_string _pref_kernels_file)
        : OOOFilterCache(_numSets, _numLines, _cc, _array, _rp, _accLat,
                         _invLat, _name)
        {
            numLinesNLP = _numLinesNLP;
            zeroLatencyCache = _zeroLatencyCache;
            pref_degree = _pref_degree;
	    //TODO: Enable selection of arbitrary prefetchers
            if (pref_degree) {
                dataflow_prefetcher =
                    new DataflowPrefetcher("dataflow_" + _name,
                                           _pref_degree, _name, true,
                                           false, _pref_constant,
                                           _pref_complex,
                                           _limit_prefetching,
                                           _var_degree,this,
                                           _pref_kernels_file);
            }
        }
#endif

    inline uint64_t load(Address vAddr, uint64_t curCycle,
                         uint64_t dispatchCycle, Address pc,
                         OOOCoreRecorder *cRec) {
        Address vLineAddr = vAddr >> lineBits;
        //L1 latency as returned by load() is zero, hence add accLat
        uint64_t respCycle = FilterCache::load(vAddr, dispatchCycle, pc);
        cRec->record(curCycle, dispatchCycle, respCycle);

        //Support legacy prefetching flow for backwards compatibility
        executePrefetch(curCycle, dispatchCycle, 0, cRec);

        // Access based Next Line Prefetcher
        // Default value for number of lines to prefetch is 0,
        // in which case we don't prefetch anything.
        // How many next lines to prefetch can be configured in the config.
        for (uint32_t numLines = 1; numLines <= numLinesNLP; numLines++) {
            Address pLineAddr = procMask | vLineAddr;
            Address nextPLineAddr = pLineAddr + numLines;
            issuePrefetch(nextPLineAddr, 0/*prefetch into L1*/, curCycle,
                          dispatchCycle, cRec, 0 /*No PC*/, false);
        }
#ifdef TRACE_BASED
        //Access Dataflow Prefetcher
        if (pref_degree) {
            MESIState dummyState = MESIState::I;
            MemReq req = {pc, vLineAddr, GETS, 1, &dummyState, dispatchCycle,
                          NULL, dummyState, getSourceId(), 0};
            dataflow_prefetcher->prefetch(req);
        }
#endif

        if (zeroLatencyCache) {
            return dispatchCycle;
        }

        return respCycle;
    }

    inline uint64_t store(Address vAddr, uint64_t curCycle,
                          uint64_t dispatchCycle, Address pc,
                          OOOCoreRecorder *cRec) {
        uint64_t respCycle = FilterCache::store(vAddr, dispatchCycle, pc);
        cRec->record(curCycle, dispatchCycle, respCycle);

        //Support legacy prefetching flow for backwards compatibility
        executePrefetch(curCycle, dispatchCycle, 0, cRec);

        if (zeroLatencyCache) {
            return dispatchCycle;
        }

        return respCycle;
    }

#ifdef TRACE_BASED
    inline void postInit() {
        if (pref_degree) {
            dataflow_prefetcher->postInit();
        }
    }

    inline void initStats(AggregateStat* parentStat) {
        AggregateStat* cacheStat = new AggregateStat();
        cacheStat->init(name.c_str(), "Filter cache stats");

        ProxyStat* fgetsStat = new ProxyStat();
        fgetsStat->init("fhGETS", "Filtered GETS hits", &fGETSHit);
        ProxyStat* fgetxStat = new ProxyStat();
        fgetxStat->init("fhGETX", "Filtered GETX hits", &fGETXHit);
        cacheStat->append(fgetsStat);
        cacheStat->append(fgetxStat);

        if (pref_degree) {
            dataflow_prefetcher->initStats(cacheStat);
        }

        initCacheStats(cacheStat);
        parentStat->append(cacheStat);
    }
#endif

    uint64_t issuePrefetch(Address lineAddr, uint32_t skip, uint64_t curCycle,
                           uint64_t dispatchCycle, OOOCoreRecorder *cRec,
                           uint64_t pc, bool isSW) {
        uint64_t respCycle;
        futex_lock(&filterLock);
        MESIState dummyState = MESIState::I;
        uint32_t flags = MemReq::PREFETCH | MemReq::SPECULATIVE;
        if (isSW) {
            flags |= MemReq::SW_SPECULATIVE;
        }
        MemReq req = {pc, lineAddr, GETS, 0, &dummyState,
                      dispatchCycle, &filterLock, dummyState, srcId,
                      flags, skip};
        respCycle = access(req);
        cRec->record(curCycle, dispatchCycle, respCycle);
        futex_unlock(&filterLock);
        return respCycle;
    }

    inline uint32_t getAccLat() { return accLat; }
};

#endif
