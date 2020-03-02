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

#include "lbr.h"

#include <list>
#include <unordered_map>

struct prefetched_info
{
    Address vAddrLine;
    uint64_t availCycle;
    prefetched_info(Address _vAddrLine, uint64_t _availCycle)
    {
        vAddrLine = _vAddrLine;
        availCycle = _availCycle;
    }
};


class PrefetchBuffer
{
private:
    uint64_t capacity;
    std::list<prefetched_info> dq;
    std::unordered_map<Address,std::list<prefetched_info>::iterator> index;
public:
    PrefetchBuffer(uint64_t size)
    {
        capacity = size;
    }
    bool prefetch(Address addr, uint64_t availCycle)
    {
        bool was_present;
        uint64_t effectiveAvailCycle = availCycle;
        if(index.find(addr)==index.end())
        {
            was_present = false;
            if(likely(dq.size()==capacity))
            {
                Address to_be_removed = dq.back().vAddrLine;
                dq.pop_back();
                index.erase(to_be_removed);
            }
        }
        else
        {
            was_present = true;
            effectiveAvailCycle = MIN(index[addr]->availCycle, effectiveAvailCycle);
            dq.erase(index[addr]);
        }
        dq.push_front(prefetched_info(addr, effectiveAvailCycle));
        index[addr]=dq.begin();
        return was_present;
    }
    bool transfer(Address addr, uint64_t &availCycle)
    {
        if(index.find(addr)==index.end())
        {
            return false;
        }
        availCycle = index[addr]->availCycle;
        dq.erase(index[addr]);
        index.erase(addr);
        return true;
    }
    void clear()
    {
        index.clear();
        dq.clear();
    }
    ~PrefetchBuffer()
    {
        clear();
    }
};

class OOOFilterCache : public FilterCache {
private:
    //Number of lines to prefetch for a Next line prefetcher
    uint32_t numLinesNLP;
    bool zeroLatencyCache;
    uint32_t pref_degree;
#ifdef TRACE_BASED
    DataflowPrefetcher *dataflow_prefetcher;
#endif
    PrefetchBuffer *prefetch_buffer = nullptr;

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
            prefetch_buffer = nullptr;
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
            prefetch_buffer = nullptr;
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
                         OOOCoreRecorder *cRec, LBR_Stack *lbr=nullptr, bool no_update_timestamp=false, bool is_prefetch=false) {
        
        if(is_prefetch)
        {
            Address vLineAddr = vAddr >> lineBits;
            Address prefetchLineAddr = procMask | vLineAddr;
            return issuePrefetch(prefetchLineAddr, 0/*prefetch into L1*/, curCycle, dispatchCycle, cRec, 0 /*No PC*/, true, no_update_timestamp);
        }
        else
        {
            Address vLineAddr = vAddr >> lineBits;
            uint64_t respCycle = FilterCache::load(vAddr, dispatchCycle, pc,lbr,no_update_timestamp,is_prefetch);
            cRec->record(curCycle, dispatchCycle, respCycle);

            //Support legacy prefetching flow for backwards compatibility
            executePrefetch(curCycle, dispatchCycle, 0, cRec);
            for (uint32_t numLines = 1; numLines <= numLinesNLP; numLines++) {
                Address pLineAddr = procMask | vLineAddr;
                Address nextPLineAddr = pLineAddr + numLines;
                issuePrefetch(nextPLineAddr, 0/*prefetch into L1*/, curCycle, dispatchCycle, cRec, 0 /*No PC*/, false);
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
                return dispatchCycle + accLat;
            }
            uint64_t prefetchBufferCycle;
            if((prefetch_buffer!=nullptr) && prefetch_buffer->transfer(vLineAddr, prefetchBufferCycle))
            {
                return MIN(respCycle,prefetchBufferCycle);
            }

            return respCycle;
        }
    }

    inline uint64_t store(Address vAddr, uint64_t curCycle,
                          uint64_t dispatchCycle, Address pc,
                          OOOCoreRecorder *cRec) {
        uint64_t respCycle = FilterCache::store(vAddr, dispatchCycle, pc);
        cRec->record(curCycle, dispatchCycle, respCycle);

        //Support legacy prefetching flow for backwards compatibility
        executePrefetch(curCycle, dispatchCycle, 0, cRec);

        if (zeroLatencyCache) {
            return dispatchCycle + accLat;
        }
        
        uint64_t prefetchBufferCycle;
        Address vLineAddr = vAddr >> lineBits;
        if((prefetch_buffer!=nullptr) && prefetch_buffer->transfer(vLineAddr, prefetchBufferCycle))
        {
            return MIN(respCycle,prefetchBufferCycle);
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
                           uint64_t pc, bool isSW, bool no_update_timestamp=false) {
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
        req.no_update_timestamp = no_update_timestamp;
        respCycle = access(req);
        cRec->record(curCycle, dispatchCycle, respCycle);
        futex_unlock(&filterLock);
        return respCycle;
    }

    inline uint32_t getAccLat() { return accLat; }

    void setPrefetchBuffer(uint64_t capacity)
    {
        if(prefetch_buffer==nullptr)
        {
            prefetch_buffer = new PrefetchBuffer(capacity);
        }
    }
    void clearPrefetchBuffer()
    {
        if(prefetch_buffer!=nullptr)
        {
            prefetch_buffer->clear();
            delete prefetch_buffer;
        }
    }

    inline void prefetch_into_buffer(Address vAddr, uint64_t curCycle)
    {
        const uint64_t prefetch_cost = 7;
        Address vLineAddr = vAddr >> lineBits;
        if(prefetch_buffer!=nullptr)
        {
            prefetch_buffer->prefetch(vLineAddr, curCycle+prefetch_cost);
        }
    }
};

#endif
