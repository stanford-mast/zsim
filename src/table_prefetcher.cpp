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

#include "table_prefetcher.h"

#include "cache.h"
#include "g_std/g_list.h"
#include "hash.h"
#include "zsim.h"

#include <fstream>
#include <cstdlib>

#ifndef DBG
//#define DBG(args...) info(args)
#define DBG(args...)
#endif

TablePrefetcher::TablePrefetcher(const g_string& _name, const g_string& _target, bool _monitor_GETS, bool _monitor_GETX,
                                 const g_string& _table_file, double _conf_thresh, double _prob_thresh, uint32_t _degree,
                                 bool _stride_detection)
    : CachePrefetcher(_name, _target),  monitor_GETS_(_monitor_GETS), monitor_GETX_(_monitor_GETX),
    conf_thresh_(_conf_thresh), prob_thresh_(_prob_thresh), degree_(_degree), stride_detection_(_stride_detection) {
    readTable(_table_file);
}

void TablePrefetcher::initStats(AggregateStat* _parentStat) {
    AggregateStat* s = new AggregateStat();
    s->init(name_.c_str(), "Table prefetcher stats");
    prof_emitted_prefetches_.init("pf", "Total prefetches emitted");
    s->append(&prof_emitted_prefetches_);
    _parentStat->append(s);
}

uint64_t TablePrefetcher::access(MemReq& _req) {
    MemReq req = _req;
    req.childId = childId_;
    uint32_t bank = CacheBankHash::hash(req.lineAddr, parents_.size());
    uint64_t resp_cycle = parents_[bank]->access(req);
    if (_req.flags & MemReq::SPECULATIVE) {
        return resp_cycle;
    }
    bool monitored = (monitor_GETS_ && _req.type == GETS) || (monitor_GETX_ && _req.type == GETX);
    if (!monitored) {
        return resp_cycle;  //Ignore other requests
    }

    //Prefetch based on the table info indexed by the PC
    auto table_entry_iter = table_.find(_req.pc);
    if (table_entry_iter != table_.end()) {
        for (uint64_t delta : table_entry_iter->second) {
            assert(d_caches_.size() > _req.srcId);
            uint64_t pref_addr = _req.lineAddr + delta;
            schedulePrefetch(pref_addr, _req);

            //Amplify by degree without stride detection
            if (!stride_detection_) {
                for (uint32_t i = 1; i < degree_; i++) {
                    pref_addr += delta;
                    schedulePrefetch(pref_addr, _req);
                }
            }
        }
        //Amplify by degree with stride detection if so configured
        if (stride_detection_ && degree_ > 1) {
            auto &hist = history_[_req.pc];
            uint64_t delta = hist.second - hist.first;  //Could be 'negative'
            if ((delta == (_req.lineAddr - hist.second)) && table_entry_iter->second.count(delta)) {
                uint64_t pref_addr = _req.lineAddr + 2 * delta;
                for (uint32_t i = 1; i < degree_; i++) {
                    schedulePrefetch(pref_addr, _req);
                    pref_addr += delta;
                }
            }
            hist.first = hist.second;
            hist.second = _req.lineAddr;
        }
    }
    return resp_cycle;
}

void TablePrefetcher::readTable(const g_string& _name) {
    std::ifstream input(_name.c_str());
    if (!input) {
        panic("Couldn't read input file '%s'", _name.c_str());
        return;
    }
    uint64_t addr;
    int64_t delta;
    double confidence, probability;
    char sep1, sep2, sep3;
    uint64_t count = 0;
    uint64_t used = 0;
    uint32_t block_offset = ilog2(zinfo->lineSize);

    //Allow decimal, octal, or hex
    input.unsetf(std::ios::dec);
    input.unsetf(std::ios::oct);
    input.unsetf(std::ios::hex);

    while (input >> addr >> sep1 >> delta >> sep2 >> confidence >> sep3 >> probability) {
        count++;
        if (sep1 != ',' || sep2 != ',' || sep3 != ',') {
            panic("Invalid data in input file '%s'", _name.c_str());
            break;
        }
        if (confidence >= conf_thresh_ && probability >= prob_thresh_) {
            table_[addr].insert(static_cast<uint64_t>(delta) >> block_offset);
            used++;
        }
    }
    if (!input.eof()) {
        panic("Format error reading input file '%s'", _name.c_str());
    }
    info("'%s' using %lu of %lu prefetch table entries", name_.c_str(), used, count);
}

void TablePrefetcher::schedulePrefetch(uint64_t _addr, const MemReq &_req) {
    auto &queue_info = d_caches_[_req.srcId];  //This is pair<cache-ptr, skip>
    DBG("'%s' Prefetch PC 0x%lx, miss 0x%lx, target 0x%lx", name_.c_str(), _req.pc,
        _req.lineAddr << ilog2(zinfo->lineSize), _addr << ilog2(zinfo->lineSize));
    queue_info.first->schedulePrefetch(_addr, queue_info.second);
    prof_emitted_prefetches_.inc();
}
