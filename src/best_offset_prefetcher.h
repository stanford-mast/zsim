/** $lic$
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

#ifndef BEST_OFFSET_PREFETCHER_H_
#define BEST_OFFSET_PREFETCHER_H_

#ifndef TEST 
#include "cache_prefetcher.h"
#include "filter_cache.h"
#include "g_std/g_string.h"
#include "memory_hierarchy.h"
#include "stats.h" 
#include "hash.h"
// hopefully don't need to include this for tests in the .cpp
#else 
#include <map>
#include <vector>
#include <iostream>
#include <cmath> 
#include <cmath> 
#endif

const uint64_t L3_ACCESS_TIME = 49;
// num run throughs to make before round over
const uint64_t RND_MAX = 100;
// max score before round ended
///const uint64_t MAX_SCORE = 10;
const uint64_t MAX_SCORE = 31;
const uint64_t BAD_SCORE = 1;
// offset to start prefetching with
const uint64_t INIT_OFFSET = 1;
const uint64_t NUM_OFFSETS = 63;
const uint64_t MAX_SIZE = RND_MAX * NUM_OFFSETS;

const uint64_t PAGE_SIZE = pow(2, 12);
const uint64_t  PAGE_MASK = 0xFFFFFFFFFFFFFFFF - PAGE_SIZE + 1;
// 64 for normal zsim operation
// rounds to wait until can prefetch again
const uint64_t ROUNDS_TO_WAIT = 2;

class RR {
    public:
        void insert(uint64_t, uint64_t);
        bool exists(uint64_t, uint64_t);
        RR(); 
        uint64_t size = 0;
    private:
        std::map<uint64_t, std::pair<uint64_t,uint64_t>> hash;
        std::deque<std::pair<uint64_t, uint64_t>> list;
        void deletion();
        uint64_t id = 0; 
};

#ifdef TEST
class BestOffsetPrefetcher {
public:
    explicit BestOffsetPrefetcher();
    uint64_t access(uint64_t _req);
    void prefetch(uint64_t _req);
    uint64_t learn(uint64_t);
#else
class BestOffsetPrefetcher : public CachePrefetcher {
public:
    explicit BestOffsetPrefetcher(const g_string& _name, const g_string& _target,
                                bool _monitor_GETS, bool _monitor_GETX,
                                uint32_t _degree);
    void initStats(AggregateStat* _parentStat) override;
    uint64_t access(MemReq& _req) override;
    void prefetch(MemReq& _req) override;
    void learn(uint64_t,uint64_t);
#endif

    void reset_prefetcher();
private:
    // recent requests list implementation
    RR recent_requests; 
    // functions
    void move_test_offset_ptr();
    void reset_offsets();
    uint64_t find_max_score();
    bool ok_to_prefetch();
    void print_scores();
    // private vars
    bool monitor_GETS_;
    bool monitor_GETX_;
    uint32_t degree_;
    std::vector<std::pair<const uint64_t, uint64_t>> offset_scores;
    int rounds_to_wait;
    uint64_t current_round;
    uint64_t test_offset_index;
    uint64_t current_offset;
#ifndef TEST
    Counter prof_emitted_prefetches_;
#endif 
};

#endif
