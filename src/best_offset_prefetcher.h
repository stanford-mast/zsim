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

/*
This file contains the function declarations of both the best offset prefetcher
as well as the recent requests table.  
*/


#ifndef BEST_OFFSET_PREFETCHER_H_
#define BEST_OFFSET_PREFETCHER_H_

#include "cache_prefetcher.h"
#include "filter_cache.h"
#include "g_std/g_string.h"
#include "memory_hierarchy.h"
#include "stats.h" 
#include "hash.h"

// declarations for the recent requests table implemenation
class RR {
    public:
        RR(); 
        void insert(uint64_t _addr, uint64_t _cycle);
        void clear();
        bool exists(uint64_t _addr, uint64_t _cycle);
        uint64_t size_ = 0;
        uint64_t max_size_;
        bool use_new_rr_;
    private:
        std::map<uint64_t, std::pair<uint64_t,uint64_t>> hash_;
        std::deque<std::pair<uint64_t, uint64_t>> list_;
        void deletion();
        uint64_t id_ = 0; 
};

class BestOffsetPrefetcher : public CachePrefetcher {
public:
    explicit BestOffsetPrefetcher(const g_string& _name,
                                const g_string& _target,
                                bool _monitor_GETS,
                                bool _monitor_GETX,
                                uint32_t _degree,
                                uint64_t _round_max,
                                uint64_t _max_score,
                                uint64_t _init_offset,
                                uint64_t _rr_size,
                                bool _new_rr,
                                bool _page_offsets,
                                bool _scanning_algorithm
                                );
    void initStats(AggregateStat* _parentStat) override;
    uint64_t access(MemReq& _req) override;
    void prefetch(MemReq& _req) override;
    void learn(uint64_t _addr, uint64_t _cycle); 
    void resetPrefetcher();
    uint64_t num_offsets_;
private:
    // recent requests list object
    RR recent_requests_; 
    // offset score object
    std::vector<std::pair<const uint64_t, uint64_t>> offset_scores_;
    // functions
    void moveTestOffsetPtr();
    void resetOffsets();
    uint64_t findMaxScore();
    void printScores();
    // private vars
    bool monitor_GETS_;
    bool monitor_GETX_;
    uint32_t degree_;
    uint64_t round_max_;
    const uint64_t max_score_;
    const uint64_t init_offset_;
    bool use_page_offsets_;
    bool use_scanning_algorithm_;
    uint64_t current_round_;
    uint64_t test_offset_index_;
    uint64_t current_offset_;
    uint64_t total_phases_ = 0;
    uint64_t total_rounds_ = 0;
    uint64_t high_score_total_ = 0;
    float average_hits_per_round__ = 0; 
    Counter prof_emitted_prefetches_, recent_requests_hits_, average_rounds_, average_hits_per_round_, all_time_max_score_, all_time_max_score_offset_, average_max_score_;
    std::vector<uint64_t> paper_offset_list_ {1,2,3,4,5,6,8,9,10,12,15,16,18,20,24,25,27,30,32,36,40,45,48,50,54,60,64,72,75,80,81,90,96,100,108,120,125,128,135,144,150,160,162,180,192,200,216,225,240,243,250,256}; 
};

#endif
