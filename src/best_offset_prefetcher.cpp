/** $lic$
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

#include "best_offset_prefetcher.h"

/*
The Best Offset Prefetcher is a prefetching algorithm proposed in the paper 
Best Offset Hardware Prefetching written by Pierre Michaud. It is a non-PC 
based prefetching algorithm that propes offsets from an access address
and selects an offset that generates the most timely prefetches. The prefetcher
monitors L2 misses and prefetches into the L3 cache.

A brief API level description: an access comes in (access function), and the amount
time of until that prefetch is ready is calculated based on the expected time until the access will
make it to the cache. If the time for the prefetch is greater than the L3 access time
(signalling that the access is going to DRAM rather than the L3 cache), then one of the 
offsets within the page (0-63) is tested to see if a prefetch to a previous access + that
test offset would have generated a timely prefetch. If so, then the "score" associated with 
that offset is increased. When an offset's "score" approaches the maximum value or 
all of the offsets have been gone through, the offset with the highest score is used to 
prefetch from all the accesses that come in. All future accesses are prefetched with that
offset from that point onwards, hopefully generating timely prefetches, and the next round of 
learning begins.
*/

// helper to reset offset scores to 0
void BestOffsetPrefetcher::reset_offsets(){
    for (auto& i : offset_scores){
        i.second = 0; 
    } 
}
void BestOffsetPrefetcher::print_scores(){
    // line break to distinguish between the last print
    std::cout << std::endl;
    for (auto& i : offset_scores){
        if (i.second != 0)
            std::cout << "Offset: " << i.first << " Score: " << i.second << std::endl;
    } 
}

void BestOffsetPrefetcher::reset_prefetcher(){
    // return all offsets to 0
    reset_offsets();
    // set current round back to 0
    this->current_round = 0;
    // set the test offset back to the end index (for timely prefetches)
    this->test_offset_index = 0;
}

uint64_t BestOffsetPrefetcher::find_max_score(){
    std::pair<uint64_t,uint64_t>  max_score = std::pair<uint64_t,uint64_t>(1, BAD_SCORE); 
    for (auto s : offset_scores){
        if (s.second >= max_score.second){
            // set the offset to the offset value
            max_score.first = s.first;
            // set the second to the score
            max_score.second = s.second; 
        } 
    }
    return max_score.first; 
}

void BestOffsetPrefetcher::move_test_offset_ptr(){
    // if the current offset is the total minus 1
    if (this->test_offset_index == (NUM_OFFSETS - 1) ){
        // reset the pointer and set the round up
        this->test_offset_index = 0;
        this->current_round++;
    // if max out the nuber of rounds, reset the prefetcher
         if (this->current_round == ( RND_MAX + 1 ) ){
            this->current_offset = find_max_score(); 
            reset_prefetcher();
        }
    }
     else{
        // move address in test_offset_point along
        this->test_offset_index++;
    }
}


void BestOffsetPrefetcher::learn(uint64_t addr, uint64_t cycle){
    // offset to currently test
    uint64_t test_offset = (this->offset_scores[this->test_offset_index]).first;
    // if we're going off the page, dont' get it
    uint64_t base_addr = addr - test_offset;
    // if we're going off the page, dont' get it
    if ( ((addr - test_offset) < 0 ) || ( (addr & PAGE_MASK) != ((base_addr) & PAGE_MASK))){
        // move this offset along
        move_test_offset_ptr();
        return;
    }

    // check if address in the recent requests table
    if (this->recent_requests.exists(base_addr, cycle) == true){
        // if increasing that index by 1 would put us over the max score
            // reset the prefetcher
        if (this->offset_scores[this->test_offset_index].second == (MAX_SCORE-1)){
            print_scores();
            // increase the score for that offset
            this->current_offset = this->offset_scores[this->test_offset_index].first;
            reset_prefetcher();
            return;
        }
        // otherwise, if found the address but not maxed out, increment
            // its score
        else{
            this->offset_scores[this->test_offset_index].second++; 
        } 
    }
    move_test_offset_ptr();
}

BestOffsetPrefetcher::BestOffsetPrefetcher(const g_string& _name,
                                       const g_string& _target,
                                       bool _monitor_GETS, bool _monitor_GETX,
                                       uint32_t _degree):
    CachePrefetcher(_name, _target),  monitor_GETS_(_monitor_GETS),
    monitor_GETX_(_monitor_GETX), degree_(1) {
    if (_degree != 1) {
        std::cout << "Ignoring degree parameter, using default degree of 1" <<std::endl;
    }
    // init recent requests hash table
    this->recent_requests = RR();
    this->offset_scores = std::vector<std::pair<const uint64_t, uint64_t>>();
    // load up the offset scores table with the offset and score
    for (uint64_t i = 1; i <= NUM_OFFSETS; ++i){ 
        offset_scores.push_back(std::pair<const uint64_t, uint64_t>(i, 0));
    }
    // set the current round to 0
    this->current_round = 0;
    // set the first test offset to the first index in the offset list
    this->test_offset_index = 0;
    // set the prefetch offset to its init value (defined in header file)
    this->current_offset = INIT_OFFSET;
}

    // create the stats for the prefetcher with clean data

void BestOffsetPrefetcher::initStats(AggregateStat* _parentStat) {
    AggregateStat* s = new AggregateStat();
    s->init(name_.c_str(), "Best Offset prefetcher stats");
    prof_emitted_prefetches_.init("pf", "Emitted prefetches");
    s->append(&prof_emitted_prefetches_);
    _parentStat->append(s);
} 
// note that this function returns the data to whatever caller it wants,
//      while prefetch gets the next line

uint64_t BestOffsetPrefetcher::access(MemReq& _req) {
    MemReq req = _req;
    req.childId = childId_;
    // create a hash for which cache bank to access
    uint32_t bank = CacheBankHash::hash(req.lineAddr, parents_.size());
    // if a speculative request, return and don't prefetch
    uint64_t resp_cycle = parents_[bank]->access(req); 

    if (_req.flags & MemReq::SPECULATIVE) {
        return resp_cycle;
    }
    //Only do data prefetches for now
    //if an instruction fetch, don't prefetch
    if (req.flags & MemReq::IFETCH) {
        return resp_cycle;
    }
    // if not monitored, don't prefetch either
    bool monitored = (monitor_GETS_ && _req.type == GETS) ||
        (monitor_GETX_ && _req.type == GETX);
    if (!monitored) {
        return resp_cycle;  //Ignore other requests
    } 

    // if not an instruction fetch, monitored, or a speculative request
    //      execute a prefetch
    // need to figure out if the response cycle is going to DRAM or not
    prefetch(_req); 
    // run the prefetch on the address (better when the first thing you do)
    uint64_t time_to_fetch = resp_cycle - req.cycle;
    // put the line address and the completion time in the RR
    if (time_to_fetch >= L3_ACCESS_TIME){
        this->recent_requests.insert(req.lineAddr, resp_cycle); 
            // if it's going to DRAM, put it in the RR
            // otherwise, put it in the L3
        // run the learning algo with the current cycle in mind
        learn(req.lineAddr, req.cycle);
    }
    return resp_cycle;
}

// work starts in here
void BestOffsetPrefetcher::prefetch(MemReq& _req) {
        auto &queue_info = d_caches_[_req.srcId];  //This is pair<cache-ptr, skip>
        uint64_t prefetch_location = _req.lineAddr + this->current_offset;
        if ((prefetch_location & PAGE_MASK) == (_req.lineAddr & PAGE_MASK)){
            queue_info.first->schedulePrefetch(prefetch_location, queue_info.second);
            // increment the number of prefetches made
            prof_emitted_prefetches_.inc();
        }
}
// this actually needs to check it it's already in the hash map

void RR::insert(uint64_t addr, uint64_t cycle){
    // to handle deleting
    if (this->list.size() == MAX_SIZE){
        this->deletion(); 
    }
    // put the most recent address in the back of the list
    this->list.push_back(std::pair<uint64_t, uint64_t>(addr, id));
    // see if this address already exists in the map
    auto found = this->hash.find(addr);
    // if the thing exists in the map already
    if (found != this->hash.end()){
        // set the hash id to the last element in the list's id
        found->second.first = this->list.back().second;
    }else { // if the map doesn't have this already
        // put the address of the back of the list in the map
        auto new_pair = std::make_pair(addr, std::pair<uint64_t, uint64_t>(id++, cycle));
        this->hash.insert(new_pair);
    }
    this->size++;
}

bool RR::exists(uint64_t addr, uint64_t cycle){
    auto found = this->hash.find(addr); 
    if (found != this->hash.end()){
        // make sure the completion cycle for the requet 
            // is less than the current one
        uint64_t completion = found->second.second;
        if (completion <= cycle){
            return true;
        }
    }
   return false;
}

void RR::deletion(){
    // get the front of the list (thing to delete)
    std::pair<uint64_t, uint64_t> front = list.front();
    // find the corresponding address in the hash map
    auto found = hash.find(front.first);
    // should always be true
    if (found != hash.end()){
        // check if the map entry is pointing to the front 
        if (found->second.first== list.front().second){
            // if it's pointing to the front, delete it
            hash.erase(front.first); 
        }
        // remove the first element from the list
        list.pop_front(); 
    }
    this->size--;
}

RR::RR(){
    this->hash = std::map<uint64_t, std::pair<uint64_t, uint64_t>>();
    this->list = std::deque<std::pair<uint64_t,uint64_t>>(); 
}

