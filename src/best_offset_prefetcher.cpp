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
* The Best Offset Prefetcher is a prefetching algorithm proposed in the paper 
* Best Offset Hardware Prefetching written by Pierre Michaud. It is a non-PC 
* based prefetching algorithm that propes offsets from an access address
* and selects an offset that generates the most timely prefetches. The prefetcher
* monitors L2 misses and prefetches into the L3 cache.
* 
* A brief API level description: an access comes in (access function), and the amount
* time of until that prefetch is ready is calculated based on the expected time until the access will
* make it to the cache. If the time for the prefetch is greater than the L3 access time
* (signalling that the access is going to DRAM rather than the L3 cache), then one of the 
* offsets within the page (0-63) is tested to see if a prefetch to a previous access + that
* test offset would have generated a timely prefetch. If so, then the "score" associated with 
* that offset is increased. When an offset's "score" approaches the maximum value or 
* all of the offsets have been gone through, the offset with the highest score is used to 
* prefetch from all the accesses that come in. All future accesses are prefetched with that
* offset from that point onwards, hopefully generating timely prefetches, and the next round of 
* learning begins.
* 
* Note that the implemenation of the recent requests table is more complex than that of the 
* algorithm proposed in the paper. The existence of addresses is checked against a hash map
* that tracks a queue of all the most recent requests. When the list gets to capacity,
* the oldest item at the front of the list is removed, and its associated entry in the hash
* map is deleted as well. This leads to a more consistent prefetching offset than the method that 
* the paper proposed, which is to delete all history of the last round by completely clearing 
* the recent request table.

*/

// helper function to reset offset scores to 0
void BestOffsetPrefetcher::resetOffsets(){
    for (auto& i : offset_scores_){
        i.second = 0; 
    } 
}

// function used to print all scores in the ofset list
void BestOffsetPrefetcher::printScores(){
    // line break to distinguish between the last print
    std::cout << std::endl;
    for (auto& i : offset_scores_){
        if (i.second != 0)
            std::cout << "Offset: " << i.first << " Score: " << i.second << std::endl;
    } 
    std::cout << std::endl;
}

// function to reset the prefetcher, updating stats, resetting rounds and offsets
void BestOffsetPrefetcher::resetPrefetcher(){
    // compute a moving average of the amount of rounds per phase
    total_phases_++;
#ifndef TEST
    average_rounds_.set(total_rounds_ / total_phases_);
    average_hits_per_round_.set(recent_requests_hits_.get() / total_phases_);
    // set the counter value
    average_max_score_.set(high_score_total_ / total_phases_);
#endif
    // return all offsets to 0
    resetOffsets();
    // set current round back to 0
    this->current_round_ = 0;
    // set the test offset back to the end index (for timely prefetches)
    this->test_offset_index_ = 0;
    // the version to clear all history versus hang onto the history from before
    //std::cout << "New chosen offset: " << this->current_offset_ << std::endl;
}

// function to fin the most successful offset of the last learning round
uint64_t BestOffsetPrefetcher::findMaxScore(){
    // set the max score as 1 to start to make it a next line prefetcher in the worst case 
    std::pair<uint64_t,uint64_t>  max_score_ = std::pair<uint64_t,uint64_t>(1, 1); 
    // run through all the offsets
    for (auto s : offset_scores_){
        if (s.second >= max_score_.second){
            // set the offset to the offset value
            max_score_.first = s.first;
            // set the second to the score
            max_score_.second = s.second; 
        }
    }
#ifndef TEST
    // set the max score stat
    if (max_score_.second > all_time_max_score_.get()){
        all_time_max_score_.set(max_score_.second); 
        all_time_max_score_offset_.set(max_score_.first); 
    }
#endif
    // set the max average score
    high_score_total_ += max_score_.second;
    // return the highest scoring offset in the list
    return max_score_.first; 
}

// function to change the current offset that we are testing
void BestOffsetPrefetcher::moveTestOffsetPtr(){
    // if the current offset is the total minus 1
    if (this->test_offset_index_ == (num_offsets - 1) ){
        // reset the pointer and set the round up
         // if max out the nuber of rounds, reset the prefetcher
         if (this->current_round_ == ( this->round_max_ ) ){
            this->current_offset_ = findMaxScore(); 
            //std::cout << "Resetting because of max rnd" << std::endl;
            resetPrefetcher();
        }else{
            this->test_offset_index_ = 0;
            this->current_round_++;
            this->total_rounds_++;
        }
    }
     else{
        // move address in test_offset_point along
        this->test_offset_index_++;
    }
}


// main learning function for the best offset prefetching algorithm
void BestOffsetPrefetcher::learn(uint64_t _addr, uint64_t _cycle){
    // offset to currently test for a base address existing in the recent requests table
#ifndef SCANNING
    uint64_t test_offset = (this->offset_scores_[this->test_offset_index_]).first;
    // if going off the page by subtracing the offset, don't consider the offset
    uint64_t base_addr = _addr - test_offset;
#ifdef LIMIT_TO_PAGE
    if ( ((_addr - test_offset) < 0 ) || ( (_addr & page_mask) != ((base_addr) & page_mask))){
        moveTestOffsetPtr();
        return;
    }
#endif

    // check if address in the recent requests table if on current page
    if (this->recent_requests_.exists(base_addr, _cycle) == true){
        // increase the statistic for the number of recent request hits
#ifndef TEST
        recent_requests_hits_.inc();
#endif
        // if increasing that index by 1 would put us over the max score reset the prefetcher
        if (this->offset_scores_[this->test_offset_index_].second == (this->max_score_ - uint64_t(1))){
            //std::cout << "Resetting because of max score" << std::endl;
            // function to print scores at the end of the the round. useful for debugging
            //printScores();
            // increase the score for that offset
            uint64_t max_score_offset = this->offset_scores_[this->test_offset_index_].first; 
            // set the max score to be the max possible score
#ifndef TEST
            all_time_max_score_.set(max_score_); 
            all_time_max_score_offset_.set(max_score_offset); 
#endif
            this->current_offset_ = max_score_offset;
            high_score_total_ += max_score_;
            // start a new learning phase with that new offset used for prefetching
            resetPrefetcher();
            // hang onto the index to preserve phase
            return;
        } // otherwise, if found the address but not maxed out, increment its score
        else{
            this->offset_scores_[this->test_offset_index_].second++; 
        } 
    }
    moveTestOffsetPtr();
#ifdef TEST
    this->recent_requests_.insert(_addr, _cycle); 
#endif
#else
    // for implementation of scanning 
    uint64_t max_score = 1;
    uint64_t max_score_offset = offset_scores_[0].first;
    for (uint64_t i = 0; i < num_offsets; i++){
        // check if the scanned offset exists in the table
        if (this->recent_requests_.exists((_addr - offset_scores_[i].first), _cycle)){
#ifndef TEST
            recent_requests_hits_.inc();
#endif
            offset_scores_[i].second++; 
        }
        if (offset_scores_[i].second >= max_score){
            max_score_offset = offset_scores_[i].first;
            max_score = offset_scores_[i].second; 
            //std::cout << "Updated max score: " << max_score << std::endl; 
        }
    }
    //printScores();
    // figure out if we're maxing out the scores
    if (max_score == this->max_score_){
        this->current_offset_ = max_score_offset; 
#ifndef TEST
            all_time_max_score_.set(max_score); 
            all_time_max_score_offset_.set(max_score_offset); 
            high_score_total_ += max_score;
#endif
        //std::cout << "Resetting because of max score" << std::endl;
        resetPrefetcher(); 
    }else{
        // increase the round
        //std::cout << "Round value before inc: " << this->current_round_ << std::endl;
        this->current_round_ += 1;
        //std::cout << "Round value after inc: " << this->current_round_ << std::endl;
        this->total_rounds_++;
        // if we hit the max found, reset the thing
        if (this->current_round_ == ( this->round_max_ ) ){
            high_score_total_ += max_score;
            this->current_offset_ = max_score_offset;
            //std::cout << "Highest score: " << max_score << std::endl;
            //std::cout << "Resetting because of max rnd" << std::endl;
            resetPrefetcher();
        }
    }

#ifdef TEST
    this->recent_requests_.insert(_addr, _cycle); 
#endif 

#endif
}
#ifndef TEST
BestOffsetPrefetcher::BestOffsetPrefetcher(const g_string& _name,
                                       const g_string& _target,
                                       bool _monitor_GETS,
                                       bool _monitor_GETX,
                                       uint32_t _degree,
                                       uint64_t _round_max,
                                       uint64_t _max_score,
                                       uint64_t _init_offset,
                                       uint64_t _target_latency,
                                       uint64_t rr_size
                                       ):
    CachePrefetcher(_name, _target),  monitor_GETS_(_monitor_GETS),
    monitor_GETX_(_monitor_GETX), degree_(1), round_max_(_round_max),
    max_score_(_max_score), init_offset_(_init_offset), target_latency_(_target_latency)
    {

    // BO only works with a degree of 1. Ignore any other value
    if (_degree != 1) {
        std::cout << "Ignoring degree parameter, using default degree of 1" <<std::endl;
    }
#else
BestOffsetPrefetcher::BestOffsetPrefetcher(uint64_t _max_score,
                                uint64_t _init_offset,
                                uint64_t rr_size,
                                uint64_t _round_max
                                ): 
#ifndef SCANNING
    round_max_(_round_max), max_score_(_max_score), init_offset_(_init_offset) {
#else
        round_max_ = _round_max * num_offsets;
        max_score_ = _max_score * num_offsets;
        init_offset_ = _init_offset;
#endif
#endif

    // init recent requests hash table
    this->recent_requests_ = RR();
    this->recent_requests_.max_size_ = rr_size;
    this->offset_scores_ = std::vector<std::pair<const uint64_t, uint64_t>>();
    // load up the offset scores table with the offset and score
#ifdef THRASH_OFFSETS
    offset_scores_.push_back(std::pair<const uint64_t, uint64_t>(1, 0));
    offset_scores_.push_back(std::pair<const uint64_t, uint64_t>(2, 0));
    offset_scores_.push_back(std::pair<const uint64_t, uint64_t>(3, 0));
    offset_scores_.push_back(std::pair<const uint64_t, uint64_t>(8, 0));
#else
#ifdef IN_PAGE_OFFSETS
    for (uint64_t i = 1; i <= num_offsets; ++i){ 
        offset_scores_.push_back(std::pair<const uint64_t, uint64_t>(i, 0));
    }
#else
    for (uint64_t i = 0; i < num_offsets; ++i){ 
        offset_scores_.push_back(std::pair<const uint64_t, uint64_t>(paper_offset_list[i], 0)); 
    }
#endif
#endif
    // set the current round to 0 of the learning phase
    this->current_round_ = 0;
    // set the first test offset to the first index in the offset list
    this->test_offset_index_ = 0;
    // set the prefetch offset to its init value (defined in config file)
    this->current_offset_ = this->init_offset_;
}

// create the stats for the prefetcher with clean data 
#ifndef TEST
void BestOffsetPrefetcher::initStats(AggregateStat* _parentStat) {
    AggregateStat* s = new AggregateStat();
    s->init(name_.c_str(), "Best Offset prefetcher stats");

    prof_emitted_prefetches_.init("pf", "Emitted prefetches");
    recent_requests_hits_.init("bh", "Base addresses found in the recent requests table");
    average_rounds_.init("ar", "Average rounds per learning phase");
    average_hits_per_round_.init("ah", "Average addresses found in the recent requests table per round");
    all_time_max_score_.init("tm", "Maximum score of an offset in offset list");
    all_time_max_score_offset_.init("to", "Offset associated with maximum score");
    average_max_score_.init("am", "Average highest score per phase");

    s->append(&prof_emitted_prefetches_);
    s->append(&recent_requests_hits_);
    s->append(&average_rounds_);
    s->append(&average_hits_per_round_);
    s->append(&all_time_max_score_);
    s->append(&all_time_max_score_offset_);
    s->append(&average_max_score_);
    
    _parentStat->append(s);
} 

// main entry point of best offset
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

        // execute the prefetch
        prefetch(_req); 
        // run the learning algo with the current cycle in mind
        learn(req.lineAddr, req.cycle);
        // put it in the RR
        this->recent_requests_.insert(req.lineAddr, resp_cycle); 
    //}
    return resp_cycle;
}

// prefetch function to schedule prefetches
void BestOffsetPrefetcher::prefetch(MemReq& _req) {
        auto &queue_info = d_caches_[_req.srcId];  //This is pair<cache-ptr, skip>
        uint64_t prefetch_location = _req.lineAddr + this->current_offset_;
#ifdef LIMIT_TO_PAGE
        if ((prefetch_location & page_mask) == (_req.lineAddr & page_mask)){
            queue_info.first->schedulePrefetch(prefetch_location, queue_info.second);
            // increment the number of prefetches made
            prof_emitted_prefetches_.inc();
        }
#else
        queue_info.first->schedulePrefetch(prefetch_location, queue_info.second);
        // increment the number of prefetches made
        prof_emitted_prefetches_.inc();

#endif
}
#endif
// function to insert an address into the recent requests table
void RR::insert(uint64_t _addr, uint64_t _cycle){
#ifdef SLIDING
    // to handle deleting
    if (this->list.size() == max_size_){
        this->deletion(); 
    }
    // put the most recent address in the back of the list
    this->list.push_back(std::pair<uint64_t, uint64_t>(_addr, id));
    // see if this address already exists in the map
    auto found = this->hash.find(_addr);
    // if the thing exists in the map already
    if (found != this->hash.end()){
        // set the hash id to the last element in the list's id
        found->second.first = this->list.back().second;
    }else { // if the map doesn't have this already
        // put the address of the back of the list in the map
        auto new_pair = std::make_pair(_addr, std::pair<uint64_t, uint64_t>(id++, _cycle));
        this->hash.insert(new_pair);
    }
    // increase the size of the list
    this->size_++;
#else
    uint64_t mapSlot = _addr % max_size_;
    //std::cout << "Map slot on insert: " << mapSlot << std::endl;
    if (this->hash.find(mapSlot) != this->hash.end()){
        //std::cout << "Map slot thrashed." << std::endl;
        this->hash.erase(mapSlot); 
        this->size_--;
    }
    auto new_pair = std::make_pair(mapSlot, std::pair<uint64_t, uint64_t>(_addr, _cycle));
    this->hash.insert(new_pair); 
    this->size_++;
#endif
}

// check if an address exists in the recent requests table
bool RR::exists(uint64_t _addr, uint64_t _cycle){
#ifdef SLIDING
    auto found = this->hash.find(_addr); 
    if (found != this->hash.end()){
        // make sure the completion cycle for the request is less than the current one
        uint64_t completion = found->second.second;
        if (completion <= _cycle){
            //std::cout << "Found address in RR" << std::endl;
            return true;
        }
    }
   return false;
#else
    uint64_t mapSlot = _addr % max_size_;
    //std::cout << "Map slot on exists: " << mapSlot << std::endl;
    auto found = this->hash.find(mapSlot); 
    if (found != this->hash.end()){
        // make sure the stored address is the same querying for
        if (found->second.first != _addr){
            return false;
        }
        uint64_t completion = found->second.second;
        if (completion <= _cycle){
            //std::cout << "Found address in RR" << std::endl;
            return true;
        }   
    } 
    return false;
#endif
}

// function to delete the front of the recent requests table
void RR::deletion(){
#ifdef SLIDING
    // get the front of the list (thing to delete)
    std::pair<uint64_t, uint64_t> front = list.front();
    // find the corresponding address in the hash map
    auto found = hash.find(front.first);
    // should always be true
    if (found != hash.end()){
        // check if the map entry is pointing to the front by its ID
        if (found->second.first == list.front().second){
            // if it's pointing to the front, delete it
            hash.erase(front.first); 
        }
        // remove the first element from the list
        list.pop_front(); 
    }
    this->size_--;
#else

#endif
}

void RR::clear(){
#ifdef SLIDING
    this->hash.clear();
    this->list.clear();
    this->size_ = 0; 
#else 
#endif
}

// init function for the recent requests table
RR::RR()
    {
#ifdef SLIDING
    this->hash = std::map<uint64_t, std::pair<uint64_t, uint64_t>>();
    this->list = std::deque<std::pair<uint64_t,uint64_t>>(); 
#else
    this->hash = std::map<uint64_t, std::pair<uint64_t, uint64_t>>(); 
#endif
}

