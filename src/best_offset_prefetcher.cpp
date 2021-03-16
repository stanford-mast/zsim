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
* based prefetching algorithm that probes offsets from an access address
* and selects an offset that generates the most timely prefetches. The prefetcher
* monitors L2 misses and prefetches into the L3 cache.
* 
* A brief API level description: an access comes in (access function) represented by
* the address of the requested line and the cycle the access will complete. The recent
* requests table is checked to see if that address minus an offset (another address)
* was recently seen and was timely, i.e. made it to the target cache.
* 
* If so, then the "score" associated with that offset is increased.
* When an offset's "score" approaches the maximum value or all of the offsets
* have been gone through, the offset with the highest score is used to 
* prefetch from all the accesses that come in. All future accesses are prefetched with that
* offset from that point onwards, and the next round of learning begins.
* 
* Note that if the configuration of the prefetcher has the "new_rr" prefetcher config set as "true",
* the address tracking algorithm will be different than what is proposed in the paper.
* The existence of addresses is checked against a hash map that tracks a queue of
* all the most recent requests.  When the list gets to capacity, * the oldest item at the
* front of the list is removed, and its associated entry in the hash map is deleted as well.
* This leads to a more consistent prefetching offset than the method proposed by the paper.  
*
* Note that if the configuration of the prefetcher has the "page_offsets" prefetcher config set as "true",
* only offsets within the page will be used (1-63). If the option is set to "false", then the 
* paper offsets list (found in the bottom of the best_offset_prefetcher.h file) will be used.

* Note that if the configuration of the prefetcher has the "scanning_algorithm" prefetcher config paramater
* set as "true", the prefetcher will use the scanning learning algorithm rather than the original
* learning algorithm. Note that in experimental trials on SPEC2006 workloads this
* learning method did not beat the original learning algorithm (measured by IPC speed up
* over a no-prefetching system).  
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
    average_rounds_.set(total_rounds_ / total_phases_);
    average_hits_per_round_.set(recent_requests_hits_.get() / total_phases_);
    // set the counter value
    average_max_score_.set(high_score_total_ / total_phases_);
    // return all offsets to 0
    resetOffsets();
    // set current round back to 0
    this->current_round_ = 0;
    // set the test offset back to the end index (for timely prefetches)
    this->test_offset_index_ = 0;
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
    // set the max score stat
    if (max_score_.second > all_time_max_score_.get()){
        all_time_max_score_.set(max_score_.second); 
        all_time_max_score_offset_.set(max_score_.first); 
    }
    // set the max average score
    high_score_total_ += max_score_.second;
    // return the highest scoring offset in the list
    return max_score_.first; 
}

// function to change the current offset that we are testing
void BestOffsetPrefetcher::moveTestOffsetPtr(){
    // if the current offset is the total minus 1
    if (this->test_offset_index_ == (num_offsets_ - 1) ){
        // reset the pointer and set the round up
         // if max out the nuber of rounds, reset the prefetcher
         if (this->current_round_ == ( this->round_max_ ) ){
            this->current_offset_ = findMaxScore(); 
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
    if (this->use_scanning_algorithm_ == false){
        uint64_t test_offset = (this->offset_scores_[this->test_offset_index_]).first;
        // if going off the page by subtracing the offset, don't consider the offset
        uint64_t base_addr = _addr - test_offset; 
        // check if address in the recent requests table if on current page
        if (this->recent_requests_.exists(base_addr, _cycle) == true){
            // increase the statistic for the number of recent request hits
            recent_requests_hits_.inc();
            // if increasing that index by 1 would put us over the max score reset the prefetcher
            if (this->offset_scores_[this->test_offset_index_].second == (this->max_score_ - uint64_t(1))){
                // function to print scores at the end of the the round. useful for debugging
                // increase the score for that offset
                uint64_t max_score_offset = this->offset_scores_[this->test_offset_index_].first; 
                // set the max score to be the max possible score
                all_time_max_score_.set(max_score_); 
                all_time_max_score_offset_.set(max_score_offset); 
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
    }else{ 
        // for implementation of scanning 
        uint64_t max_score = 1;
        uint64_t max_score_offset = offset_scores_[0].first;
        for (uint64_t i = 0; i < num_offsets_; i++){
            // check if the scanned offset exists in the table
            if (this->recent_requests_.exists((_addr - offset_scores_[i].first), _cycle)){
                offset_scores_[i].second++; 
            }
            if (offset_scores_[i].second >= max_score){
                max_score_offset = offset_scores_[i].first;
                max_score = offset_scores_[i].second; 
            }
        }
        // figure out if we're maxing out the scores
        if (max_score == this->max_score_){
            this->current_offset_ = max_score_offset; 
                all_time_max_score_.set(max_score); 
                all_time_max_score_offset_.set(max_score_offset); 
                high_score_total_ += max_score;
                resetPrefetcher(); 
        }else{
            // increase the round
            this->current_round_ += 1;
            this->total_rounds_++;
            // if we hit the max found, reset the thing
            if (this->current_round_ == ( this->round_max_ ) ){
                high_score_total_ += max_score;
                this->current_offset_ = max_score_offset;
                resetPrefetcher();
            }
        } 
    }
}
BestOffsetPrefetcher::BestOffsetPrefetcher(const g_string& _name,
                                       const g_string& _target,
                                       bool _monitor_GETS,
                                       bool _monitor_GETX,
                                       uint32_t _degree,
                                       uint64_t _round_max,
                                       uint64_t _max_score,
                                       uint64_t _init_offset,
                                       uint64_t rr_size,
                                       bool _new_rr,
                                       bool _page_offsets,
                                       bool _scanning_algorithm
                                       ):
    CachePrefetcher(_name, _target),  monitor_GETS_(_monitor_GETS),
    monitor_GETX_(_monitor_GETX), degree_(1), round_max_(_round_max),
    max_score_(_max_score), init_offset_(_init_offset),
    use_page_offsets_(_page_offsets), use_scanning_algorithm_(_scanning_algorithm) 
    {

    // BO only works with a degree of 1. Ignore any other value
    if (_degree != 1) {
        std::cout << "Ignoring degree parameter, using default degree of 1" <<std::endl;
    }

    // init recent requests hash table
    this->recent_requests_ = RR();
    this->recent_requests_.max_size_ = rr_size;
    this->recent_requests_.use_new_rr_ = _new_rr;;
    this->offset_scores_ = std::vector<std::pair<const uint64_t, uint64_t>>();
    // load up the offset scores table with the offset and score depending on the parameter for the offset type
    if (_page_offsets == true){
        num_offsets_ = 63;
        for (uint64_t i = 1; i <= num_offsets_; ++i){ 
            offset_scores_.push_back(std::pair<const uint64_t, uint64_t>(i, 0));
        }
    }else{
        // if using the paper's proposed offset list
        num_offsets_ = paper_offset_list_.size();
        for (uint64_t i = 0; i < num_offsets_; ++i){ 
            offset_scores_.push_back(std::pair<const uint64_t, uint64_t>(paper_offset_list_[i], 0)); 
        }
    }
    // set the current round to 0 of the learning phase
    this->current_round_ = 0;
    // set the first test offset to the first index in the offset list
    this->test_offset_index_ = 0;
    // set the prefetch offset to its init value (defined in config file)
    this->current_offset_ = this->init_offset_;
}

// create the stats for the prefetcher with clean data 
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
    // Only do data prefetches for now
    // if an instruction fetch, don't prefetch
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
    return resp_cycle;
}

// prefetch function to schedule prefetches
void BestOffsetPrefetcher::prefetch(MemReq& _req) {
        auto &queue_info = d_caches_[_req.srcId];  //This is pair<cache-ptr, skip>
        uint64_t prefetch_location = _req.lineAddr + this->current_offset_;
        queue_info.first->schedulePrefetch(prefetch_location, queue_info.second);
        // increment the number of prefetches made
        prof_emitted_prefetches_.inc();
}

// function to insert an address into the recent requests table
void RR::insert(uint64_t _addr, uint64_t _cycle){
    if (this->use_new_rr_ == true){
        // to handle deleting
        if (this->list_.size() == max_size_){
            this->deletion(); 
        }
        // put the most recent address in the back of the list
        this->list_.push_back(std::pair<uint64_t, uint64_t>(_addr, id_));
        // see if this address already exists in the map
        auto found = this->hash_.find(_addr);
        // if the thing exists in the map already
        if (found != this->hash_.end()){
            // set the hash id to the last element in the list's id
            found->second.first = this->list_.back().second;
        }else { // if the map doesn't have this already
            // put the address of the back of the list in the map
            auto new_pair = std::make_pair(_addr, std::pair<uint64_t, uint64_t>(id_++, _cycle));
            this->hash_.insert(new_pair);
        }
        // increase the size of the list
        this->size_++;
    }else{
        uint64_t mapSlot = _addr % max_size_;
        if (this->hash_.find(mapSlot) != this->hash_.end()){
            this->hash_.erase(mapSlot); 
            this->size_--;
        }
        auto new_pair = std::make_pair(mapSlot, std::pair<uint64_t, uint64_t>(_addr, _cycle));
        this->hash_.insert(new_pair); 
        this->size_++;
    }
}

// check if an address exists in the recent requests table
bool RR::exists(uint64_t _addr, uint64_t _cycle){
    if (this->use_new_rr_ == true){
        auto found = this->hash_.find(_addr); 
        if (found != this->hash_.end()){
            // make sure the completion cycle for the request is less than the current one
            uint64_t completion = found->second.second;
            if (completion <= _cycle){
                return true;
            }
        }
       return false;
    }else{
        uint64_t mapSlot = _addr % max_size_;
        auto found = this->hash_.find(mapSlot); 
        if (found != this->hash_.end()){
            // make sure the stored address is the same querying for
            if (found->second.first != _addr){
                return false;
            }
            uint64_t completion = found->second.second;
            if (completion <= _cycle){
                return true;
            }   
        } 
        return false;
    }
}

// function to delete the front of the recent requests table
void RR::deletion(){
    if (this->use_new_rr_ == true){
        // get the front of the list (thing to delete)
        std::pair<uint64_t, uint64_t> front = list_.front();
        // find the corresponding address in the hash map
        auto found = hash_.find(front.first);
        // should always be true
        if (found != hash_.end()){
            // check if the map entry is pointing to the front by its ID
            if (found->second.first == list_.front().second){
                // if it's pointing to the front, delete it
                hash_.erase(front.first); 
            }
            // remove the first element from the list
            list_.pop_front(); 
        }
        this->size_--;
    }
}

void RR::clear(){
    if (this->use_new_rr_ == true){
        this->hash_.clear();
        this->list_.clear();
        this->size_ = 0; 
    }
}

// init function for the recent requests table
RR::RR()
    {
    if (this->use_new_rr_ == true){
        this->hash_ = std::map<uint64_t, std::pair<uint64_t, uint64_t>>();
        this->list_ = std::deque<std::pair<uint64_t,uint64_t>>(); 
    }else{
        this->hash_ = std::map<uint64_t, std::pair<uint64_t, uint64_t>>(); 
    }
}

