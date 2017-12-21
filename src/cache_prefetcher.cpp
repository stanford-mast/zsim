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

#include "cache_prefetcher.h"
#include "g_std/g_list.h"

#include <unordered_set>

#ifndef DBG
//#define DBG(args...) info(args)
#define DBG(args...)
#endif

void CachePrefetcher::postInit() {
    //Traverse the now stable memory hierarchy to find the prefetch target and all terminal (filter/l0) cache children
    //to use their prefetch queues
    g_list<std::pair<MemObject*, int32_t>> queue;
    std::unordered_set<MemObject*> visited;
    queue.emplace_back(this, 0);
    visited.insert(this);
    bool target_found = false;
    int32_t target_level = 0;  // Positive for parent, negative for child
    while (!queue.empty()) {
        BaseCache* cache = dynamic_cast<BaseCache*>(queue.front().first);
        int32_t cur_level = queue.front().second;
        queue.pop_front();
        if (!cache) {
            //E.g., could be main memory
            continue;
        }

        //Check for the target cache
        g_string name(cache->getName());
        if (!name.compare(0, target_prefix_.size(), target_prefix_)) {
            target_found = true;
            target_level = cur_level;
            DBG("Prefetcher '%s' found target '%s' (level %d)", name_.c_str(), name.c_str(), target_level);
        }

        //Check for a terminal cache
        FilterCache* fcache = dynamic_cast<FilterCache*>(cache);
        if (fcache) {
            uint32_t idx = fcache->getSourceId();
            if (fcache->getType() == FilterCache::Type::D) {
                DBG("Prefetcher '%s' found i-cache '%s' idx %u level %d", name_.c_str(), fcache->getName(), idx, cur_level);
                d_caches_.resize(MAX(idx + 1, d_caches_.size()));
                d_caches_[idx] = std::make_pair(fcache, cur_level);
            } else {
                DBG("Prefetcher '%s' found d-cache '%s' idx %u level %d", name_.c_str(), fcache->getName(), idx, cur_level);
                i_caches_.resize(MAX(idx + 1, i_caches_.size()));
                i_caches_[idx] = std::make_pair(fcache, cur_level);
            }
        } else {
            //Continue the search for the target and/or terminal caches
            auto* parents = cache->getParents();
            if (parents) {
                for (auto parent : *parents) {
                    if (visited.count(parent) == 0) {
                        visited.insert(parent);
                        queue.emplace_back(parent, cur_level + 1);
                    }
                }
            }
            auto* children = cache->getChildren();
            if (children) {
                for (auto child : *children) {
                    //Avoid searching other children of the parent
                    if (visited.count(child) == 0 && cur_level <= 0) {
                        visited.insert(child);
                        //Zero-base from the prefetcher's children
                        queue.emplace_back(child, ((cache == this) ? 0 : cur_level - 1));
                    }
                }
            }
        }
    }

    if (!target_found) {
        panic("Prefetcher '%s' could not find target cache with pattern '%s'", name_.c_str(), target_prefix_.c_str());
    }

    if (i_caches_.empty() || d_caches_.empty()) {
        panic("Prefetcher '%s' couldn't find any terminal caches", name_.c_str());
    }

    //Convert the filter cache levels to distances from the target
    for (auto& icache : i_caches_) {
        //Depending on the hierarchy, some of these slots may be empty/invalid,
        //however these slots should never be accessed anyway
        if (icache.first != nullptr) {
            icache.second = target_level - icache.second;
            assert(icache.second >= 0);
        }
    }
    for (auto& dcache : d_caches_) {
        //Depending on the hierarchy, some of these slots may be empty/invalid,
        //however these slots should never be accessed anyway
        if (dcache.first != nullptr) {
            dcache.second = target_level - dcache.second;
            assert(dcache.second >= 0);
        }
    }
}

void CachePrefetcher::setParents(uint32_t _childId, const g_vector<MemObject*>& _parents, Network* _network) {
    childId_ = _childId;
    if (_network) {
        panic("Network not handled");
    }
    assert(!_parents.empty());
    parents_ = _parents;
}

g_vector<MemObject*>* CachePrefetcher::getParents() {
    return &parents_;
}

void CachePrefetcher::setChildren(const g_vector<BaseCache*>& _children, Network* _network) {
    if (_network) {
        panic("Network not handled");
    }
    assert(!_children.empty());
    children_ = _children;
}

g_vector<BaseCache*>* CachePrefetcher::getChildren() {
    return &children_;
}

uint64_t CachePrefetcher::invalidate(const InvReq& _req) {
    uint64_t resp_cycle = 0;
    for (auto child : children_) {
        resp_cycle = MAX(resp_cycle, child->invalidate(_req));
    }
    return resp_cycle;
}
