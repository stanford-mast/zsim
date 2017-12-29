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

#ifndef CACHE_PREFETCHER_H_
#define CACHE_PREFETCHER_H_

#include "cache.h"
#include "filter_cache.h"
#include "g_std/g_string.h"
#include "g_std/g_vector.h"

class CachePrefetcher : public BaseCache {
    public:
        explicit CachePrefetcher(const g_string& _name, const g_string& _target)
            : name_(_name), target_prefix_(_target + '-') {};

        void postInit() override;

        const char* getName() override {
            return name_.c_str();
        };

        void setParents(uint32_t _childId, const g_vector<MemObject*>& _parents, Network* _network) override;
        g_vector<MemObject*>* getParents() override;
        void setChildren(const g_vector<BaseCache*>& _children, Network* _network) override;
        g_vector<BaseCache*>* getChildren() override;
        uint64_t invalidate(const InvReq& _req) override;
        virtual void prefetch(MemReq& _req) = 0;

    protected:
        g_string name_;
        g_vector<MemObject*> parents_;
        g_vector<BaseCache*> children_;
        g_string target_prefix_;
        g_vector<std::pair<FilterCache*, int32_t>> d_caches_;  // cache, target skip distance
        g_vector<std::pair<FilterCache*, int32_t>> i_caches_;  // cache, target skip distance
        uint32_t childId_;
};

#endif  // CACHE_PREFETCHER_H_
