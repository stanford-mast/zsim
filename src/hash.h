/** $lic$
 * Copyright (C) 2017 by Google
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

#ifndef HASH_H_
#define HASH_H_

#include <stdint.h>

#include <string>

#include "galloc.h"

class HashFamily : public GlobAlloc {
    public:
        HashFamily() {}
        virtual ~HashFamily() {}

        virtual uint64_t hash(uint32_t id, uint64_t val) = 0;
};

class H3HashFamily : public HashFamily {
    private:
        const uint32_t numFuncs;
        uint32_t resShift;
        uint64_t* hMatrix;
    public:
        H3HashFamily(uint32_t numFunctions, uint32_t outputBits, uint64_t randSeed = 123132127);
        virtual ~H3HashFamily();
        uint64_t hash(uint32_t id, uint64_t val);
};

class SHA1HashFamily : public HashFamily {
    private:
        int numFuncs;
        int numPasses;

        //SHA1 is quite expensive and returns large blocks, so we use memoization and chunk the block to implement hash function families.
        uint64_t memoizedVal;
        uint32_t* memoizedHashes;
    public:
        explicit SHA1HashFamily(int numFunctions);
        uint64_t hash(uint32_t id, uint64_t val);
};

/* Used when we don't want hashing, just return the value */
class IdHashFamily : public HashFamily {
    public:
        inline uint64_t hash(uint32_t id, uint64_t val) {return val;}
};

class CacheBankHash {
public:
    static inline uint64_t hash(uint64_t val, uint64_t outputRange) {
        //Hash things a bit
        uint64_t res = 0;
        uint64_t tmp = val;
        for (uint32_t i = 0; i < 4; i++) {
            res ^= (uint32_t) ( ((uint64_t)0xffff) & tmp);
            tmp = tmp >> 16;
        }
        return (res % outputRange);
    }
};

// 64-bit FNV hashing routine
uint64_t fnv_1a_hash_64(const std::string& data, const uint64_t offset_basis,
                        bool trailing_null);
#endif  // HASH_H_
