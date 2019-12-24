#ifndef LBR_H_
#define LBR_H_

#include <bits/stdc++.h>

using namespace std;

#include "memory_hierarchy.h"

#define ENABLE_LBR
#define LBR_CAPACITY 32

class LBREntry
{
private:
    Address _bbl_address;
    uint64_t _cycles; /*elapsed core clocks since last update to the LBR stack*/
public:
    LBREntry(Address bbl_address, uint64_t cycles)
    {
        _bbl_address = bbl_address;
        _cycles = cycles;
    }
    string get_string()
    {
        ostringstream os;
        os<<_bbl_address<<";"<<_cycles;
        return os.str();
    }
};

class LBR_Stack
{
private:
    deque<LBREntry> _queue;
    uint64_t last_cycle;
public:
    LBR_Stack()
    {
        last_cycle = 0;
        _queue.clear();
    }
    void push(Address bbl_address=0, uint64_t cur_cycle=0)
    {
        uint64_t result = cur_cycle;
        if(cur_cycle!=0)
        {
            assert(cur_cycle>=last_cycle);
            result=cur_cycle-last_cycle;
            last_cycle = cur_cycle;
        }
        LBREntry new_entry(bbl_address, result);
        if(likely(_queue.size()==LBR_CAPACITY))
        {
            _queue.pop_front();
        }
        _queue.push_back(new_entry);
    }
    string get_string()
    {
        ostringstream os;
        for(int i=_queue.size()-1; i>-1; i--)
        {
            os<<_queue[i].get_string()<<",";
        }
        return os.str();
    }
};

#endif  // LBR_H_
