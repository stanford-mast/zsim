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
    Address _from_address;
    Address _to_address;
    bool _prediction_result; /*false->mispredicted, true->predicted*/
    uint64_t _cycles; /*elapsed core clocks since last update to the LBR stack*/
public:
    LBREntry(Address from, Address to, bool result, uint64_t cycles)
    {
        _from_address = from;
        _to_address = to;
        _prediction_result = result;
        _cycles = cycles;
    }
    string get_string()
    {
        ostringstream os;
        os<<_from_address<<";"<<_to_address<<";"<<_prediction_result<<";"<<_cycles;
        return os.str();
    }
};

class LBR_Stack
{
private:
    deque<LBREntry> _queue;
public:
    LBR_Stack()
    {
    }
    void push(Address from=0, Address to=0, bool result=true, uint64_t cycles=0)
    {
        LBREntry new_entry(from, to, result, cycles);
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