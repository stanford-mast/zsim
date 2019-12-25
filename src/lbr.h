#ifndef LBR_H_
#define LBR_H_

#include <stdint.h>
#include <deque>
#include <sstream>
#include <string>
#include <fstream>

#define ENABLE_LBR
#define LBR_CAPACITY 32

class LBREntry
{
private:
    uint64_t _bbl_address;
    uint64_t _cycles; /*elapsed core clocks since last update to the LBR stack*/
public:
    LBREntry(uint64_t bbl_address, uint64_t cycles)
    {
        _bbl_address = bbl_address;
        _cycles = cycles;
    }
    std::string get_string()
    {
        std::ostringstream os;
        os<<_bbl_address<<";"<<_cycles;
        return os.str();
    }
};

class LBR_Stack
{
private:
    std::deque<LBREntry> _queue;
    uint64_t last_cycle;
    std::ofstream log_file;
public:
    LBR_Stack()
    {
        last_cycle = 0;
        _queue.clear();
    }
    void set_log_file(const char *path_name)
    {
        log_file.open(path_name);
    }
    void push(uint64_t bbl_address=0, uint64_t cur_cycle=0)
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
    std::string get_string()
    {
        std::ostringstream os;
        for(int i=_queue.size()-1; i>-1; i--)
        {
            os<<_queue[i].get_string()<<",";
        }
        return os.str();
    }
    void log_event(uint64_t pc)
    {
        if(log_file.is_open())log_file<<pc<<","<<get_string()<<"\n";
    }
    ~LBR_Stack()
    {
        if(log_file.is_open())log_file.close();
    }
};

#endif  // LBR_H_
