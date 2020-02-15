#ifndef LBR_H_
#define LBR_H_

#include <stdint.h>
#include <deque>
#include <sstream>
#include <string>
#include <fstream>
#include <iostream>
#include <set>
#include <unordered_map>
#include <string>

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
    std::ofstream full_log_file;
    std::ofstream bbl_info_file;
    std::ofstream self_modifying_bbl_info_file;
    std::set<uint64_t> observed_bbls;
    std::unordered_map<uint64_t,std::set<uint32_t>> bbl_size_difference_check;
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
    void set_full_log_file(const char *path_name)
    {
        full_log_file.open(path_name);
    }
    void set_bbl_info_file(const char *path_name)
    {
        bbl_info_file.open(path_name);
        std::string tmp(path_name);
        tmp+="_self_modifying_info";
        self_modifying_bbl_info_file.open(tmp.c_str());
    }
    void push(uint64_t bbl_address=0, uint64_t cur_cycle=0, uint32_t instrs=0, uint32_t bytes=0)
    {
        uint64_t result = cur_cycle;
        if(cur_cycle!=0)
        {
            assert(cur_cycle>=last_cycle);
            result=cur_cycle-last_cycle;
            last_cycle = cur_cycle;
        }
        if(full_log_file.is_open())full_log_file<<bbl_address<<","<<result<<std::endl;
        LBREntry new_entry(bbl_address, result);
        if(likely(_queue.size()==LBR_CAPACITY))
        {
            _queue.pop_front();
        }
        _queue.push_back(new_entry);
        if(observed_bbls.find(bbl_address)==observed_bbls.end())
        {
            observed_bbls.insert(bbl_address);
            if(bbl_info_file.is_open())bbl_info_file<<bbl_address<<","<<instrs<<","<<bytes<<std::endl;
            bbl_size_difference_check[bbl_address]=std::set<uint32_t>();
            bbl_size_difference_check[bbl_address].insert(instrs);
        }
        else if (bbl_size_difference_check[bbl_address].find(instrs)==bbl_size_difference_check[bbl_address].end())
        {
            bbl_size_difference_check[bbl_address].insert(instrs);
            if(self_modifying_bbl_info_file.is_open())self_modifying_bbl_info_file<<bbl_address<<","<<instrs<<bytes<<std::endl;
        }
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
    void log_event(uint64_t pc,uint64_t miss_cl_address)
    {
        if(log_file.is_open())log_file<<miss_cl_address<<","<<pc<<","<<get_string()<<std::endl;
    }
    ~LBR_Stack()
    {
        if(log_file.is_open())log_file.close();
        if(full_log_file.is_open())full_log_file.close();
        if(bbl_info_file.is_open())bbl_info_file.close();
        observed_bbls.clear();
        if(self_modifying_bbl_info_file.is_open())self_modifying_bbl_info_file.close();
        for(auto it: bbl_size_difference_check)it.second.clear();
        bbl_size_difference_check.clear();
    }
};

#endif  // LBR_H_
