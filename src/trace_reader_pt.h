#ifndef ZSIM_TRACE_READER_PT_H
#define ZSIM_TRACE_READER_PT_H
#include <zlib.h>
#include <stdlib.h>

#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include "trace_reader.h"
#include "log.h"

#define GZ_BUFFER_SIZE 80

struct PTInst
{
    uint64_t pc;
    uint8_t size;
    uint8_t inst_bytes[16];
};

class TraceReaderPT : public TraceReader
{
private:
    gzFile raw_file = NULL;
    InstInfo next_instruction;
public:
    bool read_next_line(PTInst &inst)
    {
        if(raw_file==NULL)return false;
        char buffer[GZ_BUFFER_SIZE];
        if(gzgets(raw_file,buffer,GZ_BUFFER_SIZE) == Z_NULL)return false;
        std::string line = buffer;
        boost::trim_if(line, boost::is_any_of("\n"));
        std::vector<std::string> parsed;
        boost::split(parsed,line,boost::is_any_of(" \n"),boost::token_compress_on);
        if(parsed.size()<3)panic("TraceReaderPT: GZ File line has less than 3 items");
        inst.pc = strtoul(parsed[0].c_str(), NULL, 16);
        inst.size = strtoul(parsed[1].c_str(), NULL, 10);
        for(uint8_t i = 0; i<inst.size; i++)
        {
            inst.inst_bytes[i] = strtoul(parsed[i+2].c_str(), NULL, 16);
        }
        return true;
    }
    void processInst(InstInfo *_info, PTInst &next_line)
    {
        // Get the XED info from the cache, creating it if needed
        auto xed_map_iter = xed_map_.find(next_line.pc);
        if (xed_map_iter == xed_map_.end()) {
            fillCache(next_line.pc, next_line.size, next_line.inst_bytes);
            xed_map_iter = xed_map_.find(next_line.pc);
            assert((xed_map_iter != xed_map_.end()));
        }
        bool unknown_type, cond_branch;
        int mem_ops_;
        xed_decoded_inst_t *xed_ins;
        auto &xed_tuple = (*xed_map_iter).second;
        tie(mem_ops_, unknown_type, cond_branch, std::ignore, std::ignore) = xed_tuple;
        xed_ins = std::get<MAP_XED>(xed_tuple).get();
        _info->pc = next_line.pc;
        _info->ins = xed_ins;
        _info->pid = 0;
        _info->tid = 0;
        _info->target = 0;  // Set when the next instruction is evaluated
        _info->taken = cond_branch;  // Patched when the next instruction is evaluated
        _info->mem_addr[0] = 0;
        _info->mem_addr[1] = 0;
        _info->mem_used[0] = false;
        _info->mem_used[1] = false;
        _info->unknown_type = unknown_type;
    }
    TraceReaderPT(const std::string &_trace)
    {
        raw_file = gzopen(_trace.c_str(), "rb");
        if(!raw_file)panic("TraceReaderPT: Invalid GZ File");
    }
    const InstInfo *getNextInstruction() override
    {
        PTInst next_line;
        if(read_next_line(next_line)==false)return &invalid_info_;
        processInst(&next_instruction, next_line);
        return &next_instruction;
    }
    void binaryGroupPathIs(const std::string &_path) override
    {
        //do nothing
    }
    bool initTrace() override
    {
        //do nothing
        return true;
    }
    bool locationForVAddr(uint64_t _vaddr, uint8_t **_loc, uint64_t *_size) override
    {
        //do nothing
        return true;
    }
    ~TraceReaderPT()
    {
        if(raw_file!=NULL)gzclose(raw_file);
    }
};

#endif
