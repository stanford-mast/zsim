#ifndef ZSIM_TRACE_READER_MEMTRACE_H
#define ZSIM_TRACE_READER_MEMTRACE_H

#include "trace_reader.h"

//#include "instrument.h"
#include "analyzer.h"
#include "raw2trace_directory.h"
#include "raw2trace.h"

class TraceReaderMemtrace : public TraceReader {
  public:
    const InstInfo *getNextInstruction() override;
    TraceReaderMemtrace(const std::string &_trace, const std::string &_binary,
                        uint64_t _offset, uint32_t _bufsize);
    TraceReaderMemtrace(const std::string &_trace,
                        const std::string &_binary_group_path, uint32_t _bufsize);
    ~TraceReaderMemtrace();

  private:
    void binaryGroupPathIs(const std::string &_path) override;
    bool initTrace() override;
    bool locationForVAddr(uint64_t _vaddr, uint8_t **_loc, uint64_t *_size) override;
    void init(const std::string &_trace);
    static const char *parse_buildid_string(const char *src, OUT void **data);
    bool getNextInstruction__(InstInfo *_info, InstInfo *_prior);
    void processInst(InstInfo *_info);
    bool typeIsMem(trace_type_t _type);

    std::unique_ptr<module_mapper_t> module_mapper_;
    raw2trace_directory_t directory_;
    void *dcontext_;
    unsigned int knob_verbose_;

    enum class MTState {
        INST, MEM1, MEM2,
    };

    std::unique_ptr<analyzer_t> mt_reader_;
    reader_t *mt_iter_;
    reader_t *mt_end_;
    MTState mt_state_;
    memref_t mt_ref_;
    int mt_mem_ops_;
    uint64_t mt_seq_;
    uint32_t mt_prior_isize_;
    InstInfo mt_info_a_;
    InstInfo mt_info_b_;
    bool mt_using_info_a_;
    uint64_t mt_warn_target_;
};

#endif
