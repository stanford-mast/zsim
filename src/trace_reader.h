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
#ifndef ZSIM_TRACE_READER_H
#define ZSIM_TRACE_READER_H

#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <deque>
#include <iostream>
#include <cstring>

extern "C" {
#include "public/xed/xed-interface.h"
}

#if __cplusplus < 201402L
template<typename T, typename... Args>
static std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
#else
using std::make_unique;
#endif

// Indices to 'xed_map_' cached features
static constexpr int MAP_MEMOPS = 0;
static constexpr int MAP_UNKNOWN = 1;
static constexpr int MAP_COND = 2;
static constexpr int MAP_REP = 3;
static constexpr int MAP_XED = 4;

enum class CustomOp : uint8_t {
  NONE,
  PREFETCH_CODE
};

struct InstInfo {
  uint64_t pc;                    // instruction address
  const xed_decoded_inst_t *ins;  // XED info
  uint64_t pid;                   // process ID
  uint64_t tid;                   // thread ID
  uint64_t target;                // branch target
  uint64_t mem_addr[2];           // memory addresses
  bool mem_used[2];               // mem address usage flags
  CustomOp custom_op;             // Special or non-x86 ISA instruction
  bool taken;                     // branch taken
  bool unknown_type;              // No available decode info (presents a nop)
  bool valid;                     // True until the end of the sequence
};

class TraceReader {
 public:
  enum returnValue : uint8_t{
    ENTRY_VALID,
    ENTRY_NOT_FOUND,
    ENTRY_FIRST,
    ENTRY_OUT_OF_SEGMENT,
  };
  using bufferEntry = std::deque<InstInfo>::iterator;

  // The default-constructed object will not return valid instructions
  TraceReader();
  // A trace and single-binary object
  TraceReader(const std::string &_trace, const std::string &_binary,
              uint64_t _offset, uint32_t _buf_size = 0);
  // A trace and multi-binary object which reads 'binary-info.txt' from the
  // input path. This file contains one '<binary> <offset>' pair per line.
  TraceReader(const std::string &_trace, const std::string &_binary_group_path,
              uint32_t _buf_size = 0);
  ~TraceReader();
  // A constructor that fails will cause operator! to return true
  bool operator!();
  const InstInfo *nextInstruction();
  const returnValue findPCInSegment(bufferEntry &ref, uint64_t _pc,
                                    uint64_t _termination_pc);
  const returnValue findPC(bufferEntry &ref, uint64_t _pc);
  bufferEntry bufferStart();

 private:
  virtual const InstInfo *getNextInstruction() = 0;
  virtual void binaryGroupPathIs(const std::string &_path) = 0;
  virtual bool initTrace() = 0;
  virtual bool locationForVAddr(uint64_t _vaddr, uint8_t **_loc, uint64_t *_size) = 0;

  void init_buffer();
  void binaryFileIs(const std::string &_binary, uint64_t _offset);

  std::unique_ptr<xed_decoded_inst_t> makeNop(uint8_t _length);

 protected:
  std::string trace_;
  InstInfo info_;
  InstInfo invalid_info_;
  bool trace_ready_;
  bool binary_ready_;
  xed_state_t xed_state_;
  std::unordered_map<std::string, std::pair<uint8_t *, uint64_t>> binaries_;
  std::vector<std::tuple<uint64_t, uint64_t, uint8_t *>> sections_;
  std::unordered_map<uint64_t, std::tuple<int, bool, bool, bool,
      std::unique_ptr<xed_decoded_inst_t>>> xed_map_;
  int warn_not_found_;
  uint64_t skipped_;
  uint32_t buf_size_;
  std::deque <InstInfo> ins_buffer;

  void init(const std::string &_trace);
  bool initBinary(const std::string &_name, uint64_t _offset);
  void clearBinaries();
  void fillCache(uint64_t _vAddr, uint8_t _reported_size, uint8_t *inst_bytes=NULL);
  void traceFileIs(const std::string &_trace);
};

#endif  // EXPERIMENTAL_USERS_GRANTA_TREMBLER_TREMBLER_H_
