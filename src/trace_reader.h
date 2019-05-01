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
#ifdef ZSIM_USE_YT
#include "experimental/users/granta/yt/element-reader.h"
#endif  // ZSIM_USE_YT
#include "analyzer.h"
extern "C" {
#include "public/xed/xed-interface.h"
}

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

enum class TraceType {
#ifdef ZSIM_USE_YT
  YT,
#endif  // ZSIM_USE_YT
  MEMTRACE
};

class TraceReader {
 public:
  // The default-constructed object will not return valid instructions
  TraceReader();
  // A trace and single-binary object
  TraceReader(const std::string &_trace, TraceType _type,
           const std::string &_binary, uint64_t _offset);
  // A trace and multi-binary object which reads 'binary-info.txt' from the
  // input path. This file contains one '<binary> <offset>' pair per line.
  TraceReader(const std::string &_trace, TraceType _type,
           const std::string &_binary_group_path);
  ~TraceReader();
  // A constructor that fails will cause operator! to return true
  bool operator!();
#ifdef ZSIM_USE_YT
  void skipAmountIs(uint64_t _count);
#endif  // ZSIM_USE_YT
  const InstInfo *nextInstruction();

 private:
  enum class MTState {
    INST, MEM1, MEM2
  };

 private:
  void init();
  void traceFileIs(const std::string &_trace, TraceType _type);
  void binaryFileIs(const std::string &_binary, uint64_t _offset);
  void binaryGroupPathIs(const std::string &_path);
  void clearBinaries();
  bool initTrace();
  bool initBinary(const std::string &_name, uint64_t _offset, uint64_t _file_offset);
  void fillCache(uint64_t _vAddr, uint8_t _reported_size);
  std::unique_ptr<xed_decoded_inst_t> makeNop(uint8_t _length);
  bool locationForVAddr(uint64_t _vaddr, uint8_t **_loc, uint64_t *_size);
  const InstInfo *nextInstructionMT();
#ifdef ZSIM_USE_YT
  const InstInfo *nextInstructionYT();
#endif  // ZSIM_USE_YT
  bool MTGetNextInstruction(InstInfo *_info, InstInfo *_prior);
  void MTProcessInst(InstInfo *_info);
  bool MTTypeIsMem(trace_type_t _type);
  xed_state_t xed_state_;
  std::string trace_;
  TraceType trace_type_;
  std::unordered_map<std::string, std::pair<uint8_t *, uint64_t>> binaries_;
  bool trace_ready_;
  bool binary_ready_;
  std::vector<std::tuple<uint64_t, uint64_t, uint8_t *>> sections_;
  InstInfo info_;
  InstInfo invalid_info_;
  std::unordered_map<uint64_t, std::tuple<int, bool, bool, bool,
      std::unique_ptr<xed_decoded_inst_t>>> xed_map_;
  int warn_not_found_;
  uint64_t skipped_;
  //--- YT
#ifdef ZSIM_USE_YT
  std::unique_ptr<GenericElementReader> yt_reader_;
#endif  // ZSIM_USE_YT
  //--- MT
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

#endif  // EXPERIMENTAL_USERS_GRANTA_TREMBLER_TREMBLER_H_
