#include "trace_reader.h"
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include <fstream>
#include <mutex>
#include <utility>
#ifdef ZSIM_USE_YT
#include "experimental/users/granta/yt/chunkio/xz-chunk-reader.h"
#include "experimental/users/granta/yt/element-reader.h"
#endif  // ZSIM_USE_YT
#include "elf.h"
#include "log.h"

// Remove and include <memory> when using C++14 or later
template<typename T, typename... Args>
static std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

using std::endl;
using std::get;
using std::ifstream;
using std::ignore;
using std::make_pair;
using std::tie;
using std::unique_ptr;

static bool xedInitDone = false;
static std::mutex initMutex;

// Indices to 'xed_map_' cached features
static constexpr int MAP_MEMOPS = 0;
static constexpr int MAP_UNKNOWN = 1;
static constexpr int MAP_COND = 2;
static constexpr int MAP_REP = 3;
static constexpr int MAP_XED = 4;

// A non-reader
TraceReader::TraceReader()
  : trace_ready_(false), binary_ready_(false), skipped_(0),
  mt_warn_target_(0) {
  init();
}

// Trace + single binary
TraceReader::TraceReader(const std::string &_trace, TraceType _type,
                   const std::string &_binary, uint64_t _offset)
  : trace_type_(_type), trace_ready_(false), binary_ready_(true),
  warn_not_found_(1), skipped_(0), mt_iter_(nullptr), mt_end_(nullptr),
  mt_state_(MTState::INST), mt_mem_ops_(0), mt_seq_(0),
  mt_prior_isize_(0), mt_using_info_a_(true), mt_warn_target_(0) {
  init();
  binaryFileIs(_binary, _offset);
  traceFileIs(_trace, _type);
}

// Trace + multiple binaries
TraceReader::TraceReader(const std::string &_trace, TraceType _type,
                   const std::string &_binary_group_path)
  : trace_type_(_type), trace_ready_(false), binary_ready_(true),
  warn_not_found_(1), skipped_(0), mt_iter_(nullptr), mt_end_(nullptr),
  mt_state_(MTState::INST), mt_mem_ops_(0), mt_seq_(0),
  mt_prior_isize_(0), mt_using_info_a_(true), mt_warn_target_(0) {
  init();
  binaryGroupPathIs(_binary_group_path);
  traceFileIs(_trace, _type);
}

TraceReader::~TraceReader() {
  clearBinaries();
  if (skipped_ > 0) {
    warn("Skipped %lu stray memory references\n", skipped_);
  }
  if (mt_warn_target_ > 0) {
    warn("Set %lu conditional branches to 'not-taken' due to pid/tid gaps\n",
         mt_warn_target_);
  }
}

bool TraceReader::operator!() {
  // Return true if there was an initialization error
  return !(trace_ready_ && binary_ready_);
}

#ifdef ZSIM_USE_YT
void TraceReader::skipAmountIs(uint64_t _count) {
  // Skip forward '_count' instructions
  if (_count == 0) {
    return;
  }
  uint64_t skipped = 0;
  Element e = yt_reader_->nextElement();
  while (e.valid) {
    if (Trace::isInst(e.type)) {
      skipped++;
    }
    if (skipped == _count) {
      break;
    }
    e = yt_reader_->nextElement();
  }
}
#endif  // ZSIM_USE_YT

const InstInfo *TraceReader::nextInstruction() {
  if (trace_type_ == TraceType::MEMTRACE) {
    return nextInstructionMT();
#ifdef ZSIM_USE_YT
  } else if (trace_type_ == TraceType::YT) {
    return nextInstructionYT();
#endif  // ZSIM_USE_YT
  } else {
      warn("Unknown trace format type");
      return nullptr;
  }
}

void TraceReader::init() {
  // Initialize XED only once
  initMutex.lock();
  if (!xedInitDone) {
    xed_tables_init();
    xedInitDone = true;
  }
  initMutex.unlock();

  // Set the XED machine mode to 64-bit
  xed_state_init2(&xed_state_, XED_MACHINE_MODE_LONG_64, XED_ADDRESS_WIDTH_64b);

#ifdef ZSIM_USE_YT
  // YT
  info_.custom_op = CustomOp::NONE;  // Future feature
  info_.valid = true;
#endif  // ZSIM_USE_YT

  // MT
  mt_info_a_.custom_op = CustomOp::NONE;
  mt_info_b_.custom_op = CustomOp::NONE;
  mt_info_a_.valid = true;
  mt_info_b_.valid = true;

  // Clear the 'invalid' record (memset() would do too)
  invalid_info_.pc = 0;
  invalid_info_.ins = nullptr;
  invalid_info_.pid = 0;
  invalid_info_.tid = 0;
  invalid_info_.target = 0;
  invalid_info_.mem_addr[0] = 0;
  invalid_info_.mem_addr[1] = 0;
  invalid_info_.mem_used[0] = false;
  invalid_info_.mem_used[1] = false;
  invalid_info_.custom_op = CustomOp::NONE;
  invalid_info_.taken = false;
  invalid_info_.unknown_type = false;
  invalid_info_.valid = false;
}

void TraceReader::traceFileIs(const std::string &_trace, TraceType _type) {
  trace_type_ = _type;
  trace_ = _trace;
  trace_ready_ = initTrace();
}

void TraceReader::binaryFileIs(const std::string &_binary, uint64_t _offset) {
  clearBinaries();
  if (_binary.empty()) {
    // An absent binary is allowed
    binary_ready_ = true;
  } else {
    binary_ready_ = initBinary(_binary, _offset);
  }
}

void TraceReader::binaryGroupPathIs(const std::string &_path) {
  clearBinaries();
  binary_ready_ = true;  // An absent binary is allowed
  if (!_path.empty()) {
    // Check for 'binary-info.txt' and all of the files it specifies
    std::string info_name = _path + "/binary-info.txt";
    ifstream info_file(info_name);
    if (!info_file.is_open()) {
      panic("Could not open binary collection info file '%s': %s",
            info_name.c_str(), strerror(errno));
    }
    std::string name;
    uint64_t offset;
    while (info_file >> name >> std::hex >> offset >> std::dec) {
      binary_ready_ &= initBinary(_path + "/" + name, offset);
    }
  }
}

void TraceReader::clearBinaries() {
  // Unmap all existing files
  for (auto &binary : binaries_) {
    auto &map_info = binary.second;
    if (munmap(map_info.first, map_info.second) == -1) {
      panic("munmap: %s", strerror(errno));
    }
  }
  binaries_.clear();
  sections_.clear();
}

bool TraceReader::initTrace() {
  if (trace_type_ == TraceType::MEMTRACE) {
    mt_reader_ = make_unique<analyzer_t>(trace_);
    if (!(*mt_reader_)) {
      panic("Failure starting memtrace reader");
      return false;
    }
    mt_iter_ = &(mt_reader_->begin());
    mt_end_ = &(mt_reader_->end());

    // Set info 'A' to the first complete instruction.
    // It will initially lack branch target information.
    MTGetNextInstruction(&mt_info_a_, &mt_info_b_);
    mt_using_info_a_ = false;
    return true;
#ifdef ZSIM_USE_YT
  }
    else if (trace_type_ == TraceType::YT) {
    // Determine the trace type by the file extension
      std::string name = trace_;
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    bool isRawYT = name.size() > 3 &&
        (name.compare(name.size() - 3, 3, ".yt") == 0);
    bool isXzYT = name.size() > 6 &&
        (name.compare(name.size() - 6, 6, ".yt.xz") == 0);
    if (isXzYT) {
      yt_reader_ = make_unique<ElementReader<Util::XzChunkReader>>(
          trace_, 100*1048576UL);
    } else if (isRawYT) {
      yt_reader_ = make_unique<ElementReader<Util::BasicChunkReader>>(
          trace_, 100*1048576UL);
    } else {
      panic("Input file '%s' doesn't appear to be in the YT format (.yt or .yt.xz)",
            trace_.c_str());
      return false;
    }
    info_.pid = 0;  // YT supports TIDs but not PIDs
    return true;
#endif  // ZSIM_USE_YT
  } else {
    return false;
  }
}

bool TraceReader::initBinary(const std::string &_name, uint64_t _offset) {
  // Load the input file to memory
  int fd = open(_name.c_str(), O_RDONLY);
  if (fd == -1) {
    panic("Could not open '%s': %s", _name.c_str(), strerror(errno));
    return false;
  }
  struct stat sb;
  if (fstat(fd, &sb) == -1) {
    panic("fstat: %s", strerror(errno));
    return false;
  }
  uint64_t size = static_cast<uint64_t>(sb.st_size);
  if (size == 0) {
    warn("Input file '%s' is empty", _name.c_str());
    if (close(fd) == -1) {
      panic("close: %s", strerror(errno));
    }
    return false;
  }
  uint8_t *data = static_cast<uint8_t *>(mmap(nullptr, size, PROT_READ,
                                          MAP_SHARED, fd, 0));
  if (data == MAP_FAILED) {
    panic("mmap: %s", strerror(errno));
    return false;
  }
  if (close(fd) == -1) {
    panic("close: %s", strerror(errno));
    return false;
  }
  binaries_.emplace(_name, make_pair(data, size));

  // Parse the ELF structures in the file
  if (size < sizeof(Elf64_Ehdr)) {
    panic("File is too small to hold an ELF header");
    return false;
  }
  Elf64_Ehdr* hdr = reinterpret_cast<Elf64_Ehdr*>(data);
  if (hdr->e_machine != EM_X86_64) {
    panic("Expected ELF binary type 'EM_X86_64'");
    return false;
  }
  Elf64_Off shoff = hdr->e_shoff;  // section header table offset
  if (size < shoff + (hdr->e_shnum * sizeof(Elf64_Shdr))) {
    panic("ELF file is too small for section headers");
    return false;
  }
  Elf64_Shdr* shdr = reinterpret_cast<Elf64_Shdr*>(data + shoff);
  for (Elf64_Half i = 0; i < hdr->e_shnum; i++) {
    if ((shdr[i].sh_type == SHT_PROGBITS) &&
        (shdr[i].sh_flags & SHF_EXECINSTR)) {
      // An executable ("text") section
      Elf64_Off sec_offset = shdr[i].sh_offset;
      Elf64_Xword sec_size = shdr[i].sh_size;
      if (size < sec_offset + sec_size) {
        panic("ELF file is too small for section %u", i);
        return false;
      }
      // Save the starting virtual address, size, and location in memory
      uint64_t base_addr = shdr[i].sh_addr + _offset;
      sections_.emplace_back(base_addr, sec_size, data + sec_offset);
    }
  }
  return true;
}

void TraceReader::fillCache(uint64_t _vAddr, uint8_t _reported_size) {
  uint64_t size;
  uint8_t *loc;
  if (locationForVAddr(_vAddr, &loc, &size)) {
    xed_map_.emplace(_vAddr, make_tuple(
        0, false, false, false, make_unique<xed_decoded_inst_t>()));
    xed_decoded_inst_t *ins = get<MAP_XED>(xed_map_.at(_vAddr)).get();
    xed_decoded_inst_zero_set_mode(ins, &xed_state_);
    xed_error_enum_t res = xed_decode(ins, loc, size);

    if (res != XED_ERROR_NONE) {
      warn("XED decode error for 0x%lx: %s", _vAddr, xed_error_enum_t2str(res));
    }
    // Record if this instruction requires memory operands, since the trace
    // will deliver it in additional pieces
    uint32_t n_mem_ops = xed_decoded_inst_number_of_memory_operands(ins);
    if (n_mem_ops > 0) {
      // NOPs are special and don't actually cause memory accesses
      xed_category_enum_t category = xed_decoded_inst_get_category(ins);
      if (category != XED_CATEGORY_NOP && category != XED_CATEGORY_WIDENOP) {
        uint32_t n_used_mem_ops = 0;  // 'lea' doesn't actually touch memory
        for (uint32_t i = 0; i < n_mem_ops; i++) {
          if (xed_decoded_inst_mem_read(ins, i)) {
            n_used_mem_ops++;
          }
          if (xed_decoded_inst_mem_written(ins, i)) {
            n_used_mem_ops++;
          }
        }
        if (n_used_mem_ops > 0) {
          if (n_used_mem_ops > 2) {
            warn("Unexpected %u memory operands for 0x%lx\n",
                 n_used_mem_ops, _vAddr);
          }
          get<MAP_MEMOPS>(xed_map_.at(_vAddr)) = n_used_mem_ops;
        }
      }
    }
    auto &xed_tuple = xed_map_[_vAddr];
    // Record if this instruction is a conditional branch
    bool is_cond_br = (xed_decoded_inst_get_category(ins) ==
                       XED_CATEGORY_COND_BR);
    get<MAP_COND>(xed_tuple) = is_cond_br;

    // Record if this instruction is a 'rep' type, which may indicate a
    // variable number of memory records for input formats like memtrace
    bool is_rep = xed_decoded_inst_get_attribute(ins, XED_ATTRIBUTE_REP) > 0;
    get<MAP_REP>(xed_tuple) = is_rep;
  } else {
    if (warn_not_found_ > 0) {
      warn_not_found_ -= 1;
      if (warn_not_found_ > 0) {
        warn("No information for instruction at address 0x%lx", _vAddr);
      } else {
        warn("No information for instruction at address 0x%lx. "
             "Suppressing further messages", _vAddr);
      }
    }
    // Replace the unknown instruction with a NOP
    // NOTE: Unknown memory records are skipped, so 'rep' needs no special
    // handling here
    xed_map_.emplace(_vAddr, make_tuple(0, true, false, false,
                                        makeNop(_reported_size)));
  }
}

unique_ptr<xed_decoded_inst_t> TraceReader::makeNop(uint8_t _length) {
  // A 10-to-15-byte NOP instruction (direct XED support is only up to 9)
  static const char *nop15 =
      "\x66\x66\x66\x66\x66\x66\x2e\x0f\x1f\x84\x00\x00\x00\x00\x00";

  auto ptr = make_unique<xed_decoded_inst_t>();
  xed_decoded_inst_t *ins = ptr.get();
  xed_decoded_inst_zero_set_mode(ins, &xed_state_);
  xed_error_enum_t res;

  // The reported instruction length must be 1-15 bytes
  _length &= 0xf;
  assert(_length > 0);
  if (_length > 9) {
    int offset = 15 - _length;
    const uint8_t *pos = reinterpret_cast<const uint8_t *>(nop15 + offset);
    res = xed_decode(ins, pos, 15 - offset);
  } else {
    uint8_t buf[10];
    res = xed_encode_nop(&buf[0], _length);
    if (res != XED_ERROR_NONE) {
      warn("XED NOP encode error: %s", xed_error_enum_t2str(res));
    }
    res = xed_decode(ins, buf, sizeof(buf));
  }
  if (res != XED_ERROR_NONE) {
    warn("XED NOP decode error: %s", xed_error_enum_t2str(res));
  }
  return ptr;
}

bool TraceReader::locationForVAddr(uint64_t _vaddr, uint8_t **_loc, uint64_t *_size) {
  // Find the binary image bytes corresponding to a virtual address
  uint64_t secBaseVAddr, secSize;
  uint8_t *secLoc;
  for (auto &section : sections_) {
    tie(secBaseVAddr, secSize, secLoc) = section;
    if ((secBaseVAddr <= _vaddr) && ((secBaseVAddr + secSize) > _vaddr)) {
      *_loc = secLoc + (_vaddr - secBaseVAddr);
      *_size = secSize - (_vaddr - secBaseVAddr);
      return true;
    }
  }
  return false;
}

const InstInfo *TraceReader::nextInstructionMT() {
  InstInfo &info = (mt_using_info_a_ ? mt_info_a_ : mt_info_b_);
  InstInfo &prior = (mt_using_info_a_ ? mt_info_b_ : mt_info_a_);
  mt_using_info_a_ = !mt_using_info_a_;
  if (MTGetNextInstruction(&info, &prior)) {
    return &prior;
  } else {
    return &invalid_info_;
  }
}

#ifdef ZSIM_USE_YT
const InstInfo *TraceReader::nextInstructionYT() {
  // Grab the next instruction element.
  //
  // YT guarantees that data elements follow their corresponding instruction
  // elements without gaps, even if the trace interleaves multiple threads.
  //
  // NOTE: Data-only YT traces are not supported! This tool must have an
  // instruction trace or instruction + data trace.
  Element e = yt_reader_->nextElement();
  while (e.valid) {
    if (Trace::isInst(e.type)) {
      break;
    } else {
      // Print an error if the stray element belongs to a known instruction,
      // or silently skip if it's part of an unknown instruction.
      if (xed_map_.find(e.inst) == xed_map_.end()) {
        fillCache(e.inst, e.isize);
      }
      if (!get<MAP_UNKNOWN>(xed_map_.at(e.inst))) {
        warn("Unexpected non-instruction element of type %u, PC: 0x%lx",
             static_cast<uint32_t>(Trace::value(e.type)), e.inst);
      }
    }
    e = yt_reader_->nextElement();
  }
  if (!e.valid) {
    return &invalid_info_;
  }
  // Get the XED info from the cache, creating it if needed
  auto xed_map_iter = xed_map_.find(e.inst);
  if (xed_map_iter == xed_map_.end()) {
    fillCache(e.inst, e.isize);
    xed_map_iter = xed_map_.find(e.inst);
    assert((xed_map_iter != xed_map_.end()));
  }
  int n_mem_ops;
  bool unknown_type;
  xed_decoded_inst_t *xed_ins;
  auto &xed_tuple = (*xed_map_iter).second;
  tie(n_mem_ops, unknown_type, ignore, ignore, ignore) = xed_tuple;
  xed_ins = get<MAP_XED>(xed_tuple).get();
  info_.tid = e.tid;
  info_.pc = e.inst;
  info_.ins = xed_ins;
  info_.target = e.target;  // Target address, valid for call/jmp/ret/etc.
  info_.mem_addr[0] = 0;
  info_.mem_addr[1] = 0;
  info_.mem_used[0] = false;
  info_.mem_used[1] = false;
  info_.taken = e.taken;
  info_.unknown_type = unknown_type;
  // Check for memory operands which come in later records
  assert(n_mem_ops <= 2);
  for (int i = 0; i < n_mem_ops; i++) {
    e = yt_reader_->nextElement();
    if (!e.valid) {
      warn("Incomplete memory records after instruction 0x%lx", e.inst);
      info_.valid = false;  // Applies to all future records as well
      break;
    }
    assert(e.valid);
    assert(Trace::isData(e.type));
    assert(e.inst == info_.pc);
    info_.mem_addr[i] = e.target;
    info_.mem_used[i] = true;
  }
  return &info_;
}
#endif  // ZSIM_USE_YT

bool TraceReader::MTGetNextInstruction(InstInfo *_info, InstInfo *_prior) {
  uint32_t prior_isize = mt_prior_isize_;
  bool complete = false;
  while (*mt_iter_ != *mt_end_) {
    switch (mt_state_) {
      case (MTState::INST):
        mt_ref_ = **mt_iter_;
        if (type_is_instr(mt_ref_.instr.type)) {
          MTProcessInst(_info);
          if (mt_mem_ops_ > 0) {
            mt_state_ = MTState::MEM1;
          } else {
            complete = true;
          }
        } else if (MTTypeIsMem(mt_ref_.data.type)) {
          // Skip flush and thread exit types, patch rep instructions, and
          // silently ignore memory operands of unknown instructions
          if (!_prior->unknown_type) {
            bool is_rep = get<MAP_REP>(xed_map_.at(_prior->pc));
            if (is_rep &&
                ((uint32_t)mt_ref_.data.pid == _prior->pid) &&
                ((uint32_t)mt_ref_.data.tid == _prior->tid) &&
                (mt_ref_.data.pc == _prior->pc)) {
              *_info = *_prior;
              _info->mem_addr[0] = mt_ref_.data.addr;
              _info->mem_used[0] = true;
              if (mt_mem_ops_ > 1) {
                mt_state_ = MTState::MEM2;
              } else {
                _info->mem_addr[1] = 0;
                _info->mem_used[1] = false;
                complete = true;
              }
            } else {
              if (skipped_ == 0) {
                warn("Stray memory record detected at seq. %lu: PC: 0x%lx, "
                     "PID: %lu, TID: %lu, Addr: 0x%lx. "
                     "Suppressing further messages.\n",
                     mt_seq_, mt_ref_.data.pc, mt_ref_.data.pid,
                     mt_ref_.data.tid, mt_ref_.data.addr);
              }
              skipped_++;
            }
          }
        }
        break;
      case (MTState::MEM1):
        mt_ref_ = **mt_iter_;
        if (MTTypeIsMem(mt_ref_.data.type)) {
          if (((uint32_t)_info->pid == mt_ref_.data.pid) &&
              ((uint32_t)_info->tid == mt_ref_.data.tid) &&
              (_info->pc == mt_ref_.data.pc)) {
            _info->mem_addr[0] = mt_ref_.data.addr;
            _info->mem_used[0] = true;
            if (mt_mem_ops_ > 1) {
              mt_state_ = MTState::MEM2;
            } else {
              mt_state_ = MTState::INST;
              complete = true;
            }
          } else {
            warn("Unexpected PID/TID/PC switch following 0x%lx\n", _info->pc);
            mt_state_ = MTState::INST;
          }
        } else {
          warn("Expected data but found type '%s'\n",
               trace_type_names[mt_ref_.data.type]);
          mt_state_ = MTState::INST;
        }
        break;
      case (MTState::MEM2):
        mt_ref_ = **mt_iter_;
        if (MTTypeIsMem(mt_ref_.data.type)) {
          if (((uint32_t)_info->pid == mt_ref_.data.pid) &&
              ((uint32_t)_info->tid == mt_ref_.data.tid) &&
              (_info->pc == mt_ref_.data.pc)) {
            _info->mem_addr[1] = mt_ref_.data.addr;
            _info->mem_used[1] = true;
            assert(mt_mem_ops_ <= 2);
            mt_state_ = MTState::INST;
            complete = true;
          } else {
            warn("Unexpected PID/TID/PC switch following 0x%lx\n", _info->pc);
            mt_state_ = MTState::INST;
          }
        } else {
          warn("Expected data2 but found type '%s'\n",
               trace_type_names[mt_ref_.data.type]);
          mt_state_ = MTState::INST;
        }
        break;
    }
    mt_seq_++;
    ++(*mt_iter_);
    if (complete) {
      break;
    }
  }
  // Compute the branch target information for the prior instruction
  _prior->target = _info->pc;  // TODO(granta): Invalid for pid/tid switch
  if (_prior->taken) {  // currently set iif conditional branch
    bool non_seq = _info->pc != (_prior->pc + prior_isize);
    bool new_gid = (_prior->tid != _info->tid) || (_prior->pid != _info->pid);
    if (new_gid) {
      // TODO(granta): If there are enough of these, it may make sense to
      // delay conditional branch instructions until the thread resumes even
      // though this alters the apparent order of the trace.
      // (Seeking ahead to resolve the branch info is a non-starter.)
      if (mt_warn_target_ == 0) {
        warn("Detected a conditional branch preceding a pid/tid change "
             "at seq. %lu. Assuming not-taken. Suppressing further "
             "messages.\n", mt_seq_ - 1);
      }
      mt_warn_target_++;
      non_seq = false;
    }
    _prior->taken = non_seq;
  }

  _info->valid &= complete;
  return complete;
}

void TraceReader::MTProcessInst(InstInfo *_info) {
  // Get the XED info from the cache, creating it if needed
  auto xed_map_iter = xed_map_.find(mt_ref_.instr.addr);
  if (xed_map_iter == xed_map_.end()) {
    fillCache(mt_ref_.instr.addr, mt_ref_.instr.size);
    xed_map_iter = xed_map_.find(mt_ref_.instr.addr);
    assert((xed_map_iter != xed_map_.end()));
  }
  bool unknown_type, cond_branch;
  xed_decoded_inst_t *xed_ins;
  auto &xed_tuple = (*xed_map_iter).second;
  tie(mt_mem_ops_, unknown_type, cond_branch, ignore, ignore) = xed_tuple;
  mt_prior_isize_ = mt_ref_.instr.size;
  xed_ins = get<MAP_XED>(xed_tuple).get();
  _info->pc = mt_ref_.instr.addr;
  _info->ins = xed_ins;
  _info->pid = mt_ref_.instr.pid;
  _info->tid = mt_ref_.instr.tid;
  _info->target = 0;  // Set when the next instruction is evaluated
  _info->taken = cond_branch;  // Patched when the next instruction is evaluated
  _info->mem_addr[0] = 0;
  _info->mem_addr[1] = 0;
  _info->mem_used[0] = false;
  _info->mem_used[1] = false;
  _info->unknown_type = unknown_type;
}

bool TraceReader::MTTypeIsMem(trace_type_t _type) {
  return ((_type == TRACE_TYPE_READ) || (_type == TRACE_TYPE_WRITE) ||
          type_is_prefetch(_type));
}
