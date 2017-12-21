/** $lic$
 * Copyright (C) 2017 by Google
 * Copyright (C) 2012-2015 by Massachusetts Institute of Technology
 * Copyright (C) 2010-2013 by The Board of Trustees of Stanford University
 *
 * Author: heinerl@google.com (Heiner Litz)
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


/*
 * Wrapper file to enable support for different instruction decoders.
 * Currently we support PIN (execution driven) and Intel xed (trace driven)
 */

#ifndef THIRD_PARTY_ZSIM_SRC_WRAPPED_PIN_H_
#define THIRD_PARTY_ZSIM_SRC_WRAPPED_PIN_H_

#ifdef EXTERNAL_CACHE_MODEL
#include <stddef.h>
typedef uint64_t Address;
typedef uint64_t ADDRINT;
typedef void* BBL;
typedef bool BOOL;
typedef void* INS;
typedef uint32_t THREADID;

#elif TRACE_BASED
#include <stdint.h>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <pthread.h>

#include "bithacks.h"
#include "trace_reader.h"

extern "C" {
    #include "public/xed/xed-interface.h"
}

#define INS xed_decoded_inst_t *
#define REG xed_reg_enum_t

#define INS_Nop(ins) (INS_Category(ins) == XED_CATEGORY_NOP || INS_Category(ins) == XED_CATEGORY_WIDENOP)
#define INS_LEA(ins) (INS_Opcode(ins) == XO(LEA))
#define INS_Opcode(ins) xed_decoded_inst_get_iclass(ins)
#define INS_Category(ins) xed_decoded_inst_get_category(ins)
#define INS_IsAtomicUpdate(ins) xed_decoded_inst_get_attribute(ins, XED_ATTRIBUTE_LOCKED)
#define INS_IsRep(ins) xed_decoded_inst_get_attribute(ins, XED_ATTRIBUTE_REP)
#define INS_OperandCount(ins) xed_decoded_inst_noperands(ins)
#define INS_OperandIsImmediate(ins, op) xed_operand_values_has_immediate(ins)
#define INS_OperandRead(ins, op) xed_operand_read(xed_inst_operand(xed_decoded_inst_inst(ins), op))
#define INS_OperandWritten(ins, op) xed_operand_written(xed_inst_operand(xed_decoded_inst_inst(ins), op))
#define INS_OperandIsReg(ins, op) xed_operand_is_register( xed_operand_name (xed_inst_operand(xed_decoded_inst_inst(ins), op)) )
#define INS_OperandIsMemory(ins, op) ((xed_decoded_inst_mem_read(ins, op) | xed_decoded_inst_mem_written(ins, op)))
#define INS_IsMemory(ins) (xed_decoded_inst_number_of_memory_operands(ins))
#define INS_OperandReg(ins, op) xed_decoded_inst_get_reg(ins, xed_operand_name (xed_inst_operand(xed_decoded_inst_inst(ins), op)))
#define INS_OperandMemoryBaseReg(ins, op) xed_decoded_inst_get_base_reg(ins, op)
#define INS_OperandMemoryIndexReg(ins, op) xed_decoded_inst_get_index_reg(ins, op)
#define INS_LockPrefix(ins) xed_decoded_inst_get_attribute(ins, XED_ATTRIBUTE_LOCKED)
#define INS_OperandWidth(ins, op) xed_decoded_inst_get_operand_width(ins)
#define INS_IsDirectBranch(ins) xed3_operand_get_brdisp_width(ins)
#define INS_Size(ins) xed_decoded_inst_get_length(ins)
#define INS_Valid(ins) xed_decoded_inst_valid(ins)
/* Just like PIN we break BBLs on a number of additional instructions such as REP */
#define INS_ChangeControlFlow(ins) (INS_Category(ins) == XC(COND_BR) || INS_Category(ins) == XC(UNCOND_BR) || INS_Category(ins) == XC(CALL) || INS_Category(ins) == XC(RET) || INS_Category(ins) == XC(SYSCALL) || INS_Category(ins) == XC(SYSRET) || INS_Opcode(ins) == XO(CPUID) || INS_Opcode(ins) == XO(POPF) || INS_Opcode(ins) == XO(POPFD) || INS_Opcode(ins) == XO(POPFQ) || INS_IsRep(ins))
#define REG_FullRegName(reg) xed_get_largest_enclosing_register(reg)

//Used in decoder.h. zsim (pin) requires REG_LAST whereas zsim_trace requires XED_REG_LAST
#define REG_LAST XED_REG_LAST
#define BBL std::vector<InstInfo> *


//XED expansion macros (enable us to type opcodes at a reasonable speed)
#define XC(cat) (XED_CATEGORY_##cat)
#define XO(opcode) (XED_ICLASS_##opcode)

#define ADDRINT uint64_t
#define THREADID uint32_t
#define BOOL bool

/* Function pointer magic to cast void (*) into a void *(*) */
extern pthread_t contention_thread;
static inline void *wrap_thread_func(void * fa) {
    std::pair <void (*)(void *), void *> *p = (std::pair <void (*)(void *), void *> *)fa;

    p->first(p->second);
    return NULL;
}
#define PIN_SpawnInternalThread(func, arg, stackSize, threadid) pthread_create(&contention_thread, NULL, &wrap_thread_func, (void *)(new std::pair <void (*)(void *), void *>(&func, arg)));


#else
#include "pin.H"
extern "C" {
#include "third_party/pin/pin-2.14-71313/extras/xed-intel64/include/xed-interface.h"
}

#endif

#endif  // THIRD_PARTY_ZSIM_SRC_WRAPPED_PIN_H_
