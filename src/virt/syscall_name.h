/** $lic$
 * Copyright (C) 2017 by Google
 * Copyright (C) 2012-2015 by Massachusetts Institute of Technology
 * Copyright (C) 2010-2013 by The Board of Trustees of Stanford University
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

#ifndef VIRT_SYSCALL_NAME_H_
#define VIRT_SYSCALL_NAME_H_

/* TODO: Grab the defines below from Google3 */
#define SYS_google_syscall_switchto 602
#define SYS_google_syscall_get_gtid 603

#define SYS_SWITCHTO_SET_FLAGS           -1
#define SYS_SWITCHTO_SETID        0
#define SYS_SWITCHTO_WAIT        1
#define SYS_SWITCHTO_SWITCH        2
#define SYS_SWITCHTO_RESUME        3
#define SYS_SWITCHTO_SWITCH_CROSS_PROC  4
#define SYS_SWITCHTO_RESUME_CROSS_PROC  5
#define SYS_SWITCHTO_SWITCH_POLLED  6

const char* GetSyscallName(uint32_t syscall);

#endif  // VIRT_SYSCALL_NAME_H_
