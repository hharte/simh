/* 3b2_defs.h: AT&T 3B2 Model 400 Simulator Definitions

   Copyright (c) 2015, Seth J. Morabito

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   ROBERT M SUPNIK BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the name of the author shall not be
   used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from the author.
*/

#ifndef _3B2_DEFS_H_
#define _3B2_DEFS_H_

#include "sim_defs.h"

#define FALSE 0
#define TRUE 1

#define MAX_HIST_SIZE  50000
#define DEF_HIST_SIZE  100
#define MAXMEMSIZE	   (1 << 22)             /* 4 MB */
#define MEMSIZE		   (cpu_unit.capac)		 /* actual memory size */
#define UNIT_V_MSIZE   (UNIT_V_UF)

#define UNIT_MSIZE     (1 << UNIT_V_MSIZE)

/* Register numbers */
#define NUM_FP         9
#define NUM_AP         10
#define NUM_PSW        11
#define NUM_SP         12
#define NUM_PCBP       13
#define NUM_ISP        14
#define NUM_PC         15

/* Simulator stop codes */
#define STOP_RSRV   1
#define STOP_IBKPT  2
#define STOP_OPCODE 3
#define STOP_IRQ    4

/* Debug flags */
#define READ_MSG    (1 << 0)
#define WRITE_MSG   (1 << 1)
#define DECODE_MSG  (1 << 2)
#define EXECUTE_MSG (1 << 3)
#define INIT_MSG    (1 << 4)
#define IRQ_MSG     (1 << 5)
#define IO_D_MSG    (1 << 6)

#endif
