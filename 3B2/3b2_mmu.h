/* 3b2_mmu.c: AT&T 3B2 Model 400 MMU (WE32101) Header

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

#ifndef _3B2_MMU_H
#define _3B2_MMU_H

#include "3b2_defs.h"

#define MMUBASE 0x40000
#define MMUSIZE 0x1000

typedef struct {
    t_bool enabled;
} MMU_STATE;

extern DEVICE mmu_dev;

uint32 mmu_read(uint32 pa, uint8 size);
void mmu_write(uint32 pa, uint32 val, uint8 size);

uint8  pread_b(uint32 pa);
uint16 pread_h(uint32 pa);
uint32 pread_w(uint32 pa);
uint32 pread_w_u(uint32 pa);

void   pwrite_b(uint32 pa, uint8 val);
void   pwrite_h(uint32 pa, uint16 val);
void   pwrite_w(uint32 pa, uint32 val);
void   pwrite_w_u(uint32 pa, uint32 val);

uint8  vread_b(uint32 va);
uint16 vread_h(uint32 va);
uint32 vread_w(uint32 va);
void   vwrite_b(uint32 va);
void   vwrite_h(uint32 va);
void   vwrite_w(uint32 va);

t_bool addr_is_rom(uint32 pa);
t_bool addr_is_mem(uint32 pa);
t_bool addr_is_io(uint32 pa);

void mmu_enable();
void mmu_disable();

#endif
