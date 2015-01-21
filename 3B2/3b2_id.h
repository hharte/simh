/* 3b2_cpu.h: AT&T 3B2 Model 400 Hard Disk (2797) Header

   Copyright (c) 2014, Seth J. Morabito

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

#ifndef __3B2_ID_H__
#define __3B2_ID_H__

#include "3b2_defs.h"
#include "3b2_sysdev.h"

extern DEVICE id_dev;
extern DEBTAB sys_deb_tab[];

#define IDBASE 0x4a000
#define IDSIZE 0x2

#define ID_DSK_SIZE 1024 * 1024 * 74

/* Function prototypes */

t_stat id_svc(UNIT *uptr);
t_stat id_reset(DEVICE *dptr);
t_stat id_attach(UNIT *uptr, char *cptr);
t_stat id_boot(int32 unitno, DEVICE *dptr);
uint32 id_read(uint32 pa, uint8 size);
void id_write(uint32 pa, uint32 val, uint8 size);

#endif
