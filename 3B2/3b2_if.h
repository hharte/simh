/* 3b2_cpu.h: AT&T 3B2 Model 400 Floppy (TMS2797NL) Header

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

   Except as contained in this notice, the name of Charles E. Owen shall not be
   used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from Charles E. Owen.
*/

#ifndef __3B2_IF_H__
#define __3B2_IF_H__

#include "3b2_defs.h"
#include "3b2_sysdev.h"

extern DEVICE if_dev;
extern DEBTAB sys_deb_tab[];

#define IFBASE 0x4d000
#define IFSIZE 0x10

/* Constants */

#define DSK_SECTSIZE  1024
#define DSK_SECT      9
#define DSK_SURF      1
#define DSK_CYL       80
#define DSK_SIZE      (DSK_SECT * DSK_SURF * DSK_CYL * DSK_SECTSIZE)

/* Function prototypes */

t_stat if_svc(UNIT *uptr);
t_stat if_reset(DEVICE *dptr);
t_stat if_attach(UNIT *uptr, char *cptr);
t_stat if_boot(int32 unitno, DEVICE *dptr);
uint32 if_read(uint32 pa, uint8 size);
void if_write(uint32 pa, uint32 val, uint8 size);

#endif
