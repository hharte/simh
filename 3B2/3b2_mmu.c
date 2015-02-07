/* 3b2_mmu.c: AT&T 3B2 Model 400 MMU (WE32101) Implementation

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

#include "3b2_mmu.h"

/*******************************************************************
 *
 * The WE32101 MMU divides the virtual address space into four
 * sections. Virtual address bits 30 and 31 determine the section.
 *
 * To initialize the MMU, the operating system must:
 *
 *   - Write SDTs
 *
 *
 * Vocabulary:
 *
 *    SID: Section ID. Can be one of 0, 1, 2, or 3
 *      - SSL: Segment Select. 8K segments within the section.
 *
 *        Continguous Segment Addressing:
 *          - SOT: Segment Offset. 128K addresses within segment.
 *
 *        Paged Segment Addressing:
 *          - PSL: Page Select. 64 pages within the section.
 *          - POT: Page Offset. 2K addresses within the page.
 *
 *    SD: Segment Descriptor. Both Contiguous and Paged addressing
 *        use Segment Descriptors located in a Segment Descriptor
 *        Table (SDT) to describe the physical layout of memory.
 *
 *    SDT: Segment Descriptor Table.
 *
 *    PDT: Page Descriptor Table.
 *    PD: Page Descriptor. When using Paged addressing,
 *
 *******************************************************************/

t_bool mmu_en = FALSE;  /* MMU Enabled */

/*
 * TODO: MMU support. Currently only physical addresses are supported.
 */

t_bool addr_is_rom(uint32 pa)
{
    return (pa < ROM_SIZE);
}

t_bool addr_is_mem(uint32 pa)
{
    return (pa >= PHYS_MEM_BASE &&
            pa < (PHYS_MEM_BASE + MEM_SIZE));
}

t_bool addr_is_io(uint32 pa)
{
    return ((pa >= IO_BASE && pa < IO_BASE + IO_SIZE) ||
            (pa >= IOB_BASE && pa < IOB_BASE + IOB_SIZE));
}

/*
 * Raw physical reads and writes.
 *
 * The WE32100 is a BIG-endian machine, meaning that words are
 * arranged in increasing address from most-significant byte to
 * least-significant byte.
 */

/*
 * Read Word (Physical Address, Unaligned)
 */
uint32 pread_w_u(uint32 pa)
{
    uint32 *m;
    uint32 index;

    if (addr_is_io(pa)) {
        return io_read(pa, 32);
    }

    if (addr_is_rom(pa)) {
        m = ROM;
        index = pa >> 2;
    } else if (addr_is_mem(pa)) {
        m = RAM;
        index = (pa - PHYS_MEM_BASE) >> 2;
    } else {
        cpu_set_exception(NORMAL_EXCEPTION, EXTERNAL_MEMORY_FAULT);
        return 0;
    }

    return m[index];
}

/*
 * Read Word (Physical Address)
 */
uint32 pread_w(uint32 pa)
{
    /* Alignment exception */
    if (pa & 3) {
        cpu_set_exception(NORMAL_EXCEPTION, EXTERNAL_MEMORY_FAULT);
        return 0;
    }

    return pread_w_u(pa);
}

/*
 * Write Word (Physical Address, Unaligned)
 */
void pwrite_w_u(uint32 pa, uint32 val)
{
    uint32 *m;
    uint32 index;

    if (addr_is_io(pa)) {
        io_write(pa, val, 32);
        return;
    }

    if (addr_is_rom(pa)) {
        m = ROM;
        index = pa >> 2;
    } else if (addr_is_mem(pa)) {
        m = RAM;
        index = (pa - PHYS_MEM_BASE) >> 2;
    } else {
        cpu_set_exception(NORMAL_EXCEPTION, EXTERNAL_MEMORY_FAULT);
    }

    m[index] = val;
}

/*
 * Write Word (Physical Address)
 */
void pwrite_w(uint32 pa, uint32 val)
{
    /* Alignment exception */
    if (pa & 3) {
        cpu_set_exception(NORMAL_EXCEPTION, EXTERNAL_MEMORY_FAULT);
        return;
    }

    pwrite_w_u(pa, val);
}

/*
 * Read Halfword (Physical Address)
 */
uint16 pread_h(uint32 pa)
{
    uint32 *m;
    uint32 index;

    if (pa & 1) {
        cpu_set_exception(NORMAL_EXCEPTION, EXTERNAL_MEMORY_FAULT);
    }

    if (addr_is_io(pa)) {
        return io_read(pa, 16);
    }

    if (addr_is_rom(pa)) {
        m = ROM;
        index = pa >> 2;
    } else if (addr_is_mem(pa)) {
        m = RAM;
        index = (pa - PHYS_MEM_BASE) >> 2;
    } else {
        cpu_set_exception(NORMAL_EXCEPTION, EXTERNAL_MEMORY_FAULT);
        return 0;
    }

    if (pa & 2) {
        return m[index] & HALF_MASK;
    } else {
        return (m[index] >> 16) & HALF_MASK;
    }
}

/*
 * Write Halfword (Physical Address)
 */
void pwrite_h(uint32 pa, uint16 val)
{
    uint32 *m;
    uint32 index;
    uint32 wval = (uint32)val;

    if (pa & 1) {
        cpu_set_exception(NORMAL_EXCEPTION, EXTERNAL_MEMORY_FAULT);
        return;
    }

    if (addr_is_io(pa)) {
        io_write(pa, val, 16);
        return;
    }

    if (addr_is_rom(pa)) {
        m = ROM;
        index = pa >> 2;
    } else if (addr_is_mem(pa)) {
        m = RAM;
        index = (pa - PHYS_MEM_BASE) >> 2;
    } else {
        cpu_set_exception(NORMAL_EXCEPTION, EXTERNAL_MEMORY_FAULT);
        return;
    }

    if (pa & 2) {
        m[index] = (m[index] & ~HALF_MASK) | wval;
    } else {
        m[index] = (m[index] & HALF_MASK) | (wval << 16);
    }
}

/*
 * Read Byte (Physical Address)
 */
uint8 pread_b(uint32 pa)
{
    int32 data;
    int32 sc = (~(pa & 3) << 3) & 0x1f;

    if (addr_is_io(pa)) {
        return io_read(pa, 8);
    }

    if (addr_is_rom(pa)) {
        data = ROM[pa >> 2];
    } else if (addr_is_mem(pa)) {
        data = RAM[(pa - PHYS_MEM_BASE) >> 2];
    } else {
        cpu_set_exception(NORMAL_EXCEPTION, EXTERNAL_MEMORY_FAULT);
        return 0;
    }

    return (data >> sc) & BYTE_MASK;
}

/*
 * Write Byte (Physical Address)
 */
void pwrite_b(uint32 pa, uint8 val)
{
    uint32 *m;
    int32 index;
    int32 sc = (~(pa & 3) << 3) & 0x1f;
    int32 mask = 0xff << sc;

    if (addr_is_io(pa)) {
        io_write(pa, val, 8);
        return;
    }

    if (addr_is_rom(pa)) {
        m = ROM;
        index = pa >> 2;
    } else if (addr_is_mem(pa)) {
        m = RAM;
        index = (pa - PHYS_MEM_BASE) >> 2;
    } else {
        cpu_set_exception(NORMAL_EXCEPTION, EXTERNAL_MEMORY_FAULT);
        return;
    }

    m[index] = (m[index] & ~mask) | (val << sc);
}
