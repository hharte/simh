/* 3b2_cpu.h: AT&T 3B2 Model 400 Floppy (TMS2797NL) Implementation

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

#include "3b2_if.h"

#define IF_STATUS_REG    0
#define IF_CMD_REG       0
#define IF_TRACK_REG     1
#define IF_SECTOR_REG    2
#define IF_DATA_REG      3

#define IF_BUSY          0x01
#define IF_DRQ           0x02
#define IF_TK_0          0x04
#define IF_CRC_ER        0x08
#define IF_SK_ER         0x10
#define IF_HD_LD         0x20
#define IF_WP            0x40
#define IF_NRDY          0x80

/* Type I Commands */
#define IF_RESTORE       0x00
#define IF_SEEK          0x10
#define IF_STEP          0x20
#define IF_STEP_T        0x30
#define IF_STEP_IN       0x40
#define IF_STEP_IN_T     0x50
#define IF_STEP_OUT      0x60
#define IF_STEP_OUT_T    0x70

/* Type II Commands */
#define IF_READ_SEC      0x80
#define IF_READ_SEC_M    0x90
#define IF_WRITE_SEC     0xA0
#define IF_WRITE_SEC_M   0xB0

/* Type III Commands */
#define IF_READ_ADDR     0xC0
#define IF_READ_TRACK    0xE0
#define IF_WRITE_TRACK   0xF0

/* Type IV Command */
#define IF_FORCE_INT     0xD0

/* Unit, Register, Device descriptions */

typedef struct {
    uint8 data;
    uint8 cmd;
    uint8 cmd_type;
    uint8 status;
    uint8 track;
    uint8 sector;
} IF_STATE;

IF_STATE if_state;

UNIT if_unit[] = {
    { UDATA (&if_svc, UNIT_FIX+UNIT_ATTABLE, DSK_SIZE) },
    { NULL }
};

REG if_reg[] = {
    { NULL }
};

DEVICE if_dev = {
    "IF", if_unit, if_reg, NULL,
    1, 16, 31, 1, 16, 8,
    NULL, NULL, &if_reset,
    NULL, &if_attach, NULL, NULL,
    DEV_DEBUG, 0, sys_deb_tab
};

/* Function implementation */

t_stat if_svc(UNIT *uptr)
{
    return SCPE_OK;
}

t_stat if_reset(DEVICE *dptr)
{
    if_state.status = (IF_TK_0 | IF_HD_LD);
    if_state.track = 0;
    if_state.sector = 0x0a;
    return SCPE_OK;
}

t_stat if_attach(UNIT *uptr, char *cptr)
{
    /* TODO: Optional code to determine whether disk image is 9, 10,
       or 11 sectors per track. */
    attach_unit(uptr, cptr);
    return SCPE_OK;
}


uint32 if_read(uint32 pa, uint8 size) {
    uint8 reg, data;

    reg = pa - IFBASE;

    switch (reg) {
    case IF_STATUS_REG:
        sim_debug(READ_MSG, &if_dev, "STATUS REGISTER (0x%02x)\n", if_state.status);
        data = if_state.status;
        break;
    case IF_TRACK_REG:
        sim_debug(READ_MSG, &if_dev, "TRACK REGISTER (0x%02x)\n", if_state.track);
        data = if_state.track;
    case IF_SECTOR_REG:
        sim_debug(READ_MSG, &if_dev, "SECTOR REGISTER (0x%02x)\n", if_state.sector);
        data = if_state.sector;
    case IF_DATA_REG:
        sim_debug(READ_MSG, &if_dev, "DATA REGISTER\n");
        data = 0x1f;
        break;
    default:
        sim_debug(READ_MSG, &if_dev, "UNKNOWN REGISTER [%08x]\n", pa);
        break;
    }

    return data;
}

/* Handle the most recently received command */
void if_handle_command()
{
    switch(if_state.cmd & 0xf0) {
    case IF_RESTORE:
    case IF_SEEK:
    case IF_STEP:
    case IF_STEP_T:
    case IF_STEP_IN:
    case IF_STEP_IN_T:
    case IF_STEP_OUT:
    case IF_STEP_OUT_T:
        if_state.cmd_type = 1;
        if_state.status &= ~(IF_CRC_ER | IF_SK_ER | IF_DRQ);
        break;
    case IF_FORCE_INT:
        if_state.cmd_type = 4;
        break;
    }

    switch(if_state.cmd & 0xf0) {
    case IF_SEEK:
        if_state.track = if_state.data;

        if (if_state.track == 0) {
            if_state.status |= IF_TK_0;
        } else {
            if_state.status &= ~(IF_TK_0);
        }

        if_state.status |= IF_DRQ;

        break;
    case IF_FORCE_INT:
        break;
    }
}

void if_write(uint32 pa, uint32 val, uint8 size)
{
    uint8 reg;

    reg = pa - IFBASE;

    switch (reg) {
    case IF_CMD_REG:
        sim_debug(WRITE_MSG, &if_dev, "COMMAND REGISTER = %02x\n", val);
        if_state.cmd = val & 0xff;
        if_handle_command();
        break;
    case IF_TRACK_REG:
        sim_debug(WRITE_MSG, &if_dev, "TRACK REGISTER = %02x\n", val);
        if_state.track = val & 0xff;
        break;
    case IF_SECTOR_REG:
        sim_debug(WRITE_MSG, &if_dev, "SECTOR REGISTER = %02x\n", val);
        if_state.sector = val & 0xff;
        break;
    case IF_DATA_REG:
        sim_debug(WRITE_MSG, &if_dev, "DATA REGISTER = %02x\n", val);
        if_state.data = val & 0xff;
        break;
    default:
        sim_debug(WRITE_MSG, &if_dev, "UNKNOWN REGISTER [%08x] = %02x\n", pa, val);
        break;
    }

}
