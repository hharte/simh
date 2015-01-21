/* 3b2_cpu.h: AT&T 3B2 Model 400 Hard Disk (2797) Implementation

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

#include "3b2_id.h"

/* Command Codes (bits 3-7 of command byte) */

#define ID_CMD_AUX      0x00  /* Auxiliary Command */
#define ID_CMD_SIS      0x01  /* Sense int. status */
#define ID_CMD_SPEC     0x02  /* Specify           */
#define ID_CMD_SUS      0x03  /* Sense unit status */
#define ID_CMD_DERR     0x04  /* Detect Error      */
#define ID_CMD_RECAL    0x05  /* Recalibrate       */
#define ID_CMD_SEEK     0x06  /* Seek              */
#define ID_CMD_FMT      0x07  /* Format            */
#define ID_CMD_VID      0x08  /* Verify ID         */
#define ID_CMD_RID      0x09  /* Read ID           */
#define ID_CMD_RDIAG    0x0A  /* Read Diagnostic   */
#define ID_CMD_RDATA    0x0B  /* Read Data         */
#define ID_CMD_CHECK    0x0C  /* Check             */
#define ID_CMD_SCAN     0x0D  /* Scan              */
#define ID_CMD_VDATA    0x0E  /* Verify Data       */
#define ID_CMD_WDATA    0x0F  /* Write Data        */

#define ID_AUX_RST      0x01
#define ID_AUX_CLB      0x02
#define ID_AUX_HSRQ     0x04
#define ID_AUX_CLCE     0x08

#define ID_STAT_DRQ     0x01
#define ID_STAT_NCI     0x02
#define ID_STAT_IER     0x04
#define ID_STAT_RRQ     0x08
#define ID_STAT_SRQ     0x10
#define ID_STAT_CEL     0x20
#define ID_STAT_CEH     0x40
#define ID_STAT_CB      0x80

/* Unit, Register, Device descriptions */

#define ID_FIFO_LEN 8

uint8 id_cmd;
uint8 id_data[ID_FIFO_LEN];   /* 32-byte FIFO */
uint8 id_data_p = 0;          /* FIFO write pointer */
uint8 id_status;
uint16 id_track;

UNIT id_unit[] = {
    { UDATA (&id_svc, UNIT_FIX+UNIT_ATTABLE, ID_DSK_SIZE) },
    { NULL }
};

REG id_reg[] = {
    { HRDATAD(CMD, id_cmd, 8, "Command buffer") },
    { BRDATAD(FIFO, id_data, 16, 8, ID_FIFO_LEN, "FIFO data buffer") }
};

DEVICE id_dev = {
    "ID", id_unit, id_reg, NULL,
    1, 16, 31, 1, 16, 8,
    NULL, NULL, &id_reset,
    &id_boot, &id_attach, NULL, NULL,
    DEV_DEBUG, 0, sys_deb_tab
};

/* Function implementation */

t_stat id_svc(UNIT *uptr)
{
    return SCPE_OK;
}

t_stat id_reset(DEVICE *dptr)
{
    memset(&id_data, 0, sizeof(uint8) * ID_FIFO_LEN);
    id_data_p = 0;
    return SCPE_OK;
}

t_stat id_attach(UNIT *uptr, char *cptr)
{
    attach_unit(uptr, cptr);
    return SCPE_OK;
}

t_stat id_boot(int32 unitno, DEVICE *dptr)
{
    /* TODO: Set PC, etc. */
    return SCPE_OK;
}

uint32 id_read(uint32 pa, uint8 size) {
    uint8 data, reg;

    data = 0;
    reg = pa - IDBASE;

    switch(reg) {
    case 0:     /* Data Buffer Register */
        if (id_data_p < ID_FIFO_LEN) {
            data = id_data[id_data_p++];
            sim_debug(READ_MSG, &id_dev, ">>> READ DATA: %02x\n", data);
            id_status &= ~(ID_STAT_CEL | ID_STAT_CEH);
        }

        break;
    case 1:     /* Status Register */
        data = id_status;
        sim_debug(READ_MSG, &id_dev, ">>> READ STATUS %02x\n", data);
        break;
    }

    return data;
}
static int counter = 2;

void id_handle_command(uint8 val)
{
    uint8 dev, aux_cmd, data;

    id_data_p = 0;

    id_cmd = (val >> 4) & 0xf;
    dev = val & 0x07;

    switch(id_cmd) {
    case ID_CMD_AUX: /* Auxiliary Command */
        aux_cmd = val & 0x0f;

        if (aux_cmd & ID_AUX_CLCE) {
            /* Clear CE bits of the status register */
            sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: (AUX) CLEAR CE BITS\n");
            id_status &= ~(ID_STAT_CEL | ID_STAT_CEH);
        }

        if (aux_cmd & ID_AUX_HSRQ) {
            /* Deactivate interrupt request output */
            sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: (AUX) DEACTIVATE INT. REQ. OUTPUT\n");
        }

        if (aux_cmd & ID_AUX_CLB) {
            /* Clear data buffer */
            sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: (AUX) CLEAR DATA BUFFER\n");
            memset(&id_data, 0, sizeof(uint8) * ID_FIFO_LEN);
            id_data_p = 0;
        }

        if (aux_cmd & ID_AUX_RST) {
            sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: (AUX) RESET\n");
            id_status &= ~(ID_STAT_CEL | ID_STAT_CEH | ID_STAT_SRQ);
        }

        break;
    case ID_CMD_SIS:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: SENSE INTERRUPT STATUS\n");
        id_status |= ID_STAT_CEH; /* Command complete */
        break;
    case ID_CMD_SPEC:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: SPECIFY\n");

        /* Inspect the data, reset the pointers */
        for (id_data_p = 0; id_data_p < 8; id_data_p++) {
            sim_debug(WRITE_MSG, &id_dev,
                      ">>>    Processing byte: %02x\n", id_data[id_data_p]);
        }

        id_status |= ID_STAT_CEH; /* Command complete */
        break;
    case ID_CMD_SUS:
        id_data_p = 0;

        data = 0x1a;

        if (id_track == 0) {
            data |= 0x04;
        }

        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: SENSE UNIT STATUS: %02x\n", data);

        id_data[id_data_p++] = data;
        id_status |= ID_STAT_CEH; /* Command complete */
        break;
    case ID_CMD_DERR:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: DETECT ERROR\n");
        id_status |= ID_STAT_CEH; /* Command complete */
        break;
    case ID_CMD_RECAL:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: RECALIBRATE\n");
        id_track = 0;
        id_track = id_data[0] << 8 | id_data[1];
        id_status &= ~ID_STAT_SRQ;
        id_status |= ID_STAT_CEH; /* Command complete */
        break;
    case ID_CMD_SEEK:
        data = id_data[0] << 8 | id_data[1];
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: SEEK TO %04x\n", data);
        id_track = (id_data[0] << 8 | id_data[1]);
        id_status |= (ID_STAT_CEH | ID_STAT_SRQ);
        break;
    case ID_CMD_FMT:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: FORMAT\n");
        break;
    case ID_CMD_VID:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: VERIFY ID\n");
        break;
    case ID_CMD_RID:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: READ ID\n");
        break;
    case ID_CMD_RDIAG:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: READ DIAG\n");
        break;
    case ID_CMD_RDATA:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: READ DATA\n");
        id_status = (ID_STAT_CEH | ID_STAT_DRQ); /* command complete */
        break;
    case ID_CMD_CHECK:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: READ CHECK\n");
        break;
    case ID_CMD_SCAN:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: READ SCAN\n");
        break;
    case ID_CMD_VDATA:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: VERIFY DATA\n");
        break;
    case ID_CMD_WDATA:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: WRITE DATA\n");
        break;
    default:
        sim_debug(WRITE_MSG, &id_dev, ">>> COMMAND: %02x\n", id_cmd);
        break;
    }
}


void id_handle_data(uint8 val)
{
    sim_debug(WRITE_MSG, &id_dev, ">>> DATA=%02x\n", val);

    if (id_data_p < ID_FIFO_LEN) {
        id_data[id_data_p++] = val & 0xff;
    } else {
        sim_debug(WRITE_MSG, &id_dev, ">>> FIFO FULL!\n");
    }
}


void id_write(uint32 pa, uint32 val, uint8 size)
{
    uint8 reg;

    reg = pa - IDBASE;

    switch(reg) {
    case 0:     /* Data Buffer Register */
        id_handle_data((uint8)val);
        break;
    case 1:     /* Command Buffer */
        id_handle_command((uint8)val);
        break;
    default:
        break;
    }
}
