/*************************************************************************
 *                                                                       *
 * Copyright (c) 2022-2023 Howard M. Harte.                              *
 * https://github.com/hharte                                             *
 *                                                                       *
 * Permission is hereby granted, free of charge, to any person obtaining *
 * a copy of this software and associated documentation files (the       *
 * "Software"), to deal in the Software without restriction, including   *
 * without limitation the rights to use, copy, modify, merge, publish,   *
 * distribute, sublicense, and/or sell copies of the Software, and to    *
 * permit persons to whom the Software is furnished to do so, subject to *
 * the following conditions:                                             *
 *                                                                       *
 * The above copyright notice and this permission notice shall be        *
 * included in all copies or substantial portions of the Software.       *
 *                                                                       *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       *
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    *
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-            *
 * INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE   *
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN       *
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN     *
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE      *
 * SOFTWARE.                                                             *
 *                                                                       *
 * Except as contained in this notice, the names of The Authors shall    *
 * not be used in advertising or otherwise to promote the sale, use or   *
 * other dealings in this Software without prior written authorization   *
 * from the Authors.                                                     *
 *                                                                       *
 * Module Description:                                                   *
 *     SD-SPI Memory Card module for SIMH.                               *
 *                                                                       *
 *************************************************************************/

#include "altairz80_defs.h"
#include "sim_imd.h"

/* Debug flags */
#define ERROR_MSG   (1 << 0)
#define SEEK_MSG    (1 << 1)
#define CMD_MSG     (1 << 2)
#define RD_DATA_MSG (1 << 3)
#define WR_DATA_MSG (1 << 4)
#define SPI_MSG     (1 << 5)
#define VERBOSE_MSG (1 << 6)

#define RC_SDCARD_MAX_DRIVES          4       /* Maximum number of drives supported */
#define RC_SDCARD_MAX_SECLEN          512     /* Maximum of 512 bytes per sector */

#define DEV_NAME    "SD"

#define SDSPI_START             (0x40)
#define SDSPI_START_MASK        (0xC0)
#define SDSPI_CMD_MASK          (0x3F)

#define SDSPI_DATA_TOKEN        (0xFE)

#define SDSPI_STATE_IDLE        (0)
#define SDSPI_STATE_CMD         (1)
#define SDSPI_STATE_RX_CRC      (2)
#define SDSPI_STATE_TX_DATA     (3)
#define SDSPI_STATE_TX_DATA_RSP (4)

#define SDCARD_STATE_UNINITIALIZED  (0)
#define SDCARD_STATE_RESET      (1)
#define SDCARD_STATE_INITIALIZED (2)

#define SDSPI_CMD0              (0)
#define SDSPI_CMD1              (1)
#define SDSPI_CMD55             (55)
#define SDSPI_CMD8              (8)
#define SDSPI_CMD9              (9)         /* Read CSD */
#define SDSPI_CMD10             (10)        /* Read CID */
#define SDSPI_CMD16             (16)        /* Set block length */
#define SDSPI_CMD17             (17)        /* Read single block */
#define SDSPI_CMD24             (24)        /* Write single block */

typedef struct {
    UNIT    *uptr;
    uint8   readonly;    /* Drive is read-only? */
    uint16  sectsize;    /* sector size */
    uint8   sdspi_state;
    uint8   sdcard_state;
    uint8   sdspi_cmd;
    uint32  sdspi_arg;
    uint32  sdspi_arg_cnt;
    uint8   sdspi_crc;
    uint8   spi_bytes[520];
    uint16  spi_byte_index;
    uint16  sdspi_rsp_len;
    uint32  sdcard_blocklen;
    uint8   sdcard_detect;

} RC_SDCARD_DRIVE_INFO;

typedef struct {
    uint8   sel_drive;  /* Currently selected drive */
    uint8   ndrives;    /* Number of drives attached to the controller */
    RC_SDCARD_DRIVE_INFO drive[RC_SDCARD_MAX_DRIVES];
} RC_SDCARD_INFO;

static RC_SDCARD_INFO rc_sdcard_info_data = { 0 };
static RC_SDCARD_INFO *rc_sdcard_info = &rc_sdcard_info_data;

extern uint32 PCX;
extern t_stat set_iobase(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
extern t_stat show_iobase(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
extern uint32 sim_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                               int32 (*routine)(const int32, const int32, const int32), const char* name, uint8 unmap);
extern int32 find_unit_index(UNIT *uptr);

typedef void (*csio_write_cb_t) (uint8 data);
typedef uint8(*csio_read_cb_t) (void);

extern t_stat csio_register_callbacks(csio_write_cb_t csio_write_cb, csio_read_cb_t csio_read_cb);

void sdspi_write(uint8 data);
uint8 sdspi_read(void);
static void sdspi_execute_command(void);
static t_stat sdspi_single_block_read(void);
static t_stat sdspi_single_block_write(void);

#define UNIT_V_RC_SDCARD_VERBOSE    (UNIT_V_UF + 1) /* verbose mode, i.e. show error messages   */
#define UNIT_RC_SDCARD_VERBOSE      (1 << UNIT_V_RC_SDCARD_VERBOSE)
#define RC_SDCARD_CAPACITY          (512*4*16*512)   /* Default Disk Capacity Quantum 2020 */

static t_stat rc_sdcard_reset(DEVICE *rc_sdcard_dev);
static t_stat rc_sdcard_attach(UNIT *uptr, CONST char *cptr);
static t_stat rc_sdcard_detach(UNIT *uptr);
static t_stat rc_sdcard_unit_set_geometry(UNIT* uptr, int32 value, CONST char* cptr, void* desc);
static t_stat rc_sdcard_unit_show_geometry(FILE* st, UNIT* uptr, int32 val, CONST void* desc);

static const char* rc_sdcard_description(DEVICE *dptr);

static UNIT rc_sdcard_unit[] = {
    { UDATA (NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, RC_SDCARD_CAPACITY) },
    { UDATA (NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, RC_SDCARD_CAPACITY) },
    { UDATA (NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, RC_SDCARD_CAPACITY) },
    { UDATA (NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, RC_SDCARD_CAPACITY) }
};

static REG rc_sdcard_reg[] = {
//    { HRDATAD (TF_ERROR,    rc_sdcard_info_data.error_reg,            8, "Taskfile Error Register"), },
    { NULL }
};

#define RC_SDCARD_NAME    "RC SD Card"

static const char* rc_sdcard_description(DEVICE *dptr) {
    return RC_SDCARD_NAME;
}

static MTAB rc_sdcard_mod[] = {
    { MTAB_XTD|MTAB_VDV,    0,                      "IOBASE",   "IOBASE",
        &set_iobase, &show_iobase, NULL, "Sets disk controller I/O base address"    },
    { MTAB_XTD|MTAB_VUN|MTAB_VALR,    0,                  "GEOMETRY",     "GEOMETRY",
        &rc_sdcard_unit_set_geometry, &rc_sdcard_unit_show_geometry, NULL,
        "Set disk geometry C:nnnn/H:n/S:nnn/N:nnnn" },
    { 0 }
};

/* Debug Flags */
static DEBTAB rc_sdcard_dt[] = {
    { "ERROR",      ERROR_MSG,      "Error messages"    },
    { "SEEK",       SEEK_MSG,       "Seek messages"     },
    { "CMD",        CMD_MSG,        "Command messages"  },
    { "READ",       RD_DATA_MSG,    "Read messages"     },
    { "WRITE",      WR_DATA_MSG,    "Write messages"    },
    { "SPI",        SPI_MSG,        "SPI messages"      },
    { "VERBOSE",    VERBOSE_MSG,    "Verbose messages"  },
    { NULL,         0                                   }
};

DEVICE rc_sdcard_dev = {
    DEV_NAME, rc_sdcard_unit, rc_sdcard_reg, rc_sdcard_mod,
    RC_SDCARD_MAX_DRIVES, 10, 31, 1, RC_SDCARD_MAX_DRIVES, RC_SDCARD_MAX_DRIVES,
    NULL, NULL, &rc_sdcard_reset,
    NULL, &rc_sdcard_attach, &rc_sdcard_detach,
    &rc_sdcard_info_data, (DEV_DISABLE | DEV_DIS | DEV_DEBUG), ERROR_MSG,
    rc_sdcard_dt, NULL, NULL, NULL, NULL, NULL, &rc_sdcard_description
};

static uint8 bit_reverse(uint8 byte) {
    uint8 result = 0;
    uint8 bit;

    for (bit = 0; bit < 8; bit++) {
        result |= ((byte & (1 << bit)) >> bit) << (7 - bit);
    }
    return(result);
}


/* Reset routine */
static t_stat rc_sdcard_reset(DEVICE *dptr)
{
    int i;

    rc_sdcard_info->sel_drive = 0;

    for (i = 0; i < RC_SDCARD_MAX_DRIVES; i++) {
        RC_SDCARD_DRIVE_INFO* pDrive = &rc_sdcard_info->drive[i];

        pDrive->sdspi_state = SDSPI_STATE_IDLE;
        pDrive->sdcard_state = SDCARD_STATE_UNINITIALIZED;
        pDrive->sdspi_cmd = 0;
        pDrive->sdspi_arg = 0;
        pDrive->sdspi_arg_cnt = 0;
        pDrive->sdspi_crc = 0;
        memset(pDrive->spi_bytes, 0, sizeof(pDrive->spi_bytes));
        pDrive->spi_byte_index = 0;
        pDrive->sdspi_rsp_len = 0;
        pDrive->sdcard_blocklen = 0;
    }

    if (dptr->flags & DEV_DIS) {
        /* Disconnect CSIO Callbacks */
        csio_register_callbacks(NULL, NULL);
    }
    else {
        /* Connect CSIO Callbacks */
        csio_register_callbacks(sdspi_write, sdspi_read);
    }

    return SCPE_OK;
}


/* Attach routine */
static t_stat rc_sdcard_attach(UNIT *uptr, CONST char *cptr)
{
    t_stat r = SCPE_OK;
    RC_SDCARD_DRIVE_INFO *pDrive;
    int i = 0;

    i = find_unit_index(uptr);
    if (i == -1) {
        return (SCPE_IERR);
    }
    pDrive = &rc_sdcard_info->drive[i];

    pDrive->sectsize = 512;
    pDrive->sdcard_detect = 0;

    r = attach_unit(uptr, cptr);    /* attach unit  */
    if ( r != SCPE_OK)              /* error?       */
        return r;

    /* Determine length of this disk */
    if(sim_fsize(uptr->fileref) != 0) {
        uptr->capac = sim_fsize(uptr->fileref);
    } else {
        uptr->capac = 16 * 1024 * 1024;
    }

    pDrive->uptr = uptr;

    /* Default for new file is DSK */
    uptr->u3 = IMAGE_TYPE_DSK;

    if(uptr->capac > 0) {
        r = assignDiskType(uptr);
        if (r != SCPE_OK) {
            rc_sdcard_detach(uptr);
            return r;
        }
    }

    sim_debug(VERBOSE_MSG, &rc_sdcard_dev, DEV_NAME "%d, attached to '%s', type=DSK, len=%d\n",
        i, cptr, uptr->capac);

    pDrive->readonly = (uptr->flags & UNIT_RO) ? 1 : 0;
    pDrive->sdcard_detect = 1;
    return SCPE_OK;
}


/* Detach routine */
static t_stat rc_sdcard_detach(UNIT *uptr)
{
    RC_SDCARD_DRIVE_INFO *pDrive;
    t_stat r;
    int32 i;

    i = find_unit_index(uptr);

    if (i == -1) {
        return (SCPE_IERR);
    }

    pDrive = &rc_sdcard_info->drive[i];

    sim_debug(VERBOSE_MSG, &rc_sdcard_dev, "Detach " DEV_NAME "%d\n", i);

    r = detach_unit(uptr);  /* detach unit */
    if ( r != SCPE_OK)
        return r;

    return SCPE_OK;
}

/* Set geometry of the disk drive */
static t_stat rc_sdcard_unit_set_geometry(UNIT* uptr, int32 value, CONST char* cptr, void* desc)
{
    RC_SDCARD_DRIVE_INFO* pDrive;
    int32 i;
    int32 result;
    uint16 newCyls, newHeads, newSPT, newSecLen;

    i = find_unit_index(uptr);

    if (i == -1) {
        return (SCPE_IERR);
    }

    pDrive = &rc_sdcard_info->drive[i];

    if (cptr == NULL)
        return SCPE_ARG;

    result = sscanf(cptr, "C:%hd/H:%hd/S:%hd/N:%hd", &newCyls, &newHeads, &newSPT, &newSecLen);
    if (result != 4)
        return SCPE_ARG;

    return SCPE_OK;
}

/* Show geometry of the disk drive */
static t_stat rc_sdcard_unit_show_geometry(FILE* st, UNIT* uptr, int32 val, CONST void* desc)
{
    RC_SDCARD_DRIVE_INFO* pDrive;
    int32 i;

    i = find_unit_index(uptr);

    if (i == -1) {
        return (SCPE_IERR);
    }

    pDrive = &rc_sdcard_info->drive[i];

    fprintf(st, "C:%d/H:%d/S:%d/N:%d",
        0, 0, 0, pDrive->sectsize);

    return SCPE_OK;
}


void sdspi_write(uint8 data) {
    RC_SDCARD_DRIVE_INFO* pDrive = &rc_sdcard_info->drive[rc_sdcard_info->sel_drive];
    uint8 spi_byte = bit_reverse(data);

    sim_debug(SPI_MSG, &rc_sdcard_dev, DEV_NAME "%d: Rx SPI Byte[0x%03x]: 0x%02x\n", rc_sdcard_info->sel_drive, pDrive->spi_byte_index, spi_byte);

    switch (pDrive->sdspi_state) {
    case SDSPI_STATE_IDLE:
        if ((spi_byte & SDSPI_START_MASK) == SDSPI_START) {
            pDrive->sdspi_state = SDSPI_STATE_CMD;
            pDrive->sdspi_cmd = spi_byte & SDSPI_CMD_MASK;
            pDrive->sdspi_arg = 0;
            pDrive->sdspi_arg_cnt = 0;
        }
        break;
    case SDSPI_STATE_CMD:
        pDrive->sdspi_arg |= spi_byte << ((3 - pDrive->sdspi_arg_cnt) * 8);
        if (pDrive->sdspi_arg_cnt == 3) {
            pDrive->sdspi_state = SDSPI_STATE_RX_CRC;
        }
        pDrive->sdspi_arg_cnt++;
        pDrive->sdspi_arg_cnt &= 0x03;

        break;
    case SDSPI_STATE_RX_CRC:
        pDrive->sdspi_crc = spi_byte;
        pDrive->sdspi_state = SDSPI_STATE_IDLE;
        sdspi_execute_command();
        break;
    case SDSPI_STATE_TX_DATA:
        pDrive->spi_bytes[pDrive->spi_byte_index] = spi_byte;
        if (pDrive->spi_byte_index == 2) {
            if (spi_byte == SDSPI_DATA_TOKEN) {
                pDrive->spi_byte_index++;
            }
        }
        else {
            pDrive->spi_byte_index++;
            if (pDrive->spi_byte_index >= pDrive->sdcard_blocklen + 5) {
                pDrive->sdspi_state = SDSPI_STATE_IDLE;
                pDrive->spi_bytes[0] = 0xFF;
                pDrive->sdspi_rsp_len = 1;
                pDrive->spi_byte_index = 0;
                if (sim_fwrite(&pDrive->spi_bytes[3], 1, pDrive->sdcard_blocklen, (pDrive->uptr)->fileref) != pDrive->sdcard_blocklen) {
                    pDrive->spi_bytes[0] = 0x0D;
                }
                else {
                    pDrive->spi_bytes[0] = 0x05;
                }
            }
        }
        break;
    default:
        sim_printf("SDSPI: Error: Unhandled state 0x%02x\n", pDrive->sdspi_state);
        break;
    }
}

uint8 sdcard_csd[16] = {
    0x00, 0x2d, 0x00, 0x32, 0x5b, 0x5a, 0x83, 0xd5, 0xfe, 0xfb, 0xff, 0x80, 0x16, 0x80, 0x00, 0xcf  /* Kingston 2GB SD/SC */
};

uint8 sdcard_cid[16] = {
    0x02, 0x54, 0x4d, 0x53, 0x44, 0x30, 0x32, 0x47, 0x28, 0xad, 0x78, 0x24, 0x33, 0x00, 0x79, 0x3d  /* Kingston 2GB SD/SC */
};

static void sdspi_execute_command(void)
{
    RC_SDCARD_DRIVE_INFO* pDrive = &rc_sdcard_info->drive[rc_sdcard_info->sel_drive];

    sim_debug(CMD_MSG, &rc_sdcard_dev, DEV_NAME "%d: SDSPI: CMD=%d, arg=0x%04x, CRC=0x%02x\n",
        rc_sdcard_info->sel_drive, pDrive->sdspi_cmd, pDrive->sdspi_arg, pDrive->sdspi_crc);
    pDrive->spi_byte_index = 0;
    memset(pDrive->spi_bytes, 0, sizeof(pDrive->spi_bytes));
    pDrive->sdspi_rsp_len = 2;
    pDrive->spi_bytes[0] = 0xFF;
    pDrive->spi_bytes[1] = 0x01;

    if ((pDrive->sdcard_state == SDCARD_STATE_UNINITIALIZED) && (pDrive->sdspi_cmd != SDSPI_CMD0)) {
        sim_debug(ERROR_MSG, &rc_sdcard_dev, DEV_NAME "%d: SDSPI: Received CMD=%d while card is not initialized.\n",
            rc_sdcard_info->sel_drive, pDrive->sdspi_cmd);
        pDrive->spi_bytes[1] = 0x04;
        return;
    }

    switch (pDrive->sdspi_cmd) {
    case SDSPI_CMD0:
        pDrive->sdcard_state = SDCARD_STATE_RESET;
        break;
    case SDSPI_CMD1:
        if (pDrive->sdcard_state == SDCARD_STATE_RESET) {
            pDrive->sdcard_state = SDCARD_STATE_INITIALIZED;
        }
        else {
            pDrive->spi_bytes[1] = 0x00;
        }
        break;
    case SDSPI_CMD8:
        pDrive->sdspi_rsp_len = 6;
        break;
    case SDSPI_CMD9:
        sim_debug(CMD_MSG, &rc_sdcard_dev, DEV_NAME "%d: SDSPI: Read CSD register\n",
            rc_sdcard_info->sel_drive);
        memcpy(&pDrive->spi_bytes[3], sdcard_csd, sizeof(sdcard_csd));
        pDrive->spi_bytes[2] = SDSPI_DATA_TOKEN;
        pDrive->sdspi_rsp_len = 21;
        break;
    case SDSPI_CMD10:
        sim_debug(CMD_MSG, &rc_sdcard_dev, DEV_NAME "%d: SDSPI: Read CID register\n",
            rc_sdcard_info->sel_drive);
        memcpy(&pDrive->spi_bytes[3], sdcard_cid, sizeof(sdcard_cid));
        pDrive->spi_bytes[2] = SDSPI_DATA_TOKEN;
        pDrive->sdspi_rsp_len = 21;
        break;
    case SDSPI_CMD16:
        pDrive->sdcard_blocklen = pDrive->sdspi_arg;
        break;
    case SDSPI_CMD17:
        sdspi_single_block_read();
        break;
    case SDSPI_CMD24:
        sdspi_single_block_write();
        break;
    case SDSPI_CMD55:
        pDrive->spi_bytes[1] = 0x04;    /* Not supported, silently fail it. */
        break;
    default:
        pDrive->spi_bytes[1] = 0x04;
        sim_debug(ERROR_MSG, &rc_sdcard_dev, DEV_NAME "%d: SDSPI: Unhandled CMD=%d, arg=0x%04x, CRC=0x%02x\n",
            rc_sdcard_info->sel_drive, pDrive->sdspi_cmd, pDrive->sdspi_arg, pDrive->sdspi_crc);
        break;

    }
}

uint8 sdspi_read(void) {
    RC_SDCARD_DRIVE_INFO* pDrive = &rc_sdcard_info->drive[rc_sdcard_info->sel_drive];
    uint8 spi_byte = pDrive->spi_bytes[pDrive->spi_byte_index];

    pDrive->spi_byte_index++;
    if (pDrive->spi_byte_index > pDrive->sdspi_rsp_len)
    {
        spi_byte = 0xff;
    }

    sim_debug(SPI_MSG, &rc_sdcard_dev, DEV_NAME "%d: Tx SPI Byte: 0x%02x\n", rc_sdcard_info->sel_drive, spi_byte);

    return (bit_reverse(spi_byte));
}

static t_stat sdspi_single_block_read(void) {
    t_stat r = SCPE_OK;
    RC_SDCARD_DRIVE_INFO* pDrive = &rc_sdcard_info->drive[rc_sdcard_info->sel_drive];

    sim_debug(RD_DATA_MSG, &rc_sdcard_dev, DEV_NAME "%d: CMD17: Read single block @0x%04x, len=%d\n",
        rc_sdcard_info->sel_drive, pDrive->sdspi_arg, pDrive->sdcard_blocklen);

    if (0 == (r = sim_fseek((pDrive->uptr)->fileref, pDrive->sdspi_arg, SEEK_SET))) {
        if (sim_fread(&pDrive->spi_bytes[3], 1, pDrive->sdcard_blocklen, (pDrive->uptr)->fileref) != pDrive->sdcard_blocklen) {
            pDrive->spi_bytes[1] = 0x04;
            pDrive->sdspi_rsp_len = 2;
            return (SCPE_IOERR);
        }
    }
    else {
        pDrive->spi_bytes[1] = 0x04;
        pDrive->sdspi_rsp_len = 2;
        return (SCPE_IOERR);
    }

    pDrive->spi_bytes[0] = 0xFF;
    pDrive->spi_bytes[1] = 0x01;
    pDrive->spi_bytes[2] = SDSPI_DATA_TOKEN;

    pDrive->sdspi_rsp_len = pDrive->sdcard_blocklen + 5;
    return (r);
}

static t_stat sdspi_single_block_write(void) {
    t_stat r = SCPE_OK;
    RC_SDCARD_DRIVE_INFO* pDrive = &rc_sdcard_info->drive[rc_sdcard_info->sel_drive];

    sim_debug(WR_DATA_MSG, &rc_sdcard_dev, DEV_NAME "%d: CMD24: Write single block @0x%04x, len=%d\n",
        rc_sdcard_info->sel_drive, pDrive->sdspi_arg, pDrive->sdcard_blocklen);

    pDrive->spi_bytes[0] = 0xFF;
    pDrive->sdspi_rsp_len = 2;

    if (0 == (r = sim_fseek((pDrive->uptr)->fileref, pDrive->sdspi_arg, SEEK_SET))) {
        pDrive->sdspi_state = SDSPI_STATE_TX_DATA;
        pDrive->spi_bytes[1] = 0x01;

    }
    else {
        sim_debug(ERROR_MSG, &rc_sdcard_dev, DEV_NAME "%d: CMD24: Seek error: 0x%04x, len=%d\n",
            rc_sdcard_info->sel_drive, pDrive->sdspi_arg, pDrive->sdcard_blocklen);

        pDrive->spi_bytes[1] = 0x04;
    }

    return (r);
}
