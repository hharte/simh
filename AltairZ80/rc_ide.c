/*************************************************************************
 *                                                                       *
 * Copyright (c) 2007-2023 Howard M. Harte.                              *
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
 * SIMH Interface based on altairz80_hdsk.c, by Peter Schorn.            *
 *                                                                       *
 * Module Description:                                                   *
 *     RC2014 IDE/CF module for SIMH.  Currently only supports LBA mode. *
 *                                                                       *
 *************************************************************************/

#include "altairz80_defs.h"
#include "sim_imd.h"

/* Debug flags */
#define ERROR_MSG               (1 << 0)
#define SEEK_MSG                (1 << 1)
#define CMD_MSG                 (1 << 2)
#define RD_DATA_MSG             (1 << 3)
#define WR_DATA_MSG             (1 << 4)
#define VERBOSE_MSG             (1 << 5)
#define IDE_CTRL_MSG            (1 << 6)
#define IDE_DWR_MSG             (1 << 7)
#define IDE_DRD_MSG             (1 << 8)


#define RC_IDE_MAX_DRIVES       2       /* Maximum number of drives supported */
#define RC_IDE_MAX_SECLEN       512     /* Maximum of 512 bytes per sector */

#define RC_IDE_MAX_CYLS         0xFFFF
#define RC_IDE_MAX_HEADS        16
#define RC_IDE_MAX_SPT          256

#define DEV_NAME    "IDE"

/* Task File Register Offsets */
#define TF_DATA     0
#define TF_ERROR    1   /* Read */
#define TF_FEATURE  1   /* Write */
#define TF_SECNT    2
#define TF_SECNO    3
#define TF_CYLLO    4
#define TF_CYLHI    5
#define TF_DH       6
#define TF_STATUS   7   /* Read */
#define TF_CMD      7   /* Write */

#define RC_IDE_STATUS_BUSY         (1 << 7)
#define RC_IDE_STATUS_READY        (1 << 6)
#define RC_IDE_STATUS_WRITE_FAULT  (1 << 5)
#define RC_IDE_STATUS_SEEK_COMPL   (1 << 4)
#define RC_IDE_STATUS_DRQ          (1 << 3)
#define RC_IDE_STATUS_ERROR        (1 << 0)

#define RC_IDE_ERROR_ID_NOT_FOUND  (1 << 4)

#define RC_IDE_CMD_RECALIBRATE     0x10
#define RC_IDE_CMD_READ_SECT       0x20
#define RC_IDE_CMD_WRITE_SECT      0x30
#define RC_IDE_CMD_IDENTIFY        0xec
#define RC_IDE_SET_FEATURES        0xef

static const char* rc_ide_reg_rd_str[] = {
    "DATA    ",
    "ERROR   ",
    "SECNT   ",
    "SECNO   ",
    "CYLLO   ",
    "CYLHI   ",
    "DH      ",
    "STATUS  "
};

static const char* rc_ide_reg_wr_str[] = {
    "DATA   ",
    "FEATURE",
    "SECNT  ",
    "SECNO  ",
    "CYLLO  ",
    "CYLHI  ",
    "DH     ",
    "COMMAND"
};

typedef struct {
    UNIT *uptr;
    uint8  readonly;    /* Drive is read-only? */
    uint16 sectsize;    /* sector size */
    uint16 nsectors;    /* number of sectors/track */
    uint16 nheads;      /* number of heads */
    uint16 ncyls;       /* number of cylinders */
    uint16 cur_cyl;     /* Current cylinder */
    uint8  cur_head;    /* Current Head */
    uint8  cur_sect;    /* current starting sector of transfer */
    uint16 cur_sectsize;/* Current sector size in DH register */
    uint16 xfr_nsects;  /* Number of sectors to transfer */
    uint8 ready;        /* Is drive ready? */
    uint16 identify_data[0x100];    /* 512 bytes of drive identification data */
} RC_IDE_DRIVE_INFO;

typedef struct {
    PNP_INFO    pnp;    /* Plug and Play */
    uint8   sel_drive;  /* Currently selected drive */
    uint8   taskfile[8]; /* ATA Task File Registers */
    uint8   status_reg; /* Status Register */
    uint8   error_reg;  /* Error Register */
    uint8   retries;    /* Number of retries to attempt */
    uint8   ndrives;    /* Number of drives attached to the controller */
    uint8   sectbuf[RC_IDE_MAX_SECLEN];
    uint16  secbuf_index;
    RC_IDE_DRIVE_INFO drive[RC_IDE_MAX_DRIVES];
} RC_IDE_INFO;

static RC_IDE_INFO rc_ide_info_data = { { 0x0, 0, 0x90, 8 } };
static RC_IDE_INFO *rc_ide_info = &rc_ide_info_data;

extern uint32 PCX;
extern t_stat set_iobase(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
extern t_stat show_iobase(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
extern uint32 sim_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                               int32 (*routine)(const int32, const int32, const int32), const char* name, uint8 unmap);
extern int32 find_unit_index(UNIT *uptr);

#define UNIT_V_RC_IDE_VERBOSE    (UNIT_V_UF + 1) /* verbose mode, i.e. show error messages   */
#define UNIT_RC_IDE_VERBOSE      (1 << UNIT_V_RC_IDE_VERBOSE)
#define RC_IDE_CAPACITY          (512*4*16*512)   /* Default Disk Capacity Quantum 2020 */

static t_stat rc_ide_reset(DEVICE *rc_ide_dev);
static t_stat rc_ide_attach(UNIT *uptr, CONST char *cptr);
static t_stat rc_ide_detach(UNIT *uptr);
static t_stat rc_ide_unit_set_geometry(UNIT* uptr, int32 value, CONST char* cptr, void* desc);
static t_stat rc_ide_unit_show_geometry(FILE* st, UNIT* uptr, int32 val, CONST void* desc);
static int32 rc_idedev(const int32 port, const int32 io, const int32 data);

static uint8 RC_IDE_Read(const uint32 Addr);
static uint8 RC_IDE_Write(const uint32 Addr, uint8 cData);
static t_stat RC_IDE_doCommand(void);
static const char* rc_ide_description(DEVICE *dptr);

static UNIT rc_ide_unit[] = {
    { UDATA (NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, RC_IDE_CAPACITY) },
    { UDATA (NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, RC_IDE_CAPACITY) },
};

static REG rc_ide_reg[] = {
    { HRDATAD (TF_ERROR,    rc_ide_info_data.error_reg,            8, "Taskfile Error Register"), },
    { HRDATAD (TF_STATUS,   rc_ide_info_data.status_reg,           8, "Taskfile Status Register"), },
    { HRDATAD (TF_DATA,     rc_ide_info_data.taskfile[TF_DATA],    8, "Taskfile Data Register"), },
    { HRDATAD (TF_FEATURE,  rc_ide_info_data.taskfile[TF_FEATURE], 8, "Taskfile Precomp Register"), },
    { HRDATAD (TF_SECNT,    rc_ide_info_data.taskfile[TF_SECNT],   8, "Taskfile Sector Count Register"), },
    { HRDATAD (TF_SECNO,    rc_ide_info_data.taskfile[TF_SECNO],   8, "Taskfile Sector Number Register"), },
    { HRDATAD (TF_CYLLO,    rc_ide_info_data.taskfile[TF_CYLLO],   8, "Taskfile Cylinder Low Register"), },
    { HRDATAD (TF_CYLHI,    rc_ide_info_data.taskfile[TF_CYLHI],   8, "Taskfile Cylinder High Register"), },
    { HRDATAD (TF_DH,       rc_ide_info_data.taskfile[TF_DH],      8, "Taskfile Drive/Head Register"), },
    { HRDATAD (TF_CMD,      rc_ide_info_data.taskfile[TF_CMD],     8, "Taskfile Command Register"), },
    { NULL }
};

#define RC_IDE_NAME    "RC2014 IDE Controller"

static const char* rc_ide_description(DEVICE *dptr) {
    return RC_IDE_NAME;
}

static MTAB rc_ide_mod[] = {
    { MTAB_XTD|MTAB_VDV,    0,                      "IOBASE",   "IOBASE",
        &set_iobase, &show_iobase, NULL, "Sets disk controller I/O base address"    },
    { MTAB_XTD|MTAB_VUN|MTAB_VALR,    0,            "GEOMETRY",     "GEOMETRY",
        &rc_ide_unit_set_geometry, &rc_ide_unit_show_geometry, NULL,
        "Set disk geometry C:nnnn/H:n/S:nnn/N:nnnn" },
    { 0 }
};

/* Debug Flags */
static DEBTAB rc_ide_dt[] = {
    { "ERROR",      ERROR_MSG,      "Error messages"    },
    { "SEEK",       SEEK_MSG,       "Seek messages"     },
    { "CMD",        CMD_MSG,        "Command messages"  },
    { "READ",       RD_DATA_MSG,    "Read messages"     },
    { "WRITE",      WR_DATA_MSG,    "Write messages"    },
    { "VERBOSE",    VERBOSE_MSG,    "Verbose messages"  },
    { "CTRL",       IDE_CTRL_MSG,    "IDE Control messages" },
    { "RD_DATA",    IDE_DRD_MSG,     "IDE Data Read messages" },
    { "WR_DATA",    IDE_DWR_MSG,     "IDE Data Write messages" },
    { NULL,         0                                   }
};

DEVICE rc_ide_dev = {
    DEV_NAME, rc_ide_unit, rc_ide_reg, rc_ide_mod,
    RC_IDE_MAX_DRIVES, 10, 31, 1, RC_IDE_MAX_DRIVES, RC_IDE_MAX_DRIVES,
    NULL, NULL, &rc_ide_reset,
    NULL, &rc_ide_attach, &rc_ide_detach,
    &rc_ide_info_data, (DEV_DISABLE | DEV_DIS | DEV_DEBUG), ERROR_MSG,
    rc_ide_dt, NULL, NULL, NULL, NULL, NULL, &rc_ide_description
};

/* Reset routine */
static t_stat rc_ide_reset(DEVICE *dptr)
{
    PNP_INFO *pnp = (PNP_INFO *)dptr->ctxt;

    if(dptr->flags & DEV_DIS) { /* Disconnect I/O Ports */
        sim_map_resource(pnp->io_base, pnp->io_size, RESOURCE_TYPE_IO, &rc_idedev, "rc_idedev", TRUE);
    } else {
        /* Connect rc_ide at base address */
        if(sim_map_resource(pnp->io_base, pnp->io_size, RESOURCE_TYPE_IO, &rc_idedev, "rc_idedev", FALSE) != 0) {
            sim_printf("%s: error mapping I/O resource at 0x%04x\n", __FUNCTION__, pnp->io_base);
            return SCPE_ARG;
        }
    }

    rc_ide_info->status_reg = 0;
    rc_ide_info->error_reg = 0;
    rc_ide_info->sel_drive = 0;
    return SCPE_OK;
}


/* Attach routine */
static t_stat rc_ide_attach(UNIT *uptr, CONST char *cptr)
{
    t_stat r = SCPE_OK;
    RC_IDE_DRIVE_INFO *pDrive;
    int i = 0;

    i = find_unit_index(uptr);
    if (i == -1) {
        return (SCPE_IERR);
    }
    pDrive = &rc_ide_info->drive[i];

    /* Defaults for a 64MB Drive */
    pDrive->ready = 0;
    if (pDrive->ncyls == 0) {
        /* If geometry was not specified, default to 64MB */
        rc_ide_unit_set_geometry(uptr, 1, "C:1024/H:4/S:32/N:512", "");
    }

    r = attach_unit(uptr, cptr);    /* attach unit  */
    if ( r != SCPE_OK)              /* error?       */
        return r;

    /* Determine length of this disk */
    if(sim_fsize(uptr->fileref) != 0) {
        uptr->capac = sim_fsize(uptr->fileref);
    } else {
        uptr->capac = (pDrive->ncyls * pDrive->nsectors * pDrive->nheads * pDrive->sectsize);
    }

    pDrive->uptr = uptr;

    /* Default for new file is DSK */
    uptr->u3 = IMAGE_TYPE_DSK;

    if(uptr->capac > 0) {
        r = assignDiskType(uptr);
        if (r != SCPE_OK) {
            rc_ide_detach(uptr);
            return r;
        }
    }

    sim_debug(VERBOSE_MSG, &rc_ide_dev, DEV_NAME "%d, attached to '%s', type=DSK, len=%d\n",
        i, cptr, uptr->capac);

    pDrive->readonly = (uptr->flags & UNIT_RO) ? 1 : 0;
    rc_ide_info->error_reg = 0;
    pDrive->ready = 1;

    return SCPE_OK;
}


/* Detach routine */
static t_stat rc_ide_detach(UNIT *uptr)
{
    RC_IDE_DRIVE_INFO *pDrive;
    t_stat r;
    int32 i;

    i = find_unit_index(uptr);

    if (i == -1) {
        return (SCPE_IERR);
    }

    pDrive = &rc_ide_info->drive[i];

    pDrive->ready = 0;

    sim_debug(VERBOSE_MSG, &rc_ide_dev, "Detach " DEV_NAME "%d\n", i);

    r = detach_unit(uptr);  /* detach unit */
    if ( r != SCPE_OK)
        return r;

    return SCPE_OK;
}

/* Set geometry of the disk drive */
static t_stat rc_ide_unit_set_geometry(UNIT* uptr, int32 value, CONST char* cptr, void* desc)
{
    RC_IDE_DRIVE_INFO* pDrive;
    int32 i;
    int32 result;
    uint16 newCyls, newHeads, newSPT, newSecLen;
    uint32 total_sectors;

    i = find_unit_index(uptr);

    if (i == -1) {
        return (SCPE_IERR);
    }

    pDrive = &rc_ide_info->drive[i];

    if (cptr == NULL)
        return SCPE_ARG;

    result = sscanf(cptr, "C:%hd/H:%hd/S:%hd/N:%hd", &newCyls, &newHeads, &newSPT, &newSecLen);
    if (result != 4)
        return SCPE_ARG;

    /* Validate Cyl, Heads, Sector, Length are valid */
    if (newCyls < 1 || newCyls > RC_IDE_MAX_CYLS) {
        sim_debug(ERROR_MSG, &rc_ide_dev, DEV_NAME "%d: Number of cylinders must be 1-%d.\n",
            rc_ide_info->sel_drive, RC_IDE_MAX_CYLS);
        return SCPE_ARG;
    }
    if (newHeads < 1 || newHeads > RC_IDE_MAX_HEADS) {
        sim_debug(ERROR_MSG, &rc_ide_dev, DEV_NAME "%d: Number of heads must be 1-%d.\n",
            rc_ide_info->sel_drive, RC_IDE_MAX_HEADS);
        return SCPE_ARG;
    }
    if (newSPT < 1 || newSPT > RC_IDE_MAX_SPT) {
        sim_debug(ERROR_MSG, &rc_ide_dev, DEV_NAME "%d: Number of sectors per track must be 1-%d.\n",
            rc_ide_info->sel_drive, RC_IDE_MAX_SPT);
        return SCPE_ARG;
    }
    if (newSecLen != 512) {
        sim_debug(ERROR_MSG, &rc_ide_dev,DEV_NAME "%d: Sector length must be 512.\n",
            rc_ide_info->sel_drive);
        return SCPE_ARG;
    }

    pDrive->ncyls = newCyls;
    pDrive->nheads = newHeads;
    pDrive->nsectors = newSPT;
    pDrive->sectsize = newSecLen;

    pDrive->identify_data[0] = 0x848a;  /* CompactFlash */
    pDrive->identify_data[1] = pDrive->ncyls;
    pDrive->identify_data[3] = pDrive->nheads;
    pDrive->identify_data[6] = pDrive->nsectors;
    pDrive->identify_data[54] = pDrive->ncyls;
    pDrive->identify_data[55] = pDrive->nheads;
    pDrive->identify_data[56] = pDrive->nsectors;

    total_sectors = pDrive->ncyls * pDrive->nheads * pDrive->nsectors;
    pDrive->identify_data[60] = (uint16)total_sectors;
    pDrive->identify_data[61] = (uint16)(total_sectors >> 16);

    pDrive->identify_data[49] = 0x0200; /* LBA supported */
    return SCPE_OK;
}

/* Show geometry of the disk drive */
static t_stat rc_ide_unit_show_geometry(FILE* st, UNIT* uptr, int32 val, CONST void* desc)
{
    RC_IDE_DRIVE_INFO* pDrive;
    int32 i;

    i = find_unit_index(uptr);

    if (i == -1) {
        return (SCPE_IERR);
    }

    pDrive = &rc_ide_info->drive[i];

    fprintf(st, "C:%d/H:%d/S:%d/N:%d",
        pDrive->ncyls, pDrive->nheads, pDrive->nsectors, pDrive->sectsize);

    return SCPE_OK;
}

/* IDE Registers I/O Dispatch */
static int32 rc_idedev(const int32 port, const int32 io, const int32 data)
{
    if(io) {
        RC_IDE_Write(port, (uint8)data);
        return 0;
    } else {
        return(RC_IDE_Read(port));
    }
}


/* I/O Write to the Task File */
static uint8 RC_IDE_Write(const uint32 Addr, uint8 cData)
{
    RC_IDE_DRIVE_INFO *pDrive;
    uint8 cmd = 0;

    pDrive = &rc_ide_info->drive[rc_ide_info->sel_drive];

    switch(Addr & 0x07) {
    case TF_DATA:   /* Data FIFO */
        rc_ide_info->sectbuf[rc_ide_info->secbuf_index] = cData;
        sim_debug(VERBOSE_MSG, &rc_ide_dev,DEV_NAME ": " ADDRESS_FORMAT
            " WR TF[DATA 0x%03x]=0x%02x\n", PCX, rc_ide_info->secbuf_index, cData);
        rc_ide_info->secbuf_index++;
        if (rc_ide_info->secbuf_index == (pDrive->xfr_nsects * pDrive->sectsize)) {
            if (SCPE_OK != RC_IDE_doCommand()) {
                sim_debug(ERROR_MSG, &rc_ide_dev,DEV_NAME ": " ADDRESS_FORMAT
                    " Write command: RC_IDE_doCommand() failed.\n", PCX);
            }
        }
        break;
        case TF_DH:
            rc_ide_info->sel_drive = (cData >> 4) & 0x01;
            pDrive = &rc_ide_info->drive[rc_ide_info->sel_drive];
            /* fall through */
        case TF_FEATURE:
        case TF_SECNT:
        case TF_SECNO:
        case TF_CYLLO:
        case TF_CYLHI:
            rc_ide_info->taskfile[Addr & 0x07] = cData;
            sim_debug(VERBOSE_MSG, &rc_ide_dev,DEV_NAME ": " ADDRESS_FORMAT
                      " WR TF[%s]=0x%02x\n", PCX, rc_ide_reg_wr_str[Addr & 0x7], cData);
            break;
        case TF_CMD:
        {
            uint8 lba_mode = rc_ide_info->taskfile[TF_DH] & 0x40;
            uint32 lba;

            rc_ide_info->secbuf_index = 0;
            rc_ide_info->taskfile[TF_CMD] = cmd = cData;
            rc_ide_info->status_reg &= ~RC_IDE_STATUS_ERROR;  /* Clear error bit in status register. */
            pDrive->cur_cyl = rc_ide_info->taskfile[TF_CYLLO] | (rc_ide_info->taskfile[TF_CYLHI] << 8);
            pDrive->xfr_nsects = rc_ide_info->taskfile[TF_SECNT];

            lba = ((rc_ide_info->taskfile[TF_DH] & 0x0f) << 24) |
                   (rc_ide_info->taskfile[TF_CYLHI] << 16) |
                   (rc_ide_info->taskfile[TF_CYLLO] << 8) |
                   (rc_ide_info->taskfile[TF_SECNO]);

            sim_debug(CMD_MSG, &rc_ide_dev, DEV_NAME "%d: " ADDRESS_FORMAT
                " WR CMD=0x%02x, Mode: %s LBA=0x%08x, count=%d\n", rc_ide_info->sel_drive, PCX, cData, lba_mode ? "LBA" : "CHS", lba, rc_ide_info->taskfile[TF_SECNT]);

            /* Everything except Write commands are executed immediately. */
            if (cmd != RC_IDE_CMD_WRITE_SECT) {
                if (SCPE_OK != RC_IDE_doCommand()) {
                    sim_debug(ERROR_MSG, &rc_ide_dev,DEV_NAME ": " ADDRESS_FORMAT
                        " Non-Write command: RC_IDE_doCommand() failed.\n", PCX);
                }
            }
            else {
                /* Writes will be executed once the proper number of bytes
                   are written to the DATA FIFO. */
                rc_ide_info->secbuf_index = 0;
            }
        }
    }
    return 0;
}


/* I/O Read from the Task File */
static uint8 RC_IDE_Read(const uint32 Addr)
{
    RC_IDE_DRIVE_INFO* pDrive;
    uint8 cData = 0xFF;

    pDrive = &rc_ide_info->drive[rc_ide_info->sel_drive];

    cData = rc_ide_info->status_reg |= (pDrive->ready ? RC_IDE_STATUS_READY : 0);

    switch (Addr & 0x07) {
    case TF_DATA:   /* Data FIFO */
        cData = rc_ide_info->sectbuf[rc_ide_info->secbuf_index];
        sim_debug(VERBOSE_MSG, &rc_ide_dev,DEV_NAME ": " ADDRESS_FORMAT
            " RD TF[DATA 0x%03x]=0x%02x\n", PCX, rc_ide_info->secbuf_index, cData);
        rc_ide_info->secbuf_index++;
        if (rc_ide_info->secbuf_index > RC_IDE_MAX_SECLEN) rc_ide_info->secbuf_index = 0;
        break;
    case TF_DH:
    case TF_SECNT:
    case TF_SECNO:
    case TF_CYLLO:
    case TF_CYLHI:
    cData = rc_ide_info->taskfile[Addr & 0x07];
        sim_debug(VERBOSE_MSG, &rc_ide_dev,DEV_NAME ": " ADDRESS_FORMAT
            " RD TF[%s]=0x%02x\n", PCX, rc_ide_reg_rd_str[Addr & 0x7], cData);
        break;
    case TF_ERROR:
        cData = rc_ide_info->error_reg;
        sim_debug(VERBOSE_MSG, &rc_ide_dev,DEV_NAME ": " ADDRESS_FORMAT
            " RD TF[ERROR]=0x%02x\n", PCX, cData);
        break;
    case TF_STATUS:
        cData = rc_ide_info->status_reg;
        sim_debug(VERBOSE_MSG, &rc_ide_dev,DEV_NAME ": " ADDRESS_FORMAT
            " RD TF[STATUS]=0x%02x\n", PCX, cData);
        break;
    default:
        break;
    }

    return (cData);
}

/* Validate that Cyl, Head, Sector, Sector Length are valid for the current
 * disk drive geometry.
 */
static int RC_IDE_Validate_CHSN(RC_IDE_DRIVE_INFO* pDrive)
{
    int status = SCPE_OK;

    /* Check to make sure we're operating on a valid C/H/S/N. */
    if ((pDrive->cur_cyl >= pDrive->ncyls) ||
        (pDrive->cur_head >= pDrive->nheads) ||
        (pDrive->cur_sect >= pDrive->nsectors) ||
        (pDrive->cur_sectsize != pDrive->sectsize))
    {
        /* Set error bit in status register. */
        rc_ide_info->status_reg |= RC_IDE_STATUS_ERROR;

        /* Set ID_NOT_FOUND bit in error register. */
        rc_ide_info->error_reg |= RC_IDE_ERROR_ID_NOT_FOUND;

        sim_debug(ERROR_MSG, &rc_ide_dev,DEV_NAME "%d: " ADDRESS_FORMAT
            " ID Not Found (check disk geometry.)\n", rc_ide_info->sel_drive, PCX);

        status = SCPE_IOERR;
    }
    else {
        /* Clear ID_NOT_FOUND bit in error register. */
        rc_ide_info->error_reg &= ~RC_IDE_ERROR_ID_NOT_FOUND;
    }

    return (status);
}


/* Perform ATA Command */
static t_stat RC_IDE_doCommand(void)
{
    RC_IDE_DRIVE_INFO* pDrive;
    uint8 cmd;
    t_stat r = SCPE_OK;

    cmd = rc_ide_info->taskfile[TF_CMD];

    pDrive = &rc_ide_info->drive[rc_ide_info->sel_drive];

    pDrive->cur_head = rc_ide_info->taskfile[TF_DH] & 0x0f;
    pDrive->cur_sect = rc_ide_info->taskfile[TF_SECNO];
    rc_ide_info->error_reg &= ~RC_IDE_ERROR_ID_NOT_FOUND;

    if(pDrive->ready) {

        /* Perform command */
        switch(cmd) {
        case RC_IDE_SET_FEATURES:
            switch (rc_ide_info->taskfile[TF_FEATURE]) {
            case 0x01:
                sim_debug(CMD_MSG, &rc_ide_dev, DEV_NAME "%d: " ADDRESS_FORMAT
                    " SET_FEATURE: 8-bit transfers enabled.\n", rc_ide_info->sel_drive, PCX);
                break;
            case 0x81:
                sim_debug(CMD_MSG, &rc_ide_dev, DEV_NAME "%d: " ADDRESS_FORMAT
                    " SET_FEATURE: 8-bit transfers disabled.\n", rc_ide_info->sel_drive, PCX);
                break;
            default:
                sim_debug(ERROR_MSG, &rc_ide_dev, DEV_NAME "%d: " ADDRESS_FORMAT
                    " SET_FEATURE: Unsupported feature: 0x%02x.\n", rc_ide_info->sel_drive, PCX, rc_ide_info->taskfile[TF_FEATURE]);
                break;
            }
            break;
        case RC_IDE_CMD_IDENTIFY:
        {
            memcpy(rc_ide_info->sectbuf, pDrive->identify_data, RC_IDE_MAX_SECLEN);
            rc_ide_info->status_reg |= RC_IDE_STATUS_DRQ;
            break;
        }
        case RC_IDE_CMD_WRITE_SECT:
            /* If drive is read-only, signal a write fault. */
            if (pDrive->readonly) {
                rc_ide_info->status_reg |= RC_IDE_STATUS_ERROR;
                rc_ide_info->status_reg |= RC_IDE_STATUS_WRITE_FAULT;
                break;
            } else {
                rc_ide_info->status_reg &= ~RC_IDE_STATUS_WRITE_FAULT;
            }
            /* Fall through */
        case RC_IDE_CMD_READ_SECT:
        {
            uint32 xfr_len;
            uint32 file_offset;

            if (rc_ide_info->taskfile[TF_DH] & 0x40) { /* LBA Mode */
                file_offset = ((rc_ide_info->taskfile[TF_DH] & 0x0f) << 24) |
                               (rc_ide_info->taskfile[TF_CYLHI] << 16) |
                               (rc_ide_info->taskfile[TF_CYLLO] << 8) |
                               (rc_ide_info->taskfile[TF_SECNO]);
            }
            else { /* CHS Mode */
                /* Abort the read/write operation if C/H/S/N is not valid. */
                if (RC_IDE_Validate_CHSN(pDrive) != SCPE_OK) {
                    rc_ide_info->error_reg |= RC_IDE_ERROR_ID_NOT_FOUND;
                    rc_ide_info->status_reg |= RC_IDE_STATUS_ERROR;
                    break;
                }

                /* Calculate file offset */
                file_offset = (pDrive->cur_cyl * pDrive->nheads * pDrive->nsectors);   /* Full cylinders */
                file_offset += (pDrive->cur_head * pDrive->nsectors);   /* Add full heads */
                file_offset += (pDrive->cur_sect);  /* Add sectors for current request */
            }

            file_offset *= pDrive->sectsize;    /* Convert #sectors to byte offset */
            xfr_len = pDrive->xfr_nsects * pDrive->sectsize;

            if (0 != (r = sim_fseek((pDrive->uptr)->fileref, file_offset, SEEK_SET)))
                break;

            if (cmd == RC_IDE_CMD_READ_SECT) { /* Read */
                sim_debug(RD_DATA_MSG, &rc_ide_dev, DEV_NAME "%d: " ADDRESS_FORMAT
                    " %s SECTOR  C:%04d/H:%d/S:%04d/#:%d, offset=%5x, len=%d\n",
                    rc_ide_info->sel_drive, PCX,
                    (cmd == RC_IDE_CMD_READ_SECT) ? "READ" : "WRITE",
                    pDrive->cur_cyl, pDrive->cur_head,
                    pDrive->cur_sect, pDrive->xfr_nsects, file_offset, xfr_len);
                if (sim_fread(rc_ide_info->sectbuf, 1, xfr_len, (pDrive->uptr)->fileref) != xfr_len) {
                    r = SCPE_IOERR;
                }
            } else { /* Write */
                sim_debug(WR_DATA_MSG, &rc_ide_dev, DEV_NAME "%d: " ADDRESS_FORMAT
                    " %s SECTOR  C:%04d/H:%d/S:%04d/#:%d, offset=%5x, len=%d\n",
                    rc_ide_info->sel_drive, PCX,
                    (cmd == RC_IDE_CMD_READ_SECT) ? "READ" : "WRITE",
                    pDrive->cur_cyl, pDrive->cur_head,
                    pDrive->cur_sect, pDrive->xfr_nsects, file_offset, xfr_len);
                if (sim_fwrite(rc_ide_info->sectbuf, 1, xfr_len, (pDrive->uptr)->fileref) != xfr_len) {
                    r = SCPE_IOERR;
                }
            }
            rc_ide_info->status_reg |= RC_IDE_STATUS_DRQ;
            break;
            }
        default:
            sim_debug(ERROR_MSG, &rc_ide_dev,DEV_NAME "%d: " ADDRESS_FORMAT
                " CMD=%x Unsupported\n",
                rc_ide_info->sel_drive, PCX, cmd);
            break;
        }
    }
    else {
        rc_ide_info->status_reg |= RC_IDE_STATUS_ERROR;
    }

    return r;
}
