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
 *     Parallel FLASH module for SIMH.                                   *
 *                                                                       *
 *************************************************************************/

#include "altairz80_defs.h"
#include "sim_imd.h"

/* Debug flags */
#define ERROR_MSG   (1 << 0)
#define PROG_MSG    (1 << 1)
#define CMD_MSG     (1 << 2)
#define READ_MSG    (1 << 3)
#define WRITE_MSG   (1 << 4)
#define ERASE_MSG   (1 << 5)
#define VERBOSE_MSG (1 << 6)

#define FLASH_MAX_UNITS         4               /* Maximum number of devices supported */

#define EPROM_2708_SIZE         (1 * 1024)
#define EPROM_2716_SIZE         (2 * 1024)
#define EPROM_2732_SIZE         (4 * 1024)
#define EPROM_2764_SIZE         (8 * 1024)
#define EPROM_27128_SIZE        (16 * 1024)
#define AM28F256_SIZE           (32 * 1024)
#define I28F512_SIZE            (64 * 1024)
#define SST39F010_SIZE          (128 * 1024)
#define SST39F020_SIZE          (256 * 1024)
#define SST39F040_SIZE          (512 * 1024)
#define FLASH_MAX_SIZE          SST39F040_SIZE  /* Maximum of 512KB */

#define EPROM_2764_ID           0x9707
#define EPROM_27128_ID          0x9783
#define AM28F256_ID             0x01A1
#define I28F512_ID              0x89B8
#define SST39F010_ID            0xBFB5
#define SST39F020_ID            0xBFB6
#define SST39F040_ID            0xBFB7

#define TYPE_FLASH              0
#define TYPE_EPROM              1
#define TYPE_EEPROM             2

typedef struct {
    uint32 size;
    uint16 id;
    char* name;
    uint8 type;
} flash_info_t;

/* Supported FLASH devices, sorted from smallest to largest. */
const flash_info_t flash_devices[] = {
    { EPROM_2708_SIZE,  0,              "27C08",        TYPE_EPROM },
    { EPROM_2716_SIZE,  0,              "27C16",        TYPE_EPROM },
    { EPROM_2732_SIZE,  0,              "27C32",        TYPE_EPROM },
    { EPROM_2764_SIZE,  EPROM_2764_ID,  "TMS27C64",     TYPE_EPROM },
    { EPROM_27128_SIZE, EPROM_27128_ID, "TMS27C128",    TYPE_EPROM },
    { AM28F256_SIZE,    AM28F256_ID,    "AM28F256",     TYPE_FLASH },
    { I28F512_SIZE,     I28F512_ID,     "I28F512",      TYPE_FLASH },
    { SST39F010_SIZE,   SST39F010_ID,   "SST39F010",    TYPE_FLASH },
    { SST39F020_SIZE,   SST39F020_ID,   "SST39F020",    TYPE_FLASH },
    { SST39F040_SIZE,   SST39F040_ID,   "SST39F040",    TYPE_FLASH },
    { 0, 0, NULL }
};

#define DEV_NAME    "FLASH"

typedef struct {
    uint16  id;
    uint32  membase;
    UNIT    *uptr;
    uint8   readonly;    /* Drive is read-only? */
    uint16  sectsize;    /* sector size */
    uint8   *storage;
    uint8   cmd_mode;
    uint8   erase;
} FLASH_UNIT_INFO;

typedef struct {
    uint8   sel_unit;  /* Currently selected drive */
    FLASH_UNIT_INFO drive[FLASH_MAX_UNITS];
} FLASH_INFO;

static FLASH_INFO flash_info_data = { 0,
    .drive[0] = { SST39F010_ID, 0, 0 },
    .drive[1] = { SST39F010_ID, 0, 0 },
    .drive[2] = { SST39F010_ID, 0, 0 },
    .drive[3] = { SST39F010_ID, 0, 0 },
};
static FLASH_INFO *flash_info = &flash_info_data;

extern uint32 PCX;
extern uint32 sim_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                               int32 (*routine)(const int32, const int32, const int32), const char* name, uint8 unmap);
extern int32 find_unit_index(UNIT *uptr);

#define UNIT_V_FLASH_VERBOSE    (UNIT_V_UF + 1) /* verbose mode, i.e. show error messages   */
#define UNIT_FLASH_VERBOSE      (1 << UNIT_V_FLASH_VERBOSE)

static t_stat flash_reset(DEVICE *flash_dev);
static t_stat flash_attach(UNIT *uptr, CONST char *cptr);
static t_stat flash_detach(UNIT *uptr);
static t_stat flash_unit_set_membase(UNIT* uptr, int32 value, CONST char* cptr, void* desc);
static t_stat flash_unit_show(FILE* st, UNIT* uptr, int32 val, CONST void* desc);
static int32 flashdev(const int32 port, const int32 io, const int32 data);

static const char* flash_description(DEVICE *dptr);

static UNIT flash_unit[] = {
    { UDATA (NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, 0) },
    { UDATA (NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, 0) },
    { UDATA (NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, 0) },
    { UDATA (NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, 0) }
};

static REG flash_reg[] = {
    { HRDATAD (FLASH_ID0,    flash_info_data.drive[0].id,        16, "FLASH0 ID"), },
    { HRDATAD (FLASH_ID1,    flash_info_data.drive[1].id,        16, "FLASH1 ID"), },
    { HRDATAD (FLASH_ID2,    flash_info_data.drive[2].id,        16, "FLASH2 ID"), },
    { HRDATAD (FLASH_ID3,    flash_info_data.drive[3].id,        16, "FLASH3 ID"), },
    { NULL }
};

#define FLASH_NAME    "FLASH Memory"

static const char* flash_description(DEVICE *dptr) {
    return FLASH_NAME;
}

static MTAB flash_mod[] = {
    { MTAB_XTD|MTAB_VUN|MTAB_VALR,    0,                  "MEMBASE",     "MEMBASE",
        &flash_unit_set_membase, &flash_unit_show, NULL,
        "Set FLASH MEMBASE=xxxxx" },
    { 0 }
};

/* Debug Flags */
static DEBTAB flash_dt[] = {
    { "ERROR",      ERROR_MSG,      "Error messages"    },
    { "CMD",        CMD_MSG,        "Command messages"  },
    { "READ",       READ_MSG,       "Read messages"     },
    { "WRITE",      WRITE_MSG,      "Write messages"    },
    { "PROGRAM",    PROG_MSG,       "Programming messages" },
    { "ERASE",      ERASE_MSG,      "Erase messages" },
    { "VERBOSE",    VERBOSE_MSG,    "Verbose messages"  },
    { NULL,         0                                   }
};

DEVICE flash_dev = {
    DEV_NAME, flash_unit, flash_reg, flash_mod,
    FLASH_MAX_UNITS, 10, 31, 1, FLASH_MAX_UNITS, FLASH_MAX_UNITS,
    NULL, NULL, &flash_reset,
    NULL, &flash_attach, &flash_detach,
    &flash_info_data, (DEV_DISABLE | DEV_DIS | DEV_DEBUG), ERROR_MSG,
    flash_dt, NULL, NULL, NULL, NULL, NULL, &flash_description
};

static uint8 storage[FLASH_MAX_SIZE] = { 0 };

/* Reset routine */
static t_stat flash_reset(DEVICE *dptr)
{
    int i;

    flash_info->sel_unit = 0;

    for (i = 0; i < FLASH_MAX_UNITS; i++) {
        FLASH_UNIT_INFO* pUnit = &flash_info->drive[i];
        pUnit->cmd_mode = 0;
        pUnit->erase = 0;
    }

    if (dptr->flags & DEV_DIS) { /* Disconnect I/O Ports */
        sim_map_resource(0, FLASH_MAX_SIZE, RESOURCE_TYPE_MEMORY, &flashdev, "FLASH", TRUE);
    }
    else {
    }

    return SCPE_OK;
}


/* Attach routine */
static t_stat flash_attach(UNIT* uptr, CONST char* cptr)
{
    t_stat r = SCPE_OK;
    FLASH_UNIT_INFO* pUnit;
    int i, j = 0;

    i = find_unit_index(uptr);
    if (i == -1) {
        return (SCPE_IERR);
    }
    pUnit = &flash_info->drive[i];

    pUnit->sectsize = 512;

    r = attach_unit(uptr, cptr);    /* attach unit  */
    if (r != SCPE_OK)              /* error?       */
        return r;

    memset(storage, 0xff, FLASH_MAX_SIZE);

    /* Determine length of this disk */
    if (sim_fsize(uptr->fileref) != 0) {
        uptr->capac = sim_fsize(uptr->fileref);
    }
    else {
        uptr->capac = FLASH_MAX_SIZE;
    }

    if (uptr->capac > FLASH_MAX_SIZE) {
        sim_debug(ERROR_MSG, &flash_dev, DEV_NAME "%d, FLASH size %d too large, truncated to %d.\n", i, uptr->capac, SST39F040_SIZE);
        uptr->capac = SST39F040_SIZE;
        pUnit->id = SST39F040_ID;
    }

    pUnit->uptr = uptr;

    if (sim_fread(storage, 1, uptr->capac, uptr->fileref) != uptr->capac) {
        sim_debug(ERROR_MSG, &flash_dev, DEV_NAME "%d, Could not load file, FLASH is blank!\n", i);
        uptr->capac = FLASH_MAX_SIZE;
    }

    for (j = 0; flash_devices[j].size != 0; j++) {
        if (uptr->capac <= flash_devices[j].size) {
            uptr->capac = flash_devices[j].size;
            uptr->u4 = j;
            pUnit->id = flash_devices[j].id;
            break;
        }
    }

    sim_debug(VERBOSE_MSG, &flash_dev, DEV_NAME "%d (%s) attached to '%s', len=%d\n",
        i, flash_devices[j].name, cptr, uptr->capac);

    /* Connect FLASH at base address */
    if (sim_map_resource(pUnit->membase, uptr->capac, RESOURCE_TYPE_MEMORY, &flashdev, "FLASH", FALSE) != 0) {
        sim_printf("%s: error mapping memory resource at 0x%04x\n", __FUNCTION__, 0);
        return SCPE_ARG;
    }

    pUnit->readonly = (uptr->flags & UNIT_RO) ? 1 : 0;
    return SCPE_OK;
}


/* Detach routine */
static t_stat flash_detach(UNIT *uptr)
{
    FLASH_UNIT_INFO *pUnit;
    t_stat r;
    int32 i;

    i = find_unit_index(uptr);

    if (i == -1) {
        return (SCPE_IERR);
    }

    pUnit = &flash_info->drive[i];

    if (0 == (r = sim_fseek((pUnit->uptr)->fileref, 0, SEEK_SET))) {
        if (sim_fwrite(storage, 1, uptr->capac, uptr->fileref) != uptr->capac) {
            sim_debug(ERROR_MSG, &flash_dev, DEV_NAME "%d, Error writing FLASH.\n", i);
        }
    } else {
        sim_debug(ERROR_MSG, &flash_dev, DEV_NAME "%d, Error rewinding FLASH file.\n", i);
    }

    sim_debug(VERBOSE_MSG, &flash_dev, "Detach " DEV_NAME "%d\n", i);

    if (sim_map_resource(pUnit->membase, uptr->capac, RESOURCE_TYPE_MEMORY, &flashdev, "FLASH", TRUE) != 0) {
        sim_printf("%s: error mapping memory resource at 0x%04x\n", __FUNCTION__, 0);
        return SCPE_ARG;
    }

    r = detach_unit(uptr);  /* detach unit */
    if ( r != SCPE_OK)
        return r;

    return SCPE_OK;
}

static void flash_write(int32 Addr, uint8 data);

/* FLASH I/O Dispatch */
static int32 flashdev(const int32 Addr, const int32 write, const int32 data)
{
    FLASH_UNIT_INFO* pUnit = &flash_info->drive[flash_info->sel_unit];;

    if (write) {
        flash_write(Addr, (uint8)data);
        return 0;
    }
    else {
        if (pUnit->cmd_mode == 0x90) {
            if ((Addr & 1) == 0) {
                sim_debug(CMD_MSG, &flash_dev, DEV_NAME "Read MFG=0x%02x\n", (pUnit->id >> 8) & 0xFF);
                return ((pUnit->id >> 8) & 0xFF);
            }
            else {
                sim_debug(CMD_MSG, &flash_dev, DEV_NAME "Read DEV=0x%02x\n", pUnit->id & 0xFF);
                return (pUnit->id & 0xFF);
            }
        }
        return(storage[Addr & (FLASH_MAX_SIZE - 1)]);
    }
}

#define FLASH_CMD_MASK  (0xFFFF)

extern uint32 z180_cbar;
extern uint32 z180_cbr;
extern uint32 z180_bbr;

static void flash_write(int32 Addr, uint8 data)
{
    FLASH_UNIT_INFO* pUnit = &flash_info->drive[flash_info->sel_unit];

    sim_debug(WRITE_MSG, &flash_dev, DEV_NAME "%d, Write 0x%06x=0x%02x, mode=0x%02x, erase=%d\n",
        flash_info->sel_unit, Addr, data, pUnit->cmd_mode, pUnit->erase);

    switch (pUnit->cmd_mode) {
    case 0:
        if (((Addr & FLASH_CMD_MASK) == 0x5555) && (data == 0xaa)) {
            pUnit->cmd_mode = 1;
        }
        break;
    case 1:
        if (((Addr & FLASH_CMD_MASK) == 0x2aaa) && (data == 0x55)) {
            pUnit->cmd_mode = 2;
        }
        else {
            pUnit->cmd_mode = 0;
        }
        break;
    case 2:
        if (pUnit->erase == 0) {
            if ((Addr & FLASH_CMD_MASK) == 0x5555) {
                switch (data) {
                case 0x80:
                case 0x90:
                case 0xA0:
                    pUnit->cmd_mode = data;
                    break;
                default:
                    pUnit->cmd_mode = 0;
                    break;
                }
            }
        }
        else {  /* Chip or sector erase */
            pUnit->erase = 0;
            pUnit->cmd_mode = 0;
            switch (data) {
            case 0x30:  /* Sector erase */
                sim_debug(ERASE_MSG, &flash_dev, DEV_NAME "%d, Sector Erase: 0x%06x, cbar=0x%02x, bbr=0x%02x, cbr=0x%02x\n",
                    flash_info->sel_unit, Addr,
                    z180_cbar, z180_bbr, z180_cbr);
                memset(&storage[Addr], 0xff, 4096);
                break;
            case 0x10:  /* Chip erase */
                if (Addr == 0x5555) {
                    sim_debug(ERASE_MSG, &flash_dev, DEV_NAME "%d, Chip Erase\n",
                        flash_info->sel_unit);
                    memset(storage, 0xff, FLASH_MAX_SIZE);
                }
                else {
                    sim_debug(ERROR_MSG, &flash_dev, DEV_NAME "%d, Chip Erase with invalid address: 0x%06x\n",
                        flash_info->sel_unit, Addr);
                }
                break;
            }
        }
        break;
    case 0x80:      /* Sector Erase */
//        sim_debug(ERASE_MSG, &flash_dev, DEV_NAME "%d, Sector Erase: 0x%06x=0x%02x\n",
//            flash_info->sel_unit, Addr, data);
        pUnit->cmd_mode = 1;
        pUnit->erase = 1;
        break;
    case 0x90:      /* Software ID Entry */
        if (data == 0xF0) { /* Software ID exit */
            sim_debug(CMD_MSG, &flash_dev, DEV_NAME "Exit Command Mode.\n");
            pUnit->cmd_mode = 0;
        }
        break;
    case 0xA0:      /* Byte Program */
        sim_debug(PROG_MSG, &flash_dev, DEV_NAME "%d, Byte program: 0x%06x=0x%02x, cbar=0x%02x, bbr=0x%02x, cbr=0x%02x\n",
            flash_info->sel_unit, Addr, data,
            z180_cbar, z180_bbr, z180_cbr);
        storage[Addr] = data; //data) &=
        pUnit->cmd_mode = 0;
        break;
    default:
        sim_debug(ERROR_MSG, &flash_dev, DEV_NAME "%d, Unknown command: 0x%06x=0x%02x\n",
            flash_info->sel_unit, Addr, data);
        pUnit->cmd_mode = 0;
        break;

    }
}

/* Set geometry of the disk drive */
static t_stat flash_unit_set_membase(UNIT* uptr, int32 value, CONST char* cptr, void* desc)
{
    FLASH_UNIT_INFO* pUnit;
    int32 i;
    int32 result;

    i = find_unit_index(uptr);

    if (i == -1) {
        return (SCPE_IERR);
    }

    pUnit = &flash_info->drive[i];

    if (cptr == NULL)
        return SCPE_ARG;

    result = sscanf(cptr, "%x", &pUnit->membase);
    if (result != 1)
        return SCPE_ARG;

    return SCPE_OK;
}

/* Show geometry of the disk drive */
static t_stat flash_unit_show(FILE* st, UNIT* uptr, int32 val, CONST void* desc)
{
    FLASH_UNIT_INFO* pUnit;
    int32 i;

    i = find_unit_index(uptr);

    if (i == -1) {
        return (SCPE_IERR);
    }

    pUnit = &flash_info->drive[i];

    if (uptr->capac > 0) {
        fprintf(st, "MEM: 0x%05x-0x%05x, ID=0x%04x, %s",
            pUnit->membase, pUnit->membase + uptr->capac - 1, pUnit->id, flash_devices[uptr->u4].name);
    }
    return SCPE_OK;
}
