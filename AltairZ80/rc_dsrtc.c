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
 *     RC2014 DS1302 RTC module for SIMH.                                *
 *                                                                       *
 *************************************************************************/

#include "altairz80_defs.h"
#include "sim_imd.h"

/* Debug flags */
#define ERROR_MSG           (1 << 0)
#define CLK_MSG             (1 << 1)
#define RD_MSG              (1 << 2)
#define WR_MSG              (1 << 3)
#define VERBOSE_MSG         (1 << 4)

#define RC_DSRTC_MAX_UNITS  1       /* Maximum number of drives supported */

#define DEV_NAME            "DSRTC"

#define DSRTC_SEC           0
#define DSRTC_MIN           1
#define DSRTC_HR            2
#define DSRTC_DAY           3
#define DSRTC_MON           4
#define DSRTC_WDAY          5
#define DSRTC_YEAR          6
#define DSRTC_WP            7
#define DSRTC_TCS           8
#define DSRTC_NVRAM         32

#define DSRTC_DOUT_BIT      (1 << 7)
#define DSRTC_CLK_BIT       (1 << 6)
#define DSRTC_WE_BIT        (1 << 5)
#define DSRTC_CE_BIT        (1 << 4)

#define RTC_BCD(x)          ((((x) / 10) << 4) | ((x) % 10))

typedef struct {
    PNP_INFO    pnp;        /* Plug and Play */
    uint8   latch;
    uint16  bit;            /* RTC bit counter */
    uint8   addr;           /* RTC Address */
    uint8   wdata;          /* Write data byte */
    uint8   rdata;          /* Read data byte */
    uint8   din_bit;        /* Serial data in bit */
    uint8   reg[64];        /* RTC registers */
} RC_DSRTC_INFO;

static RC_DSRTC_INFO dsrtc_info_data = { { 0x0, 0, 0x0c, 1 } };
static RC_DSRTC_INFO *dsrtc_info = &dsrtc_info_data;

extern uint32 PCX;
extern t_stat set_iobase(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
extern t_stat show_iobase(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
extern uint32 sim_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                               int32 (*routine)(const int32, const int32, const int32), const char* name, uint8 unmap);

#define UNIT_V_RC_DSRTC_VERBOSE    (UNIT_V_UF + 1) /* verbose mode, i.e. show error messages   */
#define UNIT_RC_DSRTC_VERBOSE      (1 << UNIT_V_RC_DSRTC_VERBOSE)
#define RC_DSRTC_CAPACITY          (DSRTC_NVRAM)   /* NVRAM Capacity */

static t_stat rc_dsrtc_reset(DEVICE *rc_dsrtc_dev);
static t_stat rc_dsrtc_attach(UNIT *uptr, CONST char *cptr);
static t_stat rc_dsrtc_detach(UNIT *uptr);
static int32 rc_dsrtcdev(const int32 port, const int32 io, const int32 data);

static uint8 RC_DSRTC_Read(const uint32 Addr);
static uint8 RC_DSRTC_Write(const uint32 Addr, uint8 cData);

static const char* rc_dsrtc_description(DEVICE *dptr);

static UNIT rc_dsrtc_unit[] = {
    { UDATA (NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, RC_DSRTC_CAPACITY) },
    { UDATA (NULL, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, RC_DSRTC_CAPACITY) },
};

static REG rc_dsrtc_reg[] = {
    { HRDATAD (RTC_SEC,     dsrtc_info_data.reg[DSRTC_SEC],       8, "RTC Second Register"), },
    { HRDATAD (RTC_MIN,     dsrtc_info_data.reg[DSRTC_MIN],       8, "RTC Minute Register"), },
    { HRDATAD (RTC_HR,      dsrtc_info_data.reg[DSRTC_HR],        8, "RTC Hour Register"), },
    { HRDATAD (RTC_DAY,     dsrtc_info_data.reg[DSRTC_DAY],       8, "RTC Day Register"), },
    { HRDATAD (RTC_MON,     dsrtc_info_data.reg[DSRTC_MON],       8, "RTC Month Register"), },
    { HRDATAD (RTC_WDAY,    dsrtc_info_data.reg[DSRTC_WDAY],      8, "RTC Weekday Register"), },
    { HRDATAD (RTC_YEAR,    dsrtc_info_data.reg[DSRTC_YEAR],      8, "RTC Year Register"), },
    { HRDATAD (RTC_WP,      dsrtc_info_data.reg[DSRTC_WP],        8, "RTC Write Protect Register"), },
    { HRDATAD (RTC_TCS,     dsrtc_info_data.reg[DSRTC_TCS],       8, "RTC Trickle Charge Register"), },
    { BRDATAD (NVRAM,      &dsrtc_info_data.reg[DSRTC_NVRAM], 16, 8, 32, "RTC NVRAM"), },
    { NULL }
};

#define RC_DSRTC_NAME    "RC2014 DS1302 Real Time Clock"

static const char* rc_dsrtc_description(DEVICE *dptr) {
    return RC_DSRTC_NAME;
}

static MTAB rc_dsrtc_mod[] = {
    { MTAB_XTD|MTAB_VDV, 0, "IOBASE",   "IOBASE",
        &set_iobase, &show_iobase, NULL, "Sets RTC I/O base address"    },
    { 0 }
};

/* Debug Flags */
static DEBTAB rc_dsrtc_dt[] = {
    { "ERROR",      ERROR_MSG,      "Error messages"    },
    { "CLOCK",      CLK_MSG,        "Serial Clock messages"  },
    { "READ",       RD_MSG,         "Read messages"     },
    { "WRITE",      WR_MSG,         "Write messages"    },
    { "VERBOSE",    VERBOSE_MSG,    "Verbose messages"  },
    { NULL,         0                                   }
};

DEVICE rc_dsrtc_dev = {
    DEV_NAME, rc_dsrtc_unit, rc_dsrtc_reg, rc_dsrtc_mod,
    RC_DSRTC_MAX_UNITS, 10, 31, 1, RC_DSRTC_MAX_UNITS, RC_DSRTC_MAX_UNITS,
    NULL, NULL, &rc_dsrtc_reset,
    NULL, &rc_dsrtc_attach, &rc_dsrtc_detach,
    &dsrtc_info_data, (DEV_DISABLE | DEV_DIS | DEV_DEBUG), ERROR_MSG,
    rc_dsrtc_dt, NULL, NULL, NULL, NULL, NULL, &rc_dsrtc_description
};

/* Reset routine */
static t_stat rc_dsrtc_reset(DEVICE *dptr)
{
    PNP_INFO *pnp = (PNP_INFO *)dptr->ctxt;

    if(dptr->flags & DEV_DIS) { /* Disconnect I/O Ports */
        sim_map_resource(pnp->io_base, pnp->io_size, RESOURCE_TYPE_IO, &rc_dsrtcdev, "rc_dsrtcdev", TRUE);
    } else {
        /* Connect RTC at base address */
        if(sim_map_resource(pnp->io_base, pnp->io_size, RESOURCE_TYPE_IO, &rc_dsrtcdev, "rc_dsrtcdev", FALSE) != 0) {
            sim_printf("%s: error mapping I/O resource at 0x%04x\n", __FUNCTION__, pnp->io_base);
            return SCPE_ARG;
        }
    }

    return SCPE_OK;
}

/* Attach routine */
static t_stat rc_dsrtc_attach(UNIT *uptr, CONST char *cptr)
{
    t_stat r = SCPE_OK;
    int j = 0;

    r = attach_unit(uptr, cptr);    /* attach unit  */
    if (r != SCPE_OK)              /* error?       */
        return r;

    memset(&dsrtc_info->reg[DSRTC_NVRAM], 0, DSRTC_NVRAM);

    /* Determine length of this disk */
    if (sim_fsize(uptr->fileref) != 0) {
        uptr->capac = sim_fsize(uptr->fileref);
    }
    else {
        uptr->capac = DSRTC_NVRAM;
    }

    if (uptr->capac > DSRTC_NVRAM) {
        sim_debug(ERROR_MSG, &rc_dsrtc_dev, DEV_NAME " NVRAM size %d too large, truncated to %d.\n", uptr->capac, DSRTC_NVRAM);
    }

    if (sim_fread(&dsrtc_info->reg[DSRTC_NVRAM], 1, uptr->capac, uptr->fileref) != uptr->capac) {
        sim_debug(ERROR_MSG, &rc_dsrtc_dev, DEV_NAME " Could not load file, NVRAM is blank!\n");
        uptr->capac = DSRTC_NVRAM;
    }

    sim_debug(VERBOSE_MSG, &rc_dsrtc_dev, DEV_NAME " attached to '%s', len=%d\n",
        cptr, uptr->capac);

    return SCPE_OK;
}


/* Detach routine */
static t_stat rc_dsrtc_detach(UNIT *uptr)
{
    t_stat r;

    if (0 == (r = sim_fseek((uptr)->fileref, 0, SEEK_SET))) {
        if (sim_fwrite(&dsrtc_info->reg[DSRTC_NVRAM], 1, uptr->capac, uptr->fileref) != uptr->capac) {
            sim_debug(ERROR_MSG, &rc_dsrtc_dev, DEV_NAME " Error writing NVRAM.\n");
        }
    }
    else {
        sim_debug(ERROR_MSG, &rc_dsrtc_dev, DEV_NAME " Error rewinding NVRAM file.\n");
    }

    sim_debug(VERBOSE_MSG, &rc_dsrtc_dev, " Detach " DEV_NAME "\n");


    r = detach_unit(uptr);  /* detach unit */
    if (r != SCPE_OK)
        return r;

    return SCPE_OK;
}

/* DSRTC Register I/O Dispatch */
static int32 rc_dsrtcdev(const int32 port, const int32 io, const int32 data)
{
    if(io) {
        RC_DSRTC_Write(port, (uint8)data);
        return 0;
    } else {
        return(RC_DSRTC_Read(port));
    }
}

/* I/O Write to the RTC */
static uint8 RC_DSRTC_Write(const uint32 Addr, uint8 cData)
{
    sim_debug(WR_MSG, &rc_dsrtc_dev,DEV_NAME ": " ADDRESS_FORMAT
        " WR DSRTC=0x%02x: RTC_DOUT=%d, RTC_WE#=%d, RTC_CE#=%d\n", PCX, cData, cData >> 7, (cData >> 5) & 1, (cData >> 4) & 1);

    if ((cData ^ dsrtc_info->latch) & DSRTC_CLK_BIT) {
        if (cData & DSRTC_CLK_BIT) {    /* Clock is high */
            sim_debug(CLK_MSG, &rc_dsrtc_dev, DEV_NAME ": " ADDRESS_FORMAT
                " Clock toggled %s, bit %d=%d\n", PCX, cData & DSRTC_CLK_BIT ? "HIGH" : "LOW", dsrtc_info->bit, cData >> 7);
            if (cData & DSRTC_CE_BIT) {
                if (dsrtc_info->bit < 8) {
                    dsrtc_info->addr >>= 1;
                    dsrtc_info->addr |= (cData & DSRTC_DOUT_BIT);

                    if (dsrtc_info->bit == 7) {
                        sim_debug(CLK_MSG, &rc_dsrtc_dev, DEV_NAME ": " ADDRESS_FORMAT
                            " Clock Address 0x%02x\n", PCX, dsrtc_info->addr);
                        dsrtc_info->wdata = 0;
                        if (dsrtc_info->addr == 0xbf) { /* Clock Burst */
                            dsrtc_info->addr &= 0x01;
                        }
                        if ((dsrtc_info->addr >> 1) < 7) { /* Accessing RTC, get time from system. */
                            time_t now;
                            struct tm currentTime = { 0 };

                            sim_get_time(&now);
                            currentTime = *localtime(&now);

                            dsrtc_info->reg[DSRTC_SEC]  = RTC_BCD(currentTime.tm_sec);
                            dsrtc_info->reg[DSRTC_MIN]  = RTC_BCD(currentTime.tm_min);
                            dsrtc_info->reg[DSRTC_HR]   = RTC_BCD(currentTime.tm_hour);
                            dsrtc_info->reg[DSRTC_DAY]  = RTC_BCD(currentTime.tm_mday);
                            dsrtc_info->reg[DSRTC_MON]  = RTC_BCD(currentTime.tm_mon + 1);
                            dsrtc_info->reg[DSRTC_WDAY] = currentTime.tm_wday + 1;
                            dsrtc_info->reg[DSRTC_YEAR] = RTC_BCD(currentTime.tm_year % 100);
                        }
                    }
                } else {
                    if (!(cData & DSRTC_WE_BIT)) {  /* Write */
                        dsrtc_info->wdata >>= 1;
                        dsrtc_info->wdata |= (cData & DSRTC_DOUT_BIT);
                        if ((dsrtc_info->bit & 0x7) == 0x7) {
                            uint8 ds_addr = (dsrtc_info->addr >> 1) & 0x3F;

                            sim_debug(CLK_MSG, &rc_dsrtc_dev, DEV_NAME ": " ADDRESS_FORMAT
                                " Write Address 0x%02x=0x%02x\n", PCX, ds_addr, dsrtc_info->wdata);
                            dsrtc_info->reg[ds_addr] = dsrtc_info->wdata;
                            dsrtc_info->addr += 2;
                        }
                    }
                }

                dsrtc_info->bit++;
                
            }
        } else { /* Clock transitioned low */
            if (cData & DSRTC_WE_BIT) {  /* Read */
                uint8 ds_addr = (dsrtc_info->addr >> 1) & 0x3F;
                if ((dsrtc_info->bit & 0x7) == 0) {
                    dsrtc_info->rdata = dsrtc_info->reg[ds_addr];
                    sim_debug(CLK_MSG, &rc_dsrtc_dev, DEV_NAME ": " ADDRESS_FORMAT
                        " Read Address 0x%02x=0x%02x\n", PCX, ds_addr, dsrtc_info->rdata);
                    dsrtc_info->addr += 2;
                }
                dsrtc_info->din_bit = (dsrtc_info->rdata >> (dsrtc_info->bit & 0x7) & 1);
            }
        }
    }

    if (!(cData & DSRTC_CE_BIT)) {
        dsrtc_info->bit = 0;
        dsrtc_info->addr = 0;
    }

    dsrtc_info->latch = cData;
    return 0;
}


/* I/O Read from the RTC */
static uint8 RC_DSRTC_Read(const uint32 Addr)
{
    uint8 cData = dsrtc_info->din_bit;

    sim_debug(RD_MSG, &rc_dsrtc_dev, DEV_NAME ": " ADDRESS_FORMAT
        " RD DSRTC=0x%02x (bit=%d)\n", PCX, cData, dsrtc_info->bit & 7);

    return (cData);
}
