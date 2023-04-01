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
 *     Zilog Z180 Internal I/O.                                          *
 *                                                                       *
 *************************************************************************/

#include "altairz80_defs.h"

/* Debug flags */
#define ERROR_MSG       (1 << 0)
#define TRACE_MSG       (1 << 1)
#define PIC_MSG         (1 << 2)
#define PRT_MSG         (1 << 3)
#define CSIO_MSG        (1 << 4)
#define DMA_MSG         (1 << 5)
#define ASCI_MSG        (1 << 6)
#define ASCI_RX_MSG     (1 << 7)
#define ASCI_TX_MSG     (1 << 8)
#define ASCI_STAT_MSG   (1 << 9)
#define IRQ_MSG         (1 << 10)
#define IO_MSG          (1 << 11)
#define BBR_MSG         (1 << 12)
#define CSIO_DATA_MSG   (1 << 13)

#define Z180IO_MAX_TIMERS    3

typedef struct {
    PNP_INFO    pnp;    /* Plug and Play */
} Z180IO_INFO;

static Z180IO_INFO z180io_info_data = { { 0x0, 0, 0x0, 0x38 } };

extern t_stat set_iobase(UNIT *uptr, int32 val, CONST char *cptr, void *desc);
extern t_stat show_iobase(FILE *st, UNIT *uptr, int32 val, CONST void *desc);
extern uint32 sim_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                               int32 (*routine)(const int32, const int32, const int32), const char* name, uint8 unmap);
extern int32 sio0d(const int32 port, const int32 io, const int32 data);
extern int32 sio0s(const int32 port, const int32 io, const int32 data);
extern uint32 getClockFrequency(void);
extern int32 z180asci_io(int32 addr, int32 io, int32 data);

extern uint32 PCX;
extern uint32 z180Interrupt;
extern uint32 z180_icr;

static t_stat z180io_reset(DEVICE *z180io_dev);
static t_stat z180io_svc (UNIT *uptr);
static uint8 Z180IO_Read(const uint32 Addr);
static uint8 Z180IO_Write(const uint32 Addr, uint8 cData);
int32 z180iodev(const int32 port, const int32 io, const int32 data);
void raise_z180io_interrupt(uint8 isr_index);
static const char* z180io_description(DEVICE *dptr);


typedef void (*csio_write_cb_t) (uint8 data);
typedef uint8 (*csio_read_cb_t) (void);

t_stat csio_register_callbacks(csio_write_cb_t csio_write_cb, csio_read_cb_t csio_read_cb);

csio_write_cb_t csio_write = NULL;
csio_read_cb_t csio_read = NULL;

/* Z180IO Timer notes:
 *
 * T0, T1, T2 inputs connected to 2MHz clock on Z180IO
 * T0 IRQ connected to Slave IRQ 1
 * T1 IRQ connected to Slave IRQ 2
 * T2 IRQ connected to Slave IRQ 3
 */
typedef struct {
    uint16 count[3];    /* Current counter value for each timer. */
    uint8 mode[3];      /* Current mode of each timer. */
    uint8 bcd[3];
    uint8 rl[3];
    uint8 CTL;
} I8253_REGS;

typedef struct {
    uint8 cntla;
    uint8 cntlb;
    uint8 stat;
    uint8 tdr;
    uint8 rdr;
    uint8 ext;
} ASCI_REGS;

ASCI_REGS z180_asci[2] = { 0 };

typedef struct {
    uint8 cntr;
    uint8 trdr;
    uint8 bit_count;
} CSIO_REGS;

CSIO_REGS z180_csio = { 0x07, 0x00 };

typedef struct {
    uint16 tmdr;
    uint16 rldr;
} TIMER_REGS;

TIMER_REGS z180_timer[2] = { { 0, 0xffff }, { 0, 0xffff } };
extern uint8 GetByteDMA(const uint32 Addr);
extern void PutByteDMA(const uint32 Addr, const uint32 Value);
extern void PutBYTEExtended(register uint32 Addr, const register uint32 Value);
extern uint32 GetBYTEExtended(register uint32 Addr);

void Z180_CSIO_Receive(uint8 databit);

static uint8 z180_tcr = 0;
static uint8 z180_frc = 0xff;
static uint8 z180_dcntl = 0xf0;
static uint8 z180_dstat = 0x0;
static uint8 z180_dmode = 0x0;
static uint8 z180_il = 0x0;
extern uint8 z180_itc;
static uint8 z180_rcr = 0xc0;
static uint32 z180_sar0 = 0;
static uint32 z180_dar0 = 0;
static uint32 z180_bcr0 = 0;
static uint32 z180_mar1 = 0;
static uint32 z180_iar1 = 0;
static uint32 z180_bcr1 = 0;
extern uint32 z180_cbar;
extern uint32 z180_cbr;
extern uint32 z180_bbr;
extern uint32 z180_omcr;

static UNIT z180io_unit[] = {
    { UDATA (&z180io_svc, UNIT_FIX | UNIT_DISABLE | UNIT_DIS | UNIT_ROABLE, 0) },
    { UDATA (&z180io_svc, UNIT_FIX | UNIT_DISABLE | UNIT_DIS | UNIT_ROABLE, 0) },
    { UDATA (&z180io_svc, UNIT_FIX | UNIT_DISABLE | UNIT_DIS | UNIT_ROABLE, 0) },
    { UDATA (&z180io_svc, UNIT_FIX | UNIT_DISABLE | UNIT_DIS | UNIT_ROABLE, 0) }
};

static REG z180io_reg[] = {
    { HRDATAD (CNTLA0,      z180_asci[0].cntla,             8,  "ASCI0 Control register A"),     },
    { HRDATAD (CNTLB0,      z180_asci[0].cntlb,             8,  "ASCI0 Control register B"),     },
    { HRDATAD (STAT0,       z180_asci[0].stat,              8,  "ASCI0 Status register"),        },
    { HRDATAD (TDR0,        z180_asci[0].tdr,               8,  "ASCI0 Transmit Data register"), },
    { HRDATAD (RDR0,        z180_asci[0].rdr,               8,  "ASCI0 Receive Data register"),  },
    { HRDATAD (ASEXT0,      z180_asci[0].ext,               8,  "ASCI0 Extension Control Register"), },
    { HRDATAD (CNTLA1,      z180_asci[1].cntla,             8,  "ASCI1 Control register A"),     },
    { HRDATAD (CNTLB1,      z180_asci[1].cntlb,             8,  "ASCI1 Control register B"),     },
    { HRDATAD (STAT1,       z180_asci[1].stat,              8,  "ASCI1 Status register"),        },
    { HRDATAD (TDR1,        z180_asci[1].tdr,               8,  "ASCI1 Transmit Data register"), },
    { HRDATAD (RDR1,        z180_asci[1].rdr,               8,  "ASCI1 Receive Data register"),  },
    { HRDATAD (ASEXT1,      z180_asci[0].ext,               8,  "ASCI1 Extension Control Register"), },
    { HRDATAD (CNTR,        z180_csio.cntr,                 8,  "CSIO Control/Status Register"), },
    { HRDATAD (TRDR,        z180_csio.trdr,                 8,  "CSIO Tx/Rx Data Register"),     },
    { HRDATAD (TMDR0,       z180_timer[0].tmdr,            16,  "Timer0 Data Register"),         },
    { HRDATAD (RLDR0,       z180_timer[0].rldr,            16,  "Timer0 Reload Register"),       },
    { HRDATAD (TMDR1,       z180_timer[1].tmdr,            16,  "Timer1 Data Register"),         },
    { HRDATAD (RLDR1,       z180_timer[1].rldr,            16,  "Timer1 Reload Register"),       },
    { HRDATAD (TCR,         z180_tcr,                       8,  "Timer Control Register"),       },
    { HRDATAD (FRC,         z180_frc,                       8,  "Free Running Counter"),         },
    { HRDATAD (DSTAT,       z180_dstat,                     8,  "DMA Status Register"),          },
    { HRDATAD (DMODE,       z180_dmode,                     8,  "DMA Mode Register"),            },
    { HRDATAD (SAR0,        z180_sar0,                     20,  "DMA0 Source Address Register"), },
    { HRDATAD (DAR0,        z180_dar0,                     20,  "DMA0 Destination Address Register"), },
    { HRDATAD (BCR0,        z180_bcr0,                     16,  "DMA0 Byte Count Register"),     },
    { HRDATAD (MAR1,        z180_mar1,                     20,  "DMA1 Memory Address Register"), },
    { HRDATAD (IAR1,        z180_iar1,                     20,  "DMA1 I/O Address Register"),    },
    { HRDATAD (BCR1,        z180_bcr1,                     16,  "DMA1 Byte Count Register"),     },
    { HRDATAD (DCNTL,       z180_dcntl,                     8,  "DMA/WAIT Control Register"),    },
    { HRDATAD (IL,          z180_il,                        8,  "Interrupt Vector Low Register"),},
    { HRDATAD (ITC,         z180_itc,                       8,  "INT/TRAP Control Register"),    },
    { HRDATAD (RCR,         z180_rcr,                       8,  "Refresh Control Register"),     },
    { NULL }
};

static const char* z180io_description(DEVICE *dptr) {
    return "Zilog Z180 internal I/O";
}

static MTAB z180io_mod[] = {
    { MTAB_XTD|MTAB_VDV,    0,              "IOBASE",   "IOBASE",
        &set_iobase, &show_iobase, NULL, "Sets system support module base address" },
    { 0 }
};

/* Debug Flags */
static DEBTAB z180io_dt[] = {
    { "ERROR",  ERROR_MSG,  "Error messages"    },
    { "TRACE",  TRACE_MSG,  "Trace messages"    },
    { "PIC",    PIC_MSG,    "PIC messages"      },
    { "PRT",    PRT_MSG,    "PRT timer messages" },
    { "CSIO",   CSIO_MSG,   "CSIO messages"     },
    { "CSIO_DATA",CSIO_DATA_MSG,   "CSIO data transfer messages"     },
    { "DMA",    DMA_MSG,    "DMA messages"      },
    { "ASCI",   ASCI_MSG,   "ASCI messages"     },
    { "ASCI_RX",ASCI_RX_MSG,"ASCI Rx Data messages" },
    { "ASCI_TX",ASCI_TX_MSG,"ASCI Tx Data messages" },
    { "ASCI_STATUS",ASCI_STAT_MSG,"ASCI Status messages" },
    { "IRQ",    IRQ_MSG,    "IRQ messages"      },
    { "IO",     IO_MSG,     "I/O messages"      },
    { "BBR",    BBR_MSG,    "Bank Base Register messages" },
    { NULL,     0                               }
};

DEVICE z180io_dev = {
    "Z180IO", z180io_unit, z180io_reg, z180io_mod,
    Z180IO_MAX_TIMERS, 10, 31, 1, Z180IO_MAX_TIMERS, Z180IO_MAX_TIMERS,
    NULL, NULL, &z180io_reset,
    NULL, NULL, NULL,
    &z180io_info_data, (DEV_DISABLE | DEV_DIS | DEV_DEBUG), ERROR_MSG,
    z180io_dt, NULL, NULL, NULL, NULL, NULL, &z180io_description
};

t_stat csio_register_callbacks(csio_write_cb_t csio_write_cb, csio_read_cb_t csio_read_cb)
{
    csio_write = csio_write_cb;
    csio_read = csio_read_cb;

    return (SCPE_OK);
}

/* Reset routine */
static t_stat z180io_reset(DEVICE *dptr)
{
    PNP_INFO *pnp = (PNP_INFO *)dptr->ctxt;

    sim_cancel(&dptr->units[0]);
    sim_cancel(&dptr->units[1]);
    sim_cancel(&dptr->units[2]);
    sim_cancel(&dptr->units[3]);

    pnp->io_base = z180_icr & 0xc0;
    pnp->io_size = 0x40;

    if(dptr->flags & DEV_DIS) { /* Disconnect I/O Ports */
        sim_map_resource(pnp->io_base, pnp->io_size, RESOURCE_TYPE_IO, &z180iodev, "z180iodev", TRUE);
    } else {
        /* Connect Z180IO at base address */
        if(sim_map_resource(pnp->io_base, pnp->io_size, RESOURCE_TYPE_IO, &z180iodev, "z180iodev", FALSE) != 0) {
            sim_printf("%s: error mapping I/O resource at 0x%04x\n", __FUNCTION__, pnp->io_base);
            return SCPE_ARG;
        } else {
            z180io_unit[0].u4 = 0;
            z180io_unit[1].u4 = 1;
            z180io_unit[2].u4 = 2;
            z180io_unit[3].u4 = 3;
        }
    }

    z180_tcr = 0;
    z180_frc = 0xff;
    z180_dcntl = 0xf0;
    z180_dstat = 0x0;
    z180_dmode = 0x0;
    z180_il = 0x0;
    z180_itc = 0x01;
    z180_rcr = 0xc0;
    z180_sar0 = 0;
    z180_dar0 = 0;
    z180_bcr0 = 0;
    z180_mar1 = 0;
    z180_iar1 = 0;
    z180_bcr1 = 0;

    return SCPE_OK;
}

#define Z180_ASCI_CNTLA0    0x00
#define Z180_ASCI_CNTLA1    0x01
#define Z180_ASCI_CNTLB0    0x02
#define Z180_ASCI_CNTLB1    0x03
#define Z180_ASCI_STAT0     0x04
#define Z180_ASCI_STAT1     0x05
#define Z180_ASCI_TDR0      0x06
#define Z180_ASCI_TDR1      0x07
#define Z180_ASCI_RDR0      0x08
#define Z180_ASCI_RDR1      0x09
#define Z180_CNTR           0x0A
#define Z180_TRDR           0x0B
#define Z180_TMDR0L         0x0C
#define Z180_TMDR0H         0x0D
#define Z180_RLDR0L         0x0E
#define Z180_RLDR0H         0x0F
#define Z180_TCR            0x10
#define Z180_ASEXT0         0x12
#define Z180_ASEXT1         0x13
#define Z180_TMDR1L         0x14
#define Z180_TMDR1H         0x15
#define Z180_RLDR1L         0x16
#define Z180_RLDR1H         0x17
#define Z180_FRC            0x18
#define Z180_SAR0L          0x20
#define Z180_SAR0H          0x21
#define Z180_SAR0B          0x22
#define Z180_DAR0L          0x23
#define Z180_DAR0H          0x24
#define Z180_DAR0B          0x25
#define Z180_BCR0L          0x26
#define Z180_BCR0H          0x27
#define Z180_MAR1L          0x28
#define Z180_MAR1H          0x29
#define Z180_MAR1B          0x2a
#define Z180_IAR1L          0x2b
#define Z180_IAR1H          0x2c
#define Z180_IAR1B          0x2d
#define Z180_BCR1L          0x2e
#define Z180_BCR1H          0x2f
#define Z180_DSTAT          0x30
#define Z180_DMODE          0x31
#define Z180_DCNTL          0x32
#define Z180_IL             0x33
#define Z180_ITC            0x34
#define Z180_RCR            0x36
#define Z180_CBR            0x38
#define Z180_BBR            0x39
#define Z180_CBAR           0x3a
#define Z180_OMCR           0x3e
#define Z180_ICR            0x3f

static const char* z180io_RegToString[0x40] = {
    "CNTLA0",
    "CNTLA1",
    "CNTLB0",
    "CNTLB1",
    "STAT0",
    "STAT1",
    "TDR0",
    "TDR1",
    "RDR0",     /* 0x08 */
    "RDR1",
    "CNTR",
    "TRDR",
    "TMDR0L",
    "TMDR0H",
    "RLDR0L",
    "RLDR0H",
    "TCR",      /* 0x10 */
    "0x11",
    "ASEXT0",
    "ASEXT1",
    "TMDR1L",   /* 0x14 */
    "TMDR1H",
    "RLDR1L",
    "RLDR1H",
    "FRC",      /* 0x18 */
    "0x19",
    "0x1a",
    "0x1b",
    "0x1c",
    "0x1d",
    "0x1e",
    "0x1f",
    "SAR0L",    /* 0x20 */
    "SAR0H",
    "SAR0B",
    "DAR0L",
    "DAR0H",    /* 0x24 */
    "DAR0B",
    "BCR0L",
    "BCR0H",
    "MAR1L",    /* 0x28 */
    "MAR1H",
    "MAR1B",
    "IAR1L",
    "IAR1H",    /* 0x2c */
    "IAR1B",
    "BCR1L",
    "BCR1H",
    "DSTAT",    /* 0x30 */
    "DMODE",
    "DCNTL",
    "IL",
    "ITC",      /* 0x34 */
    "0x35",
    "RCR",
    "0x37",
    "CBR",
    "BBR",
    "CBAR",
    "0x3b",
    "0x3c",
    "0x3d",
    "OMCR",
    "ITC"
};

static struct tm currentTime;

#define CNTR_WRITE_MASK     0x77
#define Z180_CNTR_EF_BIT    7
#define Z180_CNTR_EF_MASK   (1 << Z180_CNTR_EF_BIT)
#define Z180_CNTR_EIE_BIT   6
#define Z180_CNTR_RE_BIT    5
#define Z180_CNTR_TE_BIT    4
#define Z180_CNTR_TE_MASK   (1 << Z180_CNTR_TE_BIT)
#define Z180_CNTR_SS_MASK   (0x07)

#define Z180_BIT(v, b)      ((v >> b) & 1)

#define Z180_CSIO_IRQ       (6)
#define Z180_ASCI0_IRQ      (1 << 7)
#define Z180_ASCI1_IRQ      (1 << 8)

#define DMODE_SM_MASK   0x0c
#define DMODE_SM_INC    0x00
#define DMODE_SM_DEC    0x04
#define DMODE_SM_FIX    0x08
#define DMODE_SM_IO     0x0c

#define DMODE_DM_MASK   0x30
#define DMODE_DM_INC    0x00
#define DMODE_DM_DEC    0x10
#define DMODE_DM_FIX    0x20
#define DMODE_DM_IO     0x30

#define Z180_ASCI_RDRF  (1 << 7)
#define Z180_ASCI_RIE   (1 << 3)
#define Z180_ASCI_TDRE  (1 << 1)
#define Z180_ASCI_TIE   (1 << 0)


void Z180_CSIO_Receive(uint8 databit)
{
    z180_csio.bit_count--;
    z180_csio.trdr >>= 1;
    z180_csio.trdr &= 0x7f;
    z180_csio.trdr |= (databit << 7);

    if (z180_csio.bit_count == 0) {
        z180_csio.cntr &= ~Z180_CNTR_RE_BIT;
        z180_csio.cntr |= Z180_CNTR_EF_MASK;

        sim_debug(CSIO_DATA_MSG, &z180io_dev, ADDRESS_FORMAT
                " CSIO Rx: 0x%02x.\n", PCX, z180_csio.trdr);

        if (Z180_BIT(z180_csio.cntr, Z180_CNTR_EIE_BIT)) {
//            sim_debug(CSIO_MSG, &z180io_dev, ADDRESS_FORMAT
//                " Generate CSIO Interrupt\n", PCX);
            z180Interrupt |= 1 << Z180_CSIO_IRQ;
         }
    }

}

uint8 Z180_CSIO_Transmit(void)
{
    uint8 txs_bit;
    z180_csio.bit_count--;

    txs_bit = (z180_csio.trdr >> z180_csio.bit_count) & 1;

    if (z180_csio.bit_count == 0) {
        z180_csio.cntr &= ~Z180_CNTR_TE_BIT;
        z180_csio.cntr |= Z180_CNTR_EF_MASK;

        sim_debug(CSIO_DATA_MSG, &z180io_dev, ADDRESS_FORMAT
            " CSIO Tx: 0x%02x.\n", PCX, z180_csio.trdr);

        if (Z180_BIT(z180_csio.cntr, Z180_CNTR_EIE_BIT)) {
//            sim_debug(CSIO_MSG, &z180io_dev, ADDRESS_FORMAT
//                " Generate CSIO Interrupt (Transmit)\n", PCX);
            z180_csio.bit_count = 8;
            z180Interrupt |= 1 << Z180_CSIO_IRQ;
        }
    }


    return (txs_bit);
}

int32 z180iodev(const int32 port, const int32 io, const int32 data)
{
//    sim_debug(IO_MSG, &z180io_dev, ADDRESS_FORMAT
//        " I/O %s 0x%02x (%s) = 0x%02x.\n", PCX, io ? "WR" : "RD", port, z180io_RegToString[port & 0x3f], data);
        if (io) {
            Z180IO_Write(port, data);
            return 0;
        }
        else {
            return(Z180IO_Read(port));
        }
}


static uint8 Z180IO_Read(const uint32 Addr)
{
    uint8 cData = 0xff;

    int8 sel_asci = 1;
    uint8 sel_tc = 0;
    uint8 sel_timer = 1;

    switch (Addr & 0x3f) {
    case Z180_ASCI_CNTLA0:
    case Z180_ASCI_CNTLA1:
    case Z180_ASCI_CNTLB0:
    case Z180_ASCI_CNTLB1:
    case Z180_ASCI_STAT0:
    case Z180_ASCI_STAT1:
    case Z180_ASCI_TDR0:
    case Z180_ASCI_TDR1:
    case Z180_ASCI_RDR0:
    case Z180_ASCI_RDR1:
    case Z180_ASEXT0:
    case Z180_ASEXT1:
        cData = z180asci_io(Addr, 0, 0);
        break;
    case Z180_CNTR:
        z180_csio.cntr &= ~Z180_CNTR_EF_MASK;
        z180_csio.cntr &= ~0x30;
        cData = z180_csio.cntr;
#if 0
        sim_debug(CSIO_MSG, &z180io_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (%s) = 0x%02x: EF=%d, EIE=%d, RE=%d, TE=%d, SS=%d\n", PCX, Addr, z180io_RegToString[Addr & 0x3f],
            z180_csio.cntr,
            Z180_BIT(z180_csio.cntr, Z180_CNTR_EF_BIT),
            Z180_BIT(z180_csio.cntr, Z180_CNTR_EIE_BIT),
            Z180_BIT(z180_csio.cntr, Z180_CNTR_RE_BIT),
            Z180_BIT(z180_csio.cntr, Z180_CNTR_TE_BIT),
            cData & Z180_CNTR_SS_MASK);
#endif
        break;
    case Z180_TRDR:
        if (csio_read != NULL) {
            cData = csio_read();
        }
        else {
            cData = z180_csio.trdr;
        }
        z180_csio.cntr &= ~Z180_CNTR_EF_MASK;
        sim_debug(CSIO_MSG, &z180io_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
        break;
    case Z180_TMDR0L:
        sel_timer = 0;
    case Z180_TMDR1L:
        cData = z180_timer[sel_timer].tmdr & 0xFF;
        sim_debug(PRT_MSG, &z180io_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
        break;
    case Z180_TMDR0H:
        sel_timer = 0;
    case Z180_TMDR1H:
        cData = z180_timer[sel_timer].tmdr >> 8;
        sim_debug(PRT_MSG, &z180io_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
        break;
    case Z180_RLDR0L:
        sel_timer = 0;
    case Z180_RLDR1L:
        cData = z180_timer[sel_timer].rldr & 0xFF;
        sim_debug(PRT_MSG, &z180io_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
        break;
    case Z180_RLDR0H:
        sel_timer = 0;
    case Z180_RLDR1H:
        cData = z180_timer[sel_timer].rldr >> 8; 
        sim_debug(PRT_MSG, &z180io_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
        break;
    case Z180_TCR:
        cData = z180_tcr;
        z180_tcr &= 0x3F;   /* Clear TIF - Needs more work. */
        sim_debug(PRT_MSG, &z180io_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
        break;
    case Z180_SAR0L:
        cData = z180_sar0 & 0xff;
        break;
    case Z180_SAR0H:
        cData = (z180_sar0 >> 8) & 0xff;
        break;
    case Z180_SAR0B:
        cData = (z180_sar0 >> 16) & 0xff;
        break;
    case Z180_DAR0L:
        cData = z180_dar0 & 0xff;
        break;
    case Z180_DAR0H:
        cData = (z180_dar0 >> 8) & 0xff;
        break;
    case Z180_DAR0B:
        cData = (z180_dar0 >> 16) & 0xff;
        break;
    case Z180_BCR0L:
        cData = z180_bcr0 & 0xff;
        break;
    case Z180_BCR0H:
        cData = (z180_bcr0 >> 8) & 0xff;
//        sim_debug(DMA_MSG, &z180io_dev, ADDRESS_FORMAT
//            " I/O RD 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
        break;
    case Z180_MAR1L:
        cData = z180_mar1 & 0xff;
        break;
    case Z180_MAR1H:
        cData = (z180_mar1 >> 8) & 0xff;
        break;
    case Z180_MAR1B:
        cData = (z180_mar1 >> 16) & 0xff;
        break;
    case Z180_IAR1L:
        cData = z180_iar1 & 0xff;
        break;
    case Z180_IAR1H:
        cData = (z180_iar1 >> 8) & 0xff;
        break;
    case Z180_IAR1B:
        cData = (z180_iar1 >> 16) & 0xff;
        break;
    case Z180_BCR1L:
        cData = z180_bcr1 & 0xff;
        break;
    case Z180_BCR1H:
        cData = (z180_bcr1 >> 8) & 0xff;
        break;
    case Z180_DSTAT:
        cData = z180_dstat;
//        sim_debug(DMA_MSG, &z180io_dev, ADDRESS_FORMAT
//            " I/O RD 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
        break;
    case Z180_DMODE:
        cData = z180_dmode;
//        sim_debug(DMA_MSG, &z180io_dev, ADDRESS_FORMAT
//            " I/O RD 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
        break;
    case Z180_DCNTL:
        cData = z180_dcntl;
//        sim_debug(DMA_MSG, &z180io_dev, ADDRESS_FORMAT
//            " I/O RD 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
        break;
    case Z180_IL:
        cData = z180_il;
        sim_debug(IRQ_MSG, &z180io_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
        break;
    case Z180_ITC:
        cData = z180_itc;
        sim_debug(IRQ_MSG, &z180io_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
        break;
    case Z180_RCR:
        cData = z180_rcr;
        break;
    case Z180_CBR: /* CBR */
        cData = z180_cbr;
        break;
    case Z180_BBR: /* BBR */
        cData = z180_bbr;
        break;
    case Z180_CBAR: /* CBAR */
        cData = z180_cbar;
        break;
    case Z180_OMCR: /* OMCR */
        cData = z180_omcr;
        break;
    case Z180_ICR: /* ICR */
        cData = z180_icr;
        sim_printf(ADDRESS_FORMAT "Read ICR=0x%02x\n", PCX, z180_icr);
        break;
    default:
        break;
    }
    return (cData);

}

static void generate_z180io_interrupt(void);


const char* asci_ss_str_lut[8] = {
    "/1",
    "/2",
    "/4",
    "/8",
    "/16",
    "/32",
    "/64",
    "External Clock",
};

static uint8 Z180IO_Write(const uint32 Addr, uint8 cData)
{

    uint8 sel_asci = 1;
    uint8 sel_tc = 0;
    uint8 sel_timer = 1;

    switch(Addr & 0x3f) {
        case Z180_ASCI_CNTLA0:
        case Z180_ASCI_CNTLA1:
        case Z180_ASCI_CNTLB0:
        case Z180_ASCI_CNTLB1:
        case Z180_ASCI_STAT0:
        case Z180_ASCI_STAT1:
        case Z180_ASCI_TDR0:
        case Z180_ASCI_TDR1:
        case Z180_ASCI_RDR0:
        case Z180_ASCI_RDR1:
        case Z180_ASEXT0:
        case Z180_ASEXT1:
            z180asci_io(Addr, 1, cData);
            break;
        case Z180_CNTR:
            if ((cData ^ (1 << Z180_CNTR_RE_BIT)) && (cData & (1 << Z180_CNTR_RE_BIT))) {
//                sim_debug(CSIO_MSG, &z180io_dev, ADDRESS_FORMAT
//                    " CSIO: RE changed to 1.\n", PCX);
                z180_csio.bit_count = 8;
            }
            z180_csio.cntr &= ~(CNTR_WRITE_MASK | Z180_CNTR_EF_MASK);   /* Clear EF bit */
            z180_csio.cntr |= (cData & CNTR_WRITE_MASK);
#if 1
            sim_debug(CSIO_MSG, &z180io_dev, ADDRESS_FORMAT
                " I/O WR 0x%02x (%s) = 0x%02x: EF=%d, EIE=%d, RE=%d, TE=%d, SS=%d\n", PCX, Addr, z180io_RegToString[Addr & 0x3f],
                z180_csio.cntr,
                Z180_BIT(z180_csio.cntr, Z180_CNTR_EF_BIT),
                Z180_BIT(z180_csio.cntr, Z180_CNTR_EIE_BIT),
                Z180_BIT(z180_csio.cntr, Z180_CNTR_RE_BIT),
                Z180_BIT(z180_csio.cntr, Z180_CNTR_TE_BIT),
                cData & Z180_CNTR_SS_MASK);
#endif
            break;
        case Z180_TRDR:
            z180_csio.trdr = cData;
            if (csio_write != NULL) {
                csio_write(cData);
            }

            z180_csio.cntr &= ~Z180_CNTR_EF_MASK;
            sim_debug(CSIO_MSG, &z180io_dev, ADDRESS_FORMAT
                " I/O WR 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
            if (z180_csio.cntr & Z180_CNTR_TE_MASK) {
                z180_csio.cntr |= Z180_CNTR_EF_MASK;
            }
            break;
        case Z180_TMDR0L:
            sel_timer = 0;
        case Z180_TMDR1L:
            z180_timer[sel_timer].tmdr &= 0xFF00;
            z180_timer[sel_timer].tmdr |= cData;
            sim_debug(PRT_MSG, &z180io_dev, ADDRESS_FORMAT
                " I/O WR 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
            break;
        case Z180_TMDR0H:
            sel_timer = 0;
        case Z180_TMDR1H:
            z180_timer[sel_timer].tmdr &= 0x00FF;
            z180_timer[sel_timer].tmdr |= (cData << 8);
            sim_debug(PRT_MSG, &z180io_dev, ADDRESS_FORMAT
                " I/O WR 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
            break;
        case Z180_RLDR0L:
            sel_timer = 0;
        case Z180_RLDR1L:
            z180_timer[sel_timer].rldr &= 0xFF00;
            z180_timer[sel_timer].rldr |= cData;
            sim_debug(PRT_MSG, &z180io_dev, ADDRESS_FORMAT
                " I/O WR 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
            break;
        case Z180_RLDR0H:
            sel_timer = 0;
        case Z180_RLDR1H:
            z180_timer[sel_timer].rldr &= 0x00FF;
            z180_timer[sel_timer].rldr |= (cData << 8);
            sim_debug(PRT_MSG, &z180io_dev, ADDRESS_FORMAT
                " I/O WR 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
            break;
        case Z180_TCR:
            z180_tcr = cData;
            sim_debug(PRT_MSG, &z180io_dev, ADDRESS_FORMAT
                " I/O WR 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
            if (z180_tcr & 0x01) {
                uint32 clockFrequency = getClockFrequency();

                sim_activate_after(&z180io_unit[0], 1250);
            }
            if (z180_tcr & 0x02) {
                sim_activate_after(&z180io_unit[1], 3);
            }
            break;
        case Z180_SAR0L:
            z180_sar0 &= 0x0fff00;
            z180_sar0 |= cData;
            break;
        case Z180_SAR0H:
            z180_sar0 &= 0x0f00ff;
            z180_sar0 |= cData << 8;
            break;
        case Z180_SAR0B:
            z180_sar0 &= 0x00ffff;
            z180_sar0 |= cData << 16;
            break;
        case Z180_DAR0L:
            z180_dar0 &= 0x0fff00;
            z180_dar0 |= cData;
            break;
        case Z180_DAR0H:
            z180_dar0 &= 0x0f00ff;
            z180_dar0 |= cData << 8;
            break;
        case Z180_DAR0B:
            z180_dar0 &= 0x00ffff;
            z180_dar0 |= cData << 16;
            break;
        case Z180_BCR0L:
            z180_bcr0 &= 0xff00;
            z180_bcr0 |= cData;
            break;
        case Z180_BCR0H:
            z180_bcr0 &= 0x00ff;
            z180_bcr0 |= cData << 8;
//            sim_debug(DMA_MSG, &z180io_dev, ADDRESS_FORMAT
//                " I/O WR 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
            break;
        case Z180_MAR1L:
            z180_mar1 &= 0x0fff00;
            z180_mar1 |= cData;
            break;
        case Z180_MAR1H:
            z180_mar1 &= 0x0f00ff;
            z180_mar1 |= cData << 8;
            break;
        case Z180_MAR1B:
            z180_mar1 &= 0x00ffff;
            z180_mar1 |= cData << 16;
            break;
        case Z180_IAR1L:
            z180_iar1 &= 0x0fff00;
            z180_iar1 |= cData;
            break;
        case Z180_IAR1H:
            z180_iar1 &= 0x0f00ff;
            z180_iar1 |= cData << 8;
            break;
        case Z180_IAR1B:
            z180_iar1 &= 0x00ffff;
            z180_iar1 |= cData << 16;
            break;
        case Z180_BCR1L:
            z180_bcr1 &= 0xff00;
            z180_bcr1 |= cData;
            break;
        case Z180_BCR1H:
            z180_bcr1 &= 0x00ff;
            z180_bcr1 |= cData << 8;
            break;
        case Z180_DSTAT:
            z180_dstat = cData;
            switch (z180_dstat) {
            case 0x44:
                sim_debug(DMA_MSG, &z180io_dev, ADDRESS_FORMAT
                    " DMA DSTAT=0x%02x, DMODE=0x%02x, DCNTL=0x%02x, Src: 0x%05x, Dest: 0x%05x, Len=%d\n", PCX,
                    z180_dstat, z180_dmode, z180_dcntl,
                    z180_sar0, z180_dar0, z180_bcr0);
                while (z180_bcr0 > 0) {
                    PutBYTEExtended(z180_dar0, GetBYTEExtended(z180_sar0));
                    switch (z180_dmode & DMODE_SM_MASK) {
                    case DMODE_SM_INC:
                        z180_sar0++;
                        break;
                    case DMODE_SM_DEC:
                        z180_sar0--;
                        break;
                    case DMODE_SM_FIX:
                        break;
                    case DMODE_SM_IO:
                        sim_printf(ADDRESS_FORMAT " Error: DMA0 from I/O not supported.\n", PCX);
                        break;
                    }

                    switch (z180_dmode & DMODE_DM_MASK) {
                    case DMODE_DM_INC:
                        z180_dar0++;
                        break;
                    case DMODE_DM_DEC:
                        z180_dar0--;
                        break;
                    case DMODE_DM_FIX:
                        break;
                    case DMODE_DM_IO:
                        sim_printf(ADDRESS_FORMAT " Error: DMA0 to I/O not supported.\n", PCX);
                        break;
                    }
                    z180_bcr0--;

                }
            case 0x00:  /* Stop DMA */
                break;
            default:
                sim_debug(DMA_MSG, &z180io_dev, ADDRESS_FORMAT
                    " Unhandled DMA operation: I/O WR 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
            }
            break;
        case Z180_DMODE:
            z180_dmode = cData;
//            sim_debug(DMA_MSG, &z180io_dev, ADDRESS_FORMAT
//                " I/O WR 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
            break;
        case Z180_DCNTL:
            z180_dcntl = cData;
//            sim_debug(DMA_MSG, &z180io_dev, ADDRESS_FORMAT
//                " I/O WR 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
            break;
        case Z180_IL:
            z180_il = cData;
            sim_debug(IRQ_MSG, &z180io_dev, ADDRESS_FORMAT
                " I/O WR 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
            break;
        case Z180_ITC:
            if (cData ^ z180_itc) {
                sim_debug(IRQ_MSG, &z180io_dev, ADDRESS_FORMAT
                    " I/O WR 0x%02x (%s) = 0x%02x.\n", PCX, Addr, z180io_RegToString[Addr & 0x3f], cData);
            }
            z180_itc  = (z180_itc & 0x80) ? (cData & 0x87) : (cData & 0x07);
            z180_itc |= 1;   // Fixme: Always keep internal interrupts active.
            break;
        case Z180_RCR:
            z180_rcr = cData;
            break;
        case Z180_CBR: /* CBR */
            z180_cbr = cData;
            break;
        case Z180_BBR: /* BBR */
            z180_bbr = cData;
            sim_debug(BBR_MSG, &z180io_dev, ADDRESS_FORMAT
                " ROM CBAR=0x%02x, BBR=0x%02x, CBR=0x%02x: Bank Base = 0x%04x-0x%04x, phys: 0x%05x\n",
                PCX,
                z180_cbar,
                z180_bbr,
                z180_cbr,
                (z180_cbar & 0x0f) << 12,
                ((z180_cbar & 0xf0) << 8) - 1,
                (z180_bbr << 12));
            break;
        case Z180_CBAR: /* CBAR */
            z180_cbar = cData;
            break;
        case Z180_OMCR: /* OMCR */
            z180_omcr = cData | 0x1f;
            break;
        case Z180_ICR: /* ICR */
        {
            uint32 z180_io_base = (z180_icr & 0xC0);
            uint32 z180_mmu_base = z180_io_base | 0x38;

            if ((z180_icr ^ cData) & 0xc0) {
                /* Unmap Z180 registers from current I/O base address */
                sim_map_resource(z180_io_base, 0x40, RESOURCE_TYPE_IO, &z180iodev, "z180iodev", TRUE);

                /* Set new Z180 I/O base address */
                z180_icr = cData | 0x1f;

                z180_io_base = (z180_icr & 0xC0);
                z180_mmu_base = z180_io_base | 0x38;
                /* Map Z180 MMU to new I/O base address */
                sim_map_resource(z180_io_base, 0x40, RESOURCE_TYPE_IO, &z180iodev, "z180iodev", FALSE);
            }
            break;
        }
        default:
            break;
    }

    return(0);
}

void raise_z180io_interrupt(uint8 isr_index)
{
    generate_z180io_interrupt();
}
extern void cpu_raise_interrupt(uint32 irq);

static void generate_z180io_interrupt(void)
{

}

/* Unit service routine */
/* Unit 0-2 = Timer0-2, Unit3=ISR queue */
static t_stat z180io_svc (UNIT *uptr)
{
    switch(uptr->u4) {
        case 0:
        case 1:
//            z180_timer[uptr->u4].tmdr--;
//            if (z180_timer[uptr->u4].tmdr == 0) {
//                sim_debug(IRQ_MSG, &z180io_dev, ADDRESS_FORMAT
//                    " Timer%d = 0, generate interrupt.\n", PCX, uptr->u4);

                /* Set TCR TIF */
                z180_tcr |= (uptr->u4 == 0) ? 0x40 : 0x80;
                /* Generate interrupt */
                z180Interrupt |= 1 << (uptr->u4 + 2);
                z180_timer[uptr->u4].tmdr = z180_timer[uptr->u4].rldr;
//            }
            sim_activate_after(uptr, 1250);
            break;
        case 2:
        {
            int32 key;

            key = sim_poll_kbd();

            if (key & SCPE_KFLAG) {
//                sim_printf("Key pressed = 0x%x\n", key);
                z180_asci[0].rdr = key & 0xFF;
                z180_asci[0].stat |= Z180_ASCI_RDRF;
                if (z180_asci[0].stat & Z180_ASCI_RIE)
                {
                    z180Interrupt |= Z180_ASCI0_IRQ;
                }
            }
            sim_activate_after(uptr, 2000);
            break;
        }
        case 3:
            break;
    }

    return SCPE_OK;
}

