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
 *     Zilog Z180 ASCI serial ports.                                     *
 *                                                                       *
 *************************************************************************/

#include <stdio.h>

#include "altairz80_defs.h"
#include "z180asci.h"
#include "sim_tmxr.h"

#define Z180ASCI_NAME       "Zilog Z180 ASCI"
#define Z180ASCI_SNAME      "Z180ASCI"
#define Z180ASCI_MAX_UNITS  2

#define Z180ASCI_WAIT       500           /* Service Wait Interval */

#define Z180_ASCI0_IRQ      (1 << 7)
#define Z180_ASCI1_IRQ      (1 << 8)

#define Z180_ASCI_RDRF      (1 << 7)
#define Z180_ASCI_OVRN      (1 << 6)        /* Overrun                      */
#define Z180_ASCI_PE        (1 << 5)        /* Parity Error                 */
#define Z180_ASCI_FE        (1 << 4)        /* Framing Error                */
#define Z180_ASCI_RIE       (1 << 3)
#define Z180_ASCI_DCD       (1 << 2)        /* Data Carrier Detect          */
#define Z180_ASCI_TDRE      (1 << 1)        /* Transmit Data Register Empty */
#define Z180_ASCI_TIE       (1 << 0)

#define Z180_ASCI_CNTLA_RE  (1 << 6)        /* CNTLA Receive Enable */

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
#define Z180_ASEXT0         0x12
#define Z180_ASEXT1         0x13

#define Z180_ASCI_CNTLB_CTS (1 << 5)       /* Clear to Send                */

#define Z180ASCI_IRQ        0x80           /* Interrupt Request            */
#define Z180ASCI_RESET      0x03           /* Reset                        */
#define Z180ASCI_CLK1       0x00           /* Divide Clock by 1            */
#define Z180ASCI_CLK16      0x01           /* Divide Clock by 16           */
#define Z180ASCI_CLK64      0x02           /* Divide Clock by 64           */
#define Z180ASCI_72E        0x00           /* 7-2-E                        */
#define Z180ASCI_72O        0x04           /* 7-2-O                        */
#define Z180ASCI_71E        0x08           /* 7-1-E                        */
#define Z180ASCI_71O        0x0C           /* 7-1-O                        */
#define Z180ASCI_82N        0x10           /* 8-2-N                        */
#define Z180ASCI_81N        0x14           /* 8-1-N                        */
#define Z180ASCI_81E        0x18           /* 8-1-E                        */
#define Z180ASCI_81O        0x1C           /* 8-1-O                        */
#define Z180ASCI_FMTMSK     0x1c           /* Length, Parity, Stop Mask    */
#define Z180ASCI_RTSLTID    0x00           /* RTS Low, Xmit Int Disabled   */
#define Z180ASCI_RTSLTIE    0x20           /* RTS Low, Xmit Int Enabled    */
#define Z180ASCI_RTSHTID    0x40           /* RTS High, Xmit Int Disabled  */
#define Z180ASCI_RTSHTBR    0x60           /* RTS High, Xmit Break         */
#define Z180ASCI_RTSMSK     0x60           /* RTS Bit Mask                 */
#define Z180ASCI_RIE        0x80           /* Receive Int Enabled          */

#define Z180ASCI_BAUD       9600           /* Default baud rate            */

/* Debug flags */
#define STATUS_MSG          (1 << 0)
#define ERROR_MSG           (1 << 1)
#define VERBOSE_MSG         (1 << 2)
#define ASCI_MSG            (1 << 3)
#define ASCI_RX_MSG         (1 << 4)
#define ASCI_TX_MSG         (1 << 5)
#define ASCI_STAT_MSG       (1 << 6)
#define IRQ_MSG             (1 << 7)
#define IO_MSG              (1 << 8)

/* IO Read/Write */
#define IO_RD               0x00            /* IO Read  */
#define IO_WR               0x01            /* IO Write */

typedef struct {
    uint8 cntla;
    uint8 cntlb;
    uint8 stat;
    uint8 tdr;
    uint8 rdr;
    uint8 ext;
} ASCI_REGS;

typedef struct {
    TMXR *tmxr;          /* TMXR pointer     */
    TMLN* tmln;          /* TMLN pointer     */
    int32 conn;          /* Connected Status */
    int32 baud;          /* Baud rate        */
    ASCI_REGS regs;
    int32 rts;           /* RTS Status       */
    int32 ctb;           /* Control Buffer   */
    uint8 txp;           /* Transmit Pending */
} Z180ASCI_CTX;

extern uint32 getClockFrequency(void);

extern uint32 PCX;
extern uint32 z180Interrupt;

asci_tx_cb_t asci_tx[2] = { NULL, NULL };
asci_rx_cb_t asci_rx[2] = { NULL, NULL };

static const char* z180asci_description(DEVICE *dptr);
static t_stat z180asci_svc(UNIT *uptr);
static t_stat z180asci_reset(DEVICE *dptr);
static t_stat z180asci_attach(UNIT *uptr, CONST char *cptr);
static t_stat z180asci_detach(UNIT *uptr);
static t_stat z180asci_set_baud(UNIT *uptr, int32 value, const char *cptr, void *desc);
static t_stat z180asci_show_baud(FILE *st, UNIT *uptr, int32 value, const void *desc);
static t_stat z180asci_show_cb(FILE* st, UNIT* uptr, int32 value, const void* desc);
static t_stat z180asci_config_line(UNIT *uptr);
static t_stat z180asci_config_rts(UNIT *dptr, char rts);
int32 z180asci_io(int32 addr, int32 io, int32 data);
static uint8 Z180ASCI_Read(uint32 Addr);
static uint8 Z180ASCI_Write(uint32 Addr, uint8 cData);

extern uint32 vectorInterrupt;          /* Vector Interrupt bits */
extern uint8 dataBus[MAX_INT_VECTORS];  /* Data bus value        */

/* Debug Flags */
static DEBTAB z180asci_dt[] = {
    { "STATUS",         STATUS_MSG,     "Status messages"       },
    { "ERROR",          ERROR_MSG,      "Error messages"        },
    { "VERBOSE",        VERBOSE_MSG,    "Verbose messages"      },
    { "ASCI",           ASCI_MSG,       "ASCI messages"         },
    { "ASCI_RX",        ASCI_RX_MSG,    "ASCI Rx Data messages" },
    { "ASCI_TX",        ASCI_TX_MSG,    "ASCI Tx Data messages" },
    { "ASCI_STATUS",    ASCI_STAT_MSG,  "ASCI Status messages"  },
    { "IRQ",            IRQ_MSG,        "IRQ messages"          },
    { "IO",             IO_MSG,         "I/O messages"          },
    { NULL,             0                                       }
};

/* Terminal multiplexer library descriptors */
static TMLN z180asci0_tmln[] = {                /* line descriptors */
    { 0 }
};

static TMLN z180asci1_tmln[] = {                /* line descriptors */
    { 0 }
};

static TMXR z180asci0_tmxr = {                  /* multiplexer descriptor */
    1,                                          /* number of terminal lines */
    0,                                          /* listening port (reserved) */
    0,                                          /* master socket  (reserved) */
    z180asci0_tmln,                             /* line descriptor array */
    NULL,                                       /* line connection order */
    NULL                                        /* multiplexer device (derived internally) */
};

static TMXR z180asci1_tmxr = {                  /* multiplexer descriptor */
    1,                                          /* number of terminal lines */
    0,                                          /* listening port (reserved) */
    0,                                          /* master socket  (reserved) */
    z180asci1_tmln,                             /* line descriptor array */
    NULL,                                       /* line connection order */
    NULL                                        /* multiplexer device (derived internally) */
};

#define UNIT_V_Z180ASCI_CONSOLE  (UNIT_V_UF + 0)     /* Port checks console for input */
#define UNIT_Z180ASCI_CONSOLE    (1 << UNIT_V_Z180ASCI_CONSOLE)
#define UNIT_V_Z180ASCI_BAUD     (UNIT_V_UF + 1)     /* Baud Rate */
#define UNIT_Z180ASCI_BAUD       (1 << UNIT_V_Z180ASCI_BAUD)
#define UNIT_V_Z180ASCI_DTR      (UNIT_V_UF + 2)     /* DTR follows RTS               */
#define UNIT_Z180ASCI_DTR        (1 << UNIT_V_Z180ASCI_DTR)
#define UNIT_V_Z180ASCI_DCD      (UNIT_V_UF + 3)     /* Force DCD active low          */
#define UNIT_Z180ASCI_DCD        (1 << UNIT_V_Z180ASCI_DCD)
#define UNIT_V_Z180ASCI_CB       (UNIT_V_UF + 4)     /* Callbacks */
#define UNIT_Z180ASCI_CB         (1 << UNIT_V_Z180ASCI_CB)

static MTAB z180asci_mod[] = {
    { UNIT_Z180ASCI_CONSOLE,   UNIT_Z180ASCI_CONSOLE, "CONSOLE",   "CONSOLE",   NULL, NULL, NULL,
        "Port checks for console input" },
    { UNIT_Z180ASCI_CONSOLE,0,                  "NOCONSOLE", "NOCONSOLE", NULL, NULL, NULL,
        "Port does not check for console input" },
    { UNIT_Z180ASCI_BAUD,   0,   "BAUD",  "BAUD",  &z180asci_set_baud, &z180asci_show_baud,
        NULL, "Set baud rate (default=9600)" },
    { UNIT_Z180ASCI_DTR,       UNIT_Z180ASCI_DTR,     "DTR",       "DTR",       NULL, NULL, NULL,
        "DTR follows RTS" },
    { UNIT_Z180ASCI_DTR,    0,                  "NODTR",     "NODTR",     NULL, NULL, NULL,
        "DTR does not follow RTS (default)" },
    { UNIT_Z180ASCI_DCD,       UNIT_Z180ASCI_DCD,     "DCD",       "DCD",       NULL, NULL, NULL,
        "Force DCD active low" },
    { UNIT_Z180ASCI_DCD,    0,                  "NODCD",     "NODCD",     NULL, NULL, NULL,
        "DCD follows status line (default)" },
    { UNIT_Z180ASCI_CB,     0, "CB",  NULL,  NULL, &z180asci_show_cb,
        NULL, "Show Tx/Rx Callbacks" },
    { 0 }
};

static Z180ASCI_CTX z180asci_ctx[2] = {
    { &z180asci0_tmxr, z180asci0_tmln, 0, Z180ASCI_BAUD },
    { &z180asci1_tmxr, z180asci1_tmln, 0, Z180ASCI_BAUD },
};

static UNIT z180asci_unit[] = {
        { UDATA (&z180asci_svc, UNIT_ATTABLE | UNIT_DISABLE | UNIT_Z180ASCI_CONSOLE, 0), Z180ASCI_WAIT },
        { UDATA (&z180asci_svc, UNIT_ATTABLE | UNIT_DISABLE, 0), Z180ASCI_WAIT },
};

static REG z180asci_reg[] = {
    { HRDATAD (CNTLA0,      z180asci_ctx[0].regs.cntla, 8,  "ASCI0 Control register A"),            },
    { HRDATAD (CNTLB0,      z180asci_ctx[0].regs.cntlb, 8,  "ASCI0 Control register B"),            },
    { HRDATAD (STAT0,       z180asci_ctx[0].regs.stat,  8,  "ASCI0 Status register"),               },
    { HRDATAD (TDR0,        z180asci_ctx[0].regs.tdr,   8,  "ASCI0 Transmit Data register"),        },
    { HRDATAD (RDR0,        z180asci_ctx[0].regs.rdr,   8,  "ASCI0 Receive Data register"),         },
    { HRDATAD (ASEXT0,      z180asci_ctx[0].regs.ext,   8,  "ASCI0 Extension Control Register"),    },
    { HRDATAD (CNTLA1,      z180asci_ctx[1].regs.cntla, 8,  "ASCI1 Control register A"),            },
    { HRDATAD (CNTLB1,      z180asci_ctx[1].regs.cntlb, 8,  "ASCI1 Control register B"),            },
    { HRDATAD (STAT1,       z180asci_ctx[1].regs.stat,  8,  "ASCI1 Status register"),               },
    { HRDATAD (TDR1,        z180asci_ctx[1].regs.tdr,   8,  "ASCI1 Transmit Data register"),        },
    { HRDATAD (RDR1,        z180asci_ctx[1].regs.rdr,   8,  "ASCI1 Receive Data register"),         },
    { HRDATAD (ASEXT1,      z180asci_ctx[1].regs.ext,   8,  "ASCI1 Extension Control Register"),    },
    { NULL }
};

DEVICE z180asci_dev = {
    Z180ASCI_SNAME,     /* name */
    z180asci_unit,      /* unit */
    z180asci_reg,       /* registers */
    z180asci_mod,       /* modifiers */
    Z180ASCI_MAX_UNITS, /* # units */
    10,                 /* address radix */
    31,                 /* address width */
    1,                  /* address increment */
    8,                  /* data radix */
    8,                  /* data width */
    NULL,               /* examine routine */
    NULL,               /* deposit routine */
    &z180asci_reset,    /* reset routine */
    NULL,               /* boot routine */
    &z180asci_attach,   /* attach routine */
    &z180asci_detach,   /* detach routine */
    &z180asci_ctx,      /* context */
    (DEV_DISABLE | DEV_DIS | DEV_DEBUG | DEV_MUX),  /* flags */
    0,                  /* debug control */
    z180asci_dt,        /* debug flags */
    NULL,               /* mem size routine */
    NULL,               /* logical name */
    NULL,               /* help */
    NULL,               /* attach help */
    NULL,               /* context for help */
    &z180asci_description  /* description */
};

const char* asci_ss_str_lut2[8] = {
    "/1",
    "/2",
    "/4",
    "/8",
    "/16",
    "/32",
    "/64",
    "External Clock",
};

static const char* z180asci_description(DEVICE *dptr)
{
    return Z180ASCI_NAME;
}

t_stat asci_register_callbacks(uint8 port, asci_tx_cb_t asci_tx_cb, asci_rx_cb_t asci_rx_cb)
{
    if (port >= Z180ASCI_MAX_UNITS) {
        return (SCPE_ARG);
    }

    sim_debug(VERBOSE_MSG, &z180asci_dev, "Register callbacks, port %d.\n", port);

    asci_tx[port] = asci_tx_cb;
    asci_rx[port] = asci_rx_cb;

    z180asci_unit[port].flags &= ~UNIT_ATTABLE;

    return (SCPE_OK);
}

static t_stat z180asci_reset(DEVICE *dptr)
{
    int32 c;
    uint8 port;

    if (dptr == NULL) return SCPE_IERR;

    for (port = 0; port < Z180ASCI_MAX_UNITS; port++) {
        Z180ASCI_CTX *ctx = &z180asci_ctx[port];

        /* Set DEVICE for this UNIT */
        z180asci_unit[port].dptr = dptr;
        dptr->units[port].u4 = port;
        dptr->units[port].up7 = ctx;

        c = getClockFrequency() / 5;
        dptr->units[port].wait = (c && c < 1000) ? c : 1000;

        /* Enable TMXR modem control passthru */
// Fixme: hharte: telnet connection does not work with modem control passthrough enabled. */
//        tmxr_set_modem_control_passthru(ctx->tmxr);

        /* Reset status registers */
        ctx->regs.stat = Z180_ASCI_TDRE;

        if (dptr->units[port].flags & UNIT_ATT) {
            z180asci_config_rts(&dptr->units[port], 1);    /* disable RTS */
        }

        if (!(dptr->flags & DEV_DIS)) {
            sim_activate(&dptr->units[port], dptr->units[port].wait);
        }
        else {
            sim_cancel(&dptr->units[port]);
        }

        sim_debug(STATUS_MSG, dptr, "reset port %d.\n", port);
    }
    return SCPE_OK;
}


static t_stat z180asci_svc(UNIT *uptr)
{
    Z180ASCI_CTX* ctx;
    ASCI_REGS* pRegs;
    int32 c, s;
    uint8 cntlb, stat, newstat;
    t_stat r;

    if (uptr == NULL) return SCPE_IERR;

    ctx = (Z180ASCI_CTX*)uptr->up7;
    pRegs = &ctx->regs;

    /* Check for new incoming connection */
    if (uptr->flags & UNIT_ATT) {
        if (tmxr_poll_conn(ctx->tmxr) >= 0) {      /* poll connection */

            ctx->conn = 1;          /* set connected   */

            sim_debug_unit(STATUS_MSG, uptr, "new connection.\n");
        }
    }

    /* Update incoming modem status bits */
    if (uptr->flags & UNIT_ATT) {
        tmxr_set_get_modem_bits(ctx->tmln, 0, 0, &s);
        stat = newstat = pRegs->stat;
        cntlb = pRegs->cntlb;

        pRegs->cntlb &= ~Z180_ASCI_CNTLB_CTS;
        pRegs->cntlb |= (s & TMXR_MDM_CTS) ? 0 : Z180_ASCI_CNTLB_CTS;     /* Active Low */
        if ((cntlb ^ pRegs->cntlb) & Z180_ASCI_CNTLB_CTS) {
            sim_debug_unit(STATUS_MSG, uptr, "CTS state changed to %s.\n", (pRegs->cntlb & Z180_ASCI_CNTLB_CTS) ? "LOW" : "HIGH");
        }

        newstat &= Z180_ASCI_DCD;
        newstat |= ((s & TMXR_MDM_DCD) || (uptr->flags & UNIT_Z180ASCI_DCD)) ? 0 : Z180_ASCI_DCD;     /* Active Low */
        if ((stat ^ newstat) & Z180_ASCI_DCD) {
            sim_debug_unit(STATUS_MSG, uptr, "DCD state changed to %s.\n", (pRegs->stat & Z180_ASCI_DCD) ? "LOW" : "HIGH");
            if (newstat & Z180_ASCI_DCD) {
                pRegs->stat |= Z180_ASCI_DCD;
            }
        }

        /* Enable receiver if DCD is active low */
        ctx->tmln->rcve = !(pRegs->stat & Z180_ASCI_DCD);
    }

    /* TX data */
    if (ctx->txp) {
        if (asci_tx[uptr->u4] != NULL) { /* Callback registered */
            r = asci_tx[uptr->u4](pRegs->tdr);
            ctx->txp = 0;                             /* Reset TX Pending */
        }
        else if (uptr->flags & UNIT_ATT) {
            if ((pRegs->cntlb & Z180_ASCI_CNTLB_CTS)) {    /* Active low */
                r = tmxr_putc_ln(ctx->tmln, pRegs->tdr);
                ctx->txp = 0;                         /* Reset TX Pending */
            } else {
                r = SCPE_STALL;
            }
        }
        else {
            r = sim_putchar(pRegs->tdr);
            ctx->txp = 0;                             /* Reset TX Pending */
        }

        if (r == SCPE_LOST) {
            ctx->conn = 0;          /* Connection was lost */
            sim_debug_unit(STATUS_MSG, uptr, "lost connection.\n");
        }
    }

    /* Update TDRE if not set and no character pending */
    if (!ctx->txp && !(pRegs->stat & Z180_ASCI_TDRE)) {
        uint8 new_tdre = 0;
        if (uptr->flags & UNIT_ATT) {
            tmxr_poll_tx(ctx->tmxr);
            new_tdre = (tmxr_txdone_ln(ctx->tmln) && ctx->conn) ? Z180_ASCI_TDRE : 0;
        } else {
            new_tdre = Z180_ASCI_TDRE;
        }
        pRegs->stat |= new_tdre;
        if (pRegs->stat & Z180_ASCI_TIE) {
            z180Interrupt |= (uptr->u4 == 0) ? Z180_ASCI0_IRQ : Z180_ASCI1_IRQ;
        }
    }

    /* Check for Data if RX buffer empty */
    if (!(pRegs->stat & Z180_ASCI_RDRF)) {
        if (asci_tx[uptr->u4] != NULL) { /* Callback registered */
            c = asci_rx[uptr->u4]();
        } else if (uptr->flags & UNIT_ATT) {
            tmxr_poll_rx(ctx->tmxr);

            c = tmxr_getc_ln(ctx->tmln);
        } else if (uptr->flags & UNIT_Z180ASCI_CONSOLE) {
            c = sim_poll_kbd();
        } else {
            c = 0;
        }

        if (c & (TMXR_VALID | SCPE_KFLAG)) {
            pRegs->rdr = c & 0xff;
            pRegs->stat |= Z180_ASCI_RDRF;
            pRegs->stat &= ~(Z180_ASCI_OVRN | Z180_ASCI_PE | Z180_ASCI_FE);
            if (pRegs->stat & Z180_ASCI_RIE) {
                z180Interrupt |= (uptr->u4 == 0) ? Z180_ASCI0_IRQ : Z180_ASCI1_IRQ;
            }
        }
    }

    /* Don't let TMXR clobber our wait time */
    uptr->wait = Z180ASCI_WAIT;

    sim_activate_abs(uptr, uptr->wait);

    return SCPE_OK;
}


/* Attach routine */
static t_stat z180asci_attach(UNIT *uptr, CONST char *cptr)
{
    Z180ASCI_CTX *ctx = (Z180ASCI_CTX*)uptr->up7;
    t_stat r;

    sim_debug_unit(VERBOSE_MSG, uptr, "attach (%s).\n", cptr);

    if ((r = tmxr_attach(ctx->tmxr, uptr, cptr)) == SCPE_OK) {

        if (ctx->tmln->serport) {
            r = z180asci_config_rts(uptr, ctx->rts);    /* update RTS */
        }

        ctx->tmln->rcve = 1;
    }

    return r;
}


/* Detach routine */
static t_stat z180asci_detach(UNIT *uptr)
{
    Z180ASCI_CTX* ctx;

    if (uptr->dptr == NULL) {
        return SCPE_IERR;
    }

    sim_debug(VERBOSE_MSG, uptr->dptr, "detach.\n");

    if (uptr->flags & UNIT_ATT) {
        ctx = (Z180ASCI_CTX*)uptr->up7;

        sim_cancel(uptr);

        return (tmxr_detach(ctx->tmxr, uptr));
    }

    return SCPE_UNATT;
}

static t_stat z180asci_set_baud(UNIT *uptr, int32 value, const char *cptr, void *desc)
{
    Z180ASCI_CTX *ctx;
    int32 baud;
    t_stat r = SCPE_ARG;

    ctx = (Z180ASCI_CTX *) uptr->up7;

    if (!(uptr->flags & UNIT_ATT)) {
        return SCPE_UNATT;
    }

    if (cptr != NULL) {
        if (sscanf(cptr, "%d", &baud)) {
            switch (baud) {
                case 110:
                case 150:
                case 300:
                case 1200:
                case 1800:
                case 2400:
                case 4800:
                case 9600:
                case 19200:
                case 38400:
                case 57600:
                case 115200:
                    ctx->baud = baud;
                    r = z180asci_config_line(uptr);

                    return r;

                default:
                    break;
            }
        }
    }

    return r;
}

static t_stat z180asci_show_baud(FILE *st, UNIT *uptr, int32 value, const void *desc)
{
    Z180ASCI_CTX* ctx = (Z180ASCI_CTX *) uptr->up7;

    fprintf(st, "Baud rate: %d", ctx->baud);

    return SCPE_OK;
}

static t_stat z180asci_show_cb(FILE* st, UNIT* uptr, int32 value, const void* desc)
{
    Z180ASCI_CTX* ctx = (Z180ASCI_CTX*)uptr->up7;

    if (!(uptr->flags & UNIT_ATT)) {
        fprintf(st, "TX Callback %sRegistered, RX Callabck %sRegistered", asci_tx[uptr->u4] == NULL ? "Not " : "", asci_rx[uptr->u4] == NULL ? "Not " : "");
    }

    return SCPE_OK;
}

static t_stat z180asci_config_line(UNIT *uptr)
{
    Z180ASCI_CTX* ctx = (Z180ASCI_CTX*)uptr->up7;
    char config[20];
    const char *fmt;
    t_stat r = SCPE_IERR;

    if (ctx != NULL) {
        switch (ctx->ctb & Z180ASCI_FMTMSK) {
            case Z180ASCI_72E:
                fmt = "7E2";
                break;
            case Z180ASCI_72O:
                fmt = "7O2";
                break;
            case Z180ASCI_71E:
                fmt = "7E1";
                break;
            case Z180ASCI_71O:
                fmt = "7O1";
                break;
            case Z180ASCI_82N:
                fmt = "8N2";
                break;
            case Z180ASCI_81E:
                fmt = "8E1";
                break;
            case Z180ASCI_81O:
                fmt = "8O1";
                break;
            case Z180ASCI_81N:
            default:
                fmt = "8N1";
                break;
        }

        sprintf(config, "%d-%s", ctx->baud, fmt);

        r = tmxr_set_config_line(ctx->tmln, config);

        sim_debug(STATUS_MSG, uptr->dptr, "port configuration set to '%s'.\n", config);

        /*
        ** AltairZ80 and TMXR refuse to want to play together
        ** nicely when the CLOCK register is set to anything
        ** other than 0.
        **
        ** This work-around is for those of us that may wish
        ** to run irrelevant, old software, that use TMXR and
        ** rely on some semblance of timing (Remote CP/M, BYE,
        ** RBBS, PCGET/PUT, Xmodem, MEX, Modem7, or most
        ** other communications software), on contemprary
        ** hardware.
        **
        ** Serial ports are self-limiting and sockets will run
        ** at the clocked CPU speed.
        */
        ctx->tmln->txbps = 0;   /* Get TMXR's timing out of our way */
        ctx->tmln->rxbps = 0;   /* Get TMXR's timing out of our way */
    }

    return r;
}

/*
** RTS is active low
** 0 = RTS active
** 1 = RTS inactive
*/
static t_stat z180asci_config_rts(UNIT *uptr, char rts)
{
    Z180ASCI_CTX* ctx = (Z180ASCI_CTX*)uptr->up7;
    ASCI_REGS* pRegs = &ctx->regs;
    t_stat r = SCPE_OK;
    int32 s;

    if (uptr->flags & UNIT_ATT) {
        /* RTS Control */
        s = TMXR_MDM_RTS;
        if (uptr->flags & UNIT_Z180ASCI_DTR) {
            s |= TMXR_MDM_DTR;
        }

        if (!rts) {
            r = tmxr_set_get_modem_bits(ctx->tmln, s, 0, NULL);
            if (ctx->rts) {
                sim_debug(STATUS_MSG, uptr->dptr, "RTS state changed to HIGH.\n");
            }
        } else {
            r = tmxr_set_get_modem_bits(ctx->tmln, 0, s, NULL);
            if (!ctx->rts) {
                sim_debug(STATUS_MSG, uptr->dptr, "RTS state changed to LOW.\n");
            }
        }
    }

    ctx->rts = rts;    /* Active low */

    return r;
}

int32 z180asci_io(int32 addr, int32 io, int32 data)
{
    if (io) {
        Z180ASCI_Write(addr, data);
        return 0;
    }
    else {
        return(Z180ASCI_Read(addr));
    }
}

static uint8 Z180ASCI_Read(uint32 Addr)
{
    uint8 cData = 0xff;

    int8 sel_asci = 1;
    uint8 sel_tc = 0;
    uint8 sel_timer = 1;

    Addr &= 0x1f;
    if (Addr > Z180_ASEXT1) return (0xff);

    switch (Addr) {
    case Z180_ASCI_CNTLA0:
        sel_asci = 0;    /* intentional falltrough */
    case Z180_ASCI_CNTLA1:
        cData = z180asci_ctx[sel_asci].regs.cntla;
        sim_debug(ASCI_MSG, &z180asci_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (CNTLA%d) = 0x%02x.\n", PCX, Addr, sel_asci, cData);
        break;
    case Z180_ASCI_CNTLB0:
        sel_asci = 0;    /* intentional falltrough */
    case Z180_ASCI_CNTLB1:
        cData = z180asci_ctx[sel_asci].regs.cntlb;
        sim_debug(ASCI_MSG, &z180asci_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (CNTLB%d) = 0x%02x.\n", PCX, Addr, sel_asci, cData);
        break;
    case Z180_ASCI_STAT0:
        sel_asci = 0;    /* intentional falltrough */
    case Z180_ASCI_STAT1:
        cData = z180asci_ctx[sel_asci].regs.stat;
        sim_debug(ASCI_STAT_MSG, &z180asci_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (STAT%d) = 0x%02x.\n", PCX, Addr, sel_asci, cData);
        z180asci_ctx[sel_asci].regs.stat &= ~(Z180_ASCI_DCD);   /* Clear DCD# on read */
        /* Enable receiver if DCD is active low */
        z180asci_ctx[sel_asci].tmln->rcve = !(z180asci_ctx[sel_asci].regs.stat & Z180_ASCI_DCD);
        break;
    case Z180_ASCI_TDR0:
        sel_asci = 0;    /* intentional falltrough */
    case Z180_ASCI_TDR1:
        cData = z180asci_ctx[sel_asci].regs.tdr;
        sim_debug(ASCI_TX_MSG, &z180asci_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (TDR%d) = 0x%02x.\n", PCX, Addr, sel_asci, cData);
        break;
    case Z180_ASCI_RDR0:
        sel_asci = 0;    /* intentional falltrough */
    case Z180_ASCI_RDR1:
        cData = z180asci_ctx[sel_asci].regs.rdr;
        z180asci_ctx[sel_asci].regs.stat &= ~(Z180_ASCI_RDRF | Z180_ASCI_FE | Z180_ASCI_OVRN | Z180_ASCI_PE);
        sim_debug(ASCI_RX_MSG, &z180asci_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (RDR%d) = 0x%02x.\n", PCX, Addr, sel_asci, cData);
        break;
    case Z180_ASEXT0:
        sel_asci = 0;    /* intentional falltrough */
    case Z180_ASEXT1:
        cData = z180asci_ctx[sel_asci].regs.ext;
        sim_debug(ASCI_MSG, &z180asci_dev, ADDRESS_FORMAT
            " I/O RD 0x%02x (ASEXT%d) = 0x%02x.\n", PCX, Addr, sel_asci, cData);
        break;
    default:
        break;
    }

    return (cData);
}

static uint8 Z180ASCI_Write(uint32 Addr, uint8 cData)
{

    uint8 sel_asci = 1;
    uint8 sel_tc = 0;
    uint8 sel_timer = 1;

    Addr &= 0x1f;

    switch (Addr) {
    case Z180_ASCI_CNTLA0:
        sel_asci = 0;    /* intentional falltrough */
    case Z180_ASCI_CNTLA1:
        z180asci_ctx[sel_asci].regs.cntla = cData;
        sim_debug(ASCI_MSG, &z180asci_dev, ADDRESS_FORMAT
            " I/O WR 0x%02x (CNTLA%d) = 0x%02x: MPE=%d, RE=%d, TE=%d, RTS#=%d, MPBR/EFR=%d, MODE=0x%x=%c%c%c\n", PCX, Addr, sel_asci, cData,
            (cData >> 7) & 1, (cData >> 6) & 1, (cData >> 5) & 1, (cData >> 4) & 1, (cData >> 3) & 1,
            cData & 0x07,
            (cData >> 2) & 1 ? '8' : '7',
            (cData >> 1) & 1 ? 'P' : 'N',
            (cData >> 0) & 1 ? '2' : '1');
        if (cData & 0x40) {
            sim_activate_after(&z180asci_unit[0], 2000);
        }
        break;
    case Z180_ASCI_CNTLB0:
        sel_asci = 0;    /* intentional falltrough */
    case Z180_ASCI_CNTLB1:
    {
        uint32 clockFrequency = getClockFrequency() * 1000 * 2;
        uint32 baud_prescale = (cData >> 5) & 1 ? 30 : 10;
        uint32 baud_divisor = (cData >> 3) & 1 ? 64 : 16;
        uint32 baud_ss = (cData & 0x07);
        uint32 general_divide_ratio = baud_prescale * baud_divisor * (1 << baud_ss);
        uint32 baudrate = clockFrequency / general_divide_ratio;
        z180asci_ctx[sel_asci].regs.cntlb = cData;
        sim_debug(ASCI_MSG, &z180asci_dev, ADDRESS_FORMAT
            " I/O WR 0x%02x (CNTLB%d) = 0x%02x: MPBT=%d, MP=%d, PS=%d(/%d), PE0=%c, DR=%d, SS=0x%x=%s (%d baud)\n", PCX, Addr, sel_asci, cData,
            (cData >> 7) & 1, (cData >> 6) & 1, (cData >> 5) & 1,
            baud_prescale,
            (cData >> 4) & 1 ? 'O' : 'E',
            baud_divisor,
            baud_ss,
            asci_ss_str_lut2[baud_ss],
            baudrate);
        break;
    }
    case Z180_ASCI_STAT0:
        sel_asci = 0;    /* intentional falltrough */
    case Z180_ASCI_STAT1:
        z180asci_ctx[sel_asci].regs.stat &= ~(Z180_ASCI_RIE | Z180_ASCI_TIE);
        z180asci_ctx[sel_asci].regs.stat |= cData & (Z180_ASCI_RIE | Z180_ASCI_TIE);
        sim_debug(ASCI_STAT_MSG, &z180asci_dev, ADDRESS_FORMAT
            " I/O WR 0x%02x (STAT%d) = 0x%02x.\n", PCX, Addr, sel_asci, cData);
        break;
    case Z180_ASCI_TDR0:
        sel_asci = 0;    /* intentional falltrough */
    case Z180_ASCI_TDR1:
        z180asci_ctx[sel_asci].regs.tdr = cData;
        z180asci_ctx[sel_asci].regs.stat &= ~Z180_ASCI_TDRE;
        z180asci_ctx[sel_asci].txp = 1;
        sim_debug(ASCI_TX_MSG, &z180asci_dev, ADDRESS_FORMAT
            " I/O WR 0x%02x (TDR%d) = 0x%02x.\n", PCX, Addr, sel_asci, cData);
        break;
    case Z180_ASCI_RDR0:
        sel_asci = 0;    /* intentional falltrough */
    case Z180_ASCI_RDR1:
        z180asci_ctx[sel_asci].regs.rdr = cData;
        sim_debug(ASCI_RX_MSG, &z180asci_dev, ADDRESS_FORMAT
            " I/O WR 0x%02x (RDR%d) = 0x%02x.\n", PCX, Addr, sel_asci, cData);
        break;
    case Z180_ASEXT0:
        sel_asci = 0;    /* intentional falltrough */
    case Z180_ASEXT1:
        z180asci_ctx[sel_asci].regs.ext = cData;
        sim_debug(ASCI_MSG, &z180asci_dev, ADDRESS_FORMAT
            " I/O WR 0x%02x (ASEXT%d) = 0x%02x.\n", PCX, Addr, sel_asci, cData);
        break;
    default:
        break;
    }

    return(0);
}
