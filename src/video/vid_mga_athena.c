/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Matrox MGA Impression Plus (MGA ATHENA) ISA / VLB / PCI
 *          graphics card with Gouraud-shaded, Z-buffered polygon
 *          acceleration.
 *
 *          The card is modelled as an ATHENA in "No DUBIC" mode driving
 *          a TI TVP3026 (ViewPoint) 64-bit RAMDAC directly, which is the
 *          configuration of the later Impression Plus boards. It
 *          consists of:
 *            - the ATHENA VGA core (VC-004) with its auxiliary registers
 *              at 3DEh/3DFh;
 *            - the PCI configuration space (DEVID 0D10102Bh) with the
 *              relocatable TERMBASE and ROMBASE apertures;
 *            - the 16 KB Power Graphic mode window (ISA map select
 *              AC000h-DC000h, or TERMBASE on PCI) holding the VRAM / pseudo-DMA window, the
 *              drawing engine and host registers, the memory-mapped VGA
 *              registers and the external device space (RAMDAC, DUBIC,
 *              VIWIC, clock generator);
 *            - the drawing engine: LINE / AUTOLINE, TRAP (solid,
 *              patterned, raster op, Gouraud + Z with the ZTAG fast
 *              clear), BITBLT, ILOAD and IDUMP, fed directly or through
 *              pseudo-DMA.
 *
 *          The drawing engine executes synchronously when a register in
 *          the 1D00h-1DFFh "go" range is written, so the bus FIFO always
 *          reads as empty.
 *
 *          Behaviour established by running the Matrox SXCI library (the
 *          hardware layer linked into every MGA 3D-SuperPack title) rather
 *          than taken from the specification:
 *            - REV reads A2681702h; SXCI selects its No DUBIC TVP3026
 *              path from the low nibble.
 *            - SXCI accepts only two memory codes on a No DUBIC board
 *              (bank 1, or banks 1 + 8); bank 5 flags the depth buffer.
 *            - the depth buffer is in VRAM, half the memory above each
 *              intensity pixel, and is cleared with ordinary fills.
 *            - ZI always writes depth; ZDRWEN only enables the compare.
 *            - AR0 holds 18 bits: BITBLT / ILOAD / IDUMP match the line
 *              end on the low 18 bits of the source address, and ILOAD
 *              consumes host data as one source run from AR3 to AR0.
 *            - Power Graphic CRTC offset and start address count
 *              16-pixel units.
 *
 *          Measured on real IS-ATHENA R2 PCI boards (553-03 Rev A and a
 *          revision 2 board), cold and with the SuperPack running:
 *            - board straps, CONFIG, FIFOSTATUS, INTSTS, the AUX inputs
 *              and the V4.0 video BIOS;
 *            - drawing registers read back the 64-bit destination latch;
 *              VCOUNT reads 0; 1E80-1EFF mirrors 1E00-1E7F;
 *            - with pseudo-DMA on, the redundant VRAM window reads 0;
 *            - 16-bit dithering is a per-channel 4x4 table of 0-7 offsets
 *              (engine test strips match the card pixel for pixel);
 *              intensities 256-383 saturate, 384-511 read as 0; depth
 *              saturates at FFFFh;
 *            - the depth buffer sits at half memory, 16 bits a pixel.
 *
 *          Register layout and semantics follow the Matrox "MGA ATHENA
 *          Specification", revision 1 (May 16, 1994, manual 1034-MS).
 *
 * Authors: Z2k Retro and contributors.
 *
 *          Copyright 2026 Z2k Retro.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/io.h>
#include "cpu.h"
#include <86box/timer.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/pci.h>
#include <86box/device.h>
#include <86box/plat.h>
#include <86box/video.h>
#include <86box/vid_svga.h>
#include <86box/vid_svga_render.h>

#define ROM_IMPRESSION_PLUS     "roms/video/matrox/mga_impression_plus_v36p.bin"
#define ROM_IMPRESSION_PLUS_V40 "roms/video/matrox/mga_impression_plus_pci_v40.bin"

/* Power Graphic mode window offsets. */
#define REG_DWGCTL     0x1c00
#define REG_MACCESS    0x1c04
#define REG_MCTLWTST   0x1c08
#define REG_DST0       0x1c10
#define REG_DST1       0x1c14
#define REG_ZMSK       0x1c18
#define REG_PLNWT      0x1c1c
#define REG_BCOL       0x1c20
#define REG_FCOL       0x1c24
#define REG_SRCBLT     0x1c2c
#define REG_SRC0       0x1c30
#define REG_SRC1       0x1c34
#define REG_SRC2       0x1c38
#define REG_SRC3       0x1c3c
#define REG_XYSTRT     0x1c40
#define REG_XYEND      0x1c44
#define REG_SHIFT      0x1c50
#define REG_SGN        0x1c58
#define REG_LEN        0x1c5c
#define REG_AR0        0x1c60
#define REG_AR1        0x1c64
#define REG_AR2        0x1c68
#define REG_AR3        0x1c6c
#define REG_AR4        0x1c70
#define REG_AR5        0x1c74
#define REG_AR6        0x1c78
#define REG_PITCH      0x1c8c
#define REG_YDST       0x1c90
#define REG_YDSTORG    0x1c94
#define REG_CYTOP      0x1c98
#define REG_CYBOT      0x1c9c
#define REG_CXLEFT     0x1ca0
#define REG_CXRIGHT    0x1ca4
#define REG_FXLEFT     0x1ca8
#define REG_FXRIGHT    0x1cac
#define REG_XDST       0x1cb0
#define REG_DR0        0x1cc0
#define REG_GO         0x0100

#define REG_VRAMPAGE   0x1e00
#define REG_BYTACCDATA 0x1e08
#define REG_ADRGEN     0x1e0c
#define REG_FIFOSTATUS 0x1e10
#define REG_STATUS     0x1e14
#define REG_ICLEAR     0x1e18
#define REG_INTSTS     0x1e28
#define REG_IEN        0x1e1c
#define REG_RST        0x1e40
#define REG_TEST       0x1e44
#define REG_REV        0x1e48
#define REG_CONFIG     0x1e50
#define REG_OPMODE     0x1e54
#define REG_CRTC_CTRL  0x1e5c
#define REG_VCOUNT     0x1e60

/* DWGCTL fields (ATHENA layout). */
#define DWG_OPCOD_MASK  0x0000000f
#define DWG_ATYPE_MASK  0x00000030
#define DWG_ATYPE_RPL   0x00000000
#define DWG_ATYPE_RSTR  0x00000010
#define DWG_ATYPE_ANTI  0x00000020
#define DWG_ATYPE_ZI    0x00000030
#define DWG_BLOCKM      0x00000040
#define DWG_LINEAR      0x00000080
#define DWG_BOP_SHIFT   16
#define DWG_TRANS_SHIFT 20
#define DWG_ALPHADIT    (1u << 24)
#define DWG_BLTMOD_MASK (3u << 25)
#define DWG_BMONO       (0u << 25)
#define DWG_BPLAN       (1u << 25)
#define DWG_BFCOL       (2u << 25)
#define DWG_BUCOL       (3u << 25)
#define DWG_ZDRWEN      (1u << 25)
#define DWG_ZLTE        (1u << 26)
#define DWG_HBGR        (1u << 27)
#define DWG_HCPRS       (1u << 28)
#define DWG_PATTERN     (1u << 29)
#define DWG_TRANSC      (1u << 30)

#define OPC_LINE_OPEN      0x0
#define OPC_AUTOLINE_OPEN  0x1
#define OPC_LINE_CLOSE     0x2
#define OPC_AUTOLINE_CLOSE 0x3
#define OPC_TRAP           0x4
#define OPC_BITBLT         0x8
#define OPC_ILOAD          0x9
#define OPC_IDUMP          0xa

/* CONFIG fields. */
#define CFG_NODUBIC_MASK (3u << 4)
#define CFG_BIOSEN       (1u << 9)
#define CFG_VGAEN        (1u << 10)
#define CFG_MAPSEL_SHIFT 24
#define CFG_MAPSEL_MASK  (7u << CFG_MAPSEL_SHIFT)
#define CFG_ISA          (1u << 28)

/* OPMODE fields. */
#define OPM_PSEUDODMA   (1u << 0)
#define OPM_DMAACT      (1u << 1)
#define OPM_DMAMOD_MASK (3u << 2)
#define DMAMOD_GENERAL  0
#define DMAMOD_BLIT     1
#define DMAMOD_VECTOR   2
#define DMAMOD_IDUMP    3

#define ATHENA_REV 0xa2681702 /* "TITAN" id; low byte 02h identifies the ATHENA (Impression Plus/Lite). */

enum {
    LOG_OFF = 0,
    LOG_SUMMARY,
    LOG_REGS,
    LOG_ALL
};

typedef struct athena_dwg_t {
    uint32_t dwgctl;
    uint32_t maccess;
    uint32_t mctlwtst;
    uint32_t zmsk;
    uint32_t plnwt;
    uint32_t bcol;
    uint32_t fcol;
    uint32_t src[4];
    uint32_t shift;
    uint32_t sgn;
    uint32_t len;
    int32_t  ar[7];
    uint32_t pitch;
    uint32_t ydst;
    int32_t  ydst_lin;
    uint32_t ydstorg;
    uint32_t cytop;
    uint32_t cybot;
    int32_t  cxleft;
    int32_t  cxright;
    int16_t  fxleft;
    int16_t  fxright;
    int16_t  xdst;
    int32_t  dr[16];
    int      selline;
    int      funcnt;
    int      dst_valid; /* DST0/1 read back as straps until the engine runs. */
    uint64_t dst_latch; /* afterwards: the engine's last destination quadword */
} athena_dwg_t;

typedef struct athena_t {
    svga_t        svga;
    rom_t         bios_rom;
    mem_mapping_t win_mapping;

    int      is_vlb;
    int      is_pci;
    uint8_t  pci_slot;
    uint8_t  pci_cmd;
    uint8_t  pci_option;
    uint8_t  int_line;
    uint32_t termbase;
    uint32_t rombase;
    uint32_t intsts;
    int      log_level;
    uint32_t vram_size;
    uint32_t vram_mask;

    uint64_t straps; /* VD<63:0> sampled at reset. */
    uint32_t config;
    uint32_t opmode;
    uint32_t crtc_ctrl;
    uint32_t rst;
    uint32_t test;
    uint32_t ien;
    uint32_t istat;
    uint32_t vrampage;
    uint32_t win_base;
    int      win_enabled;

    uint8_t  aux_idx;
    uint8_t  aux[16];
    uint8_t  dac_idx;   /* TVP3026 index register, as last written */
    uint8_t  dac_last;  /* last byte read from the RAMDAC */
    char     mode_desc[160]; /* last mode logged at Summary level */
    uint8_t  pos_46e8;

    /* External device space (DUBIC / VIWIC / clock generator / expansion):
       not fitted in No DUBIC mode, kept only so reads return what was
       written while the accesses are logged. */
    uint8_t  ext[0x400];

    /* Byte accumulator for dword registers reached through a 16-bit bus. */
    uint32_t acc_val;
    uint32_t acc_addr;

    athena_dwg_t dw;

    /* Pseudo-DMA address generator. */
    int      adrgen_state;
    uint32_t adrgen_word;
    int      vector_count;

    /* Host-fed operations in progress. */
    int      busy;
    int      busy_op;
    int      iload_x;
    int      iload_lines;
    uint32_t iload_src;
    uint32_t idump_src;
    int      idump_x;
    int      idump_lines;

    uint8_t  *ztag;
    uint32_t  zmask;

    uint32_t log_unhandled;
} athena_t;

static video_timings_t timing_athena_isa = { .type = VIDEO_ISA, .write_b = 3, .write_w = 3, .write_l = 6, .read_b = 5, .read_w = 5, .read_l = 10 };
static video_timings_t timing_athena_pci = { .type = VIDEO_PCI, .write_b = 4, .write_w = 4, .write_l = 4, .read_b = 10, .read_w = 10, .read_l = 10 };
static video_timings_t timing_athena_vlb = { .type = VIDEO_BUS, .write_b = 4, .write_w = 4, .write_l = 4, .read_b = 10, .read_w = 10, .read_l = 10 };

static const uint32_t mapsel_base[8] = { 0, 0xac000, 0xc8000, 0xcc000, 0xd0000, 0xd4000, 0xd8000, 0xdc000 };

static const uint8_t trans_masks[16][16] = {
  // clang-format off
    { 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1 },
    { 1,0,1,0, 0,1,0,1, 1,0,1,0, 0,1,0,1 },
    { 0,1,0,1, 1,0,1,0, 0,1,0,1, 1,0,1,0 },
    { 1,0,1,0, 0,0,0,0, 1,0,1,0, 0,0,0,0 },
    { 0,1,0,1, 0,0,0,0, 0,1,0,1, 0,0,0,0 },
    { 0,0,0,0, 1,0,1,0, 0,0,0,0, 1,0,1,0 },
    { 0,0,0,0, 0,1,0,1, 0,0,0,0, 0,1,0,1 },
    { 1,0,0,0, 0,0,0,0, 0,0,1,0, 0,0,0,0 },
    { 0,0,0,0, 0,1,0,0, 0,0,0,0, 0,0,0,1 },
    { 0,0,0,1, 0,0,0,0, 0,1,0,0, 0,0,0,0 },
    { 0,0,0,0, 0,0,1,0, 0,0,0,0, 1,0,0,0 },
    { 0,0,0,0, 1,0,0,0, 0,0,0,0, 0,0,1,0 },
    { 0,1,0,0, 0,0,0,0, 0,0,0,1, 0,0,0,0 },
    { 0,0,0,0, 0,0,0,1, 0,0,0,0, 0,1,0,0 },
    { 0,0,1,0, 0,0,0,0, 1,0,0,0, 0,0,0,0 },
    { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 }
  // clang-format on
};

/* 16-bit dithering, measured on a real ATHENA: each channel's 8-bit
   intensity gets an offset of 0-7 that depends on (y & 3, x & 3), then
   keeps its top five bits. Red uses 7 - Bayer/2; green is red upside down;
   blue has its own arrangement. */
static const uint8_t dither_r[4][4] = { { 7, 3, 6, 2 }, { 1, 5, 0, 4 }, { 6, 2, 7, 3 }, { 0, 4, 1, 5 } };
static const uint8_t dither_g[4][4] = { { 0, 4, 1, 5 }, { 6, 2, 7, 3 }, { 1, 5, 0, 4 }, { 7, 3, 6, 2 } };
static const uint8_t dither_b[4][4] = { { 2, 6, 3, 7 }, { 4, 0, 5, 1 }, { 3, 7, 2, 6 }, { 5, 1, 4, 0 } };

static inline int
dither5(int c, int off)
{
    c = (c + off) >> 3;
    return (c > 31) ? 31 : c;
}

static void athena_recalc_mapping(athena_t *athena);
static void athena_dwg_write(athena_t *athena, uint32_t addr, uint32_t val);
static void athena_iload_data(athena_t *athena, uint32_t val);
static uint32_t athena_idump_data(athena_t *athena);

/* ------------------------------------------------------------------ */
/* Logging                                                             */
/* ------------------------------------------------------------------ */

static void
athena_log(athena_t *athena, int level, const char *fmt, ...)
{
    va_list ap;

    if (athena->log_level < level)
        return;

    va_start(ap, fmt);
    pclog_ex(fmt, ap);
    va_end(ap);
}

/* ------------------------------------------------------------------ */
/* Straps and reset                                                    */
/* ------------------------------------------------------------------ */

static uint64_t
athena_build_straps(athena_t *athena, int mapsel, int product, int pcbrev, int variant)
{
    uint64_t vd = 0;
    uint32_t rambank;

    /* Board straps as read from two real IS-ATHENA R2 PCI boards (TVP3026,
       2 MB, no memory expansion): a 553-03 Rev A board reads DST0 046C09D9,
       DST1 1204DFE2 (PCB revision strap 3); a revision 2 board reads
       046D09D9 / 1246DFE2, which adds nodubic<1> and tram (variant 1). The bus
       dependent bits (55-57, 58, 61:59, 63:62) follow the ATHENA
       specification for the ISA/VLB boards. */
    vd |= 0x19;                                /* VD<4:0>   internal */
    if (!athena->is_pci)
        vd |= (uint64_t) 1 << 5;               /* VD<5>     vgaen0, 46E8 feature */
    vd |= (uint64_t) 0x027 << 6;               /* VD<15:6>  internal */
    vd |= (uint64_t) ((7 - pcbrev) & 7) << 16; /* VD<18:16> PCB revision, inverted (R2 reads 5) */
    vd |= (uint64_t) 1 << 19;                  /* VD<19>    block8/ */
    vd |= (uint64_t) (product & 0xf) << 20;    /* VD<23:20> product / platform */
    /* VD<24> vgabank0 = 0. */

    /* rambank<8:0>: bank 1 is the 2 MB on the board. The Matrox SXCI
       library accepts bank 1 alone (2 MB) or banks 1 + 8 (4 MB). */
    rambank = 1 << 1;
    if (athena->vram_size > (2 << 20))
        rambank |= (1 << 8);
    vd |= (uint64_t) (rambank & 0xff) << 25;   /* VD<32:25> */
    vd |= (uint64_t) 1 << 33;                  /* VD<34:33> ramspeed = 01 */
    vd |= (uint64_t) ((rambank >> 8) & 1) << 35;
    /* VD<36> HiRes/ = 0 (1600x1200 capable). */
    if (athena->is_pci)
        vd |= (uint64_t) 1 << 37;              /* vgaen1: VGA on, no 46E8 (PCI) */
    vd |= (uint64_t) 1 << 38;                  /* testwren pull-up */
    vd |= (uint64_t) 1 << 39;                  /* reserved, high */
    vd |= (uint64_t) 0xdf << 40;               /* VD<47:40> internal */
    /* VD<48> 200MHz = 0. */
    if (variant)
        vd |= (uint64_t) 1 << 49;              /* nodubic<1> */
    vd |= (uint64_t) 1 << 50;                  /* nodubic<0> (inverted) */
    /* VD<52:51> hyperpg = 0, VD<53> expdev = 0. */
    if (variant)
        vd |= (uint64_t) 1 << 54;              /* tram */
    if (athena->is_pci) {
        vd |= (uint64_t) 1 << 57;              /* above1meg (inverted) -> 0 */
        /* VD<58> = 0: driverdy 1, VD<61:59> mapsel 010, VD<63:62> config 00,
           as the specification requires of a PCI board. */
        vd |= (uint64_t) 2 << 59;
    } else {
        vd |= (uint64_t) 1 << 55;              /* isa */
        vd |= (uint64_t) 1 << 56;              /* pci (inverted) -> 0 */
        vd |= (uint64_t) 1 << 57;              /* above1meg (inverted) -> 0 */
        vd |= (uint64_t) 1 << 58;              /* driverdy (inverted) -> 0 */
        vd |= (uint64_t) (mapsel & 7) << 59;   /* VD<61:59> mapsel */
        vd |= (uint64_t) 1 << 62;              /* config<1:0> = 01, 16-bit */
    }

    return vd;
}

static void
athena_hard_reset(athena_t *athena)
{
    uint64_t vd = athena->straps;

    athena->config = 0;
    athena->config |= (uint32_t) ((vd >> 62) & 3);                 /* config<1:0> */
    athena->config |= (uint32_t) (!((vd >> 50) & 1)) << 4;          /* nodubic<0> */
    athena->config |= (uint32_t) (!((vd >> 58) & 1)) << 8;          /* driverdy */
    if (((vd >> 5) & 1) || ((vd >> 37) & 1))
        athena->config |= CFG_BIOSEN | CFG_VGAEN;
    athena->config |= (uint32_t) (!((vd >> 57) & 1)) << 12;         /* above1meg */
    athena->config |= (uint32_t) ((vd >> 59) & 7) << CFG_MAPSEL_SHIFT;
    athena->config |= (uint32_t) (!((vd >> 56) & 1)) << 27;         /* pci */
    athena->config |= (uint32_t) ((vd >> 55) & 1) << 28;            /* isa */

    athena->opmode    = (uint32_t) ((vd >> 24) & 1) << 11;          /* vgabank0 */
    athena->crtc_ctrl = 0;
    athena->rst       = 0;
    athena->test      = (uint32_t) (!((vd >> 38) & 1)) << 9;
    athena->ien       = 0;
    athena->istat     = 0;
    athena->vrampage  = 0;
    athena->pos_46e8  = 0x08;
    athena->intsts    = 0;

    memset(&athena->dw, 0, sizeof(athena->dw));
    athena->dw.mctlwtst  = 0xffffffff;
    athena->dw.dst_valid = 0;

    athena->adrgen_state = 0;
    athena->busy         = 0;
}

/* ------------------------------------------------------------------ */
/* VGA I/O                                                             */
/* ------------------------------------------------------------------ */

static void
athena_vga_out(uint16_t addr, uint8_t val, athena_t *athena)
{
    svga_t *svga = &athena->svga;
    uint8_t old;

    /* The auxiliary index/data ports stay at 3DEh/3DFh in both modes. */
    if ((addr != 0x3de) && (addr != 0x3df) &&
        ((addr & 0xfff0) == 0x3d0 || (addr & 0xfff0) == 0x3b0) && !(svga->miscout & 1))
        addr ^= 0x60;

    switch (addr) {
        case 0x3c6:
        case 0x3c7:
        case 0x3c8:
        case 0x3c9:
            tvp3026_ramdac_out(addr, 0, 0, val, svga->ramdac, svga);
            return;

        case 0x3d4:
            svga->crtcreg = val & 0x3f;
            return;
        case 0x3d5:
            if ((svga->crtcreg < 7) && (svga->crtc[0x11] & 0x80))
                return;
            if ((svga->crtcreg == 7) && (svga->crtc[0x11] & 0x80))
                val = (svga->crtc[7] & ~0x10) | (val & 0x10);
            old                       = svga->crtc[svga->crtcreg];
            svga->crtc[svga->crtcreg] = val;
            if (old != val) {
                if ((svga->crtcreg < 0xe) || (svga->crtcreg > 0x10)) {
                    if ((svga->crtcreg == 0xc) || (svga->crtcreg == 0xd)) {
                        svga->fullchange    = 3;
                        svga->memaddr_latch = ((svga->crtc[0xc] << 8) | svga->crtc[0xd]) + ((svga->crtc[8] & 0x60) >> 5);
                    } else {
                        svga->fullchange = changeframecount;
                        svga_recalctimings(svga);
                    }
                }
            }
            return;

        case 0x3de:
            athena->aux_idx = val & 0x0f;
            return;
        case 0x3df:
            athena_log(athena, LOG_REGS, "Athena: AUX[%02X] = %02X\n", athena->aux_idx, val);
            switch (athena->aux_idx) {
                case 0x05: /* trap flag, read only */
                case 0x06: /* CRTC FIFO read */
                case 0x07: /* aux input 1 */
                case 0x08: /* aux input 2 */
                    break;
                default:
                    athena->aux[athena->aux_idx] = val;
                    break;
            }
            switch (athena->aux_idx) {
                case 0x01:
                case 0x09:
                case 0x0c:
                    svga->read_bank = svga->write_bank = (athena->aux[0x09] & 0x0f) << 16;
                    svga_recalctimings(svga);
                    break;
                case 0x0a:
                case 0x0d:
                    svga_recalctimings(svga);
                    break;
                default:
                    break;
            }
            return;

        default:
            break;
    }

    svga_out(addr, val, svga);

    if ((addr == 0x3ce) || (addr == 0x3cf))
        athena_recalc_mapping(athena);
}

static uint8_t
athena_vga_in(uint16_t addr, athena_t *athena)
{
    svga_t *svga = &athena->svga;
    uint8_t ret;

    /* The auxiliary index/data ports stay at 3DEh/3DFh in both modes. */
    if ((addr != 0x3de) && (addr != 0x3df) &&
        ((addr & 0xfff0) == 0x3d0 || (addr & 0xfff0) == 0x3b0) && !(svga->miscout & 1))
        addr ^= 0x60;

    switch (addr) {
        case 0x3c6:
        case 0x3c7:
        case 0x3c8:
        case 0x3c9:
            return tvp3026_ramdac_in(addr, 0, 0, svga->ramdac, svga);

        case 0x3d4:
            return svga->crtcreg;
        case 0x3d5:
            return svga->crtc[svga->crtcreg];

        case 0x3de:
            return athena->aux_idx;
        case 0x3df:
            switch (athena->aux_idx) {
                case 0x05:
                case 0x06:
                    ret = 0x00;
                    break;
                case 0x07:
                    /* Monitor sense 11 on the real board (the spec says 00). */
                    ret = 0xc0;
                    break;
                case 0x08:
                    /* Revision code 101, memory config 00, display type 111,
                       as read from a real IS-ATHENA R2. */
                    ret = 0xa7;
                    break;
                default:
                    ret = athena->aux[athena->aux_idx];
                    break;
            }
            athena_log(athena, LOG_ALL, "Athena: AUX[%02X] read %02X\n", athena->aux_idx, ret);
            return ret;

        default:
            break;
    }

    return svga_in(addr, svga);
}

static void
athena_out(uint16_t addr, uint8_t val, void *priv)
{
    athena_t *athena = (athena_t *) priv;

    athena_log(athena, LOG_ALL, "Athena: OUT %03X, %02X\n", addr, val);
    if (!(athena->config & CFG_VGAEN) || (athena->is_pci && !(athena->pci_cmd & PCI_COMMAND_IO)))
        return;

    athena_vga_out(addr, val, athena);
}

static uint8_t
athena_in(uint16_t addr, void *priv)
{
    athena_t *athena = (athena_t *) priv;

    uint8_t   ret;

    if (!(athena->config & CFG_VGAEN) || (athena->is_pci && !(athena->pci_cmd & PCI_COMMAND_IO)))
        return 0xff;

    ret = athena_vga_in(addr, athena);
    athena_log(athena, LOG_ALL, "Athena: IN  %03X = %02X\n", addr, ret);
    return ret;
}

static void
athena_46e8_out(UNUSED(uint16_t addr), uint8_t val, void *priv)
{
    athena_t *athena = (athena_t *) priv;

    athena->pos_46e8 = val;
    athena_log(athena, LOG_REGS, "Athena: 46E8 = %02X\n", val);
}

static uint8_t
athena_46e8_in(UNUSED(uint16_t addr), void *priv)
{
    const athena_t *athena = (athena_t *) priv;

    return athena->pos_46e8;
}

/* ------------------------------------------------------------------ */
/* Display timing                                                      */
/* ------------------------------------------------------------------ */

static void
athena_recalctimings(svga_t *svga)
{
    athena_t *athena  = (athena_t *) svga->priv;
    int       clk_sel = (svga->miscout >> 2) & 3;

    svga->clock = (cpuclock * (double) (1ULL << 32)) / svga->getclock(clk_sel, svga->clock_gen);

    tvp3026_recalctimings(svga->ramdac, svga);

    if (athena->config & CFG_VGAEN) {
        /* VGA mode, with the VC-004 auxiliary extensions. */
        if (athena->aux[0x0a] & 0x10)
            svga->memaddr_latch |= (athena->aux[0x0a] & 3) << 16;

        if (athena->aux[0x01] & 0x08) {
            /* HR256: high resolution 256 colour, sequential pixels. */
            svga->lowres = 0;
            if (svga->bpp <= 8) {
                svga->render = svga_render_8bpp_highres;
                svga->bpp    = 8;
            }
        }
        svga->packed_chain4 = !!(athena->aux[0x01] & 0x01);
        svga->fb_only       = 0;
        if (athena->aux[0x0d] & 0x40)
            svga->interlace = 1;
    } else {
        /* Power Graphic mode: CRTC programmed on a pixel clock divided
           by 8, start address and offset in 64-bit slices. */
        int bytes_pp;

        switch (athena->crtc_ctrl & 3) {
            case 1:
                bytes_pp = 2;
                break;
            case 2:
                bytes_pp = 4;
                break;
            default:
                bytes_pp = 1;
                break;
        }

        svga->lowres         = 0;
        svga->char_width     = 8;
        svga->dots_per_clock = 8;
        svga->hdisp          = (svga->crtc[1] + 1) << 3;
        svga->hdisp_time     = svga->hdisp;
        /* The video delay (CRTC_CTRL) shifts blanking relative to the
           display enable: SXCI starts blanking one character before the
           end of the display, which still shows the full width. */
        if (svga->hblankstart <= svga->crtc[1])
            svga->hblankstart = svga->crtc[1] + 1;
        /* Offset and start address count 16-pixel units (SXCI programs
           an offset of 40 for a 640-pixel line); svga adds rowoffset x 8
           bytes per line and starts at memaddr_latch x 4 bytes. */
        svga->rowoffset      = svga->crtc[0x13] * 2 * bytes_pp;
        svga->memaddr_latch  = (((athena->aux[0x0a] & 3) << 16) | (svga->crtc[0xc] << 8) | svga->crtc[0xd]) * 4 * bytes_pp;
        svga->packed_chain4  = 1;
        svga->fb_only        = 1;

        if ((svga->bpp < 8) || (svga->bpp == 24)) {
            svga->bpp = (bytes_pp == 4) ? 32 : ((bytes_pp == 2) ? 15 : 8);
        }

        switch (svga->bpp) {
            case 15:
                svga->render = svga_render_15bpp_highres;
                break;
            case 16:
                svga->render = svga_render_16bpp_highres;
                break;
            case 32:
                svga->render = svga_render_32bpp_highres;
                break;
            default:
                svga->render = svga_render_8bpp_highres;
                break;
        }

        if (athena->crtc_ctrl & 0x18)
            svga->interlace = 1;
    }

    svga->disable_blink = (svga->bpp > 4);
    video_force_resize_set_monitor(1, svga->monitor_index);

    if (athena->log_level >= LOG_SUMMARY) {
        char   desc[160];
        double hz = (double) cpuclock * (double) (1ULL << 32) / svga->clock;

        snprintf(desc, sizeof(desc), "%s %dx%d %dbpp, pixel clock %.2f MHz, totals %dx%d, offset %d, start %X, CRTC_CTRL %X",
                 (athena->config & CFG_VGAEN) ? "VGA" : "Power Graphic", svga->hdisp, svga->dispend, svga->bpp,
                 hz / 1000000.0, svga->htotal, svga->vtotal, svga->rowoffset, svga->memaddr_latch, athena->crtc_ctrl);
        if (strcmp(desc, athena->mode_desc)) {
            strcpy(athena->mode_desc, desc);
            athena_log(athena, LOG_SUMMARY, "Athena: mode %s\n", desc);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Memory mapping                                                      */
/* ------------------------------------------------------------------ */

static void
athena_set_vga_mapping(athena_t *athena)
{
    svga_t *svga = &athena->svga;

    switch (svga->gdcreg[6] & 0x0c) {
        case 0x0: /*128k at A0000*/
            mem_mapping_set_addr(&svga->mapping, 0xa0000, 0x20000);
            svga->banked_mask = 0xffff;
            break;
        case 0x4: /*64k at A0000*/
            mem_mapping_set_addr(&svga->mapping, 0xa0000, 0x10000);
            svga->banked_mask = 0xffff;
            break;
        case 0x8: /*32k at B0000*/
            mem_mapping_set_addr(&svga->mapping, 0xb0000, 0x08000);
            svga->banked_mask = 0x7fff;
            break;
        case 0xC: /*32k at B8000*/
            mem_mapping_set_addr(&svga->mapping, 0xb8000, 0x08000);
            svga->banked_mask = 0x7fff;
            break;
        default:
            break;
    }
}

static void
athena_recalc_mapping(athena_t *athena)
{
    svga_t  *svga = &athena->svga;
    uint32_t base;
    int      enable;

    if (athena->is_pci) {
        int memspace = !!(athena->pci_cmd & PCI_COMMAND_MEM);

        if (memspace && (athena->config & CFG_VGAEN))
            athena_set_vga_mapping(athena);
        else
            mem_mapping_disable(&svga->mapping);

        if (memspace && (athena->config & CFG_BIOSEN) && (athena->rombase & 1))
            mem_mapping_set_addr(&athena->bios_rom.mapping, athena->rombase & 0xffff8000, 0x8000);
        else
            mem_mapping_disable(&athena->bios_rom.mapping);

        base   = athena->termbase & 0xffffc000;
        enable = memspace && base;
    } else {
        int mapsel = (athena->config & CFG_MAPSEL_MASK) >> CFG_MAPSEL_SHIFT;

        base   = mapsel_base[mapsel];
        enable = (base != 0);

        /* The AC000h map only decodes when the VGA does not use A0000h-AFFFFh. */
        if (enable && (base == 0xac000) && (athena->config & CFG_VGAEN) && !(svga->gdcreg[6] & 0x08))
            enable = 0;

        if ((athena->config & CFG_VGAEN) && (athena->pos_46e8 & 0x08))
            athena_set_vga_mapping(athena);
        else
            mem_mapping_disable(&svga->mapping);

        if (athena->config & CFG_BIOSEN)
            mem_mapping_enable(&athena->bios_rom.mapping);
        else
            mem_mapping_disable(&athena->bios_rom.mapping);
    }

    if (enable) {
        if (!athena->win_enabled || (athena->win_base != base))
            athena_log(athena, LOG_SUMMARY, "Athena: Power Graphic window at %08X\n", base);
        mem_mapping_set_addr(&athena->win_mapping, base, 0x4000);
    } else
        mem_mapping_disable(&athena->win_mapping);

    athena->win_base    = base;
    athena->win_enabled = enable;
}

/* ------------------------------------------------------------------ */
/* PCI configuration space                                             */
/* ------------------------------------------------------------------ */

static uint8_t
athena_pci_read(UNUSED(int func), int addr, UNUSED(int len), void *priv)
{
    const athena_t *athena = (athena_t *) priv;
    uint8_t         ret    = 0x00;

    switch (addr) {
        case 0x00:
            ret = 0x2b; /* Matrox */
            break;
        case 0x01:
            ret = 0x10;
            break;
        case 0x02:
            ret = 0x10; /* 0D10h: "ATH" in 5-bit ASCII */
            break;
        case 0x03:
            ret = 0x0d;
            break;
        case 0x04:
            ret = (athena->pci_cmd & 0x23) | 0x80; /* waitcycle reads 1 */
            break;
        case 0x07:
            ret = 0x02; /* devseltim = 01 (medium) */
            break;
        case 0x08:
            ret = 0x00; /* revision */
            break;
        case 0x0a:
            ret = (athena->config & CFG_VGAEN) ? 0x00 : 0x80;
            break;
        case 0x0b:
            ret = 0x03;
            break;
        case 0x10:
            ret = 0x00; /* memory BAR, 32-bit, not prefetchable */
            break;
        case 0x11:
            ret = (athena->termbase >> 8) & 0xc0;
            break;
        case 0x12:
            ret = (athena->termbase >> 16) & 0xff;
            break;
        case 0x13:
            ret = (athena->termbase >> 24) & 0xff;
            break;
        case 0x30:
            ret = (athena->config & CFG_BIOSEN) ? (athena->rombase & 0x01) : 0x00;
            break;
        case 0x31:
            ret = (athena->rombase >> 8) & 0x80;
            break;
        case 0x32:
            ret = (athena->rombase >> 16) & 0xff;
            break;
        case 0x33:
            ret = (athena->rombase >> 24) & 0xff;
            break;
        case 0x3c:
            ret = athena->int_line;
            break;
        case 0x3d:
            ret = PCI_INTA;
            break;
        case 0x40:
            ret = athena->pci_option & 3;
            break;
        default:
            break;
    }

    return ret;
}

static void
athena_pci_write(UNUSED(int func), int addr, UNUSED(int len), uint8_t val, void *priv)
{
    athena_t *athena = (athena_t *) priv;

    switch (addr) {
        case 0x04:
            athena->pci_cmd = val & 0x23;
            athena_recalc_mapping(athena);
            break;
        case 0x11:
            athena->termbase = (athena->termbase & 0xffff0000) | ((val & 0xc0) << 8);
            athena_recalc_mapping(athena);
            break;
        case 0x12:
            athena->termbase = (athena->termbase & 0xff00c000) | (val << 16);
            athena_recalc_mapping(athena);
            break;
        case 0x13:
            athena->termbase = (athena->termbase & 0x00ffc000) | (val << 24);
            athena_recalc_mapping(athena);
            break;
        case 0x30:
            if (athena->config & CFG_BIOSEN)
                athena->rombase = (athena->rombase & 0xffff8000) | (val & 0x01);
            athena_recalc_mapping(athena);
            break;
        case 0x31:
            athena->rombase = (athena->rombase & 0xffff0001) | ((val & 0x80) << 8);
            athena_recalc_mapping(athena);
            break;
        case 0x32:
            athena->rombase = (athena->rombase & 0xff008001) | (val << 16);
            athena_recalc_mapping(athena);
            break;
        case 0x33:
            athena->rombase = (athena->rombase & 0x00ff8001) | (val << 24);
            athena_recalc_mapping(athena);
            break;
        case 0x3c:
            athena->int_line = val;
            break;
        case 0x40:
            athena->pci_option = val & 3;
            break;
        default:
            break;
    }
}

static inline int32_t
sext18(uint32_t val)
{
    return (val & 0x20000) ? (int32_t) (val | 0xfffc0000) : (int32_t) (val & 0x3ffff);
}

static inline int32_t
sext24(uint32_t val)
{
    return (val & 0x800000) ? (int32_t) (val | 0xff000000) : (int32_t) (val & 0xffffff);
}

/* ------------------------------------------------------------------ */
/* Pixel access helpers                                                */
/* ------------------------------------------------------------------ */

static inline int
athena_pwidth(const athena_t *athena)
{
    return athena->dw.maccess & 3;
}

static inline uint32_t
athena_rd_pix(athena_t *athena, uint32_t pix)
{
    const svga_t *svga = &athena->svga;

    switch (athena_pwidth(athena)) {
        case 1:
            return *(uint16_t *) &svga->vram[(pix << 1) & athena->vram_mask];
        case 2:
            return *(uint32_t *) &svga->vram[(pix << 2) & athena->vram_mask];
        default:
            return svga->vram[pix & athena->vram_mask];
    }
}

static inline void
athena_wr_pix(athena_t *athena, uint32_t pix, uint32_t val)
{
    svga_t  *svga = &athena->svga;
    uint32_t addr;
    uint32_t m = athena->dw.plnwt;

    switch (athena_pwidth(athena)) {
        case 1:
            addr                            = (pix << 1) & athena->vram_mask;
            m &= 0xffff;
            *(uint16_t *) &svga->vram[addr] = (val & m) | (*(uint16_t *) &svga->vram[addr] & ~m);
            break;
        case 2:
            addr                            = (pix << 2) & athena->vram_mask;
            *(uint32_t *) &svga->vram[addr] = (val & m) | (*(uint32_t *) &svga->vram[addr] & ~m);
            break;
        default:
            addr             = pix & athena->vram_mask;
            m &= 0xff;
            svga->vram[addr] = (val & m) | (svga->vram[addr] & ~m);
            break;
    }
    svga->changedvram[addr >> 12] = changeframecount;
    athena->istat |= 4;                       /* pickists: the engine wrote a pixel */
    athena->dw.dst_latch = *(uint64_t *) &svga->vram[addr & ~7];
}

static inline uint32_t
athena_bitop(uint32_t src, uint32_t dst, uint32_t dwgctl)
{
    switch ((dwgctl >> DWG_BOP_SHIFT) & 0xf) {
        case 0x0:
            return 0;
        case 0x1:
            return ~(dst | src);
        case 0x2:
            return dst & ~src;
        case 0x3:
            return ~src;
        case 0x4:
            return ~dst & src;
        case 0x5:
            return ~dst;
        case 0x6:
            return dst ^ src;
        case 0x7:
            return ~(dst & src);
        case 0x8:
            return dst & src;
        case 0x9:
            return ~(dst ^ src);
        case 0xa:
            return dst;
        case 0xb:
            return dst | ~src;
        case 0xc:
        default:
            return src;
        case 0xd:
            return ~dst | src;
        case 0xe:
            return dst | src;
        case 0xf:
            return ~0;
    }
}

static inline int
athena_clip(const athena_t *athena, int x)
{
    const athena_dwg_t *dw = &athena->dw;

    return (x >= dw->cxleft) && (x <= dw->cxright) && (dw->ydst_lin >= (int32_t) dw->cytop) && (dw->ydst_lin <= (int32_t) dw->cybot);
}

/* Write one pixel through the raster unit (RPL / RSTR). */
static inline void
athena_raster_pix(athena_t *athena, uint32_t pix, uint32_t src)
{
    uint32_t dwg = athena->dw.dwgctl;

    if ((dwg & DWG_ATYPE_MASK) == DWG_ATYPE_RSTR)
        src = athena_bitop(src, athena_rd_pix(athena, pix), dwg);

    athena_wr_pix(athena, pix, src);
}

static inline void
athena_ztag_update(athena_t *athena, uint32_t pix, int blockm)
{
    uint32_t zmsk = athena->dw.zmsk;
    int      bit  = pix & 3;
    uint32_t idx  = pix & athena->zmask;

    if (!(zmsk & 0x100))
        return;
    if (!(zmsk & (0x10 << bit)))
        return;

    if (blockm ? (zmsk & 0x200) : (zmsk & (1 << bit)))
        athena->ztag[idx >> 3] |= (1 << (idx & 7));
    else
        athena->ztag[idx >> 3] &= ~(1 << (idx & 7));
}


/* Depth buffer. On the Impression Plus the 16-bit Z of a pixel lives in
   VRAM, half the memory above its intensity pixel (SXCI clears it with an
   ordinary fill at YDSTORG + 80000h on a 2 MB board). */
static inline uint32_t
athena_z_addr(const athena_t *athena, uint32_t pix)
{
    return ((pix + (athena->vram_size >> 2)) << 1) & athena->vram_mask;
}

static inline uint16_t
athena_z_read(athena_t *athena, uint32_t pix)
{
    return *(uint16_t *) &athena->svga.vram[athena_z_addr(athena, pix)];
}

static inline void
athena_z_write(athena_t *athena, uint32_t pix, uint16_t z)
{
    uint32_t a = athena_z_addr(athena, pix);

    *(uint16_t *) &athena->svga.vram[a] = z;
    athena->svga.changedvram[a >> 12]   = changeframecount;
}

static inline int
athena_pattern_bit(const athena_t *athena, int x)
{
    const athena_dwg_t *dw   = &athena->dw;
    int                 xoff = (dw->shift & 7) + x;
    int                 yoff = ((dw->shift >> 4) & 7) + dw->selline;
    int                 bit  = ((yoff & 7) * 16) + (xoff & 7);

    return (dw->src[bit >> 5] >> (bit & 31)) & 1;
}

static inline int
athena_clamp_intensity(int32_t v)
{
    switch ((v >> 22) & 3) {
        case 2:
            return 255;
        case 3:
            return 0;
        default:
            return (v >> 15) & 0xff;
    }
}

static inline uint32_t
athena_shade_color(athena_t *athena, int x)
{
    const athena_dwg_t *dw = &athena->dw;
    /* Out of range (measured): with the top two bits of the 24-bit
       intensity at 10 (256-383) it saturates at 255; at 11 (384-511, that
       is negative) it reads 0. */
    int                 r  = athena_clamp_intensity(dw->dr[4]);
    int                 g  = athena_clamp_intensity(dw->dr[8]);
    int                 b  = athena_clamp_intensity(dw->dr[12]);

    switch (athena_pwidth(athena)) {
        case 1:
            {
                int dx = x & 3;
                int dy = dw->selline & 3;

                return dither5(b, dither_b[dy][dx]) | (dither5(g, dither_g[dy][dx]) << 5) | (dither5(r, dither_r[dy][dx]) << 10);
            }
        case 2:
            {
                uint32_t a = (dw->dwgctl & DWG_ALPHADIT) ? ((uint32_t) r << 24) : (dw->fcol & 0xff000000);

                return a | (r << 16) | (g << 8) | b;
            }
        default:
            return (r & 0xe0) | ((g & 0xe0) >> 3) | (b >> 6);
    }
}

static inline uint32_t
athena_dr_signext24(uint32_t val)
{
    return (val & 0x800000) ? (val | 0xff000000) : (val & 0x00ffffff);
}

/* Advance the Y destination by one line in the requested direction. */
static inline void
athena_next_line(athena_t *athena, int down)
{
    athena_dwg_t *dw = &athena->dw;
    int32_t       iy = dw->pitch & 0x1fe0;

    if (down) {
        dw->ydst++;
        dw->ydst_lin += iy;
    } else {
        dw->ydst--;
        dw->ydst_lin -= iy;
    }
    dw->selline = (dw->selline + (down ? 1 : -1)) & 7;
}

/* ------------------------------------------------------------------ */
/* Drawing engine: LINE                                                */
/* ------------------------------------------------------------------ */

static void
athena_draw_line(athena_t *athena, int closed, int autoline)
{
    athena_dwg_t *dw    = &athena->dw;
    int           atype = dw->dwgctl & DWG_ATYPE_MASK;
    int           x     = dw->xdst;
    int32_t       err;
    int32_t       k1;
    int32_t       k2;
    int           len;
    int           start_x = dw->ar[5];
    int           start_y = dw->ar[6];
    int           end_x   = dw->ar[0];
    int           end_y   = dw->ar[2];
    int           xmajor;
    int           sdxl;
    int           sdy;
    int           stylelen = ((dw->shift >> 16) & 0x7f);

    if (autoline) {
        int dx = end_x - start_x;
        int dy = end_y - start_y;
        int adx = (dx < 0) ? -dx : dx;
        int ady = (dy < 0) ? -dy : dy;

        sdxl   = dx < 0;
        sdy    = dy < 0;
        xmajor = adx > ady;
        if (xmajor) {
            k1  = 2 * ady;
            k2  = 2 * ady - 2 * adx;
            err = 2 * ady - adx - sdy;
            len = adx;
        } else {
            k1  = 2 * adx;
            k2  = 2 * adx - 2 * ady;
            err = 2 * adx - ady - sdy;
            len = ady;
        }
        dw->sgn = (dw->sgn & ~7) | (xmajor ? 1 : 0) | (sdxl ? 2 : 0) | (sdy ? 4 : 0);
        x       = start_x;
        dw->ydst = start_y;
        dw->ydst_lin = (int32_t) (int16_t) start_y * (int32_t) (dw->pitch & 0x1fe0) + dw->ydstorg;
        dw->selline  = start_y & 7;
    } else {
        k1     = dw->ar[0];
        err    = dw->ar[1];
        k2     = dw->ar[2];
        len    = dw->len & 0xffff;
        xmajor = dw->sgn & 1;
        sdxl   = !!(dw->sgn & 2);
        sdy    = !!(dw->sgn & 4);
    }

    if (closed)
        len++;

    for (int i = 0; i < len; i++) {
        if (athena_clip(athena, x)) {
            uint32_t pix = dw->ydst_lin + x;

            if (atype == DWG_ATYPE_ZI) {
                int       pass = 1;
                uint16_t  z    = (dw->dr[0] < 0) ? 0 : (uint16_t) (((uint32_t) dw->dr[0] >> 15) > 0xffff ? 0xffff : ((uint32_t) dw->dr[0] >> 15));
                uint32_t  zi   = pix & athena->zmask;

                if (dw->dwgctl & DWG_ZDRWEN) {
                    uint16_t old_z = (athena->ztag[zi >> 3] & (1 << (zi & 7))) ? (uint16_t) (dw->fcol & 0xffff) : athena_z_read(athena, pix);

                    pass = (dw->dwgctl & DWG_ZLTE) ? (z <= old_z) : (z < old_z);
                }
                if (pass) {
                    athena_z_write(athena, pix, z);
                    athena_ztag_update(athena, pix, 0);
                    athena_wr_pix(athena, pix, athena_shade_color(athena, x));
                }
            } else {
                int style = 1;

                if (stylelen) {
                    int bit = dw->funcnt & 0x7f;

                    style = (dw->src[bit >> 5] >> (bit & 31)) & 1;
                }
                if (style || !(dw->dwgctl & DWG_TRANSC))
                    athena_raster_pix(athena, pix, style ? dw->fcol : dw->bcol);
            }
        }

        if (stylelen) {
            if (dw->funcnt == 0)
                dw->funcnt = stylelen;
            else
                dw->funcnt--;
        }

        /* Step along the major axis. */
        if (xmajor)
            x += sdxl ? -1 : 1;
        else
            athena_next_line(athena, !sdy);

        if (err >= 0) {
            if (xmajor)
                athena_next_line(athena, !sdy);
            else
                x += sdxl ? -1 : 1;
            err += k2;
            dw->dr[0] += dw->dr[3];
            dw->dr[4] = athena_dr_signext24(dw->dr[4] + dw->dr[7]);
            dw->dr[8] = athena_dr_signext24(dw->dr[8] + dw->dr[11]);
            dw->dr[12] = athena_dr_signext24(dw->dr[12] + dw->dr[15]);
        } else {
            err += k1;
            dw->dr[0] += dw->dr[2];
            dw->dr[4] = athena_dr_signext24(dw->dr[4] + dw->dr[6]);
            dw->dr[8] = athena_dr_signext24(dw->dr[8] + dw->dr[10]);
            dw->dr[12] = athena_dr_signext24(dw->dr[12] + dw->dr[14]);
        }
    }

    dw->xdst  = x;
    dw->ar[1] = err;
    if (autoline) {
        dw->xdst     = end_x;
        dw->ar[5]    = end_x;
        dw->ar[6]    = end_y;
        dw->ydst     = end_y;
        dw->ydst_lin = (int32_t) (int16_t) end_y * (int32_t) (dw->pitch & 0x1fe0) + dw->ydstorg;
        dw->selline  = end_y & 7;
    }
    dw->len = 0;
}

/* ------------------------------------------------------------------ */
/* Drawing engine: TRAP                                                */
/* ------------------------------------------------------------------ */

static void
athena_trap_step_edges(athena_t *athena)
{
    athena_dwg_t *dw = &athena->dw;

    while ((dw->ar[1] < 0) && dw->ar[0]) {
        dw->ar[1] += dw->ar[0];
        dw->fxleft += (dw->sgn & 2) ? -1 : 1;
    }
    dw->ar[1] += dw->ar[2];

    while ((dw->ar[4] < 0) && dw->ar[6]) {
        dw->ar[4] += dw->ar[6];
        dw->fxright += (dw->sgn & 0x20) ? -1 : 1;
    }
    dw->ar[4] += dw->ar[5];
}

static void
athena_draw_trap(athena_t *athena)
{
    athena_dwg_t  *dw     = &athena->dw;
    int            atype  = dw->dwgctl & DWG_ATYPE_MASK;
    int            blockm = !!(dw->dwgctl & DWG_BLOCKM);
    int            transc = !!(dw->dwgctl & DWG_TRANSC);
    int            patt   = !!(dw->dwgctl & DWG_PATTERN);
    int            tsel   = (dw->dwgctl >> DWG_TRANS_SHIFT) & 0xf;
    int            lines  = dw->len & 0xffff;
    int            down   = !(dw->sgn & 4);

    for (int y = 0; y < lines; y++) {
        const uint8_t *trans = &trans_masks[tsel][(dw->selline & 3) * 4];
        int16_t        x_l   = dw->fxleft;
        int16_t        x_r   = dw->fxright;
        int16_t        old_x_l = x_l;

        if (atype == DWG_ATYPE_ZI) {
            int32_t z_back = dw->dr[0];
            int32_t r_back = dw->dr[4];
            int32_t g_back = dw->dr[8];
            int32_t b_back = dw->dr[12];
            int     zen    = !!(dw->dwgctl & DWG_ZDRWEN);
            int     zlte   = !!(dw->dwgctl & DWG_ZLTE);

            while (x_l != x_r) {
                if (athena_clip(athena, x_l) && trans[x_l & 3]) {
                    uint32_t pix  = dw->ydst_lin + x_l;
                    int      pass = 1;

                    uint32_t zi   = pix & athena->zmask;
                    /* 17.15; values past FFFFh saturate (measured). */
                    uint32_t zraw = (uint32_t) dw->dr[0] >> 15;
                    uint16_t z    = (zraw > 0xffff) ? 0xffff : (uint16_t) zraw;

                    /* ZI always writes depth; ZDRWEN only adds the compare. */
                    if (zen) {
                        uint16_t old_z = (athena->ztag[zi >> 3] & (1 << (zi & 7))) ? (uint16_t) (dw->fcol & 0xffff) : athena_z_read(athena, pix);

                        pass = zlte ? (z <= old_z) : (z < old_z);
                    }
                    if (pass) {
                        athena_z_write(athena, pix, z);
                        athena_ztag_update(athena, pix, 0);
                        athena_wr_pix(athena, pix, athena_shade_color(athena, x_l));
                    }
                }

                dw->dr[0] += dw->dr[2];
                dw->dr[4] = athena_dr_signext24(dw->dr[4] + dw->dr[6]);
                dw->dr[8] = athena_dr_signext24(dw->dr[8] + dw->dr[10]);
                dw->dr[12] = athena_dr_signext24(dw->dr[12] + dw->dr[14]);

                if (x_l > x_r)
                    x_l--;
                else
                    x_l++;
            }

            dw->dr[0]  = z_back + dw->dr[3];
            dw->dr[4]  = athena_dr_signext24(r_back + dw->dr[7]);
            dw->dr[8]  = athena_dr_signext24(g_back + dw->dr[11]);
            dw->dr[12] = athena_dr_signext24(b_back + dw->dr[15]);

            athena_trap_step_edges(athena);

            {
                int dx = (int16_t) (dw->fxleft - old_x_l);

                dw->dr[0] += dx * dw->dr[2];
                dw->dr[4] = athena_dr_signext24(dw->dr[4] + dx * dw->dr[6]);
                dw->dr[8] = athena_dr_signext24(dw->dr[8] + dx * dw->dr[10]);
                dw->dr[12] = athena_dr_signext24(dw->dr[12] + dx * dw->dr[14]);
            }
        } else {
            int len = (x_l > x_r) ? (x_l - x_r) : (x_r - x_l);
            int x   = (x_l > x_r) ? x_r : x_l;

            for (; len > 0; len--, x++) {
                uint32_t pix;
                int      pat;

                if (!athena_clip(athena, x) || !trans[x & 3])
                    continue;

                pix = dw->ydst_lin + x;
                pat = patt ? athena_pattern_bit(athena, x) : 1;

                if (pat || !transc)
                    athena_raster_pix(athena, pix, pat ? dw->fcol : dw->bcol);
                athena_ztag_update(athena, pix, blockm);
            }

            athena_trap_step_edges(athena);
        }

        athena_next_line(athena, down);
    }

    dw->len = 0;
}

/* ------------------------------------------------------------------ */
/* Drawing engine: BITBLT                                              */
/* ------------------------------------------------------------------ */

static void
athena_draw_bitblt(athena_t *athena)
{
    athena_dwg_t *dw     = &athena->dw;
    uint32_t      bltmod = dw->dwgctl & DWG_BLTMOD_MASK;
    int           transc = !!(dw->dwgctl & DWG_TRANSC);
    int           tsel   = (dw->dwgctl >> DWG_TRANS_SHIFT) & 0xf;
    int           lines  = dw->len & 0xffff;
    int           scanleft = dw->sgn & 1;
    int           x_dir  = scanleft ? -1 : 1;
    int           down   = !(dw->sgn & 4);
    int32_t       src;

    if (dw->dwgctl & DWG_LINEAR)
        athena_log(athena, LOG_SUMMARY, "Athena: linear BITBLT not modelled (dwgctl %08X)\n", dw->dwgctl);

    src = dw->ar[3];

    for (int y = 0; y < lines; y++) {
        const uint8_t *trans = &trans_masks[tsel][(dw->selline & 3) * 4];
        int            x     = scanleft ? dw->fxright : dw->fxleft;
        int            x_end = scanleft ? dw->fxleft : dw->fxright;

        while (1) {
            if (athena_clip(athena, x) && trans[x & 3]) {
                uint32_t pix = dw->ydst_lin + x;

                switch (bltmod) {
                    case DWG_BMONO:
                        {
                            uint32_t s   = (uint32_t) src & 0x7ffffff;
                            int      bit = (athena->svga.vram[(s >> 3) & athena->vram_mask] >> (s & 7)) & 1;

                            if (bit || !transc)
                                athena_raster_pix(athena, pix, bit ? dw->fcol : dw->bcol);
                        }
                        break;
                    case DWG_BPLAN:
                        {
                            int bit = athena_rd_pix(athena, (uint32_t) src) & 1;

                            if (bit || !transc)
                                athena_raster_pix(athena, pix, bit ? dw->fcol : dw->bcol);
                        }
                        break;
                    default:
                        athena_raster_pix(athena, pix, athena_rd_pix(athena, (uint32_t) src));
                        break;
                }
            }

            if (((uint32_t) src & 0x3ffff) == ((uint32_t) dw->ar[0] & 0x3ffff)) {
                /* AR0 is an 18-bit register: line end is matched on the low
                   18 bits of the 24-bit source address. */
                dw->ar[0] = sext18(dw->ar[0] + dw->ar[5]);
                dw->ar[3] = sext24(dw->ar[3] + dw->ar[5]);
                src = dw->ar[3];
                break;
            }
            src += x_dir;

            if (x == x_end)
                break;
            x += x_dir;
        }

        athena_next_line(athena, down);
    }

    dw->len = 0;
}

/* ------------------------------------------------------------------ */
/* Drawing engine: ILOAD / IDUMP                                       */
/* ------------------------------------------------------------------ */

static void
athena_iload_start(athena_t *athena)
{
    athena_dwg_t *dw = &athena->dw;

    athena->busy        = 1;
    athena->busy_op     = OPC_ILOAD;
    athena->iload_x     = dw->fxleft;
    athena->iload_lines = dw->len & 0xffff;
    athena->iload_src   = dw->ar[3];
    if (!athena->iload_lines)
        athena->busy = 0;

    athena_log(athena, LOG_REGS, "Athena: ILOAD %d..%d x %d lines, bltmod %d, pw %d\n",
               dw->fxleft, dw->fxright, athena->iload_lines, (dw->dwgctl >> 25) & 3, athena_pwidth(athena));
}

/* Consume one source pixel. Returns 1 at the end of a source line (AR0),
   where the rest of the current host dword is discarded. */
static int
athena_iload_pixel(athena_t *athena, uint32_t val, int mono)
{
    athena_dwg_t *dw  = &athena->dw;
    int           x   = athena->iload_x;
    int           eol = 0;

    if (athena->iload_lines <= 0)
        return 1;

    if ((athena->iload_src & 0x3ffff) == ((uint32_t) dw->ar[0] & 0x3ffff)) {
        dw->ar[0]         = sext18(dw->ar[0] + dw->ar[5]);
        dw->ar[3]         = sext24(dw->ar[3] + dw->ar[5]);
        athena->iload_src = dw->ar[3];
        eol               = 1;
    } else
        athena->iload_src++;

    if (athena_clip(athena, x)) {
        uint32_t pix = dw->ydst_lin + x;

        if (mono) {
            if (val || !(dw->dwgctl & DWG_TRANSC))
                athena_raster_pix(athena, pix, val ? dw->fcol : dw->bcol);
        } else
            athena_raster_pix(athena, pix, val);
    }

    if (x >= dw->fxright) {
        athena->iload_x = dw->fxleft;
        athena->iload_lines--;
        athena_next_line(athena, !(dw->sgn & 4));
        if (athena->iload_lines <= 0) {
            athena->busy = 0;
            eol          = 1;
        }
    } else
        athena->iload_x++;

    return eol;
}

static inline uint32_t
athena_rgb_to_pw(athena_t *athena, int r, int g, int b)
{
    switch (athena_pwidth(athena)) {
        case 1:
            return ((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3);
        case 2:
            return (r << 16) | (g << 8) | b;
        default:
            return (r & 0xe0) | ((g & 0xe0) >> 3) | (b >> 6);
    }
}

static void
athena_iload_data(athena_t *athena, uint32_t val)
{
    athena_dwg_t *dw     = &athena->dw;
    uint32_t      bltmod = dw->dwgctl & DWG_BLTMOD_MASK;

    if (!athena->busy || (athena->busy_op != OPC_ILOAD)) {
        athena_log(athena, LOG_ALL, "Athena: ILOAD data %08X with no ILOAD running\n", val);
        return;
    }

    switch (bltmod) {
        case DWG_BMONO:
        case DWG_BPLAN:
            for (int i = 0; i < 32; i++) {
                int bit = (dw->dwgctl & DWG_HBGR) ? ((val >> (i ^ 7)) & 1) : ((val >> i) & 1);

                if (athena_iload_pixel(athena, bit, 1))
                    break;
            }
            break;

        case DWG_BUCOL:
            {
                int r;
                int g;
                int b;

                if (dw->dwgctl & DWG_HCPRS)
                    athena_log(athena, LOG_SUMMARY, "Athena: 24-bit compressed ILOAD treated as 32-bit\n");
                if (dw->dwgctl & DWG_HBGR) {
                    r = (val >> 16) & 0xff;
                    g = (val >> 8) & 0xff;
                    b = val & 0xff;
                } else {
                    r = val & 0xff;
                    g = (val >> 8) & 0xff;
                    b = (val >> 16) & 0xff;
                }
                athena_iload_pixel(athena, athena_rgb_to_pw(athena, r, g, b), 0);
            }
            break;

        default: /* BFCOL: data already in frame buffer format */
            switch (athena_pwidth(athena)) {
                case 1:
                    if (!athena_iload_pixel(athena, val & 0xffff, 0))
                        athena_iload_pixel(athena, val >> 16, 0);
                    break;
                case 2:
                    athena_iload_pixel(athena, val, 0);
                    break;
                default:
                    for (int i = 0; i < 4; i++) {
                        if (athena_iload_pixel(athena, (val >> (i * 8)) & 0xff, 0))
                            break;
                    }
                    break;
            }
            break;
    }
}

static void
athena_idump_start(athena_t *athena)
{
    athena_dwg_t *dw = &athena->dw;

    athena->busy        = 1;
    athena->busy_op     = OPC_IDUMP;
    athena->idump_src   = dw->ar[3];
    athena->idump_lines = dw->len & 0xffff;
    if (!athena->idump_lines)
        athena->busy = 0;

    athena_log(athena, LOG_REGS, "Athena: IDUMP from %08X to %08X x %d lines\n", dw->ar[3], dw->ar[0], athena->idump_lines);
}

static uint32_t
athena_idump_pixel(athena_t *athena, int *eol)
{
    athena_dwg_t *dw  = &athena->dw;
    uint32_t      ret = athena_rd_pix(athena, athena->idump_src);

    *eol = 0;
    if ((athena->idump_src & 0x3ffff) == ((uint32_t) dw->ar[0] & 0x3ffff)) {
        dw->ar[0] = sext18(dw->ar[0] + dw->ar[5]);
        dw->ar[3] = sext24(dw->ar[3] + dw->ar[5]);
        athena->idump_src = dw->ar[3];
        athena->idump_lines--;
        *eol = 1;
        if (athena->idump_lines <= 0)
            athena->busy = 0;
    } else
        athena->idump_src += (dw->sgn & 1) ? -1 : 1;

    return ret;
}

static uint32_t
athena_idump_data(athena_t *athena)
{
    uint32_t ret = 0;
    int      eol = 0;

    if (!athena->busy || (athena->busy_op != OPC_IDUMP))
        return 0xffffffff;

    switch (athena_pwidth(athena)) {
        case 1:
            ret = athena_idump_pixel(athena, &eol) & 0xffff;
            if (!eol && athena->busy)
                ret |= (athena_idump_pixel(athena, &eol) & 0xffff) << 16;
            break;
        case 2:
            ret = athena_idump_pixel(athena, &eol);
            break;
        default:
            for (int i = 0; (i < 4) && !eol && athena->busy; i++)
                ret |= (athena_idump_pixel(athena, &eol) & 0xff) << (i * 8);
            break;
    }

    return ret;
}

/* ------------------------------------------------------------------ */
/* Drawing engine dispatch                                             */
/* ------------------------------------------------------------------ */

static void
athena_start(athena_t *athena)
{
    athena_dwg_t *dw  = &athena->dw;
    int           opc = dw->dwgctl & DWG_OPCOD_MASK;

    dw->dst_valid = 1;

    athena_log(athena, LOG_REGS, "Athena: GO opcod %X dwgctl %08X maccess %08X len %d fx %d..%d ydst %d (lin %d) ar0 %d ar1 %d ar2 %d ar3 %d ar4 %d ar5 %d ar6 %d sgn %02X z %08X rgb %06X %06X %06X fcol %08X plnwt %08X org %d\n",
               opc, dw->dwgctl, dw->maccess, dw->len & 0xffff, dw->fxleft, dw->fxright, (int32_t) dw->ydst, dw->ydst_lin,
               dw->ar[0], dw->ar[1], dw->ar[2], dw->ar[3], dw->ar[4], dw->ar[5], dw->ar[6], dw->sgn,
               dw->dr[0], dw->dr[4] & 0xffffff, dw->dr[8] & 0xffffff, dw->dr[12] & 0xffffff, dw->fcol, dw->plnwt, dw->ydstorg);

    if (athena->busy) {
        athena_log(athena, LOG_SUMMARY, "Athena: new operation while op %X still waits for host data\n", athena->busy_op);
        athena->busy = 0;
    }

    switch (opc) {
        case OPC_LINE_OPEN:
            athena_draw_line(athena, 0, 0);
            break;
        case OPC_AUTOLINE_OPEN:
            athena_draw_line(athena, 0, 1);
            break;
        case OPC_LINE_CLOSE:
            athena_draw_line(athena, 1, 0);
            break;
        case OPC_AUTOLINE_CLOSE:
            athena_draw_line(athena, 1, 1);
            break;
        case OPC_TRAP:
            athena_draw_trap(athena);
            break;
        case OPC_BITBLT:
            athena_draw_bitblt(athena);
            break;
        case OPC_ILOAD:
            athena_iload_start(athena);
            break;
        case OPC_IDUMP:
            athena_idump_start(athena);
            break;
        default:
            athena_log(athena, LOG_SUMMARY, "Athena: reserved opcode %X (dwgctl %08X)\n", opc, dw->dwgctl);
            break;
    }
}


static void
athena_set_ydst(athena_t *athena, uint32_t val)
{
    athena_dwg_t *dw = &athena->dw;

    dw->ydst = val & 0xffffff;
    if (dw->pitch & 0x8000) {
        /* Linear format: Y x pitch >> 5. */
        dw->ydst_lin = (sext24(dw->ydst) << 5) + dw->ydstorg;
        dw->selline  = (val >> 29) & 7;
    } else {
        dw->ydst_lin = (int32_t) (int16_t) dw->ydst * (int32_t) (dw->pitch & 0x1fe0) + dw->ydstorg;
        dw->selline  = dw->ydst & 7;
    }
}

static void
athena_dwg_write(athena_t *athena, uint32_t addr, uint32_t val)
{
    athena_dwg_t *dw    = &athena->dw;
    int           go    = !!(addr & REG_GO);
    uint32_t      reg   = (addr & ~REG_GO) & 0x1ffc;

    athena_log(athena, LOG_ALL, "Athena: DWG[%04X] = %08X%s\n", reg, val, go ? " GO" : "");

    switch (reg) {
        case REG_DWGCTL:
            dw->dwgctl = val;
            break;
        case REG_MACCESS:
            dw->maccess = val;
            break;
        case REG_MCTLWTST:
            dw->mctlwtst = val;
            break;
        case REG_ZMSK:
            dw->zmsk = val;
            break;
        case REG_PLNWT:
            dw->plnwt = val;
            break;
        case REG_BCOL:
            dw->bcol = val;
            break;
        case REG_FCOL:
            dw->fcol = val;
            break;
        case REG_SRCBLT:
            athena_iload_data(athena, val);
            break;
        case REG_SRC0:
        case REG_SRC1:
        case REG_SRC2:
        case REG_SRC3:
            dw->src[(reg - REG_SRC0) >> 2] = val;
            break;
        case REG_XYSTRT:
            dw->xdst  = (int16_t) (val & 0xffff);
            dw->ar[5] = (int16_t) (val & 0xffff);
            dw->ar[6] = (int16_t) (val >> 16);
            dw->ydst  = (uint32_t) (int32_t) (int16_t) (val >> 16) & 0xffffff;
            dw->ydst_lin = (int32_t) (int16_t) (val >> 16) * (int32_t) (dw->pitch & 0x1fe0) + dw->ydstorg;
            dw->selline  = (val >> 16) & 7;
            break;
        case REG_XYEND:
            dw->ar[0] = (int16_t) (val & 0xffff);
            dw->ar[2] = (int16_t) (val >> 16);
            break;
        case REG_SHIFT:
            dw->shift  = val;
            dw->funcnt = val & 0x7f;
            break;
        case REG_SGN:
            dw->sgn = val & 0x27;
            break;
        case REG_LEN:
            dw->len = val & 0xffff;
            break;
        case REG_AR0:
            dw->ar[0] = sext18(val);
            break;
        case REG_AR1:
            dw->ar[1] = sext24(val);
            break;
        case REG_AR2:
            dw->ar[2] = sext18(val);
            break;
        case REG_AR3:
            dw->ar[3] = sext24(val);
            dw->ar[1] = dw->ar[3];
            break;
        case REG_AR4:
            dw->ar[4] = sext18(val);
            break;
        case REG_AR5:
            dw->ar[5] = sext18(val);
            break;
        case REG_AR6:
            dw->ar[6] = sext18(val);
            break;
        case REG_PITCH:
            dw->pitch = val & 0x9fff;
            break;
        case REG_YDST:
            athena_set_ydst(athena, val);
            break;
        case REG_YDSTORG:
            dw->ydstorg = val & 0x7ffffff;
            break;
        case REG_CYTOP:
            dw->cytop = val & 0x7ffffff;
            break;
        case REG_CYBOT:
            dw->cybot = val & 0x7ffffff;
            break;
        case REG_CXLEFT:
            dw->cxleft = val & 0x1fff;
            break;
        case REG_CXRIGHT:
            dw->cxright = val & 0x1fff;
            break;
        case REG_FXLEFT:
            dw->fxleft = (int16_t) (val & 0xffff);
            break;
        case REG_FXRIGHT:
            dw->fxright = (int16_t) (val & 0xffff);
            break;
        case REG_XDST:
            dw->xdst = (int16_t) (val & 0xffff);
            break;
        default:
            if ((reg >= REG_DR0) && (reg <= (REG_DR0 + 0x3c))) {
                int n = (reg - REG_DR0) >> 2;

                if (n < 4)
                    dw->dr[n] = (int32_t) val;
                else
                    dw->dr[n] = sext24(val);
            } else
                athena_log(athena, LOG_SUMMARY, "Athena: write to unknown drawing register %04X = %08X\n", reg, val);
            break;
    }

    if (go)
        athena_start(athena);
}

static uint32_t
athena_dwg_read(athena_t *athena, uint32_t reg)
{
    reg &= 0x1ffc & ~REG_GO;

    /* The drawing registers are write-only: a read of any of them returns
       DST0 or DST1 (address bit 2), the 64-bit destination latch. Until the
       engine has run it holds the board straps; on a real card it then
       holds the last destination pixels (for example 7C1F7C1F). */
    uint64_t latch = athena->dw.dst_valid ? athena->dw.dst_latch : athena->straps;

    if ((reg != REG_DST0) && (reg != REG_DST1))
        athena_log(athena, LOG_REGS, "Athena: read of drawing register %04X\n", reg);
    return (reg & 4) ? (uint32_t) (latch >> 32) : (uint32_t) latch;
}

/* ------------------------------------------------------------------ */
/* Pseudo-DMA                                                          */
/* ------------------------------------------------------------------ */

static void
athena_pdma_write(athena_t *athena, uint32_t val)
{
    switch ((athena->opmode & OPM_DMAMOD_MASK) >> 2) {
        case DMAMOD_GENERAL:
            if (athena->adrgen_state == 0) {
                athena->adrgen_word  = val;
                athena->adrgen_state = 1;
            } else {
                uint32_t a = (athena->adrgen_word >> ((athena->adrgen_state - 1) * 8)) & 0x7f;

                athena_dwg_write(athena, 0x1c00 + (a << 2), val);
                athena->adrgen_state = (athena->adrgen_state + 1) % 5;
            }
            break;

        case DMAMOD_BLIT:
            athena_iload_data(athena, val);
            break;

        case DMAMOD_VECTOR:
            if (athena->adrgen_state == 0) {
                athena->adrgen_word  = val;
                athena->adrgen_state = 1;
            } else {
                int bit = athena->adrgen_state - 1;

                if ((athena->adrgen_word >> bit) & 1)
                    athena_dwg_write(athena, REG_XYEND | REG_GO, val);
                else
                    athena_dwg_write(athena, REG_XYSTRT, val);
                athena->adrgen_state = (athena->adrgen_state + 1) % 33;
            }
            break;

        default:
            athena_log(athena, LOG_SUMMARY, "Athena: pseudo-DMA write %08X in IDUMP mode\n", val);
            break;
    }
}

/* ------------------------------------------------------------------ */
/* Host registers                                                      */
/* ------------------------------------------------------------------ */

static uint8_t
athena_host_read_b(athena_t *athena, uint32_t addr)
{
    uint32_t reg   = 0x1e00 | (addr & 0x7c);   /* 1E80-1EFF mirrors 1E00-1E7F */
    int      shift = (addr & 3) * 8;
    uint32_t val   = 0;

    switch (reg) {
        case REG_VRAMPAGE:
        case REG_VRAMPAGE + 4:
            /* 1E04 is a second address for VRAMPAGE: a real board reads the
               same value at both, and the Windows drivers set the page for
               their font cache through 1E04. */
            val = athena->vrampage & 0xffffe0;
            break;
        case REG_FIFOSTATUS:
            /* addrgenstate 21h when idle (real board), 32 free locations,
               bus FIFO empty. */
            val = 0x21000220;
            break;
        case REG_STATUS:
            val = athena->istat & 7;
            if (athena->svga.cgastat & 8)
                val |= 8;
            if (athena->busy)
                val |= (1 << 16);
            break;
        case REG_IEN:
            val = athena->ien;
            break;
        case REG_RST:
            val = athena->rst;
            break;
        case REG_TEST:
            val = athena->test;
            break;
        case REG_REV:
            val = ATHENA_REV;
            break;
        case REG_CONFIG:
            val = athena->config;
            break;
        case REG_OPMODE:
            val = athena->opmode;
            break;
        case REG_CRTC_CTRL:
            val = athena->crtc_ctrl;
            break;
        case REG_VCOUNT:
            /* Reads 0 on a real board, in VGA and Power Graphic mode alike. */
            val = 0;
            break;
        case REG_BYTACCDATA:
            val = athena->acc_val;
            break;
        case REG_INTSTS:
            if (athena->is_pci) {
                /* tirqsts<0> is active low: 1, no interrupt (real board). */
                val = (athena->intsts & 0x102) | 1;
                break;
            }
            athena_log(athena, LOG_SUMMARY, "Athena: read of PCI-only INTSTS on a non-PCI board\n");
            break;
        case REG_ADRGEN:
            val = athena->adrgen_word;
            break;
        default:
            athena_log(athena, LOG_SUMMARY, "Athena: read of unknown host register %04X\n", addr);
            break;
    }

    return (val >> shift) & 0xff;
}

static void
athena_host_write_b(athena_t *athena, uint32_t addr, uint8_t val)
{
    uint32_t reg   = 0x1e00 | (addr & 0x7c);
    int      shift = (addr & 3) * 8;
    uint32_t mask  = 0xffu << shift;
    uint32_t v     = (uint32_t) val << shift;
    uint32_t old;

    switch (reg) {
        case REG_VRAMPAGE:
        case REG_VRAMPAGE + 4:
            athena->vrampage = ((athena->vrampage & ~mask) | v) & 0xffffe0;
            if (shift == 24)
                athena_log(athena, LOG_REGS, "Athena: VRAMPAGE %06X\n", athena->vrampage);
            break;
        case REG_ICLEAR:
            athena->istat &= ~(v & 7);
            break;
        case REG_INTSTS:
            if (athena->is_pci) {
                athena->intsts &= ~(v & 2);
                athena->intsts = (athena->intsts & ~(mask & 0x100)) | (v & 0x100);
            }
            break;
        case REG_IEN:
            athena->ien = ((athena->ien & ~mask) | v) & 0xf;
            if (athena->ien)
                athena_log(athena, LOG_SUMMARY, "Athena: interrupts enabled (IEN %X), not modelled\n", athena->ien);
            break;
        case REG_RST:
            if (!shift) {
                if ((athena->rst & 1) && !(val & 1)) {
                    athena->adrgen_state = 0;
                    athena->busy         = 0;
                }
                athena->rst = val & 1;
            }
            break;
        case REG_TEST:
            athena->test = (athena->test & 0x200) | (((athena->test & ~mask) | v) & 0x3);
            break;
        case REG_CONFIG:
            old = athena->config;
            /* config<1:0>, 200MHz, misc, nodubic<1:0>, biosen, vgaen, levelirq, expdev are
               writable (SXCI sets nodubic<0> on a real board, which reads 0A000310). */
            athena->config = (athena->config & ~(mask & 0x00010e3f)) | (v & 0x00010e3f);
            if ((old ^ athena->config) & (CFG_VGAEN | CFG_BIOSEN)) {
                athena_log(athena, LOG_SUMMARY, "Athena: CONFIG %08X (VGA %s)\n", athena->config,
                           (athena->config & CFG_VGAEN) ? "enabled" : "disabled");
                athena_recalc_mapping(athena);
                svga_recalctimings(&athena->svga);
            } else
                athena_log(athena, LOG_REGS, "Athena: CONFIG %08X\n", athena->config);
            break;
        case REG_OPMODE:
            old = athena->opmode;
            /* Bit 19 holds too: SXCI leaves 04A90011 on a real board. */
            athena->opmode = ((athena->opmode & ~mask) | v) & 0x0ffb0b1f;
            if ((old ^ athena->opmode) & (OPM_PSEUDODMA | OPM_DMAMOD_MASK))
                athena->adrgen_state = 0;
            if ((athena->opmode & OPM_DMAACT) && !(old & OPM_DMAACT))
                athena_log(athena, LOG_SUMMARY, "Athena: ISA DMA requested, not modelled\n");
            athena_log(athena, LOG_ALL, "Athena: OPMODE %08X\n", athena->opmode);
            break;
        case REG_CRTC_CTRL:
            old = athena->crtc_ctrl;
            athena->crtc_ctrl = (athena->crtc_ctrl & ~mask) | v;
            if (old != athena->crtc_ctrl) {
                athena_log(athena, LOG_REGS, "Athena: CRTC_CTRL %08X\n", athena->crtc_ctrl);
                svga_recalctimings(&athena->svga);
            }
            break;
        default:
            athena_log(athena, LOG_SUMMARY, "Athena: write to unknown host register %04X = %02X\n", addr, val);
            break;
    }
}

/* ------------------------------------------------------------------ */
/* External devices: RAMDAC, DUBIC, VIWIC, clock generator             */
/* ------------------------------------------------------------------ */

static void
athena_ext_write(athena_t *athena, uint32_t addr, uint8_t val)
{
    svga_t *svga = &athena->svga;

    if ((addr & 0x3f80) == 0x3c00) {
        int      reg = (addr >> 2) & 0xf;
        uint16_t port;

        if (addr & 3)
            return;
        port = (uint16_t[]) { 0x3c8, 0x3c9, 0x3c6, 0x3c7 }[reg & 3];
        athena_log(athena, LOG_ALL, "Athena: RAMDAC[%X] = %02X\n", reg, val);
        if (reg == 0)
            athena->dac_idx = val;
        tvp3026_ramdac_out(port, !!(reg & 4), !!(reg & 8), val, svga->ramdac, svga);
        return;
    }

    athena->ext[addr & 0x3ff] = val;
    athena_log(athena, LOG_REGS, "Athena: %s[%03X] = %02X\n",
               ((addr & 0x3f80) == 0x3c80) ? "DUBIC" : (((addr & 0x3f80) == 0x3d00) ? "VIWIC" : (((addr & 0x3f80) == 0x3d80) ? "CLKGEN" : "EXPDEV")),
               addr & 0x3ff, val);
}

static uint8_t
athena_ext_read(athena_t *athena, uint32_t addr)
{
    svga_t *svga = &athena->svga;
    uint8_t ret;

    if ((addr & 0x3f80) == 0x3c00) {
        int      reg = (addr >> 2) & 0xf;
        uint16_t port;

        if (addr & 3)
            return 0xff;
        port = (uint16_t[]) { 0x3c8, 0x3c9, 0x3c6, 0x3c7 }[reg & 3];
        ret  = tvp3026_ramdac_in(port, !!(reg & 4), !!(reg & 8), svga->ramdac, svga);
        if (reg == 0x0a) {
            /* Measured on a real TVP3026: the silicon revision reads 03,
               and an index with no register behind it returns whatever
               the previous RAMDAC read left on the bus. */
            static const uint64_t echo = 0x400003ff88ff7fbdull; /* 00 02-05 07-0E 10-17 1B 1F-29 3E */

            if (athena->dac_idx == 0x01)
                ret = 0x03;
            else if (athena->dac_idx == 0x2b)
                ret |= 0x1f;            /* GPIO pins float high on the board */
            else if (athena->dac_idx == 0x3a)
                ret = 0x00;             /* sense test: no comparator trip */
            else if ((athena->dac_idx < 0x40) && ((echo >> athena->dac_idx) & 1))
                ret = athena->dac_last;
        }
        athena->dac_last = ret;
        athena_log(athena, LOG_ALL, "Athena: RAMDAC[%X] read %02X\n", reg, ret);
        return ret;
    }

    /* Nothing is fitted there in No DUBIC mode. On a real board the DUBIC
       and VIWIC ranges (3C80-3D7F) float to FF and the rest reads 00. */
    ret = (addr < 0x3d80) ? 0xff : 0x00;
    athena_log(athena, LOG_REGS, "Athena: read of unfitted device at %04X\n", addr);
    return ret;
}

/* ------------------------------------------------------------------ */
/* Window access                                                       */
/* ------------------------------------------------------------------ */

static inline uint32_t
athena_vramwin_addr(const athena_t *athena, uint32_t off)
{
    return ((athena->vrampage & 0xffffe0) + (off & 0x1fff)) & athena->vram_mask;
}

static uint8_t athena_win_readb_int(uint32_t addr, void *priv);

static uint8_t
athena_win_readb(uint32_t addr, void *priv)
{
    athena_t *athena = (athena_t *) priv;
    uint8_t   ret    = athena_win_readb_int(addr, priv);

    athena_log(athena, LOG_ALL, "Athena: [%08X] rb %04X = %02X\n", cpu_state.pc, addr & 0x3fff, ret);
    return ret;
}

static uint8_t
athena_win_readb_int(uint32_t addr, void *priv)
{
    athena_t *athena = (athena_t *) priv;
    uint32_t  off    = addr & 0x3fff;

    if (off < 0x1c00) {
        if (athena->opmode & OPM_PSEUDODMA) {
            if ((off & 3) == 0)
                athena->acc_val = athena_idump_data(athena);
            return (athena->acc_val >> ((off & 3) * 8)) & 0xff;
        }
        return athena->svga.vram[athena_vramwin_addr(athena, off)];
    }
    if (off < 0x1e00)
        return (athena_dwg_read(athena, off) >> ((off & 3) * 8)) & 0xff;
    if (off < 0x1f00)
        return athena_host_read_b(athena, off);
    if (off < 0x2000)
        return athena_vga_in(0x300 | (off & 0xff), athena);
    if (off < 0x3c00) {
        /* The redundant window only shows VRAM with pseudo-DMA off; a real
           card reads 0 there while a game leaves pseudo-DMA on. */
        if (athena->opmode & OPM_PSEUDODMA)
            return 0x00;
        return athena->svga.vram[athena_vramwin_addr(athena, off - 0x2000)];
    }
    return athena_ext_read(athena, off);
}

static uint32_t
athena_win_readl(uint32_t addr, void *priv)
{
    athena_t *athena = (athena_t *) priv;
    uint32_t  off    = addr & 0x3ffc;

    if (off < 0x1c00) {
        if (athena->opmode & OPM_PSEUDODMA)
            return athena_idump_data(athena);
        return *(uint32_t *) &athena->svga.vram[athena_vramwin_addr(athena, off)];
    }
    if (off < 0x1e00) {
        uint32_t ret = athena_dwg_read(athena, off);

        athena_log(athena, LOG_ALL, "Athena: [%08X] rl %04X = %08X\n", cpu_state.pc, off, ret);
        return ret;
    }

    return athena_win_readb(addr, priv) | (athena_win_readb(addr + 1, priv) << 8) |
           (athena_win_readb(addr + 2, priv) << 16) | ((uint32_t) athena_win_readb(addr + 3, priv) << 24);
}

static uint16_t
athena_win_readw(uint32_t addr, void *priv)
{
    athena_t *athena = (athena_t *) priv;
    uint32_t  off    = addr & 0x3ffe;

    if ((off < 0x1c00) && (athena->opmode & OPM_PSEUDODMA)) {
        if ((off & 2) == 0)
            athena->acc_val = athena_idump_data(athena);
        return (athena->acc_val >> ((off & 2) * 8)) & 0xffff;
    }

    return athena_win_readb(addr, priv) | (athena_win_readb(addr + 1, priv) << 8);
}

/* Accumulate a dword that may arrive in byte or word pieces; returns 1 when
   the dword is complete (its top byte has been written). */
static int
athena_accumulate(athena_t *athena, uint32_t off, uint32_t val, int size)
{
    uint32_t dw_off = off & ~3;
    int      sh     = (off & 3) * 8;
    uint32_t mask   = (size == 1) ? 0xff : ((size == 2) ? 0xffff : 0xffffffff);

    if (athena->acc_addr != dw_off) {
        athena->acc_addr = dw_off;
        athena->acc_val  = 0;
    }
    athena->acc_val = (athena->acc_val & ~(mask << sh)) | ((val & mask) << sh);

    return ((off & 3) + size) >= 4;
}

static void
athena_win_write_common(athena_t *athena, uint32_t off, uint32_t val, int size)
{
    svga_t *svga = &athena->svga;

    if ((off >= 0x1c00) && ((off < 0x2000) || (off >= 0x3c00)))
        athena_log(athena, LOG_ALL, "Athena: [%08X] w%d %04X = %0*X\n", cpu_state.pc, size, off, size * 2, val);

    if (off < 0x1c00) {
        if (athena->opmode & OPM_PSEUDODMA) {
            if (athena_accumulate(athena, off, val, size)) {
                athena_pdma_write(athena, athena->acc_val);
                athena->acc_addr = 0xffffffff;
            }
            return;
        }
        for (int i = 0; i < size; i++) {
            uint32_t a = athena_vramwin_addr(athena, off + i);

            svga->vram[a]               = (val >> (i * 8)) & 0xff;
            svga->changedvram[a >> 12]  = changeframecount;
        }
        return;
    }

    if (off < 0x1e00) {
        if (athena_accumulate(athena, off, val, size)) {
            athena->adrgen_state = 0;
            athena_dwg_write(athena, off & ~3, athena->acc_val);
            athena->acc_addr = 0xffffffff;
        }
        return;
    }

    for (int i = 0; i < size; i++) {
        uint32_t o = off + i;
        uint8_t  b = (val >> (i * 8)) & 0xff;

        if (o < 0x1f00)
            athena_host_write_b(athena, o, b);
        else if (o < 0x2000)
            athena_vga_out(0x300 | (o & 0xff), b, athena);
        else if (o < 0x3c00) {
            uint32_t a = athena_vramwin_addr(athena, o - 0x2000);

            if (athena->opmode & OPM_PSEUDODMA)
                continue;
            svga->vram[a]              = b;
            svga->changedvram[a >> 12] = changeframecount;
        } else
            athena_ext_write(athena, o, b);
    }
}

static void
athena_win_writeb(uint32_t addr, uint8_t val, void *priv)
{
    athena_win_write_common((athena_t *) priv, addr & 0x3fff, val, 1);
}

static void
athena_win_writew(uint32_t addr, uint16_t val, void *priv)
{
    athena_win_write_common((athena_t *) priv, addr & 0x3fff, val, 2);
}

static void
athena_win_writel(uint32_t addr, uint32_t val, void *priv)
{
    athena_win_write_common((athena_t *) priv, addr & 0x3fff, val, 4);
}

/* ------------------------------------------------------------------ */
/* Device                                                              */
/* ------------------------------------------------------------------ */

static void *
athena_init(const device_t *info)
{
    athena_t *athena = calloc(1, sizeof(athena_t));
    int       mapsel;
    int       product;

    athena->is_vlb    = !!(info->flags & DEVICE_VLB);
    athena->is_pci    = !!(info->flags & DEVICE_PCI);
    athena->vram_size = device_get_config_int("memory") << 20;
    athena->vram_mask = athena->vram_size - 1;
    athena->log_level = device_get_config_int("diag_log");

    switch (athena->is_pci ? 0 : device_get_config_hex20("base")) {
        case 0xc8000:
            mapsel = 2;
            break;
        case 0xcc000:
            mapsel = 3;
            break;
        case 0xd0000:
            mapsel = 4;
            break;
        case 0xd4000:
            mapsel = 5;
            break;
        case 0xd8000:
            mapsel = 6;
            break;
        case 0xdc000:
            mapsel = 7;
            break;
        default:
            mapsel = 1;
            break;
    }

    /* Product ID nibble: 11xx ISA, 101x VL-Bus, 0110 PCI. */
    if (athena->is_pci) {
        product = 0x6;
        mapsel  = 0;
    } else {
        product = athena->is_vlb ? 0xa : 0xc;
        product |= device_get_config_int("product_low") & (athena->is_vlb ? 1 : 3);
    }

    if (athena->is_pci) {
        int board = device_get_config_int("board");

        athena->straps = athena_build_straps(athena, mapsel, product, board & 7, board >> 4);
    } else
        athena->straps = athena_build_straps(athena, mapsel, product, device_get_config_int("pcb_rev"), 0);

    rom_init(&athena->bios_rom,
             (athena->is_pci && device_get_config_int("bios_v40")) ? ROM_IMPRESSION_PLUS_V40 : ROM_IMPRESSION_PLUS,
             0xc0000, 0x8000, 0x7fff, 0, MEM_MAPPING_EXTERNAL);
    if (athena->is_pci)
        mem_mapping_disable(&athena->bios_rom.mapping); /* placed by ROMBASE */

    video_inform(VIDEO_FLAG_TYPE_SPECIAL, athena->is_pci ? &timing_athena_pci : (athena->is_vlb ? &timing_athena_vlb : &timing_athena_isa));

    svga_init(info, &athena->svga, athena, athena->vram_size,
              athena_recalctimings, athena_in, athena_out, NULL, NULL);

    athena->svga.dac_hwcursor_draw = tvp3026_hwcursor_draw;
    athena->svga.ramdac            = device_add(&tvp3026_ramdac_device);
    athena->svga.clock_gen         = athena->svga.ramdac;
    athena->svga.getclock          = tvp3026_getclock;
    athena->svga.conv_16to32       = tvp3026_conv_16to32;
    athena->svga.decode_mask       = athena->vram_mask;
    athena->svga.vram_max          = athena->vram_size;
    athena->svga.vram_display_mask = athena->vram_mask;

    io_sethandler(0x03a0, 0x0040, athena_in, NULL, NULL, athena_out, NULL, NULL, athena);
    if (!athena->is_pci)
        io_sethandler(0x46e8, 0x0001, athena_46e8_in, NULL, NULL, athena_46e8_out, NULL, NULL, athena);

    mem_mapping_add(&athena->win_mapping, 0, 0,
                    athena_win_readb, athena_win_readw, athena_win_readl,
                    athena_win_writeb, athena_win_writew, athena_win_writel,
                    NULL, MEM_MAPPING_EXTERNAL, athena);

    /* ZTAG: one tag bit per pixel address (the depth itself is in VRAM). */
    athena->zmask = (athena->vram_size >> 1) - 1;
    athena->ztag  = calloc((athena->zmask + 1) >> 3, 1);

    athena->acc_addr = 0xffffffff;
    athena_hard_reset(athena);

    if (athena->is_pci) {
        athena->int_line = 0xff;
        pci_add_card(PCI_ADD_NORMAL, athena_pci_read, athena_pci_write, athena, &athena->pci_slot);
    }

    athena_recalc_mapping(athena);

    athena_log(athena, LOG_SUMMARY, "Athena: %s, %d MB, window %s%05X, VD straps %08X%08X\n",
               info->name, athena->vram_size >> 20, athena->is_pci ? "PCI BAR / " : "", mapsel_base[mapsel],
               (uint32_t) (athena->straps >> 32), (uint32_t) athena->straps);

    return athena;
}

static int
athena_available(void)
{
    return rom_present(ROM_IMPRESSION_PLUS);
}

static void
athena_close(void *priv)
{
    athena_t *athena = (athena_t *) priv;

    svga_close(&athena->svga);
    free(athena->ztag);
    free(athena);
}

static void
athena_speed_changed(void *priv)
{
    athena_t *athena = (athena_t *) priv;

    svga_recalctimings(&athena->svga);
}

static void
athena_force_redraw(void *priv)
{
    athena_t *athena = (athena_t *) priv;

    athena->svga.fullchange = changeframecount;
}

static const device_config_t athena_config[] = {
  // clang-format off
    {
        .name           = "memory",
        .description    = "Memory size",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 2,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "2 MB", .value = 2 },
            { .description = "4 MB", .value = 4 },
            { .description = ""                 }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "base",
        .description    = "Power Graphic window",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0xac000,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "AC000h", .value = 0xac000 },
            { .description = "C8000h", .value = 0xc8000 },
            { .description = "CC000h", .value = 0xcc000 },
            { .description = "D0000h", .value = 0xd0000 },
            { .description = "D4000h", .value = 0xd4000 },
            { .description = "D8000h", .value = 0xd8000 },
            { .description = "DC000h", .value = 0xdc000 },
            { .description = ""                         }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "pcb_rev",
        .description    = "PCB revision",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 2,
        .file_filter    = NULL,
        .spinner        = { .min = 0, .max = 7, .step = 1 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "product_low",
        .description    = "Product ID low bits strap",
        .type           = CONFIG_SPINNER,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { .min = 0, .max = 3, .step = 1 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    {
        .name           = "diag_log",
        .description    = "Diagnostic log",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = LOG_OFF,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Off",                  .value = LOG_OFF     },
            { .description = "Summary",              .value = LOG_SUMMARY },
            { .description = "Registers",            .value = LOG_REGS    },
            { .description = "Everything (slow)",    .value = LOG_ALL     },
            { .description = ""                                           }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

static const device_config_t athena_pci_config[] = {
  // clang-format off
    {
        .name           = "memory",
        .description    = "Memory size",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 2,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "2 MB", .value = 2 },
            { .description = "4 MB", .value = 4 },
            { .description = ""                 }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "bios_v40",
        .description    = "Video BIOS",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 1,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "V4.0 (IS-ATHENA R2 PCI)", .value = 1 },
            { .description = "V3.6p",                   .value = 0 },
            { .description = ""                                    }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "board",
        .description    = "Board",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 3,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "553-03 Rev A (PCB strap 3)",     .value = 3        },
            { .description = "Revision 2 board (PCB strap 2)", .value = 2 | 0x10 },
            { .description = ""                                                  }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "diag_log",
        .description    = "Diagnostic log",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = LOG_OFF,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Off",                  .value = LOG_OFF     },
            { .description = "Summary",              .value = LOG_SUMMARY },
            { .description = "Registers",            .value = LOG_REGS    },
            { .description = "Everything (slow)",    .value = LOG_ALL     },
            { .description = ""                                           }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

const device_t mga_impression_plus_isa_device = {
    .name          = "Matrox MGA Impression Plus (ISA)",
    .internal_name = "mga_impression_plus_isa",
    .flags         = DEVICE_ISA16,
    .local         = 0,
    .init          = athena_init,
    .close         = athena_close,
    .reset         = NULL,
    .available     = athena_available,
    .speed_changed = athena_speed_changed,
    .force_redraw  = athena_force_redraw,
    .config        = athena_config
};

const device_t mga_impression_plus_vlb_device = {
    .name          = "Matrox MGA Impression Plus (VLB)",
    .internal_name = "mga_impression_plus_vlb",
    .flags         = DEVICE_VLB,
    .local         = 0,
    .init          = athena_init,
    .close         = athena_close,
    .reset         = NULL,
    .available     = athena_available,
    .speed_changed = athena_speed_changed,
    .force_redraw  = athena_force_redraw,
    .config        = athena_config
};

const device_t mga_impression_plus_pci_device = {
    .name          = "Matrox MGA Impression Plus (PCI)",
    .internal_name = "mga_impression_plus_pci",
    .flags         = DEVICE_PCI,
    .local         = 0,
    .init          = athena_init,
    .close         = athena_close,
    .reset         = NULL,
    .available     = athena_available,
    .speed_changed = athena_speed_changed,
    .force_redraw  = athena_force_redraw,
    .config        = athena_pci_config
};
