/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2026  nextpnr contributors
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <vector>
#include "command.h"
#include "gtest/gtest.h"
#include "nextpnr.h"
#include "uarch/xilinx/pack.h"
#include "uarch/xilinx/xilinx.h"
#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

USING_NEXTPNR_NAMESPACE

class XilinxPackTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
        init_share_dirname();
        chipArgs.device = "xc7a50tcsg324-1";
        ctx = new Context(chipArgs);
        ctx->uarch->init(ctx);
        ctx->late_init();
        xil = reinterpret_cast<XilinxImpl *>(ctx->uarch.get());
    }

    virtual void TearDown() { delete ctx; }

    ArchArgs chipArgs;
    Context *ctx;
    XilinxImpl *xil;
};

TEST_F(XilinxPackTest, pack_cfg_startupe2)
{
    // STARTUPE2 must pack to STARTUP_STARTUP and preplace to its single
    // dedicated site (WP6)
    CellInfo *ci = ctx->createCell(ctx->id("startup"), id_STARTUPE2);
    for (auto p : {"CLK", "GSR", "GTS", "KEYCLEARB", "PACK", "USRCCLKO", "USRCCLKTS", "USRDONEO", "USRDONETS"})
        ci->addInput(ctx->id(p));
    for (auto p : {"CFGCLK", "CFGMCLK", "EOS", "PREQ"})
        ci->addOutput(ctx->id(p));

    XC7Packer p(ctx, xil);
    p.pack_cfg();

    EXPECT_EQ(ci->type, id_STARTUP_STARTUP);
    EXPECT_NE(ci->bel, BelId());
    EXPECT_EQ(ctx->getBelType(ci->bel), id_STARTUP_STARTUP);
}

TEST_F(XilinxPackTest, pack_cfg_bscane2_chain)
{
    // BSCANE2 packs to BSCAN, validates JTAG_CHAIN, and is bound to the site
    // its chain selects: USER<n> is served by BSCAN_X0Y<n-1>, so an instance
    // left on another BSCAN site answers another chain's user register (WP6).
    CellInfo *ci = ctx->createCell(ctx->id("bscan"), id_BSCANE2);
    ci->params[id_JTAG_CHAIN] = Property(2);
    XC7Packer p(ctx, xil);
    p.pack_cfg();
    EXPECT_EQ(ci->type, id_BSCAN);
    ASSERT_NE(ci->bel, BelId());
    EXPECT_EQ(xil->get_site_name(xil->get_bel_site(ci->bel)), ctx->id("BSCAN_X0Y1"));
}

TEST_F(XilinxPackTest, pack_cfg_bscane2_two_chains_do_not_share_a_site)
{
    // The design that showed the bug: a JTAG clock controller on one chain
    // and a logic analyser on another.  Each instance must get the site
    // serving its own chain, not whichever BSCAN site is free first.
    CellInfo *user3 = ctx->createCell(ctx->id("user3"), id_BSCANE2);
    user3->params[id_JTAG_CHAIN] = Property(3);
    CellInfo *user1 = ctx->createCell(ctx->id("user1"), id_BSCANE2);
    user1->params[id_JTAG_CHAIN] = Property(1);

    XC7Packer p(ctx, xil);
    p.pack_cfg();

    ASSERT_NE(user3->bel, BelId());
    ASSERT_NE(user1->bel, BelId());
    EXPECT_EQ(xil->get_site_name(xil->get_bel_site(user3->bel)), ctx->id("BSCAN_X0Y2"));
    EXPECT_EQ(xil->get_site_name(xil->get_bel_site(user1->bel)), ctx->id("BSCAN_X0Y0"));
}

TEST_F(XilinxPackTest, pack_cfg_bscane2_duplicate_chain_is_an_error)
{
    // Only one instance can serve a chain -- the chain's site has one BSCAN
    // bel -- so a second instance asking for the same chain is reported
    // rather than quietly bound to another chain's site.
    CellInfo *first = ctx->createCell(ctx->id("first"), id_BSCANE2);
    first->params[id_JTAG_CHAIN] = Property(2);
    CellInfo *second = ctx->createCell(ctx->id("second"), id_BSCANE2);
    second->params[id_JTAG_CHAIN] = Property(2);

    XC7Packer p(ctx, xil);
    EXPECT_THROW(p.pack_cfg(), log_execution_error_exception);

    // Whichever instance got there first holds the site serving USER2.
    CellInfo *placed = first->bel != BelId() ? first : second;
    ASSERT_NE(placed->bel, BelId());
    EXPECT_EQ(xil->get_site_name(xil->get_bel_site(placed->bel)), ctx->id("BSCAN_X0Y1"));
}

TEST_F(XilinxPackTest, clocking_bufh_bufhce)
{
    // BUFH and BUFHCE pack to BUFHCE_BUFHCE with CE tied active (WP3.2)
    XC7Packer p(ctx, xil);
    p.pack_constants();

    CellInfo *bufh = ctx->createCell(ctx->id("bufh"), id_BUFH);
    bufh->addInput(id_I);
    bufh->addOutput(id_O);
    CellInfo *bufhce = ctx->createCell(ctx->id("bufhce"), id_BUFHCE);
    bufhce->addInput(id_I);
    bufhce->addInput(id_CE);
    bufhce->addOutput(id_O);

    p.prepare_clocking();

    EXPECT_EQ(bufh->type, id_BUFHCE_BUFHCE);
    EXPECT_EQ(bufhce->type, id_BUFHCE_BUFHCE);
    for (auto ci : {bufh, bufhce}) {
        NetInfo *ce = ci->getPort(id_CE);
        EXPECT_NE(ce, nullptr);
        EXPECT_EQ(ce->name, ctx->id("$PACKER_VCC_NET"));
    }
}

TEST_F(XilinxPackTest, clocking_bufr)
{
    // BUFR packs to BUFR_BUFR (WP2.6)
    CellInfo *ci = ctx->createCell(ctx->id("bufr"), id_BUFR);
    ci->addInput(id_I);
    ci->addInput(id_CE);
    ci->addInput(id_CLR);
    ci->addOutput(id_O);
    XC7Packer p(ctx, xil);
    p.prepare_clocking();
    EXPECT_EQ(ci->type, id_BUFR_BUFR);
}

TEST_F(XilinxPackTest, cells_ibufgds_alias)
{
    // IBUFGDS is accepted as an alias of IBUFDS (WP3.8)
    XilinxPacker p(ctx, xil);
    CellInfo *ci = p.create_cell(id_IBUFGDS, ctx->id("ibufgds"));
    EXPECT_NE(ci, nullptr);
    EXPECT_NE(ci->ports.count(id_I), size_t(0));
    EXPECT_NE(ci->ports.count(id_IB), size_t(0));
    EXPECT_NE(ci->ports.count(id_O), size_t(0));
}

// Helper: create an SRLC32E with the ports a real (yosys) design carries.
static CellInfo *create_srlc32e(Context *ctx, const char *name)
{
    CellInfo *ci = ctx->createCell(ctx->id(name), id_SRLC32E);
    for (auto p : {id_A0, id_A1, id_A2, id_A3, id_A4})
        ci->addInput(p);
    ci->addInput(id_D);
    ci->addInput(id_CLK);
    ci->addInput(id_CE);
    ci->addOutput(id_Q);
    ci->addOutput(id_Q31);
    return ci;
}

TEST_F(XilinxPackTest, pack_srl_cascade_pair)
{
    // A two-deep SRLC32E chain (Q31 -> next D) must pack to SLICE_LUTX and be
    // constrained into one slice, head at D6LUT and member at C6LUT, exactly
    // like a carry chain (the MC31 cascade arc exists only inside a SLICEM).
    XC7Packer p(ctx, xil);
    p.pack_constants();

    CellInfo *srl1 = create_srlc32e(ctx, "srl1");
    CellInfo *srl2 = create_srlc32e(ctx, "srl2");
    NetInfo *q31 = ctx->createNet(ctx->id("q31net"));
    srl1->connectPort(id_Q31, q31);
    srl2->connectPort(id_D, q31);

    p.pack_srls();
    p.constrain_srl_cascades();

    EXPECT_EQ(srl1->type, id_SLICE_LUTX);
    EXPECT_EQ(srl2->type, id_SLICE_LUTX);
    EXPECT_EQ(str_or_default(srl1->attrs, id_X_ORIG_TYPE), "SRLC32E");

    EXPECT_TRUE(srl1->constr_abs_z);
    EXPECT_EQ(srl1->constr_z, (3 << 4) | BEL_6LUT);
    EXPECT_EQ(srl1->cluster, srl1->name);
    EXPECT_EQ(srl1->constr_children.size(), size_t(1));
    EXPECT_EQ(srl1->constr_children.at(0), srl2);

    EXPECT_EQ(srl2->cluster, srl1->name);
    EXPECT_TRUE(srl2->constr_abs_z);
    EXPECT_EQ(srl2->constr_z, (2 << 4) | BEL_6LUT);
}

TEST_F(XilinxPackTest, pack_srl_cascade_offslice)
{
    // A five-deep chain cannot fit in one slice: the link between the 4th and
    // 5th element leaves the slice. Q has no users, so the link moves onto Q
    // with the read address tied to 31, instead of leaving the A LUT through DOUTMUX.
    XC7Packer p(ctx, xil);
    p.pack_constants();

    CellInfo *srls[5];
    NetInfo *links[4];
    for (int i = 0; i < 5; i++) {
        srls[i] = create_srlc32e(ctx, stringf("srl%d", i + 1).c_str());
        if (i > 0)
            srls[i]->connectPort(id_D, links[i - 1]);
        if (i < 4) {
            links[i] = ctx->createNet(ctx->idf("link%d", i));
            srls[i]->connectPort(id_Q31, links[i]);
        }
    }

    p.pack_srls();
    p.constrain_srl_cascades();

    // Elements 1..4 cluster into one slice (D,C,B,A); element 5 does not.
    for (int i = 0; i < 4; i++) {
        EXPECT_EQ(srls[i]->cluster, srls[0]->name);
        EXPECT_EQ(srls[i]->constr_z, ((3 - i) << 4) | BEL_6LUT);
    }
    EXPECT_EQ(srls[4]->cluster, ClusterId());

    // The 4th element's Q31 is rewired onto Q, with the address tied to 31.
    EXPECT_EQ(srls[3]->getPort(id_MC31), nullptr);
    EXPECT_EQ(srls[3]->getPort(id_O6), links[3]);
    EXPECT_EQ(srls[4]->getPort(id_DI1), links[3]);
    for (auto a : {id_A2, id_A3, id_A4, id_A5, id_A6})
        EXPECT_EQ(srls[3]->getPort(a), ctx->nets.at(ctx->id("$PACKER_VCC_NET")).get());
}

class XilinxPackTestK325t : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
        init_share_dirname();
        chipArgs.device = "xc7k325tffg676-1";
        ctx = new Context(chipArgs);
        ctx->uarch->init(ctx);
        ctx->late_init();
        xil = reinterpret_cast<XilinxImpl *>(ctx->uarch.get());
    }

    virtual void TearDown() { delete ctx; }

    ArchArgs chipArgs;
    Context *ctx;
    XilinxImpl *xil;
};

TEST_F(XilinxPackTestK325t, pack_io_pcie_retype_preplace)
{
    // PCIE_2_1 must retype to PCIE_2_1_PCIE_2_1 in pack_io and then bind to
    // the unique PCIE hard-block site in pack_gbs (mirrors PS7 handling).
    CellInfo *ci = ctx->createCell(ctx->id("pcie"), id_PCIE_2_1);
    XC7Packer p(ctx, xil);
    p.pack_io();
    EXPECT_EQ(ci->type, id_PCIE_2_1_PCIE_2_1);
    p.pack_gbs();
    EXPECT_NE(ci->bel, BelId());
    EXPECT_EQ(ctx->getBelType(ci->bel), id_PCIE_2_1_PCIE_2_1);
}

// Helper: a RAMB36E1 as yosys emits it for a single-port-width memory.
static CellInfo *create_ramb36e1(Context *ctx, const char *name, int write_width_a, int read_width_b)
{
    CellInfo *ci = ctx->createCell(ctx->id(name), id_RAMB36E1);
    for (int i = 0; i < 16; i++) {
        ci->addInput(ctx->idf("ADDRARDADDR[%d]", i));
        ci->addInput(ctx->idf("ADDRBWRADDR[%d]", i));
    }
    for (int i = 0; i < 32; i++) {
        ci->addInput(ctx->idf("DIADI[%d]", i));
        ci->addInput(ctx->idf("DIBDI[%d]", i));
        ci->addOutput(ctx->idf("DOADO[%d]", i));
        ci->addOutput(ctx->idf("DOBDO[%d]", i));
    }
    for (int i = 0; i < 4; i++) {
        ci->addInput(ctx->idf("DIPADIP[%d]", i));
        ci->addInput(ctx->idf("DIPBDIP[%d]", i));
        ci->addOutput(ctx->idf("DOPADOP[%d]", i));
        ci->addOutput(ctx->idf("DOPBDOP[%d]", i));
        ci->addInput(ctx->idf("WEA[%d]", i));
    }
    for (int i = 0; i < 8; i++)
        ci->addInput(ctx->idf("WEBWE[%d]", i));
    for (auto p : {"CLKARDCLK", "CLKBWRCLK", "ENARDEN", "ENBWREN", "REGCEAREGCE", "REGCEB", "RSTRAMARSTRAM",
                   "RSTRAMB", "RSTREGARSTREG", "RSTREGB"})
        ci->addInput(ctx->id(p));
    ci->params[ctx->id("RAM_MODE")] = Property("TDP");
    ci->params[id_WRITE_WIDTH_A] = Property(write_width_a);
    ci->params[ctx->id("READ_WIDTH_B")] = Property(read_width_b);
    return ci;
}

TEST_F(XilinxPackTest, pack_bram_ramb36_width9_parity_on_both_halves)
{
    // At width 9 a RAMB36E1 keeps the ninth bit of a word in the lower
    // RAMB18 at even addresses and in the upper one at odd addresses, and the
    // upper half reads it from DIPADIP1. Left unconnected (yosys drives only
    // DIPADIP[0]), every odd address reads its parity bit back as 0.
    CellInfo *ci = create_ramb36e1(ctx, "ram_x9", 9, 9);
    NetInfo *parity = ctx->createNet(ctx->id("parity"));
    ci->connectPort(ctx->id("DIPADIP[0]"), parity);

    XC7Packer p(ctx, xil);
    p.pack_bram();

    EXPECT_EQ(ci->type, id_RAMB36E1_RAMB36E1);
    EXPECT_EQ(ci->getPort(id_DIPADIP0), parity);
    EXPECT_EQ(ci->getPort(id_DIPADIP1), parity);
}

TEST_F(XilinxPackTest, pack_bram_ramb36_width18_parity_untouched)
{
    // At width 18 the two parity pins carry two different bits of the word.
    CellInfo *ci = create_ramb36e1(ctx, "ram_x18", 18, 18);
    NetInfo *p0 = ctx->createNet(ctx->id("p0"));
    NetInfo *p1 = ctx->createNet(ctx->id("p1"));
    ci->connectPort(ctx->id("DIPADIP[0]"), p0);
    ci->connectPort(ctx->id("DIPADIP[1]"), p1);

    XC7Packer p(ctx, xil);
    p.pack_bram();

    EXPECT_EQ(ci->getPort(id_DIPADIP0), p0);
    EXPECT_EQ(ci->getPort(id_DIPADIP1), p1);
}
