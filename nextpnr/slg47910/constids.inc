// SLG47910 uarch constant IDs.
// Xilinx 7-series cell types from synth_xilinx.
X(LUT1) X(LUT2) X(LUT3) X(LUT4) X(LUT5) X(LUT6)
X(INV)
X(FDRE) X(FDSE) X(FDCE) X(FDPE)
X(CARRY4)
X(MUXF7) X(MUXF8)

// Distributed RAM / SRL cell types from synth_xilinx (without -nolutram).
X(RAM32X1D) X(RAM32M) X(SRL16E)

// Our bel / cell types.
X(SLG_LUT)
X(SLG_FF)
X(SLG_IOB)
X(SLG_CLKIN)
X(SLG_RAM)
X(SLG_RAM32X1D)
X(SLG_RAM32M)
// Phase 5: per-CLB mux_a7 / mux_c7 bels modelling the silicon's MUXF7
// network that feeds slice EXTRA_IN ports (clbl.v lines 105-108).
// Phase 5b: mux_x8 (SLG_MUX_F8) cascades the mux_a7 / mux_c7 outputs
// into slice B's EXTRA_IN.
X(SLG_MUX_F7) X(SLG_MUX_F8)

// Pin names.
X(CLK) X(D) X(Q) X(CE) X(R) X(S)
X(I0) X(I1) X(I2) X(I3) X(I4) X(I5)
X(O) X(F)
X(PAD)
X(OE)
X(INIT)

// Carry chain pins.
X(CIN) X(COUT) X(CYINIT)

// 8-FF model Phase 2: per-slice carry-out (C_BIT) and sum-out (S_BIT) bel
// outputs on each carry-mode SLG_LUT. The chipdb exposes these as additional
// LUT outputs so the XQ FF's MUX_XQ (AQ1_SEL) can capture the per-slice
// carry/sum directly without a fabric detour. See docs/silicon/reference.md
// §3.1 (MUX_XQ inputs) and §7.5 (AQ1_SEL encoding).
//
// EXTRA_IN is the MUX_XQ bit-0 source on slices A/B/C only (slice D drops
// it). It is fed by the CLB-level mux_a7 / mux_c7 / mux_x8 network, which
// is not yet modelled in the chipdb — the per-slice wire is created with
// no driver as a deliberate Phase 2 placeholder; the bit becomes reachable
// from the open flow only once the A7/C7/X8 mux network lands (Phase 3+).
X(CRY_C) X(CRY_S) X(EXTRA_IN)

X(IS_R_INVERTED) X(IS_S_INVERTED) X(IS_C_INVERTED) X(IS_D_INVERTED)

// 8-FF model: slot-binding parameter on SLG_FF cells. Value ∈ {"XQ", "OXQ", "ANY"}.
X(FF_SLOT)

// RAM32X1D ports.
X(WCLK) X(WE) X(SPO) X(DPO)
X(A0) X(A1) X(A2) X(A3) X(A4)
X(DPRA0) X(DPRA1) X(DPRA2) X(DPRA3) X(DPRA4)

// RAM32M ports (WCLK/WE reused from RAM32X1D).
// Address buses per port: ADDRA-D[4:0]; data: DIA-D[1:0], DOA-D[1:0].
X(ADDRA0) X(ADDRA1) X(ADDRA2) X(ADDRA3) X(ADDRA4)
X(ADDRB0) X(ADDRB1) X(ADDRB2) X(ADDRB3) X(ADDRB4)
X(ADDRC0) X(ADDRC1) X(ADDRC2) X(ADDRC3) X(ADDRC4)
X(ADDRD0) X(ADDRD1) X(ADDRD2) X(ADDRD3) X(ADDRD4)
X(DIA0) X(DIA1) X(DIB0) X(DIB1) X(DIC0) X(DIC1) X(DID0) X(DID1)
X(DOA0) X(DOA1) X(DOB0) X(DOB1) X(DOC0) X(DOC1) X(DOD0) X(DOD1)

// SRL16E ports (CLK reused, CE reused, D reused, Q reused).
// Address pins for SRL16E shift depth.
// A0-A3 already covered above.
