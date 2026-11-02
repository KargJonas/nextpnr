################################################################################
# IO constraints
################################################################################
# cpu_reset:0
set_property LOC AV40 [get_ports {cpu_reset}]
set_property IOSTANDARD LVCMOS18 [get_ports {cpu_reset}]

# clk200:0.p
set_property LOC E19 [get_ports {clk200_p}]
set_property IOSTANDARD LVDS [get_ports {clk200_p}]

# clk200:0.n
set_property LOC E18 [get_ports {clk200_n}]
set_property IOSTANDARD LVDS [get_ports {clk200_n}]

# serial:0.rx
set_property LOC AU33 [get_ports {serial_rx}]
set_property IOSTANDARD LVCMOS18 [get_ports {serial_rx}]

# serial:0.tx
set_property LOC AU36 [get_ports {serial_tx}]
set_property IOSTANDARD LVCMOS18 [get_ports {serial_tx}]

# ddram64:0.a
set_property LOC A20 [get_ports {ddram64_a[0]}]
set_property SLEW FAST [get_ports {ddram64_a[0]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[0]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[0]}]

# ddram64:0.a
set_property LOC B19 [get_ports {ddram64_a[1]}]
set_property SLEW FAST [get_ports {ddram64_a[1]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[1]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[1]}]

# ddram64:0.a
set_property LOC C20 [get_ports {ddram64_a[2]}]
set_property SLEW FAST [get_ports {ddram64_a[2]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[2]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[2]}]

# ddram64:0.a
set_property LOC A19 [get_ports {ddram64_a[3]}]
set_property SLEW FAST [get_ports {ddram64_a[3]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[3]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[3]}]

# ddram64:0.a
set_property LOC A17 [get_ports {ddram64_a[4]}]
set_property SLEW FAST [get_ports {ddram64_a[4]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[4]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[4]}]

# ddram64:0.a
set_property LOC A16 [get_ports {ddram64_a[5]}]
set_property SLEW FAST [get_ports {ddram64_a[5]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[5]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[5]}]

# ddram64:0.a
set_property LOC D20 [get_ports {ddram64_a[6]}]
set_property SLEW FAST [get_ports {ddram64_a[6]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[6]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[6]}]

# ddram64:0.a
set_property LOC C18 [get_ports {ddram64_a[7]}]
set_property SLEW FAST [get_ports {ddram64_a[7]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[7]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[7]}]

# ddram64:0.a
set_property LOC D17 [get_ports {ddram64_a[8]}]
set_property SLEW FAST [get_ports {ddram64_a[8]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[8]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[8]}]

# ddram64:0.a
set_property LOC C19 [get_ports {ddram64_a[9]}]
set_property SLEW FAST [get_ports {ddram64_a[9]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[9]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[9]}]

# ddram64:0.a
set_property LOC B21 [get_ports {ddram64_a[10]}]
set_property SLEW FAST [get_ports {ddram64_a[10]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[10]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[10]}]

# ddram64:0.a
set_property LOC B17 [get_ports {ddram64_a[11]}]
set_property SLEW FAST [get_ports {ddram64_a[11]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[11]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[11]}]

# ddram64:0.a
set_property LOC A15 [get_ports {ddram64_a[12]}]
set_property SLEW FAST [get_ports {ddram64_a[12]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[12]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[12]}]

# ddram64:0.a
set_property LOC A21 [get_ports {ddram64_a[13]}]
set_property SLEW FAST [get_ports {ddram64_a[13]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[13]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[13]}]

# ddram64:0.a
set_property LOC F17 [get_ports {ddram64_a[14]}]
set_property SLEW FAST [get_ports {ddram64_a[14]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[14]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[14]}]

# ddram64:0.a
set_property LOC E17 [get_ports {ddram64_a[15]}]
set_property SLEW FAST [get_ports {ddram64_a[15]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_a[15]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_a[15]}]

# ddram64:0.ba
set_property LOC D21 [get_ports {ddram64_ba[0]}]
set_property SLEW FAST [get_ports {ddram64_ba[0]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_ba[0]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_ba[0]}]

# ddram64:0.ba
set_property LOC C21 [get_ports {ddram64_ba[1]}]
set_property SLEW FAST [get_ports {ddram64_ba[1]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_ba[1]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_ba[1]}]

# ddram64:0.ba
set_property LOC D18 [get_ports {ddram64_ba[2]}]
set_property SLEW FAST [get_ports {ddram64_ba[2]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_ba[2]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_ba[2]}]

# ddram64:0.ras_n
set_property LOC E20 [get_ports {ddram64_ras_n}]
set_property SLEW FAST [get_ports {ddram64_ras_n}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_ras_n}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_ras_n}]

# ddram64:0.cas_n
set_property LOC K17 [get_ports {ddram64_cas_n}]
set_property SLEW FAST [get_ports {ddram64_cas_n}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_cas_n}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_cas_n}]

# ddram64:0.we_n
set_property LOC F20 [get_ports {ddram64_we_n}]
set_property SLEW FAST [get_ports {ddram64_we_n}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_we_n}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_we_n}]

# ddram64:0.cs_n
set_property LOC J17 [get_ports {ddram64_cs_n}]
set_property SLEW FAST [get_ports {ddram64_cs_n}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_cs_n}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_cs_n}]

# ddram64:0.dm
set_property LOC M13 [get_ports {ddram64_dm[0]}]
set_property SLEW FAST [get_ports {ddram64_dm[0]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dm[0]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_dm[0]}]

# ddram64:0.dm
set_property LOC K15 [get_ports {ddram64_dm[1]}]
set_property SLEW FAST [get_ports {ddram64_dm[1]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dm[1]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_dm[1]}]

# ddram64:0.dm
set_property LOC F12 [get_ports {ddram64_dm[2]}]
set_property SLEW FAST [get_ports {ddram64_dm[2]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dm[2]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_dm[2]}]

# ddram64:0.dm
set_property LOC A14 [get_ports {ddram64_dm[3]}]
set_property SLEW FAST [get_ports {ddram64_dm[3]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dm[3]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_dm[3]}]

# ddram64:0.dm
set_property LOC C23 [get_ports {ddram64_dm[4]}]
set_property SLEW FAST [get_ports {ddram64_dm[4]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dm[4]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_dm[4]}]

# ddram64:0.dm
set_property LOC D25 [get_ports {ddram64_dm[5]}]
set_property SLEW FAST [get_ports {ddram64_dm[5]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dm[5]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_dm[5]}]

# ddram64:0.dm
set_property LOC C31 [get_ports {ddram64_dm[6]}]
set_property SLEW FAST [get_ports {ddram64_dm[6]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dm[6]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_dm[6]}]

# ddram64:0.dm
set_property LOC F31 [get_ports {ddram64_dm[7]}]
set_property SLEW FAST [get_ports {ddram64_dm[7]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dm[7]}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_dm[7]}]

# ddram64:0.dq
set_property LOC N14 [get_ports {ddram64_dq[0]}]
set_property SLEW FAST [get_ports {ddram64_dq[0]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[0]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[0]}]

# ddram64:0.dq
set_property LOC N13 [get_ports {ddram64_dq[1]}]
set_property SLEW FAST [get_ports {ddram64_dq[1]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[1]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[1]}]

# ddram64:0.dq
set_property LOC L14 [get_ports {ddram64_dq[2]}]
set_property SLEW FAST [get_ports {ddram64_dq[2]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[2]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[2]}]

# ddram64:0.dq
set_property LOC M14 [get_ports {ddram64_dq[3]}]
set_property SLEW FAST [get_ports {ddram64_dq[3]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[3]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[3]}]

# ddram64:0.dq
set_property LOC M12 [get_ports {ddram64_dq[4]}]
set_property SLEW FAST [get_ports {ddram64_dq[4]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[4]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[4]}]

# ddram64:0.dq
set_property LOC N15 [get_ports {ddram64_dq[5]}]
set_property SLEW FAST [get_ports {ddram64_dq[5]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[5]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[5]}]

# ddram64:0.dq
set_property LOC M11 [get_ports {ddram64_dq[6]}]
set_property SLEW FAST [get_ports {ddram64_dq[6]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[6]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[6]}]

# ddram64:0.dq
set_property LOC L12 [get_ports {ddram64_dq[7]}]
set_property SLEW FAST [get_ports {ddram64_dq[7]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[7]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[7]}]

# ddram64:0.dq
set_property LOC K14 [get_ports {ddram64_dq[8]}]
set_property SLEW FAST [get_ports {ddram64_dq[8]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[8]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[8]}]

# ddram64:0.dq
set_property LOC K13 [get_ports {ddram64_dq[9]}]
set_property SLEW FAST [get_ports {ddram64_dq[9]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[9]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[9]}]

# ddram64:0.dq
set_property LOC H13 [get_ports {ddram64_dq[10]}]
set_property SLEW FAST [get_ports {ddram64_dq[10]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[10]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[10]}]

# ddram64:0.dq
set_property LOC J13 [get_ports {ddram64_dq[11]}]
set_property SLEW FAST [get_ports {ddram64_dq[11]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[11]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[11]}]

# ddram64:0.dq
set_property LOC L16 [get_ports {ddram64_dq[12]}]
set_property SLEW FAST [get_ports {ddram64_dq[12]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[12]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[12]}]

# ddram64:0.dq
set_property LOC L15 [get_ports {ddram64_dq[13]}]
set_property SLEW FAST [get_ports {ddram64_dq[13]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[13]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[13]}]

# ddram64:0.dq
set_property LOC H14 [get_ports {ddram64_dq[14]}]
set_property SLEW FAST [get_ports {ddram64_dq[14]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[14]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[14]}]

# ddram64:0.dq
set_property LOC J15 [get_ports {ddram64_dq[15]}]
set_property SLEW FAST [get_ports {ddram64_dq[15]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[15]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[15]}]

# ddram64:0.dq
set_property LOC E15 [get_ports {ddram64_dq[16]}]
set_property SLEW FAST [get_ports {ddram64_dq[16]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[16]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[16]}]

# ddram64:0.dq
set_property LOC E13 [get_ports {ddram64_dq[17]}]
set_property SLEW FAST [get_ports {ddram64_dq[17]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[17]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[17]}]

# ddram64:0.dq
set_property LOC F15 [get_ports {ddram64_dq[18]}]
set_property SLEW FAST [get_ports {ddram64_dq[18]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[18]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[18]}]

# ddram64:0.dq
set_property LOC E14 [get_ports {ddram64_dq[19]}]
set_property SLEW FAST [get_ports {ddram64_dq[19]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[19]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[19]}]

# ddram64:0.dq
set_property LOC G13 [get_ports {ddram64_dq[20]}]
set_property SLEW FAST [get_ports {ddram64_dq[20]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[20]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[20]}]

# ddram64:0.dq
set_property LOC G12 [get_ports {ddram64_dq[21]}]
set_property SLEW FAST [get_ports {ddram64_dq[21]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[21]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[21]}]

# ddram64:0.dq
set_property LOC F14 [get_ports {ddram64_dq[22]}]
set_property SLEW FAST [get_ports {ddram64_dq[22]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[22]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[22]}]

# ddram64:0.dq
set_property LOC G14 [get_ports {ddram64_dq[23]}]
set_property SLEW FAST [get_ports {ddram64_dq[23]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[23]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[23]}]

# ddram64:0.dq
set_property LOC B14 [get_ports {ddram64_dq[24]}]
set_property SLEW FAST [get_ports {ddram64_dq[24]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[24]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[24]}]

# ddram64:0.dq
set_property LOC C13 [get_ports {ddram64_dq[25]}]
set_property SLEW FAST [get_ports {ddram64_dq[25]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[25]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[25]}]

# ddram64:0.dq
set_property LOC B16 [get_ports {ddram64_dq[26]}]
set_property SLEW FAST [get_ports {ddram64_dq[26]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[26]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[26]}]

# ddram64:0.dq
set_property LOC D15 [get_ports {ddram64_dq[27]}]
set_property SLEW FAST [get_ports {ddram64_dq[27]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[27]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[27]}]

# ddram64:0.dq
set_property LOC D13 [get_ports {ddram64_dq[28]}]
set_property SLEW FAST [get_ports {ddram64_dq[28]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[28]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[28]}]

# ddram64:0.dq
set_property LOC E12 [get_ports {ddram64_dq[29]}]
set_property SLEW FAST [get_ports {ddram64_dq[29]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[29]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[29]}]

# ddram64:0.dq
set_property LOC C16 [get_ports {ddram64_dq[30]}]
set_property SLEW FAST [get_ports {ddram64_dq[30]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[30]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[30]}]

# ddram64:0.dq
set_property LOC D16 [get_ports {ddram64_dq[31]}]
set_property SLEW FAST [get_ports {ddram64_dq[31]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[31]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[31]}]

# ddram64:0.dq
set_property LOC A24 [get_ports {ddram64_dq[32]}]
set_property SLEW FAST [get_ports {ddram64_dq[32]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[32]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[32]}]

# ddram64:0.dq
set_property LOC B23 [get_ports {ddram64_dq[33]}]
set_property SLEW FAST [get_ports {ddram64_dq[33]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[33]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[33]}]

# ddram64:0.dq
set_property LOC B27 [get_ports {ddram64_dq[34]}]
set_property SLEW FAST [get_ports {ddram64_dq[34]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[34]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[34]}]

# ddram64:0.dq
set_property LOC B26 [get_ports {ddram64_dq[35]}]
set_property SLEW FAST [get_ports {ddram64_dq[35]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[35]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[35]}]

# ddram64:0.dq
set_property LOC A22 [get_ports {ddram64_dq[36]}]
set_property SLEW FAST [get_ports {ddram64_dq[36]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[36]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[36]}]

# ddram64:0.dq
set_property LOC B22 [get_ports {ddram64_dq[37]}]
set_property SLEW FAST [get_ports {ddram64_dq[37]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[37]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[37]}]

# ddram64:0.dq
set_property LOC A25 [get_ports {ddram64_dq[38]}]
set_property SLEW FAST [get_ports {ddram64_dq[38]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[38]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[38]}]

# ddram64:0.dq
set_property LOC C24 [get_ports {ddram64_dq[39]}]
set_property SLEW FAST [get_ports {ddram64_dq[39]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[39]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[39]}]

# ddram64:0.dq
set_property LOC E24 [get_ports {ddram64_dq[40]}]
set_property SLEW FAST [get_ports {ddram64_dq[40]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[40]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[40]}]

# ddram64:0.dq
set_property LOC D23 [get_ports {ddram64_dq[41]}]
set_property SLEW FAST [get_ports {ddram64_dq[41]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[41]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[41]}]

# ddram64:0.dq
set_property LOC D26 [get_ports {ddram64_dq[42]}]
set_property SLEW FAST [get_ports {ddram64_dq[42]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[42]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[42]}]

# ddram64:0.dq
set_property LOC C25 [get_ports {ddram64_dq[43]}]
set_property SLEW FAST [get_ports {ddram64_dq[43]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[43]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[43]}]

# ddram64:0.dq
set_property LOC E23 [get_ports {ddram64_dq[44]}]
set_property SLEW FAST [get_ports {ddram64_dq[44]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[44]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[44]}]

# ddram64:0.dq
set_property LOC D22 [get_ports {ddram64_dq[45]}]
set_property SLEW FAST [get_ports {ddram64_dq[45]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[45]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[45]}]

# ddram64:0.dq
set_property LOC F22 [get_ports {ddram64_dq[46]}]
set_property SLEW FAST [get_ports {ddram64_dq[46]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[46]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[46]}]

# ddram64:0.dq
set_property LOC E22 [get_ports {ddram64_dq[47]}]
set_property SLEW FAST [get_ports {ddram64_dq[47]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[47]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[47]}]

# ddram64:0.dq
set_property LOC A30 [get_ports {ddram64_dq[48]}]
set_property SLEW FAST [get_ports {ddram64_dq[48]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[48]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[48]}]

# ddram64:0.dq
set_property LOC D27 [get_ports {ddram64_dq[49]}]
set_property SLEW FAST [get_ports {ddram64_dq[49]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[49]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[49]}]

# ddram64:0.dq
set_property LOC A29 [get_ports {ddram64_dq[50]}]
set_property SLEW FAST [get_ports {ddram64_dq[50]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[50]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[50]}]

# ddram64:0.dq
set_property LOC C28 [get_ports {ddram64_dq[51]}]
set_property SLEW FAST [get_ports {ddram64_dq[51]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[51]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[51]}]

# ddram64:0.dq
set_property LOC D28 [get_ports {ddram64_dq[52]}]
set_property SLEW FAST [get_ports {ddram64_dq[52]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[52]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[52]}]

# ddram64:0.dq
set_property LOC B31 [get_ports {ddram64_dq[53]}]
set_property SLEW FAST [get_ports {ddram64_dq[53]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[53]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[53]}]

# ddram64:0.dq
set_property LOC A31 [get_ports {ddram64_dq[54]}]
set_property SLEW FAST [get_ports {ddram64_dq[54]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[54]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[54]}]

# ddram64:0.dq
set_property LOC A32 [get_ports {ddram64_dq[55]}]
set_property SLEW FAST [get_ports {ddram64_dq[55]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[55]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[55]}]

# ddram64:0.dq
set_property LOC E30 [get_ports {ddram64_dq[56]}]
set_property SLEW FAST [get_ports {ddram64_dq[56]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[56]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[56]}]

# ddram64:0.dq
set_property LOC F29 [get_ports {ddram64_dq[57]}]
set_property SLEW FAST [get_ports {ddram64_dq[57]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[57]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[57]}]

# ddram64:0.dq
set_property LOC F30 [get_ports {ddram64_dq[58]}]
set_property SLEW FAST [get_ports {ddram64_dq[58]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[58]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[58]}]

# ddram64:0.dq
set_property LOC F27 [get_ports {ddram64_dq[59]}]
set_property SLEW FAST [get_ports {ddram64_dq[59]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[59]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[59]}]

# ddram64:0.dq
set_property LOC C30 [get_ports {ddram64_dq[60]}]
set_property SLEW FAST [get_ports {ddram64_dq[60]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[60]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[60]}]

# ddram64:0.dq
set_property LOC E29 [get_ports {ddram64_dq[61]}]
set_property SLEW FAST [get_ports {ddram64_dq[61]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[61]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[61]}]

# ddram64:0.dq
set_property LOC F26 [get_ports {ddram64_dq[62]}]
set_property SLEW FAST [get_ports {ddram64_dq[62]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[62]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[62]}]

# ddram64:0.dq
set_property LOC D30 [get_ports {ddram64_dq[63]}]
set_property SLEW FAST [get_ports {ddram64_dq[63]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dq[63]}]
set_property IOSTANDARD SSTL15_T_DCI [get_ports {ddram64_dq[63]}]

# ddram64:0.dqs_p
set_property LOC N16 [get_ports {ddram64_dqs_p[0]}]
set_property SLEW FAST [get_ports {ddram64_dqs_p[0]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_p[0]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_p[0]}]

# ddram64:0.dqs_p
set_property LOC K12 [get_ports {ddram64_dqs_p[1]}]
set_property SLEW FAST [get_ports {ddram64_dqs_p[1]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_p[1]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_p[1]}]

# ddram64:0.dqs_p
set_property LOC H16 [get_ports {ddram64_dqs_p[2]}]
set_property SLEW FAST [get_ports {ddram64_dqs_p[2]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_p[2]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_p[2]}]

# ddram64:0.dqs_p
set_property LOC C15 [get_ports {ddram64_dqs_p[3]}]
set_property SLEW FAST [get_ports {ddram64_dqs_p[3]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_p[3]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_p[3]}]

# ddram64:0.dqs_p
set_property LOC A26 [get_ports {ddram64_dqs_p[4]}]
set_property SLEW FAST [get_ports {ddram64_dqs_p[4]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_p[4]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_p[4]}]

# ddram64:0.dqs_p
set_property LOC F25 [get_ports {ddram64_dqs_p[5]}]
set_property SLEW FAST [get_ports {ddram64_dqs_p[5]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_p[5]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_p[5]}]

# ddram64:0.dqs_p
set_property LOC B28 [get_ports {ddram64_dqs_p[6]}]
set_property SLEW FAST [get_ports {ddram64_dqs_p[6]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_p[6]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_p[6]}]

# ddram64:0.dqs_p
set_property LOC E27 [get_ports {ddram64_dqs_p[7]}]
set_property SLEW FAST [get_ports {ddram64_dqs_p[7]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_p[7]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_p[7]}]

# ddram64:0.dqs_n
set_property LOC M16 [get_ports {ddram64_dqs_n[0]}]
set_property SLEW FAST [get_ports {ddram64_dqs_n[0]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_n[0]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_n[0]}]

# ddram64:0.dqs_n
set_property LOC J12 [get_ports {ddram64_dqs_n[1]}]
set_property SLEW FAST [get_ports {ddram64_dqs_n[1]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_n[1]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_n[1]}]

# ddram64:0.dqs_n
set_property LOC G16 [get_ports {ddram64_dqs_n[2]}]
set_property SLEW FAST [get_ports {ddram64_dqs_n[2]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_n[2]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_n[2]}]

# ddram64:0.dqs_n
set_property LOC C14 [get_ports {ddram64_dqs_n[3]}]
set_property SLEW FAST [get_ports {ddram64_dqs_n[3]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_n[3]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_n[3]}]

# ddram64:0.dqs_n
set_property LOC A27 [get_ports {ddram64_dqs_n[4]}]
set_property SLEW FAST [get_ports {ddram64_dqs_n[4]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_n[4]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_n[4]}]

# ddram64:0.dqs_n
set_property LOC E25 [get_ports {ddram64_dqs_n[5]}]
set_property SLEW FAST [get_ports {ddram64_dqs_n[5]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_n[5]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_n[5]}]

# ddram64:0.dqs_n
set_property LOC B29 [get_ports {ddram64_dqs_n[6]}]
set_property SLEW FAST [get_ports {ddram64_dqs_n[6]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_n[6]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_n[6]}]

# ddram64:0.dqs_n
set_property LOC E28 [get_ports {ddram64_dqs_n[7]}]
set_property SLEW FAST [get_ports {ddram64_dqs_n[7]}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_dqs_n[7]}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_dqs_n[7]}]

# ddram64:0.clk_p
set_property LOC H19 [get_ports {ddram64_clk_p}]
set_property SLEW FAST [get_ports {ddram64_clk_p}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_clk_p}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_clk_p}]

# ddram64:0.clk_n
set_property LOC G18 [get_ports {ddram64_clk_n}]
set_property SLEW FAST [get_ports {ddram64_clk_n}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_clk_n}]
set_property IOSTANDARD DIFF_SSTL15 [get_ports {ddram64_clk_n}]

# ddram64:0.cke
set_property LOC K19 [get_ports {ddram64_cke}]
set_property SLEW FAST [get_ports {ddram64_cke}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_cke}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_cke}]

# ddram64:0.odt
set_property LOC H20 [get_ports {ddram64_odt}]
set_property SLEW FAST [get_ports {ddram64_odt}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_odt}]
set_property IOSTANDARD SSTL15 [get_ports {ddram64_odt}]

# ddram64:0.reset_n
set_property LOC C29 [get_ports {ddram64_reset_n}]
set_property SLEW FAST [get_ports {ddram64_reset_n}]
set_property VCCAUX_IO HIGH [get_ports {ddram64_reset_n}]
set_property IOSTANDARD LVCMOS15 [get_ports {ddram64_reset_n}]

# eth:0.rst_n
set_property LOC AJ33 [get_ports {eth_rst_n}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_rst_n}]

# eth:0.int_n
set_property LOC AL31 [get_ports {eth_int_n}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_int_n}]

# eth:0.mdio
set_property LOC AK33 [get_ports {eth_mdio}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_mdio}]

# eth:0.mdc
set_property LOC AH31 [get_ports {eth_mdc}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_mdc}]

# eth:0.rx_p
set_property LOC AM8 [get_ports {eth_rx_p}]

# eth:0.rx_n
set_property LOC AM7 [get_ports {eth_rx_n}]

# eth:0.tx_p
set_property LOC AN2 [get_ports {eth_tx_p}]

# eth:0.tx_n
set_property LOC AN1 [get_ports {eth_tx_n}]

# sgmii_clock:0.p
set_property LOC AH8 [get_ports {sgmii_clock_p}]

# sgmii_clock:0.n
set_property LOC AH7 [get_ports {sgmii_clock_n}]

# user_led:0
set_property LOC AM39 [get_ports {user_led0}]
set_property IOSTANDARD LVCMOS18 [get_ports {user_led0}]

# user_led:1
set_property LOC AN39 [get_ports {user_led1}]
set_property IOSTANDARD LVCMOS18 [get_ports {user_led1}]

# user_led:2
set_property LOC AR37 [get_ports {user_led2}]
set_property IOSTANDARD LVCMOS18 [get_ports {user_led2}]

# user_led:3
set_property LOC AT37 [get_ports {user_led3}]
set_property IOSTANDARD LVCMOS18 [get_ports {user_led3}]

# user_led:4
set_property LOC AR35 [get_ports {user_led4}]
set_property IOSTANDARD LVCMOS18 [get_ports {user_led4}]

# user_led:5
set_property LOC AP41 [get_ports {user_led5}]
set_property IOSTANDARD LVCMOS18 [get_ports {user_led5}]

# user_led:6
set_property LOC AP42 [get_ports {user_led6}]
set_property IOSTANDARD LVCMOS18 [get_ports {user_led6}]

# user_led:7
set_property LOC AU39 [get_ports {user_led7}]
set_property IOSTANDARD LVCMOS18 [get_ports {user_led7}]

################################################################################
# Design constraints
################################################################################

set_property CFGBVS VCCO [current_design]

set_property CONFIG_VOLTAGE 2.5 [current_design]

set_property BITSTREAM.STARTUP.MATCH_CYCLE NoWait [current_design]

set_property LOC GTXE2_CHANNEL_X1Y1 [get_cells liteeth_sgmii_phy.GTXE2_CHANNEL]

set_property LOC IBUFDS_GTE2_X1Y0 [get_cells liteeth_sgmii_phy.IBUFDS_GTE2]

set_property LOC MMCME2_ADV_X0Y6 [get_cells liteeth_sgmii_phy.MMCME2_ADV]

set_property LOC MMCME2_ADV_X0Y3 [get_cells liteeth_sgmii_phy.MMCME2_ADV_1]

set_property LOC BUFGCTRL_X0Y2 [get_cells liteeth_sgmii_phy.bufg_rxoutrebuf]

set_property LOC BUFGCTRL_X0Y3 [get_cells liteeth_sgmii_phy.bufg_txoutrebuf]

set_property LOC BUFGCTRL_X0Y5 [get_cells liteeth_sgmii_phy.bufg_ethrx62]

set_property LOC BUFGCTRL_X0Y6 [get_cells liteeth_sgmii_phy.bufg_ethrx125]

set_property LOC BUFGCTRL_X0Y16 [get_cells liteeth_sgmii_phy.bufg_ethtx62]

set_property LOC BUFGCTRL_X0Y17 [get_cells liteeth_sgmii_phy.bufg_ethtx125]

################################################################################
# Clock constraints
################################################################################


create_clock -name clk200_p -period 5.0 [get_ports clk200_p]

create_clock -name sgmii_clock_p -period 8.0 [get_ports sgmii_clock_p]

create_clock -name eth_tx_clk -period 8.0 [get_nets eth_tx_clk]

create_clock -name eth_rx_clk -period 8.0 [get_nets eth_rx_clk]

################################################################################
# False path constraints
################################################################################


set_false_path -quiet -to [get_cells -hierarchical -filter {mr_ff == TRUE}]

set_false_path -quiet -to [get_pins -filter {REF_PIN_NAME == PRE} -of_objects [get_cells -hierarchical -filter {ars_ff1 == TRUE || ars_ff2 == TRUE}]]

set_max_delay 2 -quiet -from [get_pins -filter {REF_PIN_NAME == C} -of_objects [get_cells -hierarchical -filter {ars_ff1 == TRUE}]] -to [get_pins -filter {REF_PIN_NAME == D} -of_objects [get_cells -hierarchical -filter {ars_ff2 == TRUE}]]

set_clock_groups -group [get_clocks -of [get_nets eth_tx_clk]] -group [get_clocks -of [get_nets eth_rx_clk]] -asynchronous

set_clock_groups -group [get_clocks -of [get_nets eth_tx_clk]] -group [get_clocks -of [get_nets sys_clk]] -asynchronous

set_clock_groups -group [get_clocks -of [get_nets eth_rx_clk]] -group [get_clocks -of [get_nets sys_clk]] -asynchronous

set_clock_groups -group [get_clocks -of [get_nets sys_clk]] -group [get_clocks -of [get_nets main_crgddr_clkin_signal]] -asynchronous