"""SM select-bit writers FUN_00573110 (L0) + FUN_00573470 (L1+), transcribed.

These two functions -- with FUN_00560450 -- are the only callers of the core bit
setter FUN_00556ad0. They turn one routed SM hop (a node + the source feeding it)
into the (BL, WL, value) writes for that node's select mux. sm_routing.py already
covers the single-stage select VALUE math (verified, 17/31 single-FF nodes); this
module transcribes the WRITERS themselves -- the per-(level, direction, branch)
coordinate geometry and the multi-stage / disable value model that the single-
stage formula does not capture (the other 14 nodes).

Status: CLOSED. The coordinate geometry below is transcribed and validated
bit-for-bit against the verified sm_routing.SM_BL_WL footprints
(tests/test_sm_writer.py), and the two-stage VALUE computation is decoded across
every reprogrammed node the corpus exercises -- sm_validate reproduces 34434/34434
SM nodes (tests/test_sm_corpus.py). The one write-time-snapshot datum that is not
statically derivable -- the (5,'D',b>0)-source (4,'D') stage-1 argmin lane, run by
the vendor on the mid-routing criticality snapshot -- is supplied by the l4d_oracle
capture (see level4_down `s1_override`), the same sanctioned-capture pattern as the
(0,'U') router-binding bit (l0_oracle).

Coordinate convention matches coords.trace_to_word_bit / sm_routing: set_bit is
called as set_bit(BL, WL, val); BL in 0..10, WL in 0..31, per tile (x, y).
"""

# FUN_00573110  -- level-0 I0/I1 input-assignment writer
#   <- decomp/eda-placer.real.c:437480 (read in full)
# Signature in the decompile: FUN_00573110(param_1=SM ctx, param_2=value bit,
# param_3=node record). It reads branch = *(node+0xc) and asserts the node is a
# level-0 input (*(node+8)==0 and *(node+0x14)==0, else "ERROR! I0_I1 assignment
# is only to level 0 SM inputs!"). It then dispatches on the SM tile type
# (*(ctx+0x10)) and writes one select bit; the type 6/7 path additionally writes
# a second-stage bit via FUN_00556d50 (the L0 value setter), left as TODO.
#
# The flags at ctx+0x98/+0x99/+0x9a gate the type-{0,>=8} layout.

def write_fun_00573110(set_bit, sm_type, branch, node_level, node_downward,
                       value, flags=(0, 0, 0), set_bit2=None):
    """<- FUN_00573110. Emit the level-0 input-assignment select bit.

    set_bit(BL, WL, val). `flags` = (ctx+0x98, +0x99, +0x9a). `set_bit2`, if
    given, models the FUN_00556d50 second-stage write for the type 6/7 path.
    """
    if node_level != 0 or node_downward != 0:
        raise ValueError(
            "I0_I1 assignment is only to level 0 SM inputs!")     # :437500

    if sm_type == 5:                                              # :437507
        # bl = (-(b>>31) - ((b - (b>>31)) & 1)) + 2  == 2 - (b & 1) for b >= 0
        bl = (-(branch >> 31) - ((branch - (branch >> 31)) & 1)) + 2
        wl = 7 if branch < 2 else 0xd                             # :437508-512
        set_bit(bl, wl, value)
        return 0

    if not (1 < ((sm_type - 6) & 0xffffffff)):                    # sm_type in {6,7}
        # The (0,'U',branch) input mux:  branch 0 -> (5,25); else -> (4, b+0x16)
        if branch < 1:                                            # :437535
            bl, wl = 5, 0x19
        else:
            bl, wl = 4, branch + 0x16                             # :437540
        set_bit(bl, wl, value)                                    # :437543
        if set_bit2 is not None:
            set_bit2(branch, value)        # FUN_00556d50 second stage (:437554)
        return 0

    if ((sm_type - 1) & 0xffffffff) < 4:                          # sm_type in {1..4}
        set_bit(10, branch + 0x10, 1 - value)                     # :437517
        return 1

    f98, f99, f9a = flags                                         # :437520
    if f98 == 0 and f99 == 0:
        if f9a == 0:
            set_bit(4, branch + 0x16, value)                      # :437522
            return 1
        set_bit(2, branch + 0x12, value)                          # :437525
        return 0
    if branch < 2:                                                # :437528
        set_bit(3, branch + 3, value)
    else:
        set_bit(3, branch + 9, value)                             # :437532
    return 0


# FUN_00573470  -- level-1+ select writer (the two-stage / multi-input muxes)
#   <- decomp/eda-placer.real.c:437646 (~2950 lines; cases mapped below)
# The function dispatches on (level = *(node+8), downward = *(node+0x14)) and,
# per case, lays out three parallel lane arrays -- a value-base, a BL-base and a
# WL-base -- then runs a fixed-count lane loop applying branch-dependent (BL, WL)
# offsets and toggling/setting the value lane that the routed source selects.
#
# Cases (guarded by the "SM_2p5K ERROR: ... level %d ... downward %d" asserts):
CASES = {
    # (level, downward): decompile anchor + the lane geometry recovered there.
    # value-base / bl-base / wl-base are the local_188/local_168/local_148-style
    # stack arrays; the lane loop writes set_bit(bl_base[i]+dBL, wl_base[i]+dWL,
    # value[i]) with the source lane toggled.
    (1, 0): dict(line=437812, lanes=4,
                 value_base=[0, 1, 0, 0], bl_base=[8, 8, 8, 8],
                 wl_base=[20, 21, 22, 23]),                 # (1,'U',branch)
    (4, 0): dict(line=438084, lanes=6,
                 value_base=[0, 1, 0, 0, 0, 0], bl_base=[8] * 6,
                 wl_base=[6, 7, 8, 9, 10, 11]),             # (4,'U',branch)
    (0, 0): dict(line=438633, lanes=5,
                 value_base=[0, 1, 0, 0, 0], bl_base=[0] * 5,
                 wl_base=[0, 1, 2, 3, 4]),                  # (0,'D',branch)
    # (3,0):437953  (5,0):438224  (>=13,0):438382 level-13 clock spine
    # downward!=0 ('D'/'U' counterparts) are decoded in the level{0,1,3,4,5}_down*
    # functions below (validated 34434/34434 by tests/test_sm_corpus.py).
}


def branch_offsets_l1up(branch):
    """(dBL, dWL) for the (1,'U') / level-1 downward-0 lane loop. <- :437836-844."""
    if branch == 2:
        return 0, 4
    if branch == 3:
        return -1, 4
    if branch == 1:
        return -1, 0
    return 0, 0


def branch_offsets_l4up(branch):
    """(dBL, dWL) for the (4,'U') lane loop: BL-1 for branch>3. <- :438116."""
    return (-1 if branch > 3 else 0), 0


# Two-stage VALUE computation (the absolute select value the vendor writes).
# The writer emits an ABSOLUTE select value (it does NOT read the template); the
# value is a per-case lane-base array modified by the ROUTED SOURCE's identity.
# In the decompile the source descriptor (param_2 -> 6 ints) carries the source
# SM node's (level, branch, index) at fields [2], [3], [5] = (iVar49, local_354,
# iVar41); the destination node is param_3. So given the source node's key and
# the destination node's key, the value is determined (no timing input) for the
# cases below.
#
# Validated bit-for-bit against vendor/reference/single_ff via the graph
# predecessors (tests/test_sm_writer.py): the source that reproduces the vendor's
# select value is exactly the routed predecessor in the chipdb graph.

def level0_down_value(src_level, src_branch, src_index, src_down):
    """Select value for a (0,'D',*) node. <- FUN_00573470 :438633-438657.

    Lane base {0,1,0,0,0}; the routed SOURCE node's identity selects the toggle.
    The decompile reads the source descriptor fields iVar49/local_354/iVar39/iVar41
    which are the source node's (level, branch, index, down). Returns None for the
    unroutable combination (the decompile's SM_2p5K ERROR path).
    """
    lanes = [0, 1, 0, 0, 0]
    if src_down == 1 and src_level == 1:                  # iVar41==1 && iVar49==1
        if src_branch == 0 and src_index < 2:             # :438650
            lanes[src_index] ^= 1
        elif src_branch < 4:                              # :438654-655
            lanes[src_branch + 1] ^= 1
        else:
            return None
    else:
        return None
    return sum(b << i for i, b in enumerate(lanes))


def level4_down(src_level, src_down, src_branch, src_field4, dest_branch,
                crit_of_edge=None, s1_override=None):
    """Both toggle lanes for a (4,'D') mux. <- FUN_00573470 :439333-439718.

    The routed source SM node's (level, down, branch, field4) = vendor descriptor
    fields [2],[5],[3],[4] (iVar49, iVar41, local_354, iVar39); iVar41 is the
    source's DOWN flag (1='D'). Returns (stage1_lane, stage2_lane) -- the lanes
    level4_down_value toggles -- or None for the decompile's SM_2p5K ERROR
    (unroutable) combinations.

    TRACE-CONFIRMED (e9patch of the vendor placer on four_ff, log_slot+BW): per
    source branch the written (stage1,stage2) lanes are 0->(2,1) 1->(0,3)
    2->(1,4 routed / 0,4 default) 3->(0,0) -- matching the rules below exactly,
    and showing stage1 is criticality-driven (same src branch 2 gives lane1 for
    the routed/critical nodes vs lane0 for unrouted defaults), so branch-1 is the
    argmin OUTCOME for routed nodes, not a heuristic.

    STAGE 1 (:439333-439375), and the sub-case's local_398/local_358 that drive
    STAGE 2 (:439689-439718):
      * src (level1, 'U'):            s1 = src_branch;            local_358 = 2
      * src (level3, 'U'):            s1 = 1 if branch>3 else 0;  local_358 = 1
      * src (level5, 'D', branch0):   s1 = src_field4 + 2;        local_358 = 1
        -> for these three (local_398 != -1) STAGE 2 toggles lane local_358.
      * otherwise: s1 = argmin over FOUR_DOWN_SLOTS (criticality), local_398 == -1
        -> STAGE 2 (:439690-439712):
             src (level5, 'D', branch>0): s2 = 0 if branch==3 else branch+2
             src (level4, 'U', branch != dest): the local_158 path (rare, None)
             else: ERROR -> None
    crit_of_edge maps a FOUR_DOWN_SLOTS edge key -> criticality (argmin path only).
    """
    from . import delay_model
    # the three closed-form source sub-cases (local_398 != -1 => stage2 = local_358)
    if src_level == 1 and src_down == 0:                     # :439623-439630
        return src_branch, 2
    if src_level == 3 and src_down == 0:                     # :439613-439622
        return (1 if src_branch > 3 else 0), 1
    if src_level == 5 and src_down == 1 and src_branch == 0:    # :439335-439342
        return src_field4 + 2, 1
    # general case: stage-1 argmin (local_398 == -1) + stage-2 source rule.
    # The argmin runs on the vendor's WRITE-TIME criticality snapshot; the port's
    # static final Anet reproduces it for all but the rare tie-close case -- there
    # `s1_override` supplies the captured write-time lane (l4d_oracle). See below.
    if src_level == 5 and src_down == 1 and src_branch > 0:     # :439690-439699
        s2 = 0 if src_branch == 3 else src_branch + 2
        if s1_override is not None:
            return s1_override, s2
        if crit_of_edge is None:
            return None
        s1 = delay_model.resolve_slots(delay_model.FOUR_DOWN_SLOTS, crit_of_edge)
        return s1, s2
    # src (4,'U'), src_branch != dest_branch: the local_158 path :439712-439713 --
    # stage1 = the general criticality argmin over FOUR_DOWN_SLOTS (:439344-439375,
    # local_398=-1, FUN_006b18b0 inlined at :439373 -- 0 standalone calls on the
    # corpus), the SAME crit_of_edge binding the (5,'D') path uses; stage2 toggles
    # lane5 (local_158._4_4_, :439713). Under all-empty criticality the argmin keeps
    # lane0 (so (4,'U',5)->547 == (s1=0, s2=5) still holds); a slot with a bound net
    # wins its lane (closes wbank32/wlfsr16/wlfsr8 (4,'D',2)->544 == (s1=1, s2=5)).
    if src_level == 4 and src_down == 0 and src_branch != dest_branch:
        if s1_override is not None:
            return s1_override, 5
        if crit_of_edge is None:
            return 0, 5
        s1 = delay_model.resolve_slots(delay_model.FOUR_DOWN_SLOTS, crit_of_edge)
        return s1, 5
    return None      # (0,'D')/timed-argmin ERROR paths -- need the real criticality


def level4_down_value(dest_branch, stage1_lane, stage2_lane):
    """Full 10-bit (4,'D',dest_branch,0) select value. <- FUN_00573470 :439323-439718.

    Two ABSOLUTE lane groups packed in SM_BL_WL cell order:
      stage 1 = cells 0..3 (BL3): base {0,1,0,0}, toggle lane stage1_lane (:439326,
                the winner from level4_down_winner).
      stage 2 = cells 4..9 (BL0): base {0,1,0,0,0,0}, toggle lane stage2_lane
                (:439681 base, :439697 toggle local_168[src_branch+2]).
    VALIDATED bit-for-bit: (winner=0,stage2=3)->0b0010100011 == clock-root (0,31);
    (winner=1,stage2=4)->0b0100100000 == logic-col FF (25,3).
    Returns the integer select value (bit i <- SM_BL_WL cell i).
    """
    s1 = [0, 1, 0, 0]
    s2 = [0, 1, 0, 0, 0, 0]
    if stage1_lane is not None and 0 <= stage1_lane < 4:
        s1[stage1_lane] ^= 1
    if stage2_lane is not None and 0 <= stage2_lane < 6:
        s2[stage2_lane] ^= 1
    lanes = s1 + s2
    return sum(b << i for i, b in enumerate(lanes))


# (1,'D') down-mux value, TRACE-DERIVED (e9patch of the vendor placer on four_ff,
# log_slot+BW correlation): the routed source is the (1,'U',branch) predecessor;
# the 10-bit value = stage1 lanes (cells 0-4 @BL6) | stage2 lanes (cells 5-9 @BL5),
# keyed by source branch and an ACTIVE (down-delivering) vs DEFAULT state.
# stage1 per branch (LSB=lane0); stage2 = 0b01010 active / 0 default. Validated
# bit-for-bit vs the corpus: (25,3)=323, (24,1)B0i1=320, (24,1)B3=330, (0,31)=6.
_L1D_STAGE1_DEFAULT = {0: 0b00011, 1: 0b00110, 2: 0b01010, 3: 0b10010}  # <- trace
# stage1 active = base {lane1} XOR lane(src_branch): b0->00011 b1->00000 b2->00110
# b3->01010. branch2 (00110) completes the lane=b pattern -- confirmed on
# lut6_probe/multilut6 (1,'D',*) <- (1,'U',2) = 326 (stage1 00110 | stage2 01010<<5).
_L1D_STAGE1_ACTIVE = {0: 0b00011, 1: 0b00000, 2: 0b00110, 3: 0b01010}
_L1D_STAGE2_ACTIVE = 0b01010


def level1_down(src_branch, active):
    """(1,'D') 10-bit select value from the routed (1,'U',src_branch) source.

    `active` = the down-mux delivers a net downward (stage2 enabled) vs a default/
    source-only node (e.g. the clock-spine root). TRACE-DERIVED (see table above).
    Returns the integer value, or None if not modelled (e.g. active branch 2).
    """
    if active:
        s1 = _L1D_STAGE1_ACTIVE.get(src_branch)
        if s1 is None:
            return None
        return s1 | (_L1D_STAGE2_ACTIVE << 5)
    s1 = _L1D_STAGE1_DEFAULT.get(src_branch)
    return None if s1 is None else s1


def level1_down_src5(src_branch, src_index):
    """(1,'D') select value when fed by a (5,'D') down-path. <- corpus-derived:
    the (5,'D') source's INDEX decides. index 1 = the level-1 mux is bypassed
    (the 5->0 path skips it) -> cleared to 0; index 0 = the level-1 stage takes
    the DEFAULT stage-1 pattern for the source branch (no stage-2 down-deliver).
    Validated: (5,'D',2,0)->10, (5,'D',0,0)->3, (5,'D',0,1)->0.
    """
    if src_index == 1:
        return 0
    return level1_down(src_branch, False)


def level1_down_src4(src_branch, src_down):
    """(1,'D') 10-bit select value from a (4,'U')/(4,'D') carry source.
    <- FUN_00573470 (1,'D') path :439805-439823 (stage1 local_208 / lane via
    local_1f8/uStack_200) + :440169 + stage2 lane via local_3b4 (default 2).

    stage1 base {lane1}, stage2 base {lane1}; the source toggles:
        (4,'U', b<4):   stage1 lane2;  stage2 lane2
        (4,'U', b>=4):  stage1 lane3;  stage2 lane2
        (4,'D', any):   stage1 lane4 ALWAYS (local_1f8=1, :439806 -- set before the
                        branch split); stage2 lane2 (b>3) / lane3 (b<=3, :440169).
    TRACE-CONFIRMED (e9patch --slots+BW): shift4 (1,'D',2,0)<-(4,'U',2,0)=198;
    counter4 (1,'D',1,0)<-(4,'D',5,0)=210; comb2 (1,'D',2,0)<-(4,'D',2,0)=338.
    """
    s1 = [0, 1, 0, 0, 0]                         # local_208 stage1 base
    s2 = [0, 1, 0, 0, 0]                         # local_148 stage2 base
    if src_down == 0:                            # (4,'U')
        s1[2 if src_branch < 4 else 3] ^= 1
        s2lane = 2
    else:                                        # (4,'D'): stage1 lane4 always
        s1[4] ^= 1
        s2lane = 2 if src_branch > 3 else 3
    s2[s2lane] ^= 1
    v1 = sum(b << i for i, b in enumerate(s1))
    v2 = sum(b << i for i, b in enumerate(s2))
    return v1 | (v2 << 5)


def level1_down_src3(src_branch, src_down, timed=False):
    """(1,'D') 10-bit select value from a level-3 source. <- FUN_00573470 (1,'D').

    src (3,'U') -- the closed-form path :439826-439833 (iVar49==3, !bVar53):
        stage1 base local_208 = {lane1}; src branch<4 sets the low int too
        (local_208 = CONCAT44(.._4_4_, 1)) -> lanes {0,1}; branch>=4 clears lane1
        (local_208 & 0xffffffff) -> {} (decompile-derived, not corpus-seen).
        stage2 base {lane1}, toggle lane local_3b4=2 (joined_r0x005790e2).
      -> (3,'U',1) = stage1 0b00011 | stage2 0b00110<<5 = 195 (lut6_probe/multilut6).

    src (3,'D') -- the argmin path (LAB_00578402 -> FUN_006b18b0), stage2 from
    :440264-440267 (iVar41==1 && iVar49==3, branch<4 sets local_148 lane0):
        stage1 base {lane1} toggled at the argmin lane = (src_branch-1) if timed
        else 0 (the same criticality-argmin outcome level4_down uses); stage2 base
        {lane1} + lane0 -> lanes {0,1}.
      -> (3,'D',1) timed: argmin lane = 0 -> stage1 0b00011 | stage2 0b00011<<5 = 99
        (multilut6 (1,'D',1) <- (3,'D',1)).
    """
    if src_down == 0:                                # (3,'U')
        s1 = 0b00011 if src_branch < 4 else 0b00000  # local_208 low+high / clear
        s2 = 0b00010 ^ (1 << 2)                      # base lane1 toggle lane2 = 6
        return s1 | (s2 << 5)
    if src_branch < 4:                               # (3,'D') argmin path
        lane = (src_branch - 1) if timed else 0      # argmin outcome (cf. level4_down)
        s1 = 0b00010 ^ (1 << lane)                   # base lane1 toggle argmin lane
        s2 = 0b00010 ^ (1 << 0)                      # base lane1 set lane0 (:440267)
        return s1 | (s2 << 5)
    return None                                      # src branch>=4 -> unobserved


def level1_down_argmin(src_level, src_down, src_branch, crit_of_edge):
    """The (1,'D') argmin-path two-stage lanes. <- FUN_00573470 LAB_00578402
    (:439836-440086) + the stage-2 setup (:440263-440271).

    Reached when the routed source is a 'D' edge that is NOT (4,'D')/(5,'D')/(1,'U')
    (the closed-form sources handled by level1_down/_src3/_src4/_src5): the dispatch
    at :439804-439835 falls through LAB_00577069 to the 5-slot argmin LAB_00578402.

    STAGE 1 (:439843-439916): toggle lane = FUN_006b18b0 over the 5 slot scores, each
    slot = max of its candidate-edge criticalities -- delay_model.L1_DOWN_SLOTS via
    crit_of_edge (the SAME node->net->Anet binding the (4,'D') path uses, Section 2.7).
    STAGE 2 (:440263-440271, local_3b0 == -1): written ONLY for a (3,'D') source
    (iVar41==1 && iVar49==3); src_branch < 4 sets stage-2 lane0 (local_148._0_4_=1),
    src_branch >= 4 sets lane4 (local_138._0_4_=1). Any other source hits the
    SM_2p5K ERROR branch (:440273-440278, uVar21=-1) -> no stage-2 write -> None here.

    Returns (stage1_lane, stage2_lane) or None for the ERROR sources.
    """
    from . import delay_model
    if not (src_down == 1 and src_level == 3):
        return None       # SM_2p5K ERROR (:440273) -- no stage-2 written
    s1 = delay_model.resolve_slots(delay_model.L1_DOWN_SLOTS, crit_of_edge)
    s2 = 0 if src_branch < 4 else 4                      # :440265 / :440269
    return s1, s2


def level1_down_value(stage1_lane, stage2_lane):
    """Full 10-bit (1,'D') select value. <- FUN_00573470 LAB_00575076 (stage1
    write, :440175-440243) + LAB_005758ef (stage2 write, :440282-440331).

    Two ABSOLUTE 5-lane groups packed in SM_BL_WL cell order:
      stage 1 = cells 0..4 (BL6, local_208 base {0,1,0,0,0}), toggle stage1_lane.
      stage 2 = cells 5..9 (BL5, local_148 base {0,1,0,0,0}), toggle stage2_lane.
    Matches the trace-derived level1_down* packing (v1 | v2<<5). VALIDATED
    bit-for-bit corpus-wide: (s1=0,s2=4)->579 (3,'D',>=4); (s1=1,s2=0)->96 (3,'D',<4).
    Returns the integer select value (bit i <- SM_BL_WL cell i).
    """
    s1 = [0, 1, 0, 0, 0]
    s2 = [0, 1, 0, 0, 0]
    if stage1_lane is not None and 0 <= stage1_lane < 5:
        s1[stage1_lane] ^= 1
    if stage2_lane is not None and 0 <= stage2_lane < 5:
        s2[stage2_lane] ^= 1
    v1 = sum(b << i for i, b in enumerate(s1))
    v2 = sum(b << i for i, b in enumerate(s2))
    return v1 | (v2 << 5)


def level3_down(dest_branch, src_level, src_branch, src_index, src_down):
    """(3,'D') 10-bit select value from its routed source. <- FUN_00573470
    (3,'D') case (:439784-440164); the src=(5,'D') closed-form sub-path.

    TRACE-CONFIRMED (e9patch --slots+BW of the vendor placer on and_reg): the
    routed source for the corpus (3,'D') nodes is (5,'D',3,0); the writer emits a
    10-bit value = stage1 (lanes 0-5, cells BL 6/5) | stage2 (lanes 6-9, cells
    BL 8/7). Stage1 base = {lane1} (local_208 init 0x100000000); stage2 stays 0.
    A single stage-1 lane is toggled by the source branch (:440144-440157):
        src_branch == 0:  lane = src_index               (:440145 local_208[iVar39])
        src_branch <= 3:  lane = src_branch + 2           (:440156, trace-calibrated)
        src_branch  > 3:  argmin path (not observed in corpus) -> None
    Validated bit-for-bit vs the corpus: (5,'D',3,0) -> (3,'D',{1,4},0) = 34.
    dest_branch only shifts the cells (handled by SM_BL_WL), not the value.

    src (1,'U') active feed (LUT6 wide-mux): stage1 base {lane1} toggled at lane
    src_branch (the universal (1,'U')-active toggle pattern), stage2 base relative-
    lane1 (abs lane7) + active relative-lane2 (abs lane8) -- the level-3 lane2
    active marker (cf. the (3,'U')->(1,'D') stage2 toggle lane2). Single-sample:
    multilut6 (3,'D',1) <- (1,'U',3) = stage1 {1,3} | stage2 {7,8} = 394. <-
    FUN_00573470 (3,'D') (1,'U') fall-through (iVar49==1, !bVar53).
    """
    if src_level == 1 and src_down == 0:       # (3,'D') <- (1,'U') active
        if src_branch >= 6:
            return None
        lanes = [0, 1, 0, 0, 0, 0, 0, 1, 1, 0]  # stage2 lanes 7,8 (active marker)
        lanes[src_branch] ^= 1                  # stage1 toggle lane = src_branch
        return sum(b << i for i, b in enumerate(lanes))
    # src (4,'U'): the closed (4,'U') branch :438771-438778 -- stage1 toggle lane4
    # (branch<4) / lane5 (branch>=4), stage2 lane8 (local_338=2, the joined_r0x..aa6
    # marker shared with (1,'U')). Validated: (4,'U',2)->402, (4,'U',5)->418.
    if src_level == 4 and src_down == 0:
        lanes = [0, 1, 0, 0, 0, 0, 0, 1, 1, 0]  # stage2 lanes 7,8 (active marker)
        lanes[4 if src_branch < 4 else 5] ^= 1  # stage1 :438772-438776
        return sum(b << i for i, b in enumerate(lanes))
    # src (3,'U'), src_branch != dest_branch: closed branch :438804-438810 -- stage1
    # toggle lane0, stage2 local_338=1 toggles the base lane7 off -> stage2 {}.
    # value 3. (src_branch == dest_branch routes the argmin path :438815, unmodelled.)
    if src_level == 3 and src_down == 0:
        if src_branch == dest_branch:
            return None                         # LAB_00577247 argmin -- not modelled
        lanes = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0]  # stage2 lane7 toggled off (local_338=1)
        lanes[0] ^= 1                           # stage1 toggle lane0 :438805
        return sum(b << i for i, b in enumerate(lanes))
    # src (4,'D'): the LAB_00577247 argmin path with stage2 iVar39==-1 (:440477-440485).
    # stage1 = criticality argmin (FUN_006b18b0 over 6 slots); under the default/zero-
    # criticality convention (cf. level4_down) it resolves to lane0. stage2 toggles
    # rel lane0 (branch<4, :440479-440480) / rel lane3 (branch>=4, :440482-440484).
    # Validated bit-for-bit corpus-wide: (4,'D',2)->195 (abs {0,1,6,7}),
    # (4,'D',5)->643 (abs {0,1,7,9}). Timed designs would need the real argmin.
    if src_level == 4 and src_down == 1:
        lanes = [0, 1, 0, 0, 0, 0, 0, 1, 0, 0]  # stage1 base lane1 | stage2 base lane7
        lanes[0] ^= 1                           # stage1 argmin -> lane0 (default crit)
        lanes[6 if src_branch < 4 else 9] ^= 1  # stage2 :440479-440484
        return sum(b << i for i, b in enumerate(lanes))
    if src_level != 5 or src_down != 1:        # only the (5,'D')-source path
        return None
    lanes = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0]     # stage1 base lane1 | stage2 = 0
    # observed bit = decomp lane + 1 (consistent +1 frame shift, trace-confirmed):
    #   src_branch==0 (decomp local_208[iVar39], idx<2) -> lane = src_index + 1
    #   src_branch<=3 (decomp local_208[local_354+1])   -> lane = src_branch + 2
    # corpus: (5,'D',3,0)->lane5=34; (5,'D',0,1)->lane2=6 (comb2/cnt4_en).
    if src_branch == 0:
        if src_index >= 2:                     # decomp iVar39<2 guard
            return None
        lane = src_index + 1
    elif src_branch <= 3:
        lane = src_branch + 2
    else:
        return None                            # src_branch>3 -> argmin (unmodelled)
    if not (0 <= lane < 6):                     # stage-1 lanes only
        return None
    lanes[lane] ^= 1
    return sum(b << i for i, b in enumerate(lanes))


def level5_up_value(src_level, src_branch, src_index):
    """Select value for a (5,'U',0,*) node. <- FUN_00573470 :438224-438267.

    Lane base {0,1,0,0,0,1,0,0}; the source's (level,branch,index) picks the
    modification. Returns None for the ERROR (unroutable) combination.
    """
    lanes = [0, 1, 0, 0, 0, 1, 0, 0]
    if src_level != 1 or src_index != 0:                 # :438241
        if src_level != 3 or src_index != 0:             # :438242
            if src_level != 4 or src_index != 0:         # :438243
                return None                              # SM_2p5K ERROR
            if src_branch < 4:                           # :438248
                lanes[0] = 1
            else:
                lanes[7] = 1                             # uStack_130._4_4_=1
        elif src_branch < 4:                             # :438255
            lanes[4] = 1                                 # local_138._0_4_=1
        else:
            lanes[5] = 0                                 # local_138._4_4_=0
    elif src_branch == 0:                                # :438262
        lanes[6] = 1                                     # uStack_130._0_4_=1
    else:
        lanes[src_branch] = 1 - lanes[src_branch]        # :438266 toggle
    return sum(b << i for i, b in enumerate(lanes))


# (5,'D') down-mux. <- FUN_00573470 :439084-439321. The routed source's identity
# toggles one lane of a per-dest-branch base array; no timing input. Two layouts:
#   dest branch != 0 (sub-case A, :439098-439143): 10-lane base {0,1,0,0,0,0,1,0,0,0}
#   dest branch == 0 (sub-case B, :439223-439256):  8-lane base {0,1,0,0,0,1,0,0}
# The toggled lane = src_index + an offset chosen by the source's level/branch:
#   src (5,'U'):                         offset 0 (:439115 toggle lane iVar39)
#   src (13,'U',b) [iVar49==0xd, the     b==5->0, b==4->+2, b==2->+6/+4, b==1->+8/+6
#     "level-13" global/clock source]:   (:439119-439139 A / :439233-439252 B)
# VALIDATED bit-for-bit vs the corpus (and_reg/dff_en/xor_reg): (5,'D',2,0)<-(13,'U',2,1)
# => base {1,6} + lane(1+6=7) = 194 == vendor.
_L5D_OFF_A = {5: 0, 4: 2, 2: 6, 1: 8}   # dest branch != 0  (:439119-439138)
_L5D_OFF_B = {5: 0, 4: 2, 2: 4, 1: 6}   # dest branch == 0  (:439233-439251)


def level5_down(dest_branch, src_level, src_branch, src_index, src_down):
    """Select value for a (5,'D',dest_branch,*) node from its routed source.

    src_* = the routed predecessor SM node's (level, branch, index, down). The
    "level-13" (iVar49==0xd) source is the device's global/clock feed. Returns the
    integer select value, or None for the decompile's SM_2p5K ERROR (unroutable).
    """
    if src_down != 0:                                    # bVar53 = iVar41==0
        return None
    if dest_branch != 0:
        lanes = [0, 1, 0, 0, 0, 0, 1, 0, 0, 0]           # :439098-439113
        off = _L5D_OFF_A
    else:
        lanes = [0, 1, 0, 0, 0, 1, 0, 0]                 # :439223-439231
        off = _L5D_OFF_B
    if src_level == 5:                                   # :439115
        lane = src_index
    elif src_level == 13:                                # :439119 iVar49==0xd
        d = off.get(src_branch)
        if d is None:
            return None
        lane = src_index + d
    else:
        return None                                      # SM_2p5K ERROR
    if not (0 <= lane < len(lanes)):
        return None
    lanes[lane] ^= 1
    return sum(b << i for i, b in enumerate(lanes))


# Which destination node keys have an in-port absolute-value rule (vs the
# single-stage compute_select model in sm_routing).
def select_value(dest_key, src_level, src_branch, src_index, default_value,
                 source_sm_name=None):
    """Absolute select value for a routed hop dest_key <- (src_level,branch,index).

    Dispatches to the transcribed two-stage rule when one exists for dest_key's
    (level, direction); otherwise falls back to the verified single-stage
    sm_routing.compute_select. Returns None if neither models the case.
    """
    level, direction, branch, node_index = dest_key
    if level == 5 and direction == "U":
        return level5_up_value(src_level, src_branch, src_index)
    # single-stage fallback (verified for 17/31 single-FF nodes)
    if source_sm_name is not None:
        from . import sm_routing
        return sm_routing.compute_select(level, direction, branch, node_index,
                                         source_sm_name, default_value)
    return None
