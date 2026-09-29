"""Device delay tables + the criticality-driven primitives that resolve the
timing-gated switch-matrix muxes.

~107 of the open SM bits are design-invariant clock/disable bits; ~112 are TIMING-
DRIVEN -- the vendor toggles a mux select by comparing net criticalities (Anet) and
picking the least-critical (argmin). This module holds:
  1. the DEVICE DELAY TABLES, transcribed verbatim from the vendor config echo
     (PNR_PACK_PLACE.log; byte-identical corpus-wide, so pure device constants); and
  2. the DECISION PRIMITIVES from the decompile: per-net criticality lookup
     (FUN_00680580), the argmin selector (FUN_006b18b0), and the two-stage slot
     structure combining them (FUN @ 0x562... region).

The Anet values they consume come from STA (ptiming.cpp); today read from the oracle
(timing_oracle), reproducing them from scratch is the remaining depth. The decision
logic is independent of where Anet comes from, so it is testable now.
"""

# --------------------------------------------------------------------------- #
# DEVICE DELAY TABLES                                                          #
# Verbatim from PNR_PACK_PLACE.log .placement (the vendor's config echo), which #
# timing_oracle verifies is identical across all 12 corpus designs => device   #
# constants. Picoseconds.                                                      #
# --------------------------------------------------------------------------- #

# Net interconnect delay. [0] = the placement-stage per-net delay used for every
# ordinary single-driver net; [1] is the long-net value. VALIDATED: every timed
# net in the corpus has load_arrival - driver_arrival == INTERCONNECT_DELAY[0].
INTERCONNECT_DELAY = [5000, 10000]

# Per-logic-port delay, indexed by the vendor's internal port-type enum [0..27].
# 10000 = a disabled/unused path sentinel; 0 = pass-through; the live cell delays
# are 1000/250/710 at [5]/[6]/[7]/[8].
LOGIC_DELAY = [
    0, 10000, 10000, 10000, 10000, 1000, 250, 1000, 710, 0,
    0, 10000, 10000, 10000, 0, 0, 0, 0, 10000, 10000,
    10000, 10000, 10000, 0, 0, 10000, 10000, 0,
]

# Routing delay per switch-matrix level. RBB_L_DELAY[level][idx]: 13 levels x 25
# indices. Levels 0..11 are a uniform 1800 per hop; level 12 (the clock spine) is
# 3000 for idx 0..23 and 710 for idx 24.
_RBB_L_ROW_UNIFORM = [1800] * 25
_RBB_L_ROW_SPINE = [3000] * 24 + [710]
RBB_L_DELAY = [list(_RBB_L_ROW_UNIFORM) for _ in range(12)] + [list(_RBB_L_ROW_SPINE)]

# Timing knobs (PNR_PACK_PLACE.log .placement).
CLK_UNCERTAINTY = 150       # clock-uncertainty guardband added to setup checks
HOLD_TIME_THRESHOLD = 150   # hold-fix threshold
MIN_NET_CRIT = 0.05         # criticality floor for routed nets
IGNORE_INPUT_OUTPUT_DELAY = 0

# Sentinel criticalities FUN_00680580 returns for non-real edges (IEEE-754 of the
# constants it returns: 0xbff0... = -1.0 empty edge, 0xc000... = -2.0 invalid).
CRIT_EMPTY = -1.0    # candidate edge list empty            (return 0xbff0000000000000)
CRIT_INVALID = -2.0  # candidate node value negative        (return 0xc000000000000000)
CRIT_UNSET = 0.0     # net index == 0x7fffffff (unrouted)   (return 0)

# MCMF (min-cost max-flow) timing-driven router delay/penalty params -- the fine
# routing-delay model used during the trial route that produces the criticality
# (Anet) the SM decisions consume. These are the binary's config DEFAULTS, read
# via FUN_004d4fe0("pppopt_mcmf_params_*", default) @decomp:357772-357810
# (place/pppopt_mcmf.cpp). Picoseconds for the *_delay entries.
MCMF_PARAMS = {
    "still_excess_penalty_coeff": 30,        # 0x1e
    "still_excess_reward_quadvac_thre": 100,
    "still_excess_reward_quadvac_coeff": 5,
    "still_excess_penalty_min": 20,          # 0x14
    "zero_excess_eval_cost_too": 1,
    "neg_excess_pipeff1_nocost_dist_thre": 0,
    "neg_excess_pipeff2_nocost_dist_thre": 0,
    "neg_excess_penalty_slack_offset": 100,
    "neg_excess_penalty_slack_coeff": 1,
    "neg_excess_quad_delay": 250,            # 0xfa
    "neg_excess_tile_cross_delay": 450,      # 0x1c2  -- per tile-boundary crossing
    "pppff_delay": 95,                       # 0x5f
    "pipeff_delay": 20,                      # 0x14
    "extra_penalty_pipeff_pppff_tile_boundary": 3,
    "extra_penalty_pipeff_load_tile_boundary": 7,
    "atbl0_driver_side_nff1_slack_to_wns_margin": 1000,
    "atbl0_load_side_slack_to_wns_margin": 200,
    "over_limit_penalty": 100,
}

# worst_timing == REQ_PERIOD - WNS. The timing context packs (REQ_PERIOD, WNS) as
# (low32, high32) at obj+0x18 (read @decomp:357768; "MCMF Parameters: REQ_PERIOD
# =%d WNS=%d" @357770). The criticality kernel's ref/wns (sta.criticality_from_
# slack) ARE this REQ_PERIOD/WNS, so ref-wns == worst_timing (validated bit-exact).
ARRIVAL_FIXED_ADDEND = 0x41a   # 1050 ps fixed addend in route arrival prop @decomp:545201


# --------------------------------------------------------------------------- #
# ELEMENTARY DELAYS                                                            #
# --------------------------------------------------------------------------- #

def interconnect_delay(long_net=False):
    """Per-net delay used by the placement-stage timer."""
    return INTERCONNECT_DELAY[1 if long_net else 0]


# --------------------------------------------------------------------------- #
# PER-NODE ROUTING DELAY (the value the detailed router minimizes)             #
# --------------------------------------------------------------------------- #
# The tables above are the PLACEMENT-stage delay model (coarse, per-level). The
# detailed router instead minimizes a per-NODE timing delay
#     node.delay = max(corner0, corner1) + base       # FUN_00b970f0 @1271916-1271933
# where (corner0, corner1) = (rise, fall) are BOTH packed in the rgraph edge weight
# and decoded by route_timing.decode_corners (rise = weight>>13, fall =
# (weight>>1)&0xfff; <- rproblem.cpp FUN_0050cbf0 @378337). node.delay /
# make_delay_cost are exposed in route_timing (kept there because they need the
# loaded rgraph, not just the device tables). The vendor inputs, all pure artifacts:
#   * route_timing.segment_corners / segment_delay -- the per-segment (rise, fall)
#       and their max, validated against the PNR_ROUTE.log #timing arrivals across
#       the corpus (max-corner match ~72%, up from ~59% for rise alone; ~58% of
#       hops match BOTH corners exactly). The residual is the path-dependent
#       trial-route layer (tile crossings / source launch), not a missing corner.
#   * branch_delay.output_delays  -- the driver-cell launch delay (branchdly_def,
#       4-corner average) added at the net source.
#   * branch_delay.cross_tile_delays -- the placement wirelength delay table
#       (placedelaylib.txt, QUAD_SIZE 16); branch_delay.timing_pins -- the per-RBB
#       pin list (timing_def) grounding the branchdly port indices.
# The vendor's 4D table at *(obj+0x6f0) is a grid-AVERAGE of these same per-edge
# corners, keyed by (level, direction, branch, index) (FUN_006a4a80 @602233-602328),
# used as the trial router's coarse congestion-cost approximation -- not a separate
# source of corner data. NODE_DELAY_BASE mirrors route_timing.NODE_DELAY_BASE for
# documentation; the per-node delay is genuinely per-edge (an rgraph node's delay
# depends on which upstream segment reaches it -- verified: 36864/45376 multi-driver
# nodes have differing incoming corners), so it lives on the edge, not one node field.
NODE_DELAY_BASE = 0        # *(obj + 0x658)            # TODO: identify non-zero placement value


# --------------------------------------------------------------------------- #
# DECISION PRIMITIVES                                                          #
# --------------------------------------------------------------------------- #

def argmin(values):
    """Index of the first minimum.

    <- FUN_006b18b0 @ decomp/eda-placer.real.c:610315
       (scan i=1..n-1, keep index where values[i] < running min; ties keep first)
    """
    if not values:
        return 0
    best_i, best = 0, values[0]
    for i in range(1, len(values)):
        if values[i] < best:
            best_i, best = i, values[i]
    return best_i


# The two-stage select decision (FUN @ decomp/eda-placer.real.c:430414-430439):
# four candidate slots, each scored as the MAX of two candidate-edge criticalities
# fetched by FUN_00680580(graph, node, A, B, 0, 0); then FUN_006b18b0 (argmin)
# picks the slot whose worst criticality is smallest, and that slot's select bit
# is toggled (local_2a8[winner] ^= 1). The (A,B) pairs that index the node's edge
# lists, per slot:
TWO_STAGE_SLOTS = (
    ((1, 1), (0, 0)),   # slot 0
    ((1, 4), (0, 1)),   # slot 1
    ((2, 2), (0, 2)),   # slot 2
    ((2, 5), (0, 3)),   # slot 3
)


# The (4,'D') two-stage slots (FUN_00573470 @decomp:439348-439372). Same shape as
# TWO_STAGE_SLOTS (4 slots, each scored max of two FUN_00680580 edge criticalities,
# then argmin -> toggle the value lane) but with this case's own (A,B,idx,mode)
# edge keys. Edge key = the 4 trailing args of FUN_00680580(graph,node,A,B,idx,mode).
FOUR_DOWN_SLOTS = (
    ((3, 1, 0, 0), (1, 0, 0, 0)),   # slot 0
    ((3, 4, 0, 0), (1, 1, 0, 0)),   # slot 1
    ((5, 0, 0, 1), (1, 2, 0, 0)),   # slot 2
    ((5, 0, 1, 1), (1, 3, 0, 0)),   # slot 3
)


# The general (1,'D')/(0,'D') down argmin (FUN_00573470 LAB_00578402
# @decomp:439843-439914): FIVE slots, each scored as the MAX of up to THREE
# candidate-edge criticalities (mixed list modes), then FUN_006b18b0(local_98, 5)
# picks the stage-1 lane toggled in local_208 (base {lane1}, @439788). Edge key =
# the 4 trailing args of FUN_00680580(graph, node, A, B, idx, mode). This is the
# binding the (1,'D') two-stage argmin needs -- the SAME derivable node->net->Anet
# binding as FOUR_DOWN_SLOTS (Section 2.7); resolve `crit_of_edge` over it for the
# stage-1 lane exactly as the (4,'D') path does.
# The STAGE-1 lane is the argmin over these 5 slots via crit_of_edge (the SAME
# node->net->Anet binding the (4,'D') path uses). The STAGE-2 lane is decided by the
# routed source: a (3,'D') source sets stage-2 lane0 (src_branch<4) / lane4 (>=4)
# at LAB_00578402's stage-2 setup (:440263-440271); other 'D' sources hit the
# writer's SM_2p5K ERROR (no stage-2). Transcribed in sm_writer.level1_down_argmin /
# level1_down_value and wired in sm_validate (closed the (1,'D') argmin family,
# +222 nodes). The FUN_0067df70/FUN_0067ddc0 calls + the @440090 -1-guarded loop are
# in the param_6 != 0 VALIDATION branch (read-back/compare), not the write path, so
# the emitted value comes from the LAB_00575076/LAB_005758ef writes, not those calls.
L1_DOWN_SLOTS = (
    ((5, 0, 0, 1), (3, 1, 0, 0), (1, 0, 0, 0)),   # slot 0
    ((5, 0, 1, 1), (3, 4, 0, 0), (1, 1, 0, 0)),   # slot 1
    ((5, 1, 0, 1), (4, 2, 0, 0), (1, 2, 0, 0)),   # slot 2
    ((5, 2, 0, 1), (4, 5, 0, 0), (1, 3, 0, 0)),   # slot 3
    ((5, 3, 0, 1), (4, 5, 0, 1), (4, 2, 0, 0)),   # slot 4
)


def slot_scores(slots, crit_of_edge):
    """Score each slot as the MAX of its candidate-edge criticalities.

    `crit_of_edge(edge_key) -> float`; edge_key is the case's (A,B,idx,mode) tuple.
    Slots may carry 2 edges (FOUR_DOWN_SLOTS/TWO_STAGE_SLOTS) or 3 (L1_DOWN_SLOTS);
    the score is the max over all of them (the nested if/else max-reductions in the
    decompile, e.g. FUN_00573470 :439843-439913)."""
    return [max(crit_of_edge(e) for e in slot) for slot in slots]


def resolve_slots(slots, crit_of_edge):
    """argmin over the slot scores -> the value lane to toggle. <- FUN_006b18b0."""
    return argmin(slot_scores(slots, crit_of_edge))


def two_stage_slot_scores(crit_of_edge):
    """Score each of the 4 slots as max of its two candidate-edge criticalities.

    `crit_of_edge((A, B)) -> float` returns the criticality of the net on the
    node's (direction A, branch B, index 0) edge -- the FUN_00680580 value,
    including the CRIT_EMPTY / CRIT_INVALID / CRIT_UNSET sentinels.

    <- FUN @ decomp/eda-placer.real.c:430414-430437 (dVar = max(call,call) x4)
    """
    scores = []
    for e1, e2 in TWO_STAGE_SLOTS:
        scores.append(max(crit_of_edge(e1), crit_of_edge(e2)))
    return scores


def resolve_two_stage(crit_of_edge):
    """Return the slot index whose select bit the vendor toggles.

    <- FUN @ decomp/eda-placer.real.c:430438 (FUN_006b18b0(local_b8,4))
    """
    return argmin(two_stage_slot_scores(crit_of_edge))


# --------------------------------------------------------------------------- #
# NOTE -- crit_of_edge: what it returns, and the precise gating piece           #
# --------------------------------------------------------------------------- #
# FUN_00680580(graph, net_array, A, B, index, mode) -- read in full
# (decomp/eda-placer.real.c:578488-578556) -- resolves an edge key to a
# CRITICALITY as follows:
#   * it indexes the node's per-category edge-list array  *(graph + 0xd8)[A]
#     (A in {0,1,2,3,5}, the TWO_STAGE/FOUR_DOWN slot's first field), then [B]
#     (branch; mode 0 takes *plVar2, mode != 0 takes plVar2[9] -- the incoming vs
#     fanout list), then [index] -- yielding an edge entry that carries a NET
#     INDEX iVar6 (:578539/578567);
#   * it returns  *(net_array[iVar6] + 0x50)  (:578553) -- the net's Anet, i.e.
#     EXACTLY timing_oracle.Net.anet (the +0x50 field). Sentinels: empty edge
#     list -> -1.0 (CRIT_EMPTY), net index < 0 -> -2.0 (CRIT_INVALID), net index
#     == 0x7fffffff -> 0 (CRIT_UNSET).
# So the criticality VALUE is oracle-exposed -- timing_oracle.Net.anet supplies
# every net's Anet, and porting the path-based STA that recomputes it
# (place/ptiming.cpp; forward arrival via clk-latency + LOGIC_DELAY +
# INTERCONNECT_DELAY/RBB_L_DELAY, backward required time, per-pin slack, [0,1]
# normalization floored at MIN_NET_CRIT) is validated against that oracle.
#
# The piece that actually GATES the 234 (1,'D')/(4,'D') argmin SM nodes is NOT the
# Anet value -- it is the per-node candidate-edge -> net BINDING that crit_of_edge
# indexes ( *(graph+0xd8)[A][B][index] ). That binding is built by the router and
# appears in NEITHER oracle: PNR_ROUTE.log carries only the single ROUTED hop per
# net (not the node's full candidate-edge population), and the .vm carries cell
# placement, not per-node edge bindings. The argmin compares the worst Anet across
# a slot's TWO candidate edges, most of which are other (or empty) nets, so the
# outcome cannot be reconstructed from the routed hop alone. Closing the 234 nodes
# therefore needs the router's candidate-edge binding pass (the +0xd8 lists), not
# merely the STA -- a located source, transcribed, NOT a corpus-fitted substitute.
