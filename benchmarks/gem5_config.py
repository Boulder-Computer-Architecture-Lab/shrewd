"""
gem5 SE-mode SimPoint-based Simulation Config for SPEC2017 RISC-V Benchmarks

Fast-forwards to a specified SimPoint interval using an ATOMIC CPU (caches
present but bypassed in atomic mode), then switches to a detailed O3 CPU
for the duration of one SimPoint interval (100M instructions).

Usage (invoked by gem5.opt):
    gem5.opt -d <outdir> gem5_config_simpoint_run.py \
        --binary <path> --simpoint-interval <N> \
        [--benchmark-args ...] [--stdin <file>]
"""
import argparse
import json
from pathlib import Path

from m5.stats import dump, reset

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.isas import ISA
from gem5.resources.resource import BinaryResource, FileResource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(
    description="gem5 SPEC2017 SimPoint runner (fast-forward + detailed)"
)
parser.add_argument("--binary", required=True, help="Path to the RISC-V binary")
parser.add_argument(
    "--simpoint-interval",
    type=int,
    required=True,
    help="SimPoint interval number (0-based) to simulate",
)
parser.add_argument(
    "--interval-size",
    type=int,
    default=100_000_000,
    help="Interval size in instructions (default: 100M)",
)
parser.add_argument("--stdin", default=None, help="Path to stdin input file")
parser.add_argument("--no-shrewd", action="store_true",
                    help="Disable Shrewd on the O3 core (for software-only protection)")
parser.add_argument("--shrewd-default-on", action="store_true",
                    help="Start with protection ON (100%% HW duplication, no secon needed)")
parser.add_argument("--secon-no-count", action="store_true",
                    help="Enable seconNoCountMode: secon/secoff toggle a no-count region")
parser.add_argument("--mechanism", default="FuDup",
                    choices=["FuDup", "Die", "Diva"],
                    help="How protected instructions are checked")
parser.add_argument("--diva-policy", default="Marked",
                    choices=["Marked", "Kernel", "All"],
                    help="Which instructions the Diva checker verifies")
parser.add_argument("--diva-width", type=int, default=2,
                    help="Diva checks started per cycle")
parser.add_argument("--diva-latency-scale", type=int, default=1,
                    help="Multiplies every Diva check latency (default 1: "
                         "the checked unit's latency + 1, at least 2)")
parser.add_argument("--protect-domain-only", action="store_true",
                    help="Default-on and AVF-guided protection cover only "
                         "ALU instructions with a destination register")
parser.add_argument("--avf-interval", type=int, default=0,
                    help="AVF monitor interval in committed instructions "
                         "(writes <cpu>.shaft_avf.csv; 0 disables)")
parser.add_argument("--avf-schedule", default=None,
                    help="AVF-guided protection: file of the intervals to "
                         "protect (needs --avf-interval)")
parser.add_argument("--dup-reg-no-count", action="store_true",
                    help="Software-duplicate binaries: instructions on "
                         "x31/f31 are not counted")
parser.add_argument("--shadow-policy", default="SpatialWait",
                    choices=["Opportunistic", "SpatialWait", "Temporal", "Hybrid"],
                    help="What a protected inst does when no partner FU is free")
parser.add_argument("--shadow-compare-latency", type=int, default=0,
                    help="Cycles to compare primary and shadow results")
parser.add_argument("--stuck-fu", default=None, metavar="CLASS[:N]",
                    help="Permanent fault: the N-th unit (default 0) with op "
                         "class CLASS (e.g. FloatAdd) corrupts its results")
parser.add_argument("--markers-only", action="store_true",
                    help="Execute secon/secoff (dropped at rename) but check "
                         "nothing: the cost of the markers alone")
parser.add_argument("--legacy-shadow", action="store_true",
                    help="Original model: Opportunistic shadows that take no "
                         "issue slot and no compare latency")
parser.add_argument("--checkpoint", default=None,
                    help="Restore this SimPoint checkpoint (make_checkpoints.py) "
                         "instead of fast-forwarding from the program start")
args, benchmark_args = parser.parse_known_args()

if benchmark_args and benchmark_args[0] == "--benchmark-args":
    benchmark_args = benchmark_args[1:]

requires(isa_required=ISA.RISCV)

INTERVAL_SIZE = args.interval_size
SIMPOINT_INTERVAL = args.simpoint_interval
FAST_FORWARD_INSTS = SIMPOINT_INTERVAL * INTERVAL_SIZE
if args.checkpoint:
    # The checkpoint sits warmup_insts before the interval; the ATOMIC core
    # runs those (warming the caches) and then the usual switch happens.
    meta = json.loads((Path(args.checkpoint) / "shaft_meta.json").read_text())
    if (meta["interval"] != SIMPOINT_INTERVAL or
            meta["interval_size"] != INTERVAL_SIZE):
        raise SystemExit(f"Checkpoint {args.checkpoint} is for interval "
                         f"{meta['interval']} (size {meta['interval_size']})")
    FAST_FORWARD_INSTS = meta["warmup_insts"]

# ---------------------------------------------------------------------------
# Components — caches are present from the start; ATOMIC mode bypasses their
# timing so fast-forward is nearly as fast as NoCache.
# ---------------------------------------------------------------------------
cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="32kB",
    l1i_size="32kB",
    l2_size="2MB",
)

memory = DualChannelDDR4_2400(size="32GB")

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.ATOMIC,
    switch_core_type=CPUTypes.O3,
    num_cores=1,
    isa=ISA.RISCV,
)

board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# ---------------------------------------------------------------------------
# Workload
# ---------------------------------------------------------------------------
binary_resource = BinaryResource(local_path=args.binary)
stdin_file = FileResource(local_path=args.stdin) if args.stdin else None

board.set_se_binary_workload(
    binary=binary_resource,
    arguments=benchmark_args,
    stdin_file=stdin_file,
    checkpoint=Path(args.checkpoint) if args.checkpoint else None,
)

# ---------------------------------------------------------------------------
# Enable Shrewd on the O3 (switch) cores unless --no-shrewd is set.
# Configure seconNoCountMode on the ATOMIC (start) cores so that
# instruction counting during fast-forward matches the O3 core's mode,
# ensuring all variants reach the same SimPoint tick.
# ---------------------------------------------------------------------------
# Per-PC committed/protected/checked counts for the measured interval
# (<cpu>.shaft_pcs.csv), used to estimate the run's SDC coverage.
for core in processor._switchable_cores["switch"]:
    core.core.shaftCommitLog = True
    core.core.avfInterval = args.avf_interval
    if args.avf_schedule:
        core.core.avfSchedule = str(Path(args.avf_schedule).resolve())
    core.core.shaftDomainOnly = args.protect_domain_only
    core.core.dupRegNoCount = args.dup_reg_no_count
    if args.stuck_fu:
        op_class, _, n = args.stuck_fu.partition(":")
        core.core.stuckFuOpClass = op_class
        core.core.stuckFuInstance = int(n or 0)
for core in processor._switchable_cores["start"]:
    core.core.dupRegNoCount = args.dup_reg_no_count

if args.markers_only:
    for core in processor._switchable_cores["switch"]:
        core.core.enableShrewd = False
        core.core.priorityToShadow = False
elif args.no_shrewd:
    for core in processor._switchable_cores["switch"]:
        core.core.seconNoCountMode = True
        core.core.enableShrewd = False
        core.core.priorityToShadow = False
        core.core.shrewdDefaultOn = False
    for core in processor._switchable_cores["start"]:
        core.core.seconNoCountMode = True
else:
    for core in processor._switchable_cores["switch"]:
        core.core.seconNoCountMode = bool(args.secon_no_count)
        core.core.enableShrewd = True
        core.core.shaftMechanism = args.mechanism
        core.core.divaPolicy = args.diva_policy
        core.core.divaWidth = args.diva_width
        core.core.divaLatencyScale = args.diva_latency_scale
        core.core.priorityToShadow = True
        if args.legacy_shadow:
            core.core.shadowPolicy = "Opportunistic"
            core.core.shadowUsesIssueSlot = False
            core.core.shadowCompareLatency = 0
        else:
            core.core.shadowPolicy = args.shadow_policy
            core.core.shadowCompareLatency = args.shadow_compare_latency
        if args.shrewd_default_on:
            core.core.shrewdDefaultOn = True
    for core in processor._switchable_cores["start"]:
        core.core.seconNoCountMode = bool(args.secon_no_count)

# ---------------------------------------------------------------------------
# Exit-event handler: manages fast-forward → switch → detailed simulation
# ---------------------------------------------------------------------------
def handle_max_insts():
    """Generator that handles sequential MAX_INSTS exit events."""
    # --- First exit: fast-forward complete ---
    print(f"Fast-forward of {FAST_FORWARD_INSTS:,} instructions complete.")
    print("Switching ATOMIC → O3 (detailed) core …")

    # Switch CPU (m5.switchCpus automatically drains and changes mem mode)
    processor.switch()

    # Reset stats so only the SimPoint interval is measured
    reset()

    # Schedule exit after one SimPoint interval
    simulator.schedule_max_insts(INTERVAL_SIZE)

    yield False  # continue simulation

    # --- Second exit: SimPoint interval complete ---
    print(f"SimPoint interval {SIMPOINT_INTERVAL} ({INTERVAL_SIZE:,} insts) complete.")
    dump()
    yield True  # terminate simulation


# ---------------------------------------------------------------------------
# Build and run simulator
# ---------------------------------------------------------------------------
simulator = Simulator(
    board=board,
    on_exit_event={ExitEvent.MAX_INSTS: handle_max_insts()},
)

print(f"SimPoint interval : {SIMPOINT_INTERVAL}")
print(f"Interval size     : {INTERVAL_SIZE:,} instructions")
print(f"Fast-forward      : {FAST_FORWARD_INSTS:,} instructions")
print(f"Binary            : {args.binary}")
print(f"Args              : {benchmark_args}")
print(f"Stdin             : {args.stdin}")
print("-" * 60)

if FAST_FORWARD_INSTS > 0:
    # Schedule exit event after fast-forward completes
    simulator.schedule_max_insts(FAST_FORWARD_INSTS)
else:
    # Interval 0: switch immediately and just run the detailed interval
    processor.switch()
    reset()
    simulator.schedule_max_insts(INTERVAL_SIZE)

simulator.run()

print("-" * 60)
print("Simulation finished.")
