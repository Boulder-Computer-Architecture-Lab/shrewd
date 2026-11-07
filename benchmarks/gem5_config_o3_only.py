"""
gem5 SE-mode O3-only Simulation Config for RISC-V binaries.

This keeps the same CLI shape as the SimPoint config, but runs a single O3 core
from start to finish (no ATOMIC warmup, no processor switch, no MAX_INSTS
scheduling).
"""
import argparse
from pathlib import Path

from m5.stats import dump, reset

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.isas import ISA
from gem5.resources.resource import BinaryResource, FileResource
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

# ---------------------------------------------------------------------------
# Argument parsing (kept compatible with gem5_config.py)
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(
    description="gem5 RISC-V O3-only runner (no fast-forward, no max insts)"
)
parser.add_argument("--binary", required=True, help="Path to the RISC-V binary")
parser.add_argument(
    "--simpoint-interval",
    type=int,
    default=0,
    help="Accepted for compatibility; ignored in O3-only mode",
)
parser.add_argument(
    "--interval-size",
    type=int,
    default=100_000_000,
    help="Accepted for compatibility; ignored in O3-only mode",
)
parser.add_argument("--stdin", default=None, help="Path to stdin input file")
parser.add_argument("--no-shrewd", action="store_true",
                    help="Disable Shrewd on the O3 core (for software-only protection)")
parser.add_argument("--shrewd-default-on", action="store_true",
                    help="Start with protection ON (100%% HW duplication, no secon needed)")
parser.add_argument("--secon-no-count", action="store_true",
                    help="Enable seconNoCountMode on O3 core")
parser.add_argument("--fault-injection-window", type=int, default=0,
                    help="Enable fault injector with given window size (0 disables)")
parser.add_argument(
    "--run-to-completion",
    action="store_true",
    help="Accepted for compatibility; O3-only mode always runs to completion",
)
parser.add_argument("--mechanism", default="FuDup",
                    choices=["FuDup", "Die", "Diva", "RfTwin"],
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
parser.add_argument("--shadow-policy", default="SpatialWait",
                    choices=["Opportunistic", "SpatialWait", "Temporal", "Hybrid"],
                    help="What a protected inst does when no partner FU is free")
parser.add_argument("--shadow-compare-latency", type=int, default=0,
                    help="Cycles to compare primary and shadow results")
parser.add_argument("--stuck-fu", default=None, metavar="CLASS[:N]",
                    help="Permanent fault: the N-th unit (default 0) with op "
                         "class CLASS (e.g. FloatAdd) corrupts its results")
parser.add_argument("--partition", default="Off",
                    choices=["Off", "Marked", "All"],
                    help="Protected memory partition: Marked, a page "
                         "joins it when a protected store to it commits "
                         "and its writes are then mirrored; All, every "
                         "write is mirrored")
parser.add_argument("--partition-migrate-cycles", type=int, default=6000,
                    help="Cycles commit stalls when a page moves into "
                         "the partition")
parser.add_argument("--markers-only", action="store_true",
                    help="Execute secon/secoff (dropped at rename) but check "
                         "nothing: the cost of the markers alone")
parser.add_argument("--legacy-shadow", action="store_true",
                    help="Original model: Opportunistic shadows that take no "
                         "issue slot and no compare latency")
args, benchmark_args = parser.parse_known_args()

if benchmark_args and benchmark_args[0] == "--benchmark-args":
    benchmark_args = benchmark_args[1:]

requires(isa_required=ISA.RISCV)

# ---------------------------------------------------------------------------
# Components
# ---------------------------------------------------------------------------
cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="32kB",
    l1i_size="32kB",
    l2_size="2MB",
)

memory = DualChannelDDR4_2400(size="32GB")

processor = SimpleProcessor(
    cpu_type=CPUTypes.O3,
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
)

# ---------------------------------------------------------------------------
# Configure O3 core features
# ---------------------------------------------------------------------------
for core in processor.get_cores():
    sim_core = core.get_simobject()
    sim_core.shaftCommitLog = True
    sim_core.avfInterval = args.avf_interval
    if args.avf_schedule:
        sim_core.avfSchedule = str(Path(args.avf_schedule).resolve())
    sim_core.shaftDomainOnly = args.protect_domain_only
    sim_core.shaftPartition = args.partition
    sim_core.partitionMigrateCycles = args.partition_migrate_cycles
    if args.stuck_fu:
        op_class, _, n = args.stuck_fu.partition(":")
        sim_core.stuckFuOpClass = op_class
        sim_core.stuckFuInstance = int(n or 0)

    if not args.no_shrewd and not args.markers_only:
        sim_core.enableShrewd = True
        sim_core.shaftMechanism = args.mechanism
        sim_core.divaPolicy = args.diva_policy
        sim_core.divaWidth = args.diva_width
        sim_core.divaLatencyScale = args.diva_latency_scale
        sim_core.priorityToShadow = True
        if args.legacy_shadow:
            sim_core.shadowPolicy = "Opportunistic"
            sim_core.shadowUsesIssueSlot = False
            sim_core.shadowCompareLatency = 0
        else:
            sim_core.shadowPolicy = args.shadow_policy
            sim_core.shadowCompareLatency = args.shadow_compare_latency
        if args.shrewd_default_on:
            sim_core.shrewdDefaultOn = True

    if args.secon_no_count:
        sim_core.seconNoCountMode = True

    if args.fault_injection_window and args.fault_injection_window > 0:
        sim_core.faultInjectionWindow = args.fault_injection_window

# ---------------------------------------------------------------------------
# Build and run simulator
# ---------------------------------------------------------------------------
simulator = Simulator(board=board)

print("Run mode          : O3-only run-to-completion")
print(f"Binary            : {args.binary}")
print(f"Args              : {benchmark_args}")
print(f"Stdin             : {args.stdin}")
print(f"Shrewd enabled    : {not args.no_shrewd}")
print(f"No-count mode     : {args.secon_no_count}")
print(f"Fault inj window  : {args.fault_injection_window}")
print("-" * 60)

reset()
simulator.run()
dump()

print("-" * 60)
print("Simulation finished.")
