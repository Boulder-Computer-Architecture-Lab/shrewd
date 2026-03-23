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
args, benchmark_args = parser.parse_known_args()

if benchmark_args and benchmark_args[0] == "--benchmark-args":
    benchmark_args = benchmark_args[1:]

requires(isa_required=ISA.RISCV)

INTERVAL_SIZE = args.interval_size
SIMPOINT_INTERVAL = args.simpoint_interval
FAST_FORWARD_INSTS = SIMPOINT_INTERVAL * INTERVAL_SIZE

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
)

# ---------------------------------------------------------------------------
# Enable Shrewd on the O3 (switch) cores unless --no-shrewd is set.
# Configure seconNoCountMode on the ATOMIC (start) cores so that
# instruction counting during fast-forward matches the O3 core's mode,
# ensuring all variants reach the same SimPoint tick.
# ---------------------------------------------------------------------------
if args.no_shrewd:
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
        core.core.priorityToShadow = True
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
