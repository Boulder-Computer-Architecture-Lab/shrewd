"""
gem5 SE-mode O3-only Simulation Config for RISC-V binaries.

This keeps the same CLI shape as the SimPoint config, but runs a single O3 core
from start to finish (no ATOMIC warmup, no processor switch, no MAX_INSTS
scheduling).
"""
import argparse

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

    if not args.no_shrewd:
        sim_core.enableShrewd = True
        sim_core.priorityToShadow = True
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
