"""
gem5 SE-mode Simulation Config for SPEC2017 RISC-V Benchmarks

This script is invoked by template.py (the orchestrator). It configures and runs
a single gem5 simulation with:
- Single core RISC-V O3 CPU
- MESI two-level cache hierarchy
- DDR4 2400 memory

Usage (invoked by gem5.opt):
    gem5.opt -d <outdir> gem5_config.py --binary <path> [--args ...] [--stdin <file>]
"""
import argparse
import sys

from m5 import stats

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import PrivateL1PrivateL2CacheHierarchy

from gem5.isas import ISA
from gem5.resources.resource import BinaryResource, FileResource
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

# Parse arguments.
# Use parse_known_args so that benchmark args starting with '--' (e.g.
# namd's --input) are not consumed by this parser.
parser = argparse.ArgumentParser(description="gem5 SPEC2017 benchmark runner")
parser.add_argument("--binary", required=True, help="Path to the RISC-V binary")
parser.add_argument("--stdin", default=None,
                    help="Path to stdin input file")
args, benchmark_args = parser.parse_known_args()
# Strip the leading '--benchmark-args' sentinel if the orchestrator passed it
if benchmark_args and benchmark_args[0] == "--benchmark-args":
    benchmark_args = benchmark_args[1:]

requires(isa_required=ISA.RISCV)

cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="32kB",   # 48kB is becoming standard (Alder Lake/Zen 4), 32kB is also fine
    l1i_size="32kB",
    l2_size="2MB",     # Large private L2
)

# Memory: Dual Channel DDR4 2400 DRAM device.
memory = DualChannelDDR4_2400(size="32GB")


#processor = SimpleProcessor(
#    cpu_type=CPUTypes.O3,
#    isa=ISA.RISCV,
#    num_cores=1,
#)

processor = SimpleProcessor(
    cpu_type=CPUTypes.ATOMIC,
    isa=ISA.RISCV,
    num_cores=1,
)

board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Set up the workload
binary_resource = BinaryResource(local_path=args.binary)
stdin_file = FileResource(local_path=args.stdin) if args.stdin else None

board.set_se_binary_workload(
    binary=binary_resource,
    arguments=benchmark_args,
    stdin_file=stdin_file,
)

# Enable Shrewd and Priority to Shadow.
# Set at the Python param level (before instantiation) so that the board
# does not need to be rooted yet — calling getCCObject() here would fail
# because the Root has not been created and Parent proxies cannot resolve.
#for core in board.processor.get_cores():
#    core.core.enableShrewd = True
#    core.core.priorityToShadow = True

# Enable SimPoint profiling — generates Basic Block Vector (BBV) data
# that can later be fed to the SimPoint tool to identify representative
# simulation intervals.  Interval size is 100 M instructions.
# Uses BaseAtomicSimpleCPU.addSimPointProbe() from
# src/cpu/simple/BaseAtomicSimpleCPU.py
for core in board.processor.get_cores():
    core.core.addSimPointProbe(interval=100_000_000)  # 100 M instructions
    core.core.max_insts_any_thread = 30_000_000_000

# Create and run the simulator
simulator = Simulator(board=board)

print(f"Starting gem5 simulation: {args.binary}")
print(f"  Args: {benchmark_args}")
print(f"  Stdin: {args.stdin}")
print("-" * 60)

simulator.run()

print("-" * 60)
print("Simulation completed successfully!")
stats.dump()
