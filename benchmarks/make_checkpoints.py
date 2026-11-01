"""
Take SimPoint checkpoints for one binary in a single ATOMIC fast-forward.

For each SimPoint interval it saves a checkpoint WARMUP instructions before
the interval starts, counted exactly like gem5_config.py (secon/secoff, and
whole SW-duplicate regions with --secon-no-count, are not counted). A run
then restores, warms the caches in ATOMIC mode and switches to O3 right at
the interval start (gem5_config.py --checkpoint).

The board and processor match gem5_config.py, so object paths match on
restore; there are no caches here, which makes the fast-forward much faster
(caches hold no state in a checkpoint).

Usage (invoked by gem5.opt):
    gem5.opt -d <outdir> make_checkpoints.py --binary <path> \
        --simpoint-intervals 15,6,231 [--warmup-insts 50000000] \
        [--secon-no-count] [--stdin <file>] [--benchmark-args ...]

Writes <outdir>/simpoint_<N>/ (a gem5 checkpoint plus shaft_meta.json).
"""
import argparse
import json
from pathlib import Path

import m5

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import BinaryResource, FileResource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

parser = argparse.ArgumentParser(
    description="Save SimPoint checkpoints (one ATOMIC pass, no caches)"
)
parser.add_argument("--binary", required=True, help="Path to the RISC-V binary")
parser.add_argument("--simpoint-intervals", required=True,
                    help="Comma-separated SimPoint interval numbers")
parser.add_argument("--interval-size", type=int, default=100_000_000,
                    help="Interval size in instructions (default: 100M)")
parser.add_argument("--warmup-insts", type=int, default=50_000_000,
                    help="Checkpoint this many instructions before each "
                         "interval, for cache warm-up after restore")
parser.add_argument("--stdin", default=None, help="Path to stdin input file")
parser.add_argument("--secon-no-count", action="store_true",
                    help="secon/secoff delimit uncounted SW-duplicate regions")
args, benchmark_args = parser.parse_known_args()

if benchmark_args and benchmark_args[0] == "--benchmark-args":
    benchmark_args = benchmark_args[1:]

requires(isa_required=ISA.RISCV)

# (checkpoint instruction count, interval) in execution order; an interval
# that starts within the warm-up of the program start needs no checkpoint.
points = sorted(
    (interval * args.interval_size - args.warmup_insts, interval)
    for interval in {int(i) for i in args.simpoint_intervals.split(",")}
    if interval * args.interval_size > args.warmup_insts
)
if not points:
    raise SystemExit("No interval starts late enough to need a checkpoint.")

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.ATOMIC,
    switch_core_type=CPUTypes.O3,
    num_cores=1,
    isa=ISA.RISCV,
)
board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=DualChannelDDR4_2400(size="32GB"),
    cache_hierarchy=NoCache(),
)
board.set_se_binary_workload(
    binary=BinaryResource(local_path=args.binary),
    arguments=benchmark_args,
    stdin_file=FileResource(local_path=args.stdin) if args.stdin else None,
)

# Count instructions the same way as gem5_config.py.
for core in processor._switchable_cores["start"]:
    core.core.seconNoCountMode = args.secon_no_count
for core in processor._switchable_cores["switch"]:
    core.core.seconNoCountMode = args.secon_no_count

outdir = Path(m5.options.outdir)


def take_checkpoints():
    previous = 0
    for count, interval in points:
        path = outdir / f"simpoint_{interval}"
        simulator.save_checkpoint(path)
        meta = {
            "binary": args.binary,
            "interval": interval,
            "interval_size": args.interval_size,
            "checkpoint_insts": count,
            "warmup_insts": interval * args.interval_size - count,
            "secon_no_count": args.secon_no_count,
        }
        (path / "shaft_meta.json").write_text(json.dumps(meta, indent=2))
        print(f"Saved {path} at {count:,} instructions")
        previous = count
        following = [c for c, _ in points if c > previous]
        if not following:
            yield True
            return
        simulator.schedule_max_insts(following[0] - previous)
        yield False


simulator = Simulator(
    board=board,
    on_exit_event={ExitEvent.MAX_INSTS: take_checkpoints()},
)
simulator.schedule_max_insts(points[0][0])
simulator.run()
print("All checkpoints saved.")
