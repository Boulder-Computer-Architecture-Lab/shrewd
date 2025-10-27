# Copyright (c) 2021 The Regents of the University of California.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
Script to run SPEC CPU2017 benchmarks with gem5.
The script expects a benchmark program name and the simulation
size. The system is fixed with 2 CPU cores, MESI Two Level system
cache and 3 GB DDR4 memory. It uses the x86 board.

This script will count the total number of instructions executed
in the ROI. It also tracks how much wallclock and simulated time.

Usage:
------
```
scons build/X86/gem5.opt
./build/X86/gem5.opt \
    configs/example/gem5_library/x86-spec-cpu2017-benchmarks.py \
    --image <full_path_to_the_spec-2017_disk_image> \
    --partition <root_partition_to_mount> \
    --benchmark <benchmark_name> \
    --size <simulation_size>
```
"""

import argparse
import os
import time
from datetime import datetime
from pathlib import Path

import m5
from m5.objects import Root
from m5.stats.gem5stats import get_simstat
from m5.util import (
    fatal,
    warn,
)

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.x86_board import X86Board
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import (
    DiskImageResource,
    obtain_resource,
)
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import (
    BootloaderResource,
    CheckpointResource,
    DiskImageResource,
    KernelResource,
    obtain_resource,
)

# PATH TO GEM5 DIRECTORY
script_path = os.path.realpath(__file__)
script_dir = os.path.dirname(script_path)
GEM5_DIR = os.path.dirname(script_dir) + "/"

def read_script(path):
    return Path(path).read_text()

# checkpoints taken using the following timer frequencies
# 100HZ, 250HZ, 650HZ, 2500HZ, 5000HZ
CHECKPOINTS = {
    "100HZ": f"{GEM5_DIR}CHECKPOINTS/HZ_100/cpt.2326543852000",
    "250HZ": f"{GEM5_DIR}CHECKPOINTS/HZ_250/cpt.2196683440400",
    "650HZ": f"{GEM5_DIR}CHECKPOINTS/HZ_650/cpt.2227680488800",
    "2500HZ": f"{GEM5_DIR}CHECKPOINTS/HZ_2500/cpt.2177215789600",
    "5000HZ": f"{GEM5_DIR}CHECKPOINTS/HZ_5000/cpt.2238599336400",
    "normal": f"{GEM5_DIR}x86_spec/boot_ckpt"
}

# kernels binaries with different timer frequencies
# 100HZ, 250HZ, 650HZ, 2500HZ, 5000HZ
KERNELS = {
    "100HZ": f"{GEM5_DIR}x86_spec/vmlinux_100HZ",
    "250HZ": f"{GEM5_DIR}x86_spec/vmlinux_250HZ",
    "650HZ": f"{GEM5_DIR}x86_spec/vmlinux_650HZ",
    "2500HZ": f"{GEM5_DIR}x86_spec/vmlinux_2500HZ",
    "5000HZ": f"{GEM5_DIR}x86_spec/vmlinux_5000HZ",
    "normal": f"{GEM5_DIR}x86_spec/vmlinux-5.2.3"
}

fast_forward_count =  {
        "AB1":  10_000_000_000,
        "AB2": 10_000_000_000,
        "AB3": 10_000_000_000,
        "AB4": 10_000_000_000,
        "AB5": 10_000_000_000,
        "AB6": 10_000_000_000,
        "AB7": 10_000_000_000,
        "AB8": 10_000_000_000,
        "AB9": 10_000_000_000,
        "AB10": 10_000_000_000,
        "AB11": 10_000_000_000,
        "AB12": 10_000_000_000,
        "CD1": 10_000_000_000,
        "CD2": 10_000_000_000,
        "CD3": 10_000_000_000,
        "CD4": 10_000_000_000,
        "CD5": 10_000_000_000,
        "CD6": 10_000_000_000,
        "CD7": 10_000_000_000,
        "CD8": 10_000_000_000,
        "CD9": 10_000_000_000,
        "CD10": 10_000_000_000,
        "CD11": 10_000_000_000,
        "CD12": 10_000_000_000,
        "505.mcf_r": 10_000_000_000,
        "hack_back_ckpt": 100_000_000_000,
        "helloworld": 1_000_000,
        "timerFrequency": 10_000_000_000,
        "pcid": 1_00_00,
        "500.perlbench_r" : 10_000_000_000,
        "502.gcc_r" : 10_000_000_000,
        "503.bwaves_r" : 10_000_000_000,
        "505.mcf_r" : 10_000_000_000,
        "507.cactuBSSN_r" : 10_000_000_000,
        "508.namd_r" : 10_000_000_000,
        "510.parest_r" : 10_000_000_000,
        "511.povray_r" : 10_000_000_000,
        "519.lbm_r" : 10_000_000_000,
        "520.omnetpp_r" : 10_000_000_000,
        "523.xalancbmk_r" : 10_000_000_000,
        "526.blender_r" : 10_000_000_000,
        "531.deepsjeng_r" : 10_000_000_000,
        "538.imagick_r" : 10_000_000_000,
        "541.leela_r" : 10_000_000_000,
        "544.nab_r" : 10_000_000_000,
        "548.exchange2_r" : 10_000_000_000,
        "549.fotonik3d_r" : 10_000_000_000,
        "554.roms_r" : 10_000_000_000,
        "557.xz_r" : 10_000_000_000,
    "synthetic": 100_000_000,
    "checkpoint_after_boot": 100_000_000_000
}

benchmark_choices = [
    "AB1",
    "AB2",
    "AB3",
    "AB4",
    "AB5",
    "AB6",
    "AB7",
    "AB8",
    "AB9",
    "AB10",
    "AB11",
    "AB12",
    "CD1",
    "CD2",
    "CD3",
    "CD4",
    "CD5",
    "CD6",
    "CD7",
    "CD8",
    "CD9",
    "CD10",
    "CD11",
    "CD12",
    "500.perlbench_r",
    "502.gcc_r",
    "503.bwaves_r",
    "505.mcf_r",
    "507.cactuBSSN_r",
    "508.namd_r",
    "510.parest_r",
    "511.povray_r",
    "519.lbm_r",
    "520.omnetpp_r",
    "523.xalancbmk_r",
    "526.blender_r",
    "531.deepsjeng_r",
    "538.imagick_r",
    "541.leela_r",
    "544.nab_r",
    "548.exchange2_r",
    "549.fotonik3d_r",
    "554.roms_r",
    "557.xz_r",
    "helloworld",
    "hack_back_ckpt",
    "pcid",
    "synthetic",
    "checkpoint_after_boot"
]

parser = argparse.ArgumentParser(
    description="Gem5 config to run arm"
)
parser.add_argument(
    "--defense",
    type=int,
    help="argument to decide whether to secure the structures",
)

parser.add_argument(
    "--benchmark",
    type=str,
    required=False,
    help="Input the benchmark program to execute.",
    choices=benchmark_choices,
)

parser.add_argument(
    "--maxInstsProcess",
    type=int,
    help="max number of instructions to exec per process",
)

parser.add_argument(
    "--invalidateCounter",
    type=int,
    help="counter for when to invalidate BTB",
)

parser.add_argument(
    "--ssbp_size",
    type=int,
    default=4096,
    help="SSBP size",
)

parser.add_argument(
    "--vt_size",
    type=int,
    default=64,
    help="SSBP VT size",
)

parser.add_argument(
    "--numWorkloads",
    type=int,
    default=2,
    help="num Workloads",
)

parser.add_argument(
    "--btbSize",
    type=int,
    default=4096,
    help="size of the BTB",
)

parser.add_argument(
    "--vltSize",
    type=int,
    default=4096,
    help="size of the VLT",
)

parser.add_argument(
    "--iMAPSize",
    type=int,
    default=4096,
    help="size of the iMAP",
)

parser.add_argument(
    "--timer_frequency",
    type=str,
    default="normal",
    help="timer frequency to use for the kernel and checkpoint",
)

parser.add_argument(
    "--enableShrewd",
    action="store_true",
    help="Enable special instruction queue functionality",
)

parser.add_argument(
    "--priorityToShadow",
    action="store_true",
    help="Enable priority to shadow functionality",
)

args = parser.parse_args()

# Associatity is 8
# Here we setup the parameters of the l1 and l2 caches.
cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="32kB",
    l1i_size="32kB",
    l2_size="2MB"
)

# cache_hierarchy = NoCache()

# Memory: Dual Channel DDR4 2400 DRAM device.
# The X86 board only supports 3 GB of main memory.
memory = DualChannelDDR4_2400(size="3GB")

# Here we setup the processor. This is a special switchable processor in which
# a starting core type and a switch core type must be specified. Once a
# configuration is instantiated a user may call `processor.switch()` to switch
# from the starting core types to the switch core types. In this simulation
# we start with KVM cores to simulate the OS boot, then switch to the Timing
# cores for the command we wish to run after boot.

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.ATOMIC,
    switch_core_type=CPUTypes.O3,
    isa=ISA.X86,
    num_cores=1,
)

# Here we setup the board. The X86Board allows for Full-System X86 simulations
kernel_resource = KernelResource(
    local_path=os.path.abspath(
        KERNELS[args.timer_frequency]
    )
)
image_resource = DiskImageResource(
    local_path=os.path.abspath(
        f"{GEM5_DIR}x86_spec/scpc_x86.img"
        # f"{GEM5_DIR}m5_binaries/disks/x86-ubuntu-24.04.img"
    ),
    root_partition="1",
)

checkpoint_resource = CheckpointResource(
    local_path=os.path.abspath(
        CHECKPOINTS["normal"]
    )
)

board = X86Board(
    clk_freq="3.5GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

board.set_kernel_disk_workload(
    kernel=kernel_resource,
    disk_image=image_resource,
     kernel_args = [
         "earlyprintk=ttyS0",
         "console=ttyS0",
         "lpj=7999923",
         # X86Board uses an IDE controller here; kernel sees it as /dev/hda.
         "root=/dev/hda1",
         "rootfstype=ext4",
         "rw",
         # Ensure the kernel waits for the disk to appear before mounting.
         "rootwait",
     ],
    # bootloader=bootloader_resource,
    checkpoint=checkpoint_resource,
    readfile_contents=read_script(
        f"{GEM5_DIR}x86_spec/spec_cpu_scripts/{args.benchmark}.rcS"
        # f"{GEM5_DIR}configs/boot/{args.benchmark}.rcS"
        # f"{GEM5_DIR}spec_cpu_scripts/{args.benchmark}.rcS"
        # f"{GEM5_DIR}configs/boot/hack_back_ckpt.rcS"
    ),
)

# Define instruction counts
# the first element shows the number of instructions to warm up the structure by
# the second element shows the number of instructions to use in the recording stats
fast_forward_inst = fast_forward_count[args.benchmark]
instruction_counts = [fast_forward_inst, 30_000_000_000]

# switch cpus after the warmup
def start_accurate_sim():
    print("\t==========\n\tswitched cpus\n\t==========\n\t")
    processor.switch()
    # Re-set enableShrewd and priorityToShadow after switching to O3 CPU
    print(f"DEBUG: Attempting to set enableShrewd = {args.enableShrewd}, priorityToShadow = {args.priorityToShadow}")
    for core in processor.get_cores():
        print(f"DEBUG: Processing core: {core}")
        try:
            print(f"DEBUG: core.core = {core.core}")
            cpp_core = core.core.getCCObject()
            print(f"DEBUG: cpp_core = {cpp_core}")
            if hasattr(cpp_core, 'setEnableShrewd'):
                print(f"DEBUG: Found setEnableShrewd method, calling it...")
                cpp_core.setEnableShrewd(args.enableShrewd)
                print(f"DEBUG: Successfully set enableShrewd = {args.enableShrewd} after CPU switch")
            else:
                print(f"DEBUG: cpp_core does not have setEnableShrewd method")
            if hasattr(cpp_core, 'setPriorityToShadow'):
                print(f"DEBUG: Found setPriorityToShadow method, calling it...")
                cpp_core.setPriorityToShadow(args.priorityToShadow)
                print(f"DEBUG: Successfully set priorityToShadow = {args.priorityToShadow} after CPU switch")
            else:
                print(f"DEBUG: cpp_core does not have setPriorityToShadow method")
        except AttributeError as e:
            print(f"DEBUG: AttributeError: {e}")
        except Exception as e:
            print(f"DEBUG: Unexpected error: {type(e).__name__}: {e}")
    m5.stats.reset()
    simulator.schedule_max_insts(instruction_counts[1])
    yield False

# exit_counter = 0  # global or external to the handler
# def handle_exit():
#     global exit_counter
#     exit_counter += 1
#     print(f"Booting linux (called {exit_counter} times)")
#     yield False  # Always continue

# Create simulator
simulator = Simulator(
    board=board,
    on_exit_event={
        ExitEvent.MAX_INSTS: start_accurate_sim(),
        # ExitEvent.EXIT: handle_exit(),
        },
)

print(f"running {args.benchmark}.rcS")
print(f"DEBUG: Initial args.enableShrewd value = {args.enableShrewd}")
print(f"DEBUG: Initial args.priorityToShadow value = {args.priorityToShadow}")
#simulator.defense(args.defense)
#simulator.maxIntsProcess(args.maxIntsProcess)
#simulator.invalidateCounter(args.invalidateCounter)
#simulator.ssbpSize(args.ssbp_size)
#simulator.vtSize(args.vt_size)
print(f"\tRunning simulation for {instruction_counts[0]} warmup instructions...")
simulator.schedule_max_insts(instruction_counts[0])
#simulator.numWorkloads(args.numWorkloads)
#simulator.btbSize(args.btbSize)
#simulator.vltSize(args.vltSize)
#simulator.iMAPSize(args.iMAPSize)
exit_event = simulator.run()
print("...COMPLETED...")
