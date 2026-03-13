#!/usr/bin/env python3
"""
SPEC2017 Performance Evaluation Orchestrator

Runs all SPEC2017 benchmarks (baseline + protected binaries at various
protection levels) in parallel via gem5 SE-mode simulations.

Each benchmark runs as a separate gem5.opt process with its own output
directory under results/<benchmark>/<variant>/.

Usage:
    python3 template.py [--gem5 <path>] [--jobs N] [--benchmarks b1,b2,...] [--variants v1,v2,...]
"""
import argparse
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
GEM5_BINARY = PROJECT_ROOT / "build" / "ALL" / "gem5.opt"
GEM5_CONFIG = SCRIPT_DIR / "gem5_config_generate_simpoints.py"
BASELINE_DIR = SCRIPT_DIR / "baseline"
PROTECTED_DIR = SCRIPT_DIR / "protected_binaries"
RESULTS_DIR = SCRIPT_DIR / "results"

PROTECTION_LEVELS = ["5pct", "10pct", "20pct", "50pct"]

# Benchmark configs: binary name, spec number, args, stdin file (relative to run dir)
# Derived from run_campaign.py BENCHMARK_CONFIG
BENCHMARK_CONFIG = {
    "bwaves": {
        "spec_num": "503.bwaves_r",
        "binary": "bwaves_r_base.riscv",
        "args": ["bwaves_1"],
        "stdin": "bwaves_1.in",
    },
    "cactuBSSN": {
        "spec_num": "507.cactuBSSN_r",
        "binary": "cactusBSSN_r_base.riscv",
        "args": ["spec_ref.par"],
        "stdin": None,
    },
    "namd": {
        "spec_num": "508.namd_r",
        "binary": "namd_r_base.riscv",
        "args": ["--input", "apoa1.input", "--output", "apoa1.ref.output",
                 "--iterations", "65"],
        "stdin": None,
    },
    "parest": {
        "spec_num": "510.parest_r",
        "binary": "parest_r_base.riscv",
        "args": ["ref.prm"],
        "stdin": None,
    },
    "povray": {
        "spec_num": "511.povray_r",
        "binary": "povray_r_base.riscv",
        "args": ["SPEC-benchmark-ref.ini"],
        "stdin": None,
    },
    "lbm": {
        "spec_num": "519.lbm_r",
        "binary": "lbm_r_base.riscv",
        "args": ["3000", "reference.dat", "0", "0", "100_100_130_ldc.of"],
        "stdin": None,
    },
    "wrf": {
        "spec_num": "521.wrf_r",
        "binary": "wrf_r_base.riscv",
        "args": [],
        "stdin": None,
    },
    "blender": {
        "spec_num": "526.blender_r",
        "binary": "blender_r_base.riscv",
        "args": ["sh3_no_char.blend", "--render-output", "sh3_no_char_",
                 "--threads", "1", "-b", "-F", "RAWTGA", "-s", "849",
                 "-e", "849", "-a"],
        "stdin": None,
    },
    "cam4": {
        "spec_num": "527.cam4_r",
        "binary": "cam4_r_base.riscv",
        "args": [],
        "stdin": None,
    },
    "imagick": {
        "spec_num": "538.imagick_r",
        "binary": "imagick_r_base.riscv",
        "args": ["-limit", "disk", "0", "refrate_input.tga", "-edge", "41",
                 "-resample", "181%", "-emboss", "31", "-colorspace", "YUV",
                 "-mean-shift", "19x19+15%", "-resize", "30%",
                 "refrate_output.tga"],
        "stdin": None,
    },
    "nab": {
        "spec_num": "544.nab_r",
        "binary": "nab_r_base.riscv",
        "args": ["1am0", "1122214447", "122"],
        "stdin": None,
    },
    "fotonik3d": {
        "spec_num": "549.fotonik3d_r",
        "binary": "fotonik3d_r_base.riscv",
        "args": [],
        "stdin": None,
    },
    "roms": {
        "spec_num": "554.roms_r",
        "binary": "roms_r_base.riscv",
        "args": [],
        "stdin": "ocean_benchmark2.in.x",
    },
}

# Protected binary naming: baseline uses <name>_base.riscv,
# protected uses <name>_base.secon_secoff.riscv
PROTECTED_BINARY_NAMES = {
    "lbm":    "lbm_r_base.secon_secoff.riscv",
    "nab":    "nab_r_base.secon_secoff.riscv",
    "namd":   "namd_r_base.secon_secoff.riscv",
    "povray": "povray_r_base.secon_secoff.riscv",
}


def build_run_list(benchmarks, variants):
    """
    Build a list of (benchmark, variant, binary_path, run_dir, args, stdin_path)
    tuples for all requested runs.
    """
    runs = []

    for bench in benchmarks:
        cfg = BENCHMARK_CONFIG[bench]
        spec_num = cfg["spec_num"]

        if "baseline" in variants:
            run_dir = BASELINE_DIR / spec_num / "run" / "run_base_refrate_riscv.0000"
            binary_path = BASELINE_DIR / spec_num / "exe" / cfg["binary"]
            stdin_path = str(run_dir / cfg["stdin"]) if cfg["stdin"] else None

            if binary_path.exists():
                runs.append((bench, "baseline", str(binary_path),
                              str(run_dir), cfg["args"], stdin_path))
            else:
                print(f"WARNING: baseline binary not found: {binary_path}")

        # Protected variants (only for benchmarks that have them)
        if bench in PROTECTED_BINARY_NAMES:
            protected_binary = PROTECTED_BINARY_NAMES[bench]
            for level in PROTECTION_LEVELS:
                if level not in variants:
                    continue
                prot_dir = PROTECTED_DIR / bench / level
                binary_path = prot_dir / protected_binary
                stdin_path = str(prot_dir / cfg["stdin"]) if cfg["stdin"] else None

                if binary_path.exists():
                    runs.append((bench, level, str(binary_path),
                                  str(prot_dir), cfg["args"], stdin_path))
                else:
                    print(f"WARNING: protected binary not found: {binary_path}")

    return runs


def run_gem5_simulation(run_spec, gem5_binary, gem5_config, results_dir):
    """
    Launch a single gem5 simulation as a subprocess.
    Returns (benchmark, variant, returncode, elapsed_seconds, outdir).
    """
    bench, variant, binary_path, run_dir, bench_args, stdin_path = run_spec

    outdir = os.path.join(results_dir, bench, variant)
    os.makedirs(outdir, exist_ok=True)

    cmd = [
        str(gem5_binary),
        "-d", outdir,
        str(gem5_config),
        "--binary", binary_path,
    ]
    if bench_args:
        cmd += ["--benchmark-args"] + bench_args
    if stdin_path:
        cmd += ["--stdin", stdin_path]

    log_path = os.path.join(outdir, "gem5_run.log")
    start = time.time()

    with open(log_path, "w") as log_f:
        result = subprocess.run(
            cmd,
            cwd=run_dir,
            stdout=log_f,
            stderr=subprocess.STDOUT,
            timeout=None,
        )

    elapsed = time.time() - start
    return bench, variant, result.returncode, elapsed, outdir


def parse_stats(stats_path):
    """Extract cycles and committed instructions from a gem5 stats.txt."""
    cycles = None
    insts = None
    if not os.path.exists(stats_path):
        return cycles, insts
    with open(stats_path, "r") as f:
        for line in f:
            if "board.processor.cores.core.numCycles" in line:
                try:
                    cycles = int(line.split()[1])
                except (IndexError, ValueError):
                    pass
            if "board.processor.cores.core.committedInsts" in line:
                try:
                    insts = int(line.split()[1])
                except (IndexError, ValueError):
                    pass
    return cycles, insts


def main():
    parser = argparse.ArgumentParser(
        description="Run SPEC2017 benchmarks on gem5 (baseline + protected)")
    parser.add_argument("--gem5", type=str, default=str(GEM5_BINARY),
                        help="Path to gem5.opt binary")
    parser.add_argument("--config", type=str, default=str(GEM5_CONFIG),
                        help="Path to gem5 config script")
    parser.add_argument("--results-dir", type=str, default=str(RESULTS_DIR),
                        help="Directory for output results")
    parser.add_argument("--jobs", "-j", type=int, default=os.cpu_count(),
                        help="Max parallel gem5 processes")
    parser.add_argument("--benchmarks", type=str, default=None,
                        help="Comma-separated list of benchmarks to run "
                             "(default: all)")
    parser.add_argument("--variants", type=str, default=None,
                        help="Comma-separated list of variants to run "
                             "(default: baseline,5pct,10pct,20pct,50pct)")
    args = parser.parse_args()

    gem5_binary = Path(args.gem5)
    gem5_config = Path(args.config)

    if not gem5_binary.exists():
        sys.exit(f"ERROR: gem5 binary not found: {gem5_binary}")
    if not gem5_config.exists():
        sys.exit(f"ERROR: gem5 config not found: {gem5_config}")

    all_benchmarks = list(BENCHMARK_CONFIG.keys())
    if args.benchmarks:
        benchmarks = [b.strip() for b in args.benchmarks.split(",")]
        for b in benchmarks:
            if b not in BENCHMARK_CONFIG:
                sys.exit(f"ERROR: Unknown benchmark '{b}'. "
                         f"Available: {', '.join(all_benchmarks)}")
    else:
        benchmarks = all_benchmarks

    all_variants = ["baseline"] + PROTECTION_LEVELS
    if args.variants:
        variants = [v.strip() for v in args.variants.split(",")]
        for v in variants:
            if v not in all_variants:
                sys.exit(f"ERROR: Unknown variant '{v}'. "
                         f"Available: {', '.join(all_variants)}")
    else:
        variants = all_variants

    runs = build_run_list(benchmarks, variants)
    if not runs:
        sys.exit("ERROR: No valid benchmark runs found. Check paths.")

    print(f"{'=' * 70}")
    print(f"SPEC2017 gem5 Performance Evaluation")
    print(f"{'=' * 70}")
    print(f"gem5 binary:  {gem5_binary}")
    print(f"Benchmarks:   {', '.join(benchmarks)}")
    print(f"Variants:     {', '.join(variants)}")
    print(f"Total runs:   {len(runs)}")
    print(f"Parallel jobs: {args.jobs}")
    print(f"Results dir:  {args.results_dir}")
    print(f"{'=' * 70}")

    os.makedirs(args.results_dir, exist_ok=True)

    completed = 0
    failed = 0
    results_summary = []

    with ProcessPoolExecutor(max_workers=args.jobs) as executor:
        future_to_run = {}
        for run_spec in runs:
            future = executor.submit(
                run_gem5_simulation, run_spec, gem5_binary, gem5_config,
                args.results_dir,
            )
            future_to_run[future] = run_spec

        for future in as_completed(future_to_run):
            run_spec = future_to_run[future]
            bench, variant = run_spec[0], run_spec[1]
            try:
                bench, variant, retcode, elapsed, outdir = future.result()
                stats_path = os.path.join(outdir, "stats.txt")
                cycles, insts = parse_stats(stats_path)

                status = "OK" if retcode == 0 else f"FAIL(rc={retcode})"
                if retcode != 0:
                    failed += 1
                else:
                    completed += 1

                results_summary.append({
                    "benchmark": bench,
                    "variant": variant,
                    "status": status,
                    "elapsed": elapsed,
                    "cycles": cycles,
                    "instructions": insts,
                    "outdir": outdir,
                })

                cyc_str = f"{cycles:,}" if cycles else "N/A"
                inst_str = f"{insts:,}" if insts else "N/A"
                print(f"  [{completed + failed}/{len(runs)}] "
                      f"{bench:12s} {variant:10s} {status:10s} "
                      f"{elapsed:8.1f}s  cycles={cyc_str}  insts={inst_str}")

            except Exception as e:
                failed += 1
                print(f"  [{completed + failed}/{len(runs)}] "
                      f"{bench:12s} {variant:10s} EXCEPTION: {e}")

    # Print summary table
    print(f"\n{'=' * 70}")
    print(f"SUMMARY: {completed} succeeded, {failed} failed out of {len(runs)}")
    print(f"{'=' * 70}")
    print(f"{'Benchmark':12s} {'Variant':10s} {'Status':10s} "
          f"{'Time(s)':>8s} {'Cycles':>15s} {'Instructions':>15s}")
    print(f"{'-' * 70}")
    for r in sorted(results_summary, key=lambda x: (x["benchmark"], x["variant"])):
        cyc_str = f"{r['cycles']:,}" if r["cycles"] else "N/A"
        inst_str = f"{r['instructions']:,}" if r["instructions"] else "N/A"
        print(f"{r['benchmark']:12s} {r['variant']:10s} {r['status']:10s} "
              f"{r['elapsed']:8.1f} {cyc_str:>15s} {inst_str:>15s}")

    # Write CSV summary
    csv_path = os.path.join(args.results_dir, "summary.csv")
    with open(csv_path, "w") as f:
        f.write("benchmark,variant,status,elapsed_s,cycles,instructions\n")
        for r in sorted(results_summary,
                        key=lambda x: (x["benchmark"], x["variant"])):
            cyc = r["cycles"] if r["cycles"] else ""
            inst = r["instructions"] if r["instructions"] else ""
            f.write(f"{r['benchmark']},{r['variant']},{r['status']},"
                    f"{r['elapsed']:.1f},{cyc},{inst}\n")
    print(f"\nResults written to {csv_path}")


if __name__ == "__main__":
    main()