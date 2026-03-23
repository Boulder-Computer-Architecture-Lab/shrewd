#!/usr/bin/env python3
"""
SPEC2017 SimPoint-based Performance Evaluation Orchestrator

For each benchmark, runs the top-3 SimPoint intervals identified in
top3_summary.txt.  Each SimPoint is executed as a separate gem5 process
that fast-forwards with an ATOMIC CPU then switches to O3 for the
100M-instruction interval.

Results are written under results/<benchmark>/<variant>/simpoint_<interval>/.

Usage:
    python3 run_campaign_simpoints.py [--gem5 <path>] [--jobs N] \
        [--benchmarks b1,b2,...] [--variants v1,v2,...] \
        [--simpoint-summary <path>]
"""
import argparse
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
GEM5_BINARY = PROJECT_ROOT / "build" / "ALL" / "gem5.opt"
GEM5_CONFIG = SCRIPT_DIR / "gem5_config.py"
BASELINE_DIR = SCRIPT_DIR / "baseline"
PROTECTED_DIR = SCRIPT_DIR / "protected_binaries"
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DUPLICATE_DIR = SCRIPT_DIR / "results_duplicate"
SIMPOINT_SUMMARY = SCRIPT_DIR / "top3_summary.txt"

PROTECTION_LEVELS = ["5pct", "10pct", "20pct", "50pct", "100pct"]
DUPLICATE_LEVELS = ["dup_5pct", "dup_10pct", "dup_20pct", "dup_50pct"]

# Benchmarks that have protected/duplicate binaries under protected_binaries/
PROTECTED_BENCHMARKS = ["bwaves", "cactuBSSN", "cam4", "lbm", "nab", "namd", "parest", "roms"]

# ── Benchmark configs (mirrors run_campaign.py) ──────────────────────────
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
    # "povray": {
    #     "spec_num": "511.povray_r",
    #     "binary": "povray_r_base.riscv",
    #     "args": ["SPEC-benchmark-ref.ini"],
    #     "stdin": None,
    # },
    "lbm": {
        "spec_num": "519.lbm_r",
        "binary": "lbm_r_base.riscv",
        "args": ["3000", "reference.dat", "0", "0", "100_100_130_ldc.of"],
        "stdin": None,
    },
    # "wrf": {
    #     "spec_num": "521.wrf_r",
    #     "binary": "wrf_r_base.riscv",
    #     "args": [],
    #     "stdin": None,
    # },
    # "blender": {
    #     "spec_num": "526.blender_r",
    #     "binary": "blender_r_base.riscv",
    #     "args": ["sh3_no_char.blend", "--render-output", "sh3_no_char_",
    #              "--threads", "1", "-b", "-F", "RAWTGA", "-s", "849",
    #              "-e", "849", "-a"],
    #     "stdin": None,
    # },
    "cam4": {
        "spec_num": "527.cam4_r",
        "binary": "cam4_r_base.riscv",
        "args": [],
        "stdin": None,
    },
    # "imagick": {
    #     "spec_num": "538.imagick_r",
    #     "binary": "imagick_r_base.riscv",
    #     "args": ["-limit", "disk", "0", "refrate_input.tga", "-edge", "41",
    #              "-resample", "181%", "-emboss", "31", "-colorspace", "YUV",
    #              "-mean-shift", "19x19+15%", "-resize", "30%",
    #              "refrate_output.tga"],
    #     "stdin": None,
    # },
    "nab": {
        "spec_num": "544.nab_r",
        "binary": "nab_r_base.riscv",
        "args": ["1am0", "1122214447", "122"],
        "stdin": None,
    },
    # "fotonik3d": {
    #     "spec_num": "549.fotonik3d_r",
    #     "binary": "fotonik3d_r_base.riscv",
    #     "args": [],
    #     "stdin": None,
    # },
    "roms": {
        "spec_num": "554.roms_r",
        "binary": "roms_r_base.riscv",
        "args": [],
        "stdin": "ocean_benchmark2.in.x",
    },
}


def _protected_binary(base_binary):
    """Derive secon_secoff binary name from baseline name."""
    return base_binary.replace(".riscv", ".secon_secoff.riscv")


def _duplicate_binary(base_binary):
    """Derive software-duplicate binary name from baseline name."""
    return base_binary.replace(".riscv", ".duplicate.riscv")


# ── Parse top3_summary.txt ───────────────────────────────────────────────

def parse_simpoint_summary(summary_path):
    """
    Parse top3_summary.txt and return a dict:
        { "blender": [(interval, weight), ...], ... }

    Benchmark names in the file have a '_baseline' suffix which is stripped.
    """
    simpoints = {}
    current_bench = None

    with open(summary_path) as f:
        for line in f:
            line = line.strip()

            # Match "Benchmark : blender_baseline"
            m = re.match(r"^Benchmark\s*:\s*(\S+)", line)
            if m:
                bench_raw = m.group(1)
                # Strip '_baseline' suffix to get the short name
                current_bench = re.sub(r"_baseline$", "", bench_raw)
                simpoints[current_bench] = []
                continue

            # Match data rows:  "1       233           0.204013    20"
            if current_bench is not None:
                m = re.match(
                    r"^\d+\s+(\d+)\s+([\d.]+)\s+\d+\s*$", line
                )
                if m:
                    interval = int(m.group(1))
                    weight = float(m.group(2))
                    simpoints[current_bench].append((interval, weight))

    return simpoints


# ── Build run list ────────────────────────────────────────────────────────

def build_run_list(benchmarks, variants, simpoints_map):
    """
    Build list of tuples:
        (bench, variant, binary_path, run_dir, args, stdin_path,
         interval, weight, no_shrewd)
    One entry per (benchmark, variant, simpoint interval).
    """
    runs = []

    for bench in benchmarks:
        cfg = BENCHMARK_CONFIG[bench]
        spec_num = cfg["spec_num"]
        sp_list = simpoints_map.get(bench, [])

        if not sp_list:
            print(f"WARNING: No simpoints for benchmark '{bench}', skipping")
            continue

        if "baseline" in variants:
            run_dir = BASELINE_DIR / spec_num / "run" / "run_base_refrate_riscv.0000"
            binary_path = BASELINE_DIR / spec_num / "exe" / cfg["binary"]
            stdin_path = str(run_dir / cfg["stdin"]) if cfg["stdin"] else None

            if binary_path.exists():
                for interval, weight in sp_list:
                    runs.append((
                        bench, "baseline", str(binary_path),
                        str(run_dir), cfg["args"], stdin_path,
                        interval, weight, "None",
                    ))
            else:
                print(f"WARNING: baseline binary not found: {binary_path}")

        # Hardware-protected variants (secon_secoff, Shrewd enabled)
        if bench in PROTECTED_BENCHMARKS:
            prot_binary = _protected_binary(cfg["binary"])
            for level in PROTECTION_LEVELS:
                if level not in variants:
                    continue
                if level == "100pct":
                    # 100% HW duplication: baseline binary + Shrewd on O3
                    # duplicates everything (no SW annotations needed)
                    run_dir = BASELINE_DIR / spec_num / "run" / "run_base_refrate_riscv.0000"
                    binary_path = BASELINE_DIR / spec_num / "exe" / cfg["binary"]
                    stdin_path = str(run_dir / cfg["stdin"]) if cfg["stdin"] else None
                    if binary_path.exists():
                        for interval, weight in sp_list:
                            runs.append((
                                bench, "100pct", str(binary_path),
                                str(run_dir), cfg["args"], stdin_path,
                                interval, weight, "shrewd_default_on",
                            ))
                    else:
                        print(f"WARNING: baseline binary not found for 100pct: {binary_path}")
                    continue
                prot_dir = PROTECTED_DIR / bench / level
                binary_path = prot_dir / prot_binary
                stdin_path = (
                    str(prot_dir / cfg["stdin"]) if cfg["stdin"] else None
                )
                if binary_path.exists():
                    for interval, weight in sp_list:
                        runs.append((
                            bench, level, str(binary_path),
                            str(prot_dir), cfg["args"], stdin_path,
                            interval, weight, "None",
                        ))
                else:
                    print(f"WARNING: protected binary not found: {binary_path}")

        # Software-duplicate variants (no Shrewd)
        if bench in PROTECTED_BENCHMARKS:
            dup_binary = _duplicate_binary(cfg["binary"])
            for dup_level in DUPLICATE_LEVELS:
                if dup_level not in variants:
                    continue
                # dup_5pct -> 5pct  (strip "dup_" prefix for directory)
                pct_dir_name = dup_level.removeprefix("dup_")
                prot_dir = PROTECTED_DIR / bench / pct_dir_name
                binary_path = prot_dir / dup_binary
                stdin_path = (
                    str(prot_dir / cfg["stdin"]) if cfg["stdin"] else None
                )
                if binary_path.exists():
                    for interval, weight in sp_list:
                        runs.append((
                            bench, dup_level, str(binary_path),
                            str(prot_dir), cfg["args"], stdin_path,
                            interval, weight, "no_shrewd",
                        ))
                else:
                    print(f"WARNING: duplicate binary not found: {binary_path}")

    return runs


# ── Single simulation launcher ────────────────────────────────────────────

def run_gem5_simulation(run_spec, gem5_binary, gem5_config, results_dir,
                        duplicate_results_dir):
    """
    Launch one gem5 SimPoint simulation.  Returns a result dict.
    """
    (bench, variant, binary_path, run_dir, bench_args,
     stdin_path, interval, weight, shrewd_param) = run_spec

    # Duplicate variants go to a separate results directory
    base_dir = duplicate_results_dir if variant.startswith("dup_") else results_dir
    # For dup_5pct store under the pct name (5pct) in the duplicate tree
    variant_dir = variant.removeprefix("dup_") if variant.startswith("dup_") else variant
    outdir = os.path.join(base_dir, bench, variant_dir,
                          f"simpoint_{interval}")
    os.makedirs(outdir, exist_ok=True)

    cmd = [
        str(gem5_binary),
        "-d", outdir,
        str(gem5_config),
        "--binary", binary_path,
        "--simpoint-interval", str(interval),
    ]
    if shrewd_param == "shrewd_default_on":
        cmd.append("--shrewd-default-on")
    elif shrewd_param == "no_shrewd":
        # SW-duplicate binaries: disable HW Shrewd, enable secon/secoff no-count
        # mode so region markers don't inflate max_insts accounting.
        cmd.append("--no-shrewd")
        cmd.append("--secon-no-count")
    elif shrewd_param != "None":
        print(f"Invalid shrewd_param value: {shrewd_param}")
        sys.exit(1) # Exits with a status code of 1 (indicating an error)

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
    return {
        "benchmark": bench,
        "variant": variant,
        "interval": interval,
        "weight": weight,
        "returncode": result.returncode,
        "elapsed": elapsed,
        "outdir": outdir,
    }


def parse_stats(stats_path):
    """Extract cycles and committed instructions from a gem5 stats.txt."""
    cycles = None
    insts = None
    if not os.path.exists(stats_path):
        return cycles, insts
    with open(stats_path, "r") as f:
        for line in f:
            if "board.processor.switch.core.numCycles" in line:
                try:
                    cycles = int(line.split()[1])
                except (IndexError, ValueError):
                    pass
            if "board.processor.switch.core.committedInsts" in line:
                try:
                    insts = int(line.split()[1])
                except (IndexError, ValueError):
                    pass
    return cycles, insts


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Run SPEC2017 SimPoint-sampled simulations on gem5")
    parser.add_argument("--gem5", type=str, default=str(GEM5_BINARY),
                        help="Path to gem5.opt binary")
    parser.add_argument("--config", type=str, default=str(GEM5_CONFIG),
                        help="Path to gem5 SimPoint config script")
    parser.add_argument("--results-dir", type=str, default=str(RESULTS_DIR),
                        help="Directory for output results")
    parser.add_argument("--duplicate-results-dir", type=str,
                        default=str(RESULTS_DUPLICATE_DIR),
                        help="Separate directory for software-duplicate results")
    parser.add_argument("--jobs", "-j", type=int, default=int(float(os.cpu_count())*0.75),
                        help="Max parallel gem5 processes")
    parser.add_argument("--benchmarks", type=str, default=None,
                        help="Comma-separated benchmarks (default: all)")
    parser.add_argument("--variants", type=str, default=None,
                        help="Comma-separated variants "
                             "(default: baseline,5pct,10pct,20pct,50pct)")
    parser.add_argument("--simpoint-summary", type=str,
                        default=str(SIMPOINT_SUMMARY),
                        help="Path to top3_summary.txt")
    cli = parser.parse_args()

    gem5_binary = Path(cli.gem5)
    gem5_config = Path(cli.config)

    if not gem5_binary.exists():
        sys.exit(f"ERROR: gem5 binary not found: {gem5_binary}")
    if not gem5_config.exists():
        sys.exit(f"ERROR: gem5 config not found: {gem5_config}")

    # Parse SimPoint summary
    summary_path = Path(cli.simpoint_summary)
    if not summary_path.exists():
        sys.exit(f"ERROR: SimPoint summary not found: {summary_path}")
    simpoints_map = parse_simpoint_summary(summary_path)

    # Determine benchmarks
    all_benchmarks = list(BENCHMARK_CONFIG.keys())
    if cli.benchmarks:
        benchmarks = [b.strip() for b in cli.benchmarks.split(",")]
        for b in benchmarks:
            if b not in BENCHMARK_CONFIG:
                sys.exit(f"ERROR: Unknown benchmark '{b}'. "
                         f"Available: {', '.join(all_benchmarks)}")
    else:
        benchmarks = all_benchmarks

    # Determine variants
    all_variants = ["baseline"] + PROTECTION_LEVELS + DUPLICATE_LEVELS
    if cli.variants:
        variants = [v.strip() for v in cli.variants.split(",")]
        for v in variants:
            if v not in all_variants:
                sys.exit(f"ERROR: Unknown variant '{v}'. "
                         f"Available: {', '.join(all_variants)}")
    else:
        variants = all_variants

    runs = build_run_list(benchmarks, variants, simpoints_map)
    if not runs:
        sys.exit("ERROR: No valid benchmark runs found. Check paths.")

    print(f"{'=' * 78}")
    print(f"SPEC2017 gem5 SimPoint-based Evaluation")
    print(f"{'=' * 78}")
    print(f"gem5 binary  : {gem5_binary}")
    print(f"gem5 config  : {gem5_config}")
    print(f"Benchmarks   : {', '.join(benchmarks)}")
    print(f"Variants     : {', '.join(variants)}")
    print(f"Total runs   : {len(runs)}")
    print(f"Parallel jobs: {cli.jobs}")
    print(f"Results dir  : {cli.results_dir}")
    print(f"{'=' * 78}")

    os.makedirs(cli.results_dir, exist_ok=True)
    os.makedirs(cli.duplicate_results_dir, exist_ok=True)

    completed = 0
    failed = 0
    results_summary = []

    with ProcessPoolExecutor(max_workers=cli.jobs) as executor:
        future_to_run = {}
        for run_spec in runs:
            future = executor.submit(
                run_gem5_simulation, run_spec, gem5_binary, gem5_config,
                cli.results_dir, cli.duplicate_results_dir,
            )
            future_to_run[future] = run_spec

        for future in as_completed(future_to_run):
            run_spec = future_to_run[future]
            bench, variant = run_spec[0], run_spec[1]
            interval = run_spec[6]
            try:
                res = future.result()
                stats_path = os.path.join(res["outdir"], "stats.txt")
                cycles, insts = parse_stats(stats_path)

                status = ("OK" if res["returncode"] == 0
                          else f"FAIL(rc={res['returncode']})")
                if res["returncode"] != 0:
                    failed += 1
                else:
                    completed += 1

                res["cycles"] = cycles
                res["instructions"] = insts
                res["status"] = status
                results_summary.append(res)

                cyc_str = f"{cycles:,}" if cycles else "N/A"
                inst_str = f"{insts:,}" if insts else "N/A"
                print(
                    f"  [{completed + failed}/{len(runs)}] "
                    f"{bench:12s} {variant:10s} sp={interval:<5d} "
                    f"{status:10s} {res['elapsed']:8.1f}s  "
                    f"cycles={cyc_str}  insts={inst_str}"
                )

            except Exception as e:
                failed += 1
                print(
                    f"  [{completed + failed}/{len(runs)}] "
                    f"{bench:12s} {variant:10s} sp={interval:<5d} "
                    f"EXCEPTION: {e}"
                )

    # ── Summary table ─────────────────────────────────────────────────
    print(f"\n{'=' * 78}")
    print(f"SUMMARY: {completed} succeeded, {failed} failed out of {len(runs)}")
    print(f"{'=' * 78}")
    header = (f"{'Benchmark':12s} {'Variant':10s} {'SP':>5s} {'Weight':>8s} "
              f"{'Status':10s} {'Time(s)':>8s} {'Cycles':>15s} "
              f"{'Instructions':>15s}")
    print(header)
    print(f"{'-' * 78}")
    for r in sorted(results_summary,
                    key=lambda x: (x["benchmark"], x["variant"],
                                   x["interval"])):
        cyc_str = f"{r['cycles']:,}" if r.get("cycles") else "N/A"
        inst_str = f"{r['instructions']:,}" if r.get("instructions") else "N/A"
        print(
            f"{r['benchmark']:12s} {r['variant']:10s} {r['interval']:5d} "
            f"{r['weight']:8.4f} {r['status']:10s} {r['elapsed']:8.1f} "
            f"{cyc_str:>15s} {inst_str:>15s}"
        )

    # ── CSV output ────────────────────────────────────────────────────
    csv_path = os.path.join(cli.results_dir, "simpoint_summary.csv")
    with open(csv_path, "w") as f:
        f.write("benchmark,variant,simpoint_interval,weight,status,"
                "elapsed_s,cycles,instructions\n")
        for r in sorted(results_summary,
                        key=lambda x: (x["benchmark"], x["variant"],
                                       x["interval"])):
            cyc = r.get("cycles", "") or ""
            inst = r.get("instructions", "") or ""
            f.write(
                f"{r['benchmark']},{r['variant']},{r['interval']},"
                f"{r['weight']:.6f},{r['status']},{r['elapsed']:.1f},"
                f"{cyc},{inst}\n"
            )
    print(f"\nResults written to {csv_path}")


if __name__ == "__main__":
    main()
