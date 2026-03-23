#!/usr/bin/env python3
"""
Recovery experiment runner for recovery binaries:
- main.baseline.riscv: baseline run (no error injection)
- main.secon.riscv:    runs with Shrewd enabled + injector enabled
- main.sw.riscv:       runs with Shrewd disabled + injector enabled

Experiments:
1. One baseline run (no error injection)
2. Recovery runs with fault injection at interval sizes:
   - 1,000,000
   - 1,000
   - 1

Outputs are written under:
results_recovery/baseline/ or results_recovery/<variant>/interval_<N>/
"""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
GEM5_BINARY = PROJECT_ROOT / "build" / "ALL" / "gem5.opt"
GEM5_CONFIG = SCRIPT_DIR / "gem5_config_o3_only.py"
RECOVERY_DIR = SCRIPT_DIR / "recovery_binaries"
RESULTS_DIR = SCRIPT_DIR / "results_recovery"

INTERVAL_SIZES = [1_000_000, 1_000, 1]


def parse_stats(stats_path: Path):
    cycles = None
    insts = None
    if not stats_path.exists():
        return cycles, insts

    with stats_path.open("r") as f:
        for line in f:
            if "numCycles" in line and "board.processor" in line:
                try:
                    cycles = int(line.split()[1])
                except (IndexError, ValueError):
                    pass
            if "committedInsts" in line and "board.processor" in line:
                try:
                    insts = int(line.split()[1])
                except (IndexError, ValueError):
                    pass

    return cycles, insts


def run_one(gem5_binary: Path, gem5_config: Path, binary: Path, variant: str | None,
            interval_size: int | None, results_dir: Path, is_baseline: bool = False):
    # Determine output directory based on baseline or recovery mode
    if is_baseline:
        outdir = results_dir / "baseline"
    else:
        outdir = results_dir / variant / f"interval_{interval_size}"
    outdir.mkdir(parents=True, exist_ok=True)

    cmd = [
        str(gem5_binary),
        "-d", str(outdir),
        str(gem5_config),
        "--binary", str(binary),
        "--run-to-completion",
    ]

    # Add fault injection parameters only for non-baseline runs
    if not is_baseline:
        cmd.insert(4, "--fault-injection-window")
        cmd.insert(5, str(interval_size))

    # Add variant-specific flags (only for non-baseline runs)
    if not is_baseline:
        if variant == "secon":
            pass
        elif variant == "sw":
            cmd.append("--no-shrewd")
            cmd.append("--secon-no-count")
        else:
            raise ValueError(f"Unknown variant: {variant}")

    log_path = outdir / "gem5_run.log"
    start = time.time()
    with log_path.open("w") as logf:
        result = subprocess.run(
            cmd,
            cwd=str(RECOVERY_DIR),
            stdout=logf,
            stderr=subprocess.STDOUT,
            timeout=None,
        )

    elapsed = time.time() - start
    cycles, insts = parse_stats(outdir / "stats.txt")

    return {
        "variant": variant,
        "interval_size": interval_size,
        "is_baseline": is_baseline,
        "returncode": result.returncode,
        "elapsed": elapsed,
        "cycles": cycles,
        "instructions": insts,
        "outdir": str(outdir),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Run recovery evaluation binaries with fixed intervals")
    parser.add_argument("--gem5", type=str, default=str(GEM5_BINARY),
                        help="Path to gem5.opt binary")
    parser.add_argument("--config", type=str, default=str(GEM5_CONFIG),
                        help="Path to gem5 config script")
    parser.add_argument("--results-dir", type=str, default=str(RESULTS_DIR),
                        help="Directory for recovery experiment results")
    parser.add_argument("--jobs", type=int, default=18,
                        help="Number of concurrent gem5 jobs (default: 18)")
    cli = parser.parse_args()

    gem5_binary = Path(cli.gem5)
    gem5_config = Path(cli.config)
    results_dir = Path(cli.results_dir)

    secon_binary = RECOVERY_DIR / "main.secon.riscv"
    sw_binary = RECOVERY_DIR / "main.sw.riscv"

    if not gem5_binary.exists():
        sys.exit(f"ERROR: gem5 binary not found: {gem5_binary}")
    if not gem5_config.exists():
        sys.exit(f"ERROR: gem5 config not found: {gem5_config}")
    if not secon_binary.exists():
        sys.exit(f"ERROR: secon binary not found: {secon_binary}")
    if not sw_binary.exists():
        sys.exit(f"ERROR: sw binary not found: {sw_binary}")
    if cli.jobs <= 0:
        sys.exit("ERROR: --jobs must be >= 1")

    results_dir.mkdir(parents=True, exist_ok=True)

    baseline_binary = RECOVERY_DIR / "main.baseline.riscv"
    if not baseline_binary.exists():
        sys.exit(f"ERROR: baseline binary not found: {baseline_binary}")

    runs = []
    
    # Add single baseline run (no error injection)
    runs.append((baseline_binary, None, None, True))
    
    # Add recovery runs with fault injection at different intervals
    for interval_size in INTERVAL_SIZES:
        runs.append((secon_binary, "secon", interval_size, False))
        runs.append((sw_binary, "sw", interval_size, False))

    print("=" * 78)
    print("Recovery Evaluation Runner")
    print("=" * 78)
    print(f"gem5 binary      : {gem5_binary}")
    print(f"gem5 config      : {gem5_config}")
    print(f"recovery dir     : {RECOVERY_DIR}")
    print("run mode         : run-to-completion (no fast-forward / no max insts)")
    print(f"parallel jobs    : {cli.jobs}")
    print(f"baseline runs    : 1 (no error injection)")
    print(f"recovery runs    : {len(INTERVAL_SIZES) * 2} (each variant × each interval size)")
    print(f"interval sizes   : {INTERVAL_SIZES}")
    print(f"total runs       : {len(runs)}")
    print("=" * 78)

    summary = [None] * len(runs)
    failed = 0

    if cli.jobs == 1:
        for idx, (binary, variant, interval_size, is_baseline) in enumerate(runs, start=1):
            res = run_one(
                gem5_binary=gem5_binary,
                gem5_config=gem5_config,
                binary=binary,
                variant=variant,
                interval_size=interval_size,
                results_dir=results_dir,
                is_baseline=is_baseline,
            )
            summary[idx - 1] = res

            status = "OK" if res["returncode"] == 0 else f"FAIL(rc={res['returncode']})"
            if res["returncode"] != 0:
                failed += 1

            cyc_str = f"{res['cycles']:,}" if res["cycles"] else "N/A"
            inst_str = f"{res['instructions']:,}" if res["instructions"] else "N/A"
            
            if is_baseline:
                print(
                    f"[{idx}/{len(runs)}] baseline         "
                    f"{status:12s} {res['elapsed']:8.1f}s cycles={cyc_str} insts={inst_str}"
                )
            else:
                print(
                    f"[{idx}/{len(runs)}] {variant:6s} interval={interval_size:>9,d} "
                    f"{status:12s} {res['elapsed']:8.1f}s cycles={cyc_str} insts={inst_str}"
                )
    else:
        future_to_meta = {}
        with ThreadPoolExecutor(max_workers=cli.jobs) as pool:
            for idx, (binary, variant, interval_size, is_baseline) in enumerate(runs, start=1):
                fut = pool.submit(
                    run_one,
                    gem5_binary,
                    gem5_config,
                    binary,
                    variant,
                    interval_size,
                    results_dir,
                    is_baseline,
                )
                future_to_meta[fut] = (idx, variant, interval_size, is_baseline)

            done_count = 0
            for fut in as_completed(future_to_meta):
                idx, variant, interval_size, is_baseline = future_to_meta[fut]
                res = fut.result()
                summary[idx - 1] = res
                done_count += 1

                status = "OK" if res["returncode"] == 0 else f"FAIL(rc={res['returncode']})"
                if res["returncode"] != 0:
                    failed += 1

                cyc_str = f"{res['cycles']:,}" if res["cycles"] else "N/A"
                inst_str = f"{res['instructions']:,}" if res["instructions"] else "N/A"
                
                if is_baseline:
                    print(
                        f"[{done_count}/{len(runs)}] #{idx} baseline         "
                        f"{status:12s} {res['elapsed']:8.1f}s cycles={cyc_str} insts={inst_str}"
                    )
                else:
                    print(
                        f"[{done_count}/{len(runs)}] #{idx} {variant:6s} "
                        f"interval={interval_size:>9,d} {status:12s} "
                        f"{res['elapsed']:8.1f}s cycles={cyc_str} insts={inst_str}"
                    )

    print("\n" + "=" * 78)
    print(f"SUMMARY: {len(runs) - failed} succeeded, {failed} failed out of {len(runs)}")
    print("=" * 78)

    csv_path = results_dir / "recovery_summary.csv"
    with csv_path.open("w") as f:
        f.write("variant,run_type,interval_size,status,elapsed_s,cycles,instructions,outdir\n")
        for res in summary:
            if res is None:
                continue
            status = "OK" if res["returncode"] == 0 else f"FAIL(rc={res['returncode']})"
            cyc = res["cycles"] if res["cycles"] is not None else ""
            inst = res["instructions"] if res["instructions"] is not None else ""
            run_type = "baseline" if res["is_baseline"] else "recovery"
            interval_size = "" if res["is_baseline"] else res["interval_size"]
            f.write(
                f"{res['variant']},{run_type},{interval_size},{status},{res['elapsed']:.1f},"
                f"{cyc},{inst},{res['outdir']}\n"
            )

    print(f"Results written to {csv_path}")


if __name__ == "__main__":
    main()
