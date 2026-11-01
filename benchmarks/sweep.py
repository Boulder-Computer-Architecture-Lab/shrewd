#!/usr/bin/env python3
"""
SHAFT ISCA'27 sweeps: SimPoint checkpoints, then every run restored from
them, with memory-aware parallelism for one large machine. Resumable: runs
and checkpoints that already finished are skipped.

    python3 sweep.py list [EXP ...]          configs, run counts, missing inputs
    python3 sweep.py checkpoints [EXP ...]   the checkpoints those runs need
    python3 sweep.py run [EXP ...]           the runs
    python3 sweep.py collect [EXP ...]       <results>/<EXPS>.csv (per run) and
                                             <EXPS>_summary.csv (per config)

Experiments (default: all):
    E1  impact vs likelihood: line-level S / L / R marker binaries, 5-50%,
        under the temporal schemes (Temporal, DIE)
    E2  mechanism spectrum on the S binaries: spatial, temporal, hybrid,
        DIE and Diva, plus everything protected
    E3  Diva provisioning: checker width x latency, all vs S-selected
    E6  the old model's unchecked fallback (opportunistic shadows)
    EB  SHAFT vs software duplication, on the existing (OLD) and the new
        binaries
    EP  permanent fault: a stuck FloatAdd unit under each mechanism
    EF  FT-GMRES (whole program): no markers, the developer's annotation
        of the reliable part, and line-level S / L selections at its budget
Each marker binary also runs with --markers-only (markers executed, nothing
checked), which splits its overhead into the markers' and the mechanism's.

Binaries, by family (each compared with its own matched baseline):
    OLD   protected_binaries/<b>/: base, prot<L>, dup<L>  (the existing
          type-level selections)
    LINE  selection_binaries/<b>/: lbase, S<L>, L<L>, R<L>, Sdup<L>
          (build_selection_binaries.py)
    FT    ftgmres/: none, annotated, S, L

Layout: <checkpoints>/<bench>/<binary>/simpoint_<N>/ and
<results>/<bench>/<config>/simpoint_<N>/ (FT-GMRES: .../full/), with
done.json once a run ends.
"""
import argparse
import csv
import json
import os
import re
import subprocess
import sys
import time
from collections import defaultdict, namedtuple
from pathlib import Path

import run_campaign as rc
import run_coverage

SCRIPT_DIR = Path(__file__).resolve().parent
GEM5_CONFIG = SCRIPT_DIR / "gem5_config.py"
GEM5_WHOLE = SCRIPT_DIR / "gem5_config_o3_only.py"
MAKE_CHECKPOINTS = SCRIPT_DIR / "make_checkpoints.py"
SELECTION_BINARIES = SCRIPT_DIR / "selection_binaries"
FTGMRES_DIR = SCRIPT_DIR / "ftgmres"

BENCHMARKS = ["bwaves", "cactuBSSN", "lbm", "nab", "namd", "parest",
              "povray", "roms"]
FTGMRES = "ftgmres"
FTGMRES_ARGS = ["64"]
CONFIG = dict(rc.BENCHMARK_CONFIG)
CONFIG.setdefault("povray", {
    "spec_num": "511.povray_r",
    "binary": "povray_r_base.riscv",
    "args": ["SPEC-benchmark-ref.ini"],
    "stdin": None,
})

# Peak gem5 RSS per benchmark (GB) until a finished run of it reports its
# own; then 1.15x the largest seen.
MEM_GB = {"bwaves": 1.3, "cactuBSSN": 1.3, "lbm": 0.7, "nab": 0.7,
          "namd": 0.7, "parest": 1.0, "povray": 0.6, "roms": 1.4,
          FTGMRES: 0.5}

BUDGETS = (5, 10, 20, 50)

# Benchmarks without LINE-family runs: roms' hot loop (lmd_swfrac) compiles
# to different instructions in the campaign's build and the pipeline's, so
# its line-level selections cannot be carried over (and its SDC risk is in
# libm's exp, which no rebuild of roms can mark).
NO_LINE_FAMILY = {"roms"}

# ── Binaries and configurations ─────────────────────────────────────────

Binary = namedtuple("Binary", "path run_dir stdin no_count")


def binary(bench, key):
    """A benchmark's binary by key (see the module docstring)."""
    if bench == FTGMRES:
        return Binary(FTGMRES_DIR / f"ftgmres.{key}.riscv", FTGMRES_DIR,
                      None, False)
    cfg = CONFIG[bench]
    stem = cfg["binary"][:-len(".riscv")]
    spec_run = (rc.BASELINE_DIR / cfg["spec_num"] / "run" /
                "run_base_refrate_riscv.0000")

    def stdin(d):
        return d / cfg["stdin"] if cfg["stdin"] else None

    if key == "base":
        return Binary(rc.PROTECTED_DIR / bench / "baseline" /
                      f"{stem}.baseline.riscv", spec_run, stdin(spec_run),
                      False)
    if key == "lbase":
        return Binary(SELECTION_BINARIES / bench / "baseline" /
                      f"{stem}.baseline.riscv", spec_run, stdin(spec_run),
                      False)
    m = re.fullmatch(r"(prot|dup)(\d+)", key)
    if m:
        level_dir = rc.PROTECTED_DIR / bench / f"{m.group(2)}pct"
        mode = "secon_secoff" if m.group(1) == "prot" else "duplicate"
        return Binary(level_dir / f"{stem}.{mode}.riscv", level_dir,
                      stdin(level_dir), m.group(1) == "dup")
    sel, dup, level = re.fullmatch(r"([SLR])(dup)?(\d+)", key).groups()
    mode = "duplicate" if dup else "secon_secoff"
    return Binary(SELECTION_BINARIES / bench / f"{sel}_line_{level}pct" /
                  f"{stem}.{mode}.riscv", spec_run, stdin(spec_run),
                  bool(dup))


MECHANISMS = {
    "base": ["--no-shrewd"],                      # unprotected binaries
    "sw": ["--no-shrewd", "--secon-no-count"],    # software duplication
    "markers": ["--markers-only"],
    "spatial": ["--mechanism", "FuDup", "--shadow-policy", "SpatialWait"],
    "temporal": ["--mechanism", "FuDup", "--shadow-policy", "Temporal"],
    "hybrid": ["--mechanism", "FuDup", "--shadow-policy", "Hybrid"],
    "opport": ["--mechanism", "FuDup", "--shadow-policy", "Opportunistic"],
    "die": ["--mechanism", "Die"],
}


def mechanism(mech):
    diva = re.fullmatch(r"diva_w(\d+)_l(\d+)", mech)
    if diva:
        return ["--mechanism", "Diva", "--diva-width", diva.group(1),
                "--diva-latency", diva.group(2)]
    return MECHANISMS[mech]


# name, binary key, gem5 config arguments, the family's unprotected config
Config = namedtuple("Config", "name binary args base")


def family_base(key):
    if key in ("none", "annotated", "S", "L"):
        return "none_base"
    if key == "base" or re.fullmatch(r"(prot|dup)\d+", key):
        return "base_base"
    return "lbase_base"


def run_config(key, mech, every=False, stuck=None):
    """Binary <key> under <mech>; every: protect everything (100%);
    stuck: a permanently faulty unit of that op class."""
    args = mechanism(mech)
    name = f"{key}_{mech}"
    if every:
        args = args + (["--diva-policy", "All"] if mech.startswith("diva")
                       else ["--shrewd-default-on"])
        name = f"{key}_all_{mech}"
    if stuck:
        args = args + ["--stuck-fu", stuck]
        name = f"{key}_stuck{stuck}_{mech}"
    return Config(name, key, args, family_base(key))


def line_family(selection, budgets, mechs):
    return [run_config(f"{selection}{b}", m) for b in budgets for m in mechs]


DIVA = [f"diva_w{w}_l{lat}" for w in (1, 2, 4, 8) for lat in (4, 8)]

EXPERIMENTS = {
    "E1": [run_config("lbase", "base")]
          + [c for s in "SLR"
             for c in line_family(s, BUDGETS, ("temporal", "die", "markers"))],
    "E2": [run_config("lbase", "base")]
          + line_family("S", BUDGETS, ("spatial", "temporal", "hybrid", "die",
                                       "diva_w2_l4", "markers"))
          + [run_config("lbase", m, every=True)
             for m in ("spatial", "temporal", "die", "diva_w2_l4")],
    "E3": [run_config("lbase", "base")]
          + [run_config("lbase", d, every=True) for d in DIVA]
          + line_family("S", BUDGETS,
                        [f"diva_w{w}_l4" for w in (1, 2, 4, 8)])
          + line_family("S", (20,), [f"diva_w{w}_l8" for w in (1, 2, 4, 8)]),
    "E6": [run_config("lbase", "base")]
          + line_family("S", BUDGETS, ("opport",)),
    "EB": [run_config("base", "base")]
          + [run_config(f"prot{b}", m) for b in BUDGETS
             for m in ("spatial", "temporal", "die", "markers")]
          + [run_config(f"dup{b}", "sw") for b in BUDGETS]
          + [run_config("base", m, every=True)
             for m in ("spatial", "temporal")]
          + [run_config("lbase", "base")]
          + [run_config(f"Sdup{b}", "sw") for b in BUDGETS],
    "EP": [run_config("lbase", "base")]
          + [run_config("S50", m, stuck="FloatAdd")
             for m in ("markers", "spatial", "temporal", "hybrid", "die",
                       "diva_w2_l4")],
    "EF": [run_config("none", "base")]
          + [run_config("none", m, every=True)
             for m in ("spatial", "temporal", "die", "diva_w1_l4",
                       "diva_w2_l4")]
          + [run_config(k, m) for k in ("annotated", "S", "L")
             for m in ("markers", "spatial", "temporal", "die",
                       "diva_w1_l4", "diva_w2_l4")]
          + [run_config("annotated", m, stuck="FloatAdd")
             for m in ("markers", "spatial", "temporal", "die",
                       "diva_w2_l4")],
}

Run = namedtuple("Run", "config bench interval weight")


def plan(exps, benches, simpoints):
    """Unique runs of the experiments (configs shared between them run
    once). FT-GMRES runs whole (interval None) for experiment EF only."""
    runs = {}
    for exp in exps:
        for cfg in EXPERIMENTS[exp]:
            if exp == "EF":
                if FTGMRES in benches:
                    runs.setdefault((cfg.name, FTGMRES, None),
                                    Run(cfg, FTGMRES, None, 1.0))
                continue
            for bench in benches:
                if bench == FTGMRES:
                    continue
                if bench in NO_LINE_FAMILY and cfg.base == "lbase_base":
                    continue
                for interval, weight in simpoints.get(bench, []):
                    runs.setdefault((cfg.name, bench, interval),
                                    Run(cfg, bench, interval, weight))
    return list(runs.values())


def run_path(args, run):
    point = "full" if run.interval is None else f"simpoint_{run.interval}"
    return args.results / run.bench / run.config.name / point


def checkpoint_path(args, bench, key, interval):
    return args.checkpoints / bench / key / f"simpoint_{interval}"


def missing_inputs(run):
    """Files a run needs that are not there."""
    b = binary(run.bench, run.config.binary)
    return [str(p) for p in (b.path, b.stdin) if p and not Path(p).exists()]


# ── Process pool with a memory budget ───────────────────────────────────

def observed_memory(args):
    """Largest peak RSS (GB) per benchmark from finished runs/passes."""
    seen = defaultdict(float)
    for root in (args.results, args.checkpoints):
        for done in root.glob("*/**/*done.json"):
            try:
                info = json.loads(done.read_text())
            except (OSError, ValueError):
                continue
            bench = done.relative_to(root).parts[0]
            seen[bench] = max(seen[bench], info.get("maxrss_kb", 0) / 2**20)
    return seen


def execute(tasks, args):
    """Run tasks (dicts: name, bench, cmd, cwd, log, done) within -j/--mem-gb.

    Peak memory per benchmark starts from MEM_GB and follows what finished
    tasks of that benchmark actually used.
    """
    seen = observed_memory(args)

    def estimate(bench):
        return seen[bench] * 1.15 if seen[bench] else MEM_GB.get(bench, 1.5)

    pending, running = list(tasks), {}
    ok = failed = 0
    while pending or running:
        i = 0
        used = sum(estimate(t["bench"]) for t, _, _ in running.values())
        while i < len(pending) and len(running) < args.jobs:
            task = pending[i]
            need = estimate(task["bench"])
            if running and used + need > args.mem_gb:
                i += 1
                continue
            pending.pop(i)
            task["log"].parent.mkdir(parents=True, exist_ok=True)
            # A memory-capped scope (systemd-run execs the command, so the
            # pid and rusage are gem5's): a runaway task dies alone instead
            # of making the kernel OOM-kill others.
            cap = ([] if not args.cap_gb else
                   ["systemd-run", "--user", "--scope", "-q",
                    "-p", f"MemoryMax={args.cap_gb}G",
                    "-p", "MemorySwapMax=0"])
            with open(task["log"], "w") as log:
                proc = subprocess.Popen(cap + task["cmd"], cwd=task["cwd"],
                                        stdout=log, stderr=subprocess.STDOUT)
            running[proc.pid] = (task, proc, time.time())
            used += need
        pid, status, usage = os.wait4(-1, 0)
        if pid not in running:
            continue
        task, proc, start = running.pop(pid)
        proc.returncode = os.waitstatus_to_exitcode(status)
        info = {"rc": proc.returncode, "elapsed_s": round(time.time() - start),
                "maxrss_kb": usage.ru_maxrss, "cmd": task["cmd"],
                "cwd": str(task["cwd"])}
        task["done"](info)
        seen[task["bench"]] = max(seen[task["bench"]],
                                  usage.ru_maxrss / 2**20)
        ok += proc.returncode == 0
        failed += proc.returncode != 0
        print(f"[{ok + failed}/{len(tasks)}] {task['name']}: "
              f"{'ok' if proc.returncode == 0 else f'FAILED rc={proc.returncode}'}"
              f" {info['elapsed_s']}s {usage.ru_maxrss / 2**20:.2f}GB",
              flush=True)
    return failed


# ── Commands ────────────────────────────────────────────────────────────

def cmd_list(args, runs):
    by_config = defaultdict(int)
    for run in runs:
        by_config[run.config.name] += 1
    if args.verbose:
        for name, count in sorted(by_config.items()):
            print(f"  {name:32s} {count:4d} runs")
    passes = checkpoint_passes(args, runs)
    print(f"{len(runs)} runs in {len(by_config)} configs; "
          f"{len(passes)} checkpoint passes still to take")
    missing = sorted({m for run in runs for m in missing_inputs(run)})
    for path in missing:
        print(f"  MISSING {path}")
    done = sum(is_done(run_path(args, run)) for run in runs)
    print(f"{done} runs finished")
    return 1 if missing else 0


def checkpoint_passes(args, runs):
    """(bench, binary key) -> SimPoint intervals whose checkpoint is
    missing."""
    need = defaultdict(set)
    for run in runs:
        if run.interval is None:
            continue
        ckpt = checkpoint_path(args, run.bench, run.config.binary,
                               run.interval)
        if not (ckpt / "shaft_meta.json").exists():
            need[(run.bench, run.config.binary)].add(run.interval)
    return need


def cmd_checkpoints(args, runs):
    tasks = []
    for (bench, key), intervals in sorted(
            checkpoint_passes(args, runs).items(),
            key=lambda item: -max(item[1])):
        b = binary(bench, key)
        out = args.checkpoints / bench / key
        cmd = [str(args.gem5), "-re", "-d", str(out), str(MAKE_CHECKPOINTS),
               "--binary", str(b.path), "--simpoint-intervals",
               ",".join(str(i) for i in sorted(intervals))]
        if b.no_count:
            cmd.append("--secon-no-count")
        if b.stdin:
            cmd += ["--stdin", str(b.stdin)]
        if CONFIG[bench]["args"]:
            cmd += ["--benchmark-args"] + CONFIG[bench]["args"]
        tag = "_".join(str(i) for i in sorted(intervals))

        def done(info, out=out, tag=tag):
            (out / f"pass_{tag}.done.json").write_text(json.dumps(info))

        tasks.append({"name": f"checkpoint {bench}/{key} {tag}",
                      "bench": bench, "cmd": cmd, "cwd": b.run_dir,
                      "log": out / f"pass_{tag}.log", "done": done})
    print(f"{len(tasks)} checkpoint passes ({args.jobs} jobs, "
          f"{args.mem_gb:.0f} GB)")
    if args.dry_run:
        for t in tasks[:10]:
            print(" ", " ".join(t["cmd"]))
        return 0
    return execute(tasks, args)


def is_done(out):
    try:
        return json.loads((out / "done.json").read_text())["rc"] == 0
    except (OSError, ValueError, KeyError):
        return False


def run_command(args, run, out):
    """gem5 command line and working directory of one run."""
    b = binary(run.bench, run.config.binary)
    if run.interval is None:
        cmd = [str(args.gem5), "-re", "-d", str(out), str(GEM5_WHOLE),
               "--binary", str(b.path)] + run.config.args
        return cmd + ["--benchmark-args"] + FTGMRES_ARGS, b.run_dir
    cmd = [str(args.gem5), "-re", "-d", str(out), str(GEM5_CONFIG),
           "--binary", str(b.path), "--simpoint-interval",
           str(run.interval)] + run.config.args
    if not args.no_checkpoint:
        cmd += ["--checkpoint", str(checkpoint_path(
            args, run.bench, run.config.binary, run.interval))]
    if b.no_count:
        cmd.append("--secon-no-count")
    if b.stdin:
        cmd += ["--stdin", str(b.stdin)]
    if CONFIG[run.bench]["args"]:
        cmd += ["--benchmark-args"] + CONFIG[run.bench]["args"]
    return cmd, b.run_dir


def cmd_run(args, runs):
    tasks, skipped = [], 0
    for run in runs:
        out = run_path(args, run)
        if is_done(out):
            continue
        if (run.interval is not None and not args.no_checkpoint and
                not (checkpoint_path(args, run.bench, run.config.binary,
                                     run.interval) / "shaft_meta.json"
                     ).exists()):
            skipped += 1
            continue
        cmd, cwd = run_command(args, run, out)

        def done(info, out=out):
            (out / "done.json").write_text(json.dumps(info))

        point = "full" if run.interval is None else \
            f"simpoint_{run.interval}"
        tasks.append({"name": f"{run.bench}/{run.config.name}/{point}",
                      "bench": run.bench, "cmd": cmd, "cwd": cwd,
                      "log": out / "gem5.log", "done": done})
    # Interleave benchmarks so memory use stays mixed.
    by_bench = defaultdict(list)
    for t in tasks:
        by_bench[t["bench"]].append(t)
    groups = list(by_bench.values())
    tasks = [g[i] for i in range(max(map(len, groups), default=0))
             for g in groups if i < len(g)]
    print(f"{len(tasks)} runs to do ({skipped} skipped: no checkpoint yet; "
          f"{args.jobs} jobs, {args.mem_gb:.0f} GB)")
    if args.dry_run:
        for t in tasks[:10]:
            print(" ", " ".join(t["cmd"]))
        return 0
    return execute(tasks, args)


# ── Results ─────────────────────────────────────────────────────────────

CORES = ("board.processor.switch.core.", "board.processor.cores.core.")
STATS = {
    "cycles": "numCycles",
    "insts": "commitStats0.numInsts",
    "protection_toggles": "commit.protectionToggles",
    "shadow_available": "shadowAvailable",
    "shadow_not_available": "shadowNotAvailable",
    "shadow_waits": "shadowWaits",
    "shadow_temporal": "shadowTemporal",
    "protected_not_duplicable": "protectedNotDuplicable",
    "die_twins": "commit.dieTwinsRetired",
    "die_twin_wait_cycles": "commit.dieTwinWaitCycles",
    "die_mismatches": "commit.dieMismatches",
    "diva_checked": "commit.divaChecked",
    "diva_gate_stall_cycles": "commit.divaGateStallCycles",
    "diva_mismatches": "commit.divaMismatches",
    "shadow_mismatches": "iew.shrewdFaultSquashes",
    "faults_committed": "commit.faultsCommitted",
    "protected_faults_committed": "commit.protectedFaultsCommitted",
    "stuck_fu_results": "stuckFuResults",
    "stuck_fu_retire_cycle": "stuckFuRetireCycle",
    "rob_full_events": "rename.ROBFullEvents",
    "iq_full_events": "rename.IQFullEvents",
    "branch_mispredicts": "commit.branchMispredicts",
}


def read_stats(path):
    """The measured core's statistics of interest (last dump)."""
    wanted = {core + suffix: key for core in CORES
              for key, suffix in STATS.items()}
    values = {}
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 2 and parts[0] in wanted:
                try:
                    values[wanted[parts[0]]] = float(parts[1])
                except ValueError:
                    pass
            elif parts and parts[0] == "hostSeconds":
                values["host_seconds"] = float(parts[1])
    return values


def cmd_collect(args, runs):
    rows = []
    for run in runs:
        out = run_path(args, run)
        if not is_done(out) or not (out / "stats.txt").exists():
            continue
        row = {"config": run.config.name, "base": run.config.base,
               "bench": run.bench, "interval": run.interval,
               "weight": run.weight}
        row.update(read_stats(out / "stats.txt"))
        if run.bench == FTGMRES:
            output = re.search(r"status=\S+ outer=\d+ inner_failures=\d+ "
                               r"relerr=\S+",
                               (out / "simout.txt").read_text())
            row["output"] = output.group(0) if output else ""
        else:
            try:
                row.update(run_coverage.coverage(out, run.bench) or {})
            except (FileNotFoundError, subprocess.CalledProcessError) as e:
                print(f"{out}: no coverage ({e})", file=sys.stderr)
        row["maxrss_gb"] = json.loads(
            (out / "done.json").read_text())["maxrss_kb"] / 2**20
        rows.append(row)
    if not rows:
        print("No finished runs.")
        return 1

    args.results.mkdir(parents=True, exist_ok=True)
    name = "_".join(args.experiments)
    fields = sorted({k for r in rows for k in r},
                    key=lambda k: (k not in ("config", "base", "bench",
                                             "interval", "weight"), k))
    with open(args.results / f"{name}.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    # SimPoint-weighted per (config, benchmark); overhead against the
    # family's unprotected binary on the same SimPoints.
    groups = defaultdict(list)
    for r in rows:
        groups[(r["config"], r["bench"])].append(r)
    summary = []
    for (config, bench), rs in sorted(groups.items()):
        base = {r["interval"]: r for r in groups.get((rs[0]["base"], bench),
                                                      [])}
        w = sum(r["weight"] for r in rs)
        cycles = sum(r["weight"] * r["cycles"] for r in rs) / w
        complete = all(r["interval"] in base for r in rs)
        base_cycles = (sum(r["weight"] * base[r["interval"]]["cycles"]
                           for r in rs) / w if complete else None)
        mass = sum(r["weight"] * r.get("sdc_mass", 0) for r in rs)
        committed = sum(r["weight"] * r.get("committed", 0) for r in rs)

        def share(key):
            return (sum(r["weight"] * r.get(key, 0) for r in rs) / committed
                    if committed else "")

        summary.append({
            "config": config, "bench": bench, "simpoints": len(rs),
            "cycles": cycles,
            "overhead": cycles / base_cycles - 1 if base_cycles else "",
            "sdc_coverage": (sum(r["weight"] * r.get("sdc_covered", 0)
                                 for r in rs) / mass if mass else ""),
            "protected_share": share("protected"),
            "checked_share": share("checked"),
            "toggles_per_kinst": (1000 * share("protection_toggles")
                                  if committed else ""),
            "faults_committed": sum(r.get("faults_committed", 0)
                                    for r in rs),
        })
    with open(args.results / f"{name}_summary.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)
    print(f"{len(rows)} runs -> {args.results / (name + '.csv')} and "
          f"{name}_summary.csv")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[1],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="\n".join(__doc__.splitlines()[2:]))
    parser.add_argument("command", choices=["list", "checkpoints", "run",
                                            "collect"])
    parser.add_argument("experiments", nargs="*",
                        help=f"Default: all ({', '.join(EXPERIMENTS)})")
    parser.add_argument("--benchmarks",
                        default=",".join(BENCHMARKS + [FTGMRES]))
    parser.add_argument("--gem5", type=Path,
                        default=rc.PROJECT_ROOT / "build" / "ALL" / "gem5.opt")
    parser.add_argument("--checkpoints", type=Path,
                        default=SCRIPT_DIR / "checkpoints")
    parser.add_argument("--results", type=Path,
                        default=SCRIPT_DIR / "results_isca27")
    parser.add_argument("--simpoint-summary", type=Path,
                        default=rc.SIMPOINT_SUMMARY)
    parser.add_argument("-j", "--jobs", type=int, default=os.cpu_count())
    parser.add_argument("--mem-gb", type=float, default=None,
                        help="Memory for gem5 processes (default: 90%% of "
                             "MemAvailable)")
    parser.add_argument("--cap-gb", type=float, default=4.0,
                        help="Kill any one gem5 process that exceeds this "
                             "(systemd-run --user scope; 0 disables)")
    parser.add_argument("--no-checkpoint", action="store_true",
                        help="Fast-forward from the start instead")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="list: every config and its run count")
    args = parser.parse_args()

    args.experiments = args.experiments or list(EXPERIMENTS)
    unknown = set(args.experiments) - set(EXPERIMENTS)
    if unknown:
        sys.exit(f"Unknown experiments: {', '.join(sorted(unknown))}")
    args.benchmarks = args.benchmarks.split(",")
    if args.mem_gb is None:
        with open("/proc/meminfo") as f:
            avail = next(int(line.split()[1]) for line in f
                         if line.startswith("MemAvailable"))
        args.mem_gb = 0.9 * avail / 2**20
    if args.cap_gb and args.command in ("checkpoints", "run") and \
            subprocess.run(["systemd-run", "--user", "--scope", "-q",
                            "true"], capture_output=True).returncode != 0:
        print("systemd-run --user unavailable: running without --cap-gb",
              file=sys.stderr)
        args.cap_gb = 0

    simpoints = rc.parse_simpoint_summary(args.simpoint_summary)
    runs = plan(args.experiments, args.benchmarks, simpoints)
    command = {"list": cmd_list, "checkpoints": cmd_checkpoints,
               "run": cmd_run, "collect": cmd_collect}[args.command]
    sys.exit(1 if command(args, runs) else 0)


if __name__ == "__main__":
    main()
