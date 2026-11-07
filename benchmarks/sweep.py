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
    E1  which instructions to protect: impact-guided markers (S, from the
        injection campaign), random markers (R), and hardware-only
        AVF-guided protection (no markers) at the same protected shares
    E2  where selectivity pays: SHAFT on S vs software duplication of the
        same instructions vs everything protected, under the
        resource-sharing mechanisms (same-unit and idle-unit re-execution,
        DIE)
    E3  spatial redundancy: the Diva checker's width (and latency) with S
        markers vs checking everything
    EP  permanent fault: a stuck FloatAdd unit under each mechanism
    ES  the same stuck unit under software duplication of S50: whether the
        duplicate lands on another unit is up to the scheduler (gem5 pairs
        each original with its duplicate: commit.swDup{Pairs,Detected,
        Undetected})
    EF  FT-GMRES (whole program): the developer's annotation of the reliable
        part (its ALU instructions, or the whole region) vs S at its budget
        vs AVF-guided protection at its share
    EV  the same for the video encoder (videnc/, whole program): everything
        but the motion search annotated; the stuck unit is an IntMult
    EC  coalesced markers: the S binaries with regions merged across short
        straight-line gaps (fewer markers, the gaps' ALU instructions
        checked too), under every mechanism but software duplication
    EL  code-layout controls: the unprotected binary with N unexecuted
        bytes in front of its code (same instructions, shifted addresses),
        the noise band for every overhead
Each marker binary also runs with --markers-only (markers executed, nothing
checked), which splits its overhead into the markers' and the mechanism's.

Every run logs the core's AVF per interval (<cpu>.shaft_avf.csv). AVF-guided
runs protect the intervals in which the unprotected run's AVF was highest:
their schedules (<results>/<bench>/avf_schedules/) are written by `run` once
that run is done, so a second `run` starts them.

Binaries (selection_binaries/<b>/, build_selection_binaries.py; all with t6
and ft11 reserved, otherwise identical code):
    lbase      unprotected
    S<L>, R<L> markers around the S or R selection of L% of the window's
               ALU instructions (select_window.py)
    Sdup<L>    software duplication of the S selection
    S<L>k<K>   S<L> with regions merged across gaps of at most K straight-line
               instructions (selection_binaries_coalesced/,
               build_coalesced_binaries.py)
    pad<N>     lbase relinked with N bytes in front of the code
               (selection_binaries_layout/, build_layout_variants.py)
Whole programs (ftgmres/, videnc/): none, annotated, S.

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
COALESCED_BINARIES = SCRIPT_DIR / "selection_binaries_coalesced"
LAYOUT_BINARIES = SCRIPT_DIR / "selection_binaries_layout"
FTGMRES_DIR = SCRIPT_DIR / "ftgmres"
VIDENC_DIR = SCRIPT_DIR / "videnc"

BENCHMARKS = ["bwaves", "cactuBSSN", "lbm", "nab", "namd", "parest",
              "povray"]
FTGMRES = "ftgmres"
VIDENC = "videnc"
# Whole programs (run whole in O3, interval None): directory, arguments,
# the experiment that runs them, and the summary line they print.
WholeProgram = namedtuple("WholeProgram", "dir args experiment output")
WHOLE = {
    FTGMRES: WholeProgram(FTGMRES_DIR, ["64"], "EF",
                          r"status=\S+ outer=\d+ inner_failures=\d+ "
                          r"relerr=\S+"),
    # The stream goes nowhere: gem5 runs only time it (no faults injected;
    # campaign.py checks streams).
    VIDENC: WholeProgram(VIDENC_DIR, ["enc", "/dev/null"], "EV",
                         r"frames=\d+ bytes=\d+ sse=\d+ fallbacks=\d+ "
                         r"stream=\S+ recon=\S+"),
}
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
          "namd": 0.7, "parest": 1.0, "povray": 0.6, FTGMRES: 0.5,
          VIDENC: 0.5}

# Protection levels: shares (%) of the window's ALU instructions.
LEVELS = (10, 25, 50, 75)
# Coalesced markers (EC): regions merged across gaps of at most this many
# straight-line instructions.
COALESCE_GAP = 2
# Layout controls (EL): bytes in front of the code.
LAYOUT_PADS = (2, 6, 16, 40)
# AVF monitor interval, in committed instructions.
AVF_INTERVAL = 100_000

# ── Binaries and configurations ─────────────────────────────────────────

Binary = namedtuple("Binary", "path run_dir stdin no_count")


def binary(bench, key):
    """A benchmark's binary by key (see the module docstring)."""
    if bench in WHOLE:
        d = WHOLE[bench].dir
        return Binary(d / f"{bench}.{key}.riscv", d, None, False)
    cfg = CONFIG[bench]
    stem = cfg["binary"][:-len(".riscv")]
    spec_run = (rc.BASELINE_DIR / cfg["spec_num"] / "run" /
                "run_base_refrate_riscv.0000")
    stdin = spec_run / cfg["stdin"] if cfg["stdin"] else None
    if key == "lbase":
        return Binary(SELECTION_BINARIES / bench / "baseline" /
                      f"{stem}.baseline.riscv", spec_run, stdin, False)
    if key.startswith("pad"):
        return Binary(LAYOUT_BINARIES / bench / key /
                      f"{stem}.baseline.riscv", spec_run, stdin, False)
    sel, dup, level, gap = re.fullmatch(r"([SR])(dup)?(\d+)(?:k(\d+))?",
                                        key).groups()
    if gap:
        return Binary(COALESCED_BINARIES / bench /
                      f"{sel}_{level}pct_k{gap}" /
                      f"{stem}.secon_secoff.riscv", spec_run, stdin, False)
    mode = "duplicate" if dup else "secon_secoff"
    return Binary(SELECTION_BINARIES / bench / f"{sel}_{level}pct" /
                  f"{stem}.{mode}.riscv", spec_run, stdin, bool(dup))


MECHANISMS = {
    "base": ["--no-shrewd"],                      # unprotected binaries
    "sw": ["--no-shrewd"],                        # software duplication
    "markers": ["--markers-only"],
    # Resource-sharing redundancy: the check reuses the core's own units.
    "temporal": ["--mechanism", "FuDup", "--shadow-policy", "Temporal"],
    "idlefu": ["--mechanism", "FuDup", "--shadow-policy", "SpatialWait"],
    "die": ["--mechanism", "Die"],
}


def mechanism(mech):
    """gem5 arguments of a mechanism; diva_w<W>[_x<S>] is the Diva checker
    (spatial redundancy) with width W and its latencies scaled by S."""
    diva = re.fullmatch(r"diva_w(\d+)(?:_x(\d+))?", mech)
    if diva:
        return ["--mechanism", "Diva", "--diva-width", diva.group(1),
                "--diva-latency-scale", diva.group(2) or "1"]
    return MECHANISMS[mech]


# name, binary key, gem5 config arguments, the family's unprotected config,
# AVF-guided protection level (None: markers or everything)
Config = namedtuple("Config", "name binary args base avf")


def family_base(key):
    return "none_base" if key in ("none", "annotated", "S") else "lbase_base"


def run_config(key, mech, every=False, stuck=None, avf=None, alu=False,
               domain=False):
    """Binary <key> under <mech>; every: protect every ALU instruction;
    avf: protect the intervals of highest AVF holding that share of them;
    alu: of the marked instructions, only the ALU ones (FT-GMRES's
    annotation marks whole regions, loads, stores and branches included);
    domain: the same without a name suffix (coalesced regions, whose gaps
    may hold loads and stores);
    stuck: a permanently faulty unit of that op class."""
    args = mechanism(mech)
    name = f"{key}_{mech}"
    if domain:
        args = args + ["--protect-domain-only"]
    if every:
        args = args + ["--shrewd-default-on", "--protect-domain-only"]
        name = f"{key}_all_{mech}"
    if avf:
        args = args + ["--protect-domain-only"]
        name = f"{key}_avf{avf}_{mech}"
    if alu:
        args = args + ["--protect-domain-only"]
        name = f"{key}_alu_{mech}"
    if stuck:
        args = args + ["--stuck-fu", stuck]
        name = f"{key}_stuck{stuck}_{mech}"
    return Config(name, key, args, family_base(key), avf)


def levels(selection, mechs, at=LEVELS):
    return [run_config(f"{selection}{lvl}", m) for lvl in at for m in mechs]


SHARING = ("temporal", "idlefu", "die")
DIVA = [f"diva_w{w}" for w in (1, 2, 4, 8)]
# FT-GMRES: the annotation's share of ALU instructions (11.7%, from its
# campaign profile), for AVF guidance at the same share.
FTGMRES_SHARE = 12
# The video encoder: the annotation's share (25.6%, videnc/campaign).
VIDENC_SHARE = 26

EXPERIMENTS = {
    "E1": [run_config("lbase", "base")]
          + levels("S", ("markers", "temporal", "die"))
          + levels("R", ("markers",))
          + [run_config("lbase", m, avf=lvl) for lvl in LEVELS
             for m in ("temporal", "die")],
    "E2": [run_config("lbase", "base")]
          + levels("S", SHARING + ("markers",))
          + levels("Sdup", ("sw",))
          + [run_config("lbase", m, every=True) for m in SHARING],
    "E3": [run_config("lbase", "base")]
          + levels("S", DIVA)
          + [run_config("lbase", d, every=True) for d in DIVA]
          + levels("S", [f"{d}_x2" for d in DIVA], at=(25,))
          + [run_config("lbase", f"{d}_x2", every=True) for d in DIVA],
    "EP": [run_config("lbase", "base")]
          + [run_config("S50", m, stuck="FloatAdd")
             for m in ("markers",) + SHARING + ("diva_w2",)],
    "ES": [run_config("lbase", "base"),
           run_config("Sdup50", "sw", stuck="FloatAdd")],
    "EF": [run_config("none", "base")]
          + [run_config("none", m, every=True)
             for m in SHARING + ("diva_w1", "diva_w2")]
          + [run_config(k, m) for k in ("annotated", "S")
             for m in ("markers",) + SHARING + ("diva_w1", "diva_w2")]
          + [run_config("annotated", m, alu=True)
             for m in ("markers",) + SHARING + ("diva_w1", "diva_w2")]
          + [run_config("none", m, avf=FTGMRES_SHARE)
             for m in SHARING + ("diva_w1", "diva_w2")]
          + [run_config("annotated", m, stuck="FloatAdd")
             for m in ("markers",) + SHARING + ("diva_w2",)],
    "EV": [run_config("none", "base")]
          + [run_config("none", m, every=True)
             for m in SHARING + ("diva_w1", "diva_w2")]
          + [run_config(k, m) for k in ("annotated", "S")
             for m in ("markers",) + SHARING + ("diva_w1", "diva_w2")]
          + [run_config("annotated", m, alu=True)
             for m in ("markers",) + SHARING + ("diva_w1", "diva_w2")]
          + [run_config("none", m, avf=VIDENC_SHARE)
             for m in SHARING + ("diva_w1", "diva_w2")]
          # Integer code: a stuck multiplier (quantization, reconstruction).
          + [run_config("annotated", m, stuck="IntMult")
             for m in ("markers",) + SHARING + ("diva_w2",)],
    "EC": [run_config("lbase", "base")]
          + [run_config(f"S{lvl}k{COALESCE_GAP}", m, domain=True)
             for lvl in LEVELS
             for m in ("markers",) + SHARING + ("diva_w1", "diva_w2")],
    "EL": [run_config("lbase", "base")]
          + [run_config(f"pad{n}", "base") for n in LAYOUT_PADS],
}

Run = namedtuple("Run", "config bench interval weight")


def plan(exps, benches, simpoints):
    """Unique runs of the experiments (configs shared between them run
    once). A whole program runs whole (interval None), for its own
    experiment only (FT-GMRES: EF; the video encoder: EV)."""
    runs = {}
    for exp in exps:
        whole = [b for b, w in WHOLE.items() if w.experiment == exp]
        for cfg in EXPERIMENTS[exp]:
            if whole:
                for b in whole:
                    if b in benches:
                        runs.setdefault((cfg.name, b, None),
                                        Run(cfg, b, None, 1.0))
                continue
            for bench in benches:
                if bench in WHOLE:
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
        good = proc.returncode == 0 or bool(info.get("crash"))
        ok += good
        failed += not good
        status = ("ok" if proc.returncode == 0 else
                  f"program crashed ({info['crash']})" if good else
                  f"FAILED rc={proc.returncode}"
                  + (f": {info['error']}" if info.get("error") else ""))
        print(f"[{ok + failed}/{len(tasks)}] {task['name']}: {status}"
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
            cmd.append("--dup-reg-no-count")
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


# gem5 SE panics when the simulated program itself faults (a wild address,
# an illegal instruction). Under a stuck unit that is the program crashing on
# its corrupted results, an outcome rather than a failed run.
# gem5 writes "panic: panic condition ... occurred: <message>".
GUEST_CRASH = re.compile(r"panic: .*?(Page table fault when accessing "
                         r"virtual address \S+|Fault \S+ encountered at pc "
                         r".*|(?:Illegal|Unknown) instruction .*)")
GEM5_ERROR = re.compile(r"(?:panic|fatal|Error): .*")


def run_info(out):
    try:
        return json.loads((out / "done.json").read_text())
    except (OSError, ValueError):
        return {}


def is_done(out):
    info = run_info(out)
    if (info.get("rc", 0) != 0 and not info.get("crash") and
            "_stuck" in out.parent.name):
        # A failed stuck-unit run: look (again) for a guest crash.
        failure(info, out, True)
        info["classified"] = True
        (out / "done.json").write_text(json.dumps(info))
    return info.get("rc") == 0 or bool(info.get("crash"))


def is_crash(out):
    return bool(run_info(out).get("crash"))


def failure(info, out, stuck):
    """Mark a failed run's info: the guest crash it records (stuck units
    only), or gem5's last error line."""
    info["classified"] = True
    if info["rc"] == 0:
        return
    try:
        err = (out / "simerr.txt").read_text(errors="replace")
    except OSError:
        err = ""
    # The error recorded when the run ended counts too: simerr.txt may since
    # belong to a rerun (or a copy taken mid-rerun).
    err = info.get("error", "") + "\n" + err
    crash = GUEST_CRASH.search(err) if stuck else None
    if crash:
        info["crash"] = crash.group(1)
    else:
        errors = GEM5_ERROR.findall(err)
        if errors:
            info["error"] = errors[-1][:300]


def schedule_path(args, run):
    """The intervals an AVF-guided run protects."""
    point = "full" if run.interval is None else f"simpoint_{run.interval}"
    return (args.results / run.bench / "avf_schedules" /
            f"{point}_{run.config.avf}pct.txt")


def avf_trace(args, run):
    """The AVF log of the unprotected run at the same SimPoint, once done."""
    base = run._replace(config=run.config._replace(name=run.config.base))
    out = run_path(args, base)
    logs = sorted(out.glob("*.shaft_avf.csv"))
    return logs[-1] if is_done(out) and logs else None


def write_schedule(trace, level, path):
    """The intervals of highest AVF (ROB + IQ + LSQ, summed as Walcott et
    al. do) that together hold level% of the run's ALU instructions."""
    with open(trace) as f:
        rows = list(csv.DictReader(f))
    total = sum(int(r["domain"]) for r in rows)
    chosen, used = [], 0
    for r in sorted(rows, key=lambda r: -(float(r["avf_rob"]) +
                                          float(r["avf_iq"]) +
                                          float(r["avf_lsq"]))):
        if used >= level / 100 * total:
            break
        chosen.append(int(r["interval"]))
        used += int(r["domain"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(f"{i}\n" for i in sorted(chosen)))


def run_command(args, run, out):
    """gem5 command line and working directory of one run."""
    b = binary(run.bench, run.config.binary)
    avf = ["--avf-interval", str(AVF_INTERVAL)]
    if run.config.avf:
        avf += ["--avf-schedule", str(schedule_path(args, run))]
    if run.interval is None:
        cmd = [str(args.gem5), "-re", "-d", str(out), str(GEM5_WHOLE),
               "--binary", str(b.path)] + run.config.args + avf
        return cmd + ["--benchmark-args"] + WHOLE[run.bench].args, b.run_dir
    cmd = [str(args.gem5), "-re", "-d", str(out), str(GEM5_CONFIG),
           "--binary", str(b.path), "--simpoint-interval",
           str(run.interval)] + run.config.args + avf
    if not args.no_checkpoint:
        cmd += ["--checkpoint", str(checkpoint_path(
            args, run.bench, run.config.binary, run.interval))]
    if b.no_count:
        cmd.append("--dup-reg-no-count")
    if b.stdin:
        cmd += ["--stdin", str(b.stdin)]
    if CONFIG[run.bench]["args"]:
        cmd += ["--benchmark-args"] + CONFIG[run.bench]["args"]
    return cmd, b.run_dir


def cmd_run(args, runs):
    tasks, skipped, waiting = [], 0, 0
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
        if run.config.avf and not schedule_path(args, run).exists():
            trace = avf_trace(args, run)
            if trace is None:
                waiting += 1
                continue
            write_schedule(trace, run.config.avf, schedule_path(args, run))
        cmd, cwd = run_command(args, run, out)

        def done(info, out=out, stuck="_stuck" in run.config.name):
            failure(info, out, stuck)
            (out / "done.json").write_text(json.dumps(info))

        point = "full" if run.interval is None else \
            f"simpoint_{run.interval}"
        tasks.append({"name": f"{run.bench}/{run.config.name}/{point}",
                      "bench": run.bench, "cmd": cmd, "cwd": cwd,
                      "log": out / "gem5.log", "done": done,
                      "base": run.config.name == run.config.base})

    def interleaved(group):
        """Benchmarks interleaved, so memory use stays mixed."""
        by_bench = defaultdict(list)
        for t in group:
            by_bench[t["bench"]].append(t)
        lists = list(by_bench.values())
        return [g[i] for i in range(max(map(len, lists), default=0))
                for g in lists if i < len(g)]

    # The unprotected runs first: AVF-guided runs need their traces.
    tasks = (interleaved([t for t in tasks if t["base"]]) +
             interleaved([t for t in tasks if not t["base"]]))
    print(f"{len(tasks)} runs to do ({skipped} skipped: no checkpoint yet; "
          f"{waiting} waiting for an unprotected run's AVF trace, which "
          f"the next `run` starts; {args.jobs} jobs, {args.mem_gb:.0f} GB)")
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
    "sw_dup_pairs": "commit.swDupPairs",
    "sw_dup_detected": "commit.swDupDetected",
    "sw_dup_undetected": "commit.swDupUndetected",
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


def read_avf(out):
    """Cycle-weighted mean AVF of a run's intervals, per structure."""
    logs = sorted(out.glob("*.shaft_avf.csv"))
    if not logs:
        return {}
    with open(logs[-1]) as f:
        rows = list(csv.DictReader(f))
    cycles = sum(int(r["cycles"]) for r in rows)
    if not cycles:
        return {}
    avf = {f"avf_{k}": sum(int(r["cycles"]) * float(r[f"avf_{k}"])
                           for r in rows) / cycles
           for k in ("rob", "iq", "lsq")}
    avf["avf_intervals"] = len(rows)
    return avf


def run_coverages(args, runs):
    """run_coverage totals per finished SPEC run. The binaries are looked
    up here (the runs' config.ini names the paths they ran with, maybe on
    another machine), once per binary for all the PCs its runs logged."""
    by_binary = defaultdict(list)
    for run in runs:
        out = run_path(args, run)
        if run.bench not in WHOLE and is_done(out) and not is_crash(out):
            by_binary[(binary(run.bench, run.config.binary).path,
                       run.bench)].append(out)
    totals = {}
    for (path, bench), outs in by_binary.items():
        logs = {out: run_coverage.read_log(out) for out in outs}
        pcs = {pc for rows in logs.values() if rows for pc, *_ in rows}
        try:
            tables = run_coverage.pc_tables(str(path), pcs)
        except (FileNotFoundError, subprocess.CalledProcessError) as e:
            print(f"{path}: no coverage ({e})", file=sys.stderr)
            continue
        for out in outs:
            if logs[out]:
                totals[out] = run_coverage.coverage(out, bench,
                                                    tables=tables)
    return totals


def cmd_collect(args, runs):
    coverages = run_coverages(args, runs)
    rows = []
    for run in runs:
        out = run_path(args, run)
        if not is_done(out):
            continue
        row = {"config": run.config.name, "base": run.config.base,
               "bench": run.bench, "interval": run.interval,
               "weight": run.weight}
        info = run_info(out)
        if info.get("crash"):
            # No statistics: gem5 stopped at the program's fault.
            rows.append({**row, "crashed": 1, "crash": info["crash"],
                         "host_seconds": info["elapsed_s"]})
            continue
        if not (out / "stats.txt").exists():
            continue
        row.update(read_stats(out / "stats.txt"))
        if run.bench in WHOLE:
            output = re.search(WHOLE[run.bench].output,
                               (out / "simout.txt").read_text())
            row["output"] = output.group(0) if output else ""
        else:
            row.update(coverages.get(out, {}))
        row.update(read_avf(out))
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
        if not r.get("crashed"):
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
            "sdc_protected": (sum(r["weight"] * r.get("sdc_protected", 0)
                                  for r in rs) / mass if mass else ""),
            "protected_share": share("protected"),
            "checked_share": share("checked"),
            "domain_protected_share": (
                sum(r["weight"] * r.get("domain_protected", 0) for r in rs) /
                sum(r["weight"] * r.get("domain_committed", 0) for r in rs)
                if committed else ""),
            "toggles_per_kinst": (1000 * share("protection_toggles")
                                  if committed else ""),
            "avf": (sum(r["weight"] * sum(r.get(f"avf_{k}", 0)
                                          for k in ("rob", "iq", "lsq"))
                        for r in rs) / w),
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
                        default=",".join(BENCHMARKS + list(WHOLE)))
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

    # gem5 runs in each benchmark's run directory, so a relative path to it
    # would resolve there.
    args.gem5 = args.gem5.resolve()
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
