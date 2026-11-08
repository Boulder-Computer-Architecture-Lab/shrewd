#!/usr/bin/env python3
"""
Area and energy of the ISCA'27 runs with McPAT. A new pipeline: the old one
in ~/Research/Mcpat (its parser, templates and results) is left untouched;
only its McPAT binary is used.

mcpat_isca27/core.xml     the gem5 O3 core of results_isca27 at 22 nm, 3 GHz
mcpat_isca27/checker.xml  the DIVA checker at width W: McPAT's in-order core
                          with W int ALUs, W multipliers and W FPUs (gem5's
                          model starts any W checks per cycle), of which only
                          the execution unit counts (register file, units,
                          bypass): checks arrive with their operands from the
                          ROB, and loads re-read the core's L1.

Per run, the activity in stats.txt fills core.xml, and energy = McPAT's
runtime dynamic power plus leakage, times cycles / f, for the processor (core,
L2, memory controllers). DIVA runs add the checker's execution unit (dynamic
plus leakage, over the same time). Activity choices:
  - markers are fetched and decoded (decode.decodedInsts) but never renamed;
  - an idle-unit or same-unit shadow is one more access to its unit (the
    per-class Shadow{Available,NotAvailable} stats) and shares the primary's
    register read and result bus;
  - a DIE twin is in gem5's IQ, ROB and issue counts already; it also reads
    its operands (2 per twin, which gem5's shared-operand twins don't count)
    and broadcasts nothing;
  - a DIVA check is one access to a checker unit of its class, 2 register
    reads (3 for an FMA); the checker's register file takes every result
    written. The count is divaChecked; its class shares are the idle-unit
    run's shadow shares (same binary, same SimPoint; shadows also issue on
    the wrong path, up to 40% more of them on branchy code, so only the
    shares are taken), or the run's own issue mix where there is no such
    run. DIE twins use the same shares.

Usage:
    python3 mcpat_isca27.py area                 # mcpat_isca27/area.csv
    python3 mcpat_isca27.py energy [-j 4] [EXP]  # mcpat_isca27/runs.csv
    python3 mcpat_isca27.py summary              # mcpat_isca27/summary.csv
"""
import argparse
import csv
import math
import os
import re
import subprocess
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
DIR = HERE / "mcpat_isca27"
WORK = DIR / "work"
RESULTS = HERE / "results_isca27"
MCPAT = Path(os.environ.get("MCPAT", Path.home() / "Research/Mcpat/mcpat/mcpat"))
CLOCK_HZ = 3.003e9
WIDTHS = (1, 2, 4, 8)
CORE = "board.processor.switch.core."
INT_FU = ("IntAlu",)
MUL_FU = ("IntMult", "IntDiv")
FP_FU = ("FloatAdd", "FloatCmp", "FloatCvt", "FloatMult", "FloatMultAcc",
         "FloatDiv", "FloatSqrt", "FloatMisc")
DIVA = re.compile(r"_diva_w(\d+)(_x\d+)?$")

# ── stats ───────────────────────────────────────────────────────────────


def read_stats(path):
    s = {}
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) >= 2 and not line.startswith("-"):
                # Whole programs (FT-GMRES) run on one O3 core, not the
                # switchable processor of the SimPoint runs.
                key = p[0].replace("board.processor.cores.core.", CORE)
                try:
                    s[key] = float(p[1])
                except ValueError:
                    pass
    return s


def c(s, key):
    return s.get(CORE + key, 0.0)


def total(s, pattern):
    rx = re.compile(pattern)
    return sum(v for k, v in s.items() if rx.search(k))


def issued(s, classes):
    return sum(c(s, f"statIssuedInstType_0::{k}") for k in classes)


def shadows(s, classes):
    return sum(c(s, f"{k}Shadow{a}") for k in classes
               for a in ("Available", "NotAvailable"))


def class_mix(s, sibling):
    """Shares of int ALU / mul-div / FP / FMA among protected checks."""
    if sibling is not None and shadows(sibling, INT_FU + MUL_FU + FP_FU):
        get = lambda cl: shadows(sibling, cl)
    else:
        get = lambda cl: issued(s, cl)
    n = {"int": get(INT_FU), "mul": get(MUL_FU), "fp": get(FP_FU),
         "fma": get(("FloatMultAcc",))}
    t = n["int"] + n["mul"] + n["fp"] or 1.0
    return {k: v / t for k, v in n.items()}


def core_activity(s, mix):
    cyc = c(s, "numCycles")
    idle = c(s, "idleCycles")
    twins = c(s, "commit.dieTwinsRetired")
    tw = {k: twins * mix[k] for k in ("int", "mul", "fp", "fma")}
    ialu = issued(s, INT_FU) + shadows(s, INT_FU)
    mul = issued(s, MUL_FU) + shadows(s, MUL_FU)
    fpu = issued(s, FP_FU) + shadows(s, FP_FU)
    lookups = c(s, "rename.lookups") or 1.0
    l1i = "board.cache_hierarchy.l1i-cache-0."
    l1d = "board.cache_hierarchy.l1d-cache-0."
    l2 = "board.cache_hierarchy.l2-cache-0."
    reads = total(s, r"mem_ctrl\d*\.readReqs$")
    writes = total(s, r"mem_ctrl\d*\.writeReqs$")
    core = {
        "total_cycles": cyc, "idle_cycles": idle, "busy_cycles": cyc - idle,
        "total_instructions": c(s, "decode.decodedInsts"),
        "int_instructions": issued(s, INT_FU + MUL_FU + ("No_OpClass",
                                                          "IprAccess")),
        "fp_instructions": issued(s, FP_FU),
        "branch_instructions": c(s, "branchPred.condPredicted"),
        "branch_mispredictions": c(s, "commit.branchMispredicts"),
        "load_instructions": issued(s, ("MemRead", "FloatMemRead")),
        "store_instructions": issued(s, ("MemWrite", "FloatMemWrite")),
        "committed_instructions": c(s, "commitStats0.numInsts"),
        "committed_int_instructions": sum(
            c(s, f"commitStats0.committedInstType::{k}")
            for k in INT_FU + MUL_FU + ("No_OpClass", "IprAccess")),
        "committed_fp_instructions": sum(
            c(s, f"commitStats0.committedInstType::{k}") for k in FP_FU),
        "ROB_reads": c(s, "rob.reads"), "ROB_writes": c(s, "rob.writes"),
        "rename_reads": c(s, "rename.intLookups"),
        "rename_writes": c(s, "rename.renamedOperands")
        * c(s, "rename.intLookups") / lookups,
        "fp_rename_reads": c(s, "rename.fpLookups"),
        "fp_rename_writes": c(s, "rename.renamedOperands")
        * c(s, "rename.fpLookups") / lookups,
        "inst_window_reads": c(s, "intInstQueueReads"),
        "inst_window_writes": c(s, "intInstQueueWrites"),
        "inst_window_wakeup_accesses": c(s, "intInstQueueWakeupAccesses"),
        "fp_inst_window_reads": c(s, "fpInstQueueReads"),
        "fp_inst_window_writes": c(s, "fpInstQueueWrites"),
        "fp_inst_window_wakeup_accesses": c(s, "fpInstQueueWakeupAccesses"),
        "int_regfile_reads": c(s, "executeStats0.numIntRegReads")
        + 2 * (tw["int"] + tw["mul"]),
        "int_regfile_writes": c(s, "executeStats0.numIntRegWrites"),
        "float_regfile_reads": c(s, "executeStats0.numFpRegReads")
        + 2 * tw["fp"] + tw["fma"],
        "float_regfile_writes": c(s, "executeStats0.numFpRegWrites"),
        "function_calls": c(s, "commit.functionCalls"),
        "ialu_accesses": ialu, "mul_accesses": mul, "fpu_accesses": fpu,
        "cdb_alu_accesses": issued(s, INT_FU) - tw["int"],
        "cdb_mul_accesses": issued(s, MUL_FU) - tw["mul"],
        "cdb_fpu_accesses": issued(s, FP_FU) - tw["fp"],
    }
    vals = {("system", k): core[k]
            for k in ("total_cycles", "idle_cycles", "busy_cycles")}
    vals.update({("system.core0", k): v for k, v in core.items()})
    icache = s.get(l1i + "overallAccesses::total", 0.0)
    dcache = s.get(l1d + "overallAccesses::total", 0.0)
    vals.update({
        ("system.core0.itlb", "total_accesses"): icache,
        ("system.core0.itlb", "total_misses"): 0,
        ("system.core0.icache", "read_accesses"): icache,
        ("system.core0.icache", "read_misses"):
            s.get(l1i + "overallMisses::total", 0.0),
        ("system.core0.dtlb", "total_accesses"): dcache,
        ("system.core0.dtlb", "total_misses"): 0,
        ("system.core0.dcache", "read_accesses"):
            s.get(l1d + "ReadReq.accesses::total", 0.0),
        ("system.core0.dcache", "write_accesses"):
            s.get(l1d + "WriteReq.accesses::total", 0.0),
        ("system.core0.dcache", "read_misses"):
            s.get(l1d + "ReadReq.mshrMisses::total", 0.0),
        ("system.core0.dcache", "write_misses"):
            s.get(l1d + "WriteReq.mshrMisses::total", 0.0),
        ("system.core0.BTB", "read_accesses"): c(s, "branchPred.BTBLookups"),
        ("system.core0.BTB", "write_accesses"): c(s, "branchPred.BTBUpdates"),
        ("system.L20", "read_accesses"):
            s.get(l2 + "demandAccesses::total", 0.0),
        ("system.L20", "write_accesses"):
            total(s, r"l2-cache-0\.Writeback\w*\.accesses::total$"),
        ("system.L20", "read_misses"): s.get(l2 + "demandMisses::total", 0.0),
        ("system.L20", "write_misses"): 0,
        ("system.mc", "memory_accesses"): reads + writes,
        ("system.mc", "memory_reads"): reads,
        ("system.mc", "memory_writes"): writes,
    })
    return vals


def checker_activity(s, mix, checked):
    cyc = c(s, "numCycles")
    n = {k: checked * mix[k] for k in ("int", "mul", "fp", "fma")}
    core = {
        "total_cycles": cyc, "idle_cycles": 0, "busy_cycles": cyc,
        "total_instructions": checked, "committed_instructions": checked,
        "int_instructions": n["int"] + n["mul"], "fp_instructions": n["fp"],
        "committed_int_instructions": n["int"] + n["mul"],
        "committed_fp_instructions": n["fp"],
        "ialu_accesses": n["int"], "mul_accesses": n["mul"],
        "fpu_accesses": n["fp"],
        "cdb_alu_accesses": n["int"], "cdb_mul_accesses": n["mul"],
        "cdb_fpu_accesses": n["fp"],
        "int_regfile_reads": 2 * (n["int"] + n["mul"]),
        "float_regfile_reads": 2 * n["fp"] + n["fma"],
        "int_regfile_writes": c(s, "executeStats0.numIntRegWrites"),
        "float_regfile_writes": c(s, "executeStats0.numFpRegWrites"),
    }
    zero = ("branch_instructions", "branch_mispredictions",
            "load_instructions", "store_instructions", "ROB_reads",
            "ROB_writes", "rename_reads", "rename_writes", "fp_rename_reads",
            "fp_rename_writes", "inst_window_reads", "inst_window_writes",
            "inst_window_wakeup_accesses", "fp_inst_window_reads",
            "fp_inst_window_writes", "fp_inst_window_wakeup_accesses",
            "function_calls")
    vals = {("system", k): core[k]
            for k in ("total_cycles", "idle_cycles", "busy_cycles")}
    vals.update({("system.core0", k): v for k, v in core.items()})
    vals.update({("system.core0", k): 0 for k in zero})
    for comp, keys in (("itlb", ("total_accesses", "total_misses")),
                       ("icache", ("read_accesses", "read_misses")),
                       ("dtlb", ("total_accesses", "total_misses")),
                       ("dcache", ("read_accesses", "write_accesses",
                                   "read_misses", "write_misses")),
                       ("BTB", ("read_accesses", "write_accesses"))):
        vals.update({(f"system.core0.{comp}", k): 0 for k in keys})
    return vals

# ── McPAT ───────────────────────────────────────────────────────────────

COMP_RE = re.compile(r'<component\s+id="([^"]+)"')
VAL_RE = re.compile(r'(<(?:param|stat)\s+name="([^"]+)"\s+value=")[^"]*(")')


def fill(template, vals, width=None):
    text = (DIR / template).read_text()
    if width is not None:
        text = text.replace("@W@", str(width))
    out, comp = [], None
    for line in text.splitlines(keepends=True):
        m = COMP_RE.search(line)
        if m:
            comp = m.group(1)
        m = VAL_RE.search(line)
        if m and (comp, m.group(2)) in vals:
            v = vals[(comp, m.group(2))]
            v = int(round(v)) if isinstance(v, float) else v
            line = VAL_RE.sub(lambda mm: f"{mm.group(1)}{v}{mm.group(3)}",
                              line, count=1)
        out.append(line)
    return "".join(out)


KEYS = ("Area", "Peak Dynamic", "Subthreshold Leakage", "Gate Leakage",
        "Runtime Dynamic")


def parse(text):
    """{block: {key: value}}, first occurrence of each block name."""
    blocks, name = {}, None
    for line in text.splitlines():
        t = line.strip()
        if not t:
            continue
        m = re.match(r"(.+?)\s*=\s*([-\d.e+]+)\s*(mm\^2|W)$", t)
        if m and name is not None:
            if m.group(1) in KEYS:
                blocks[name].setdefault(m.group(1), float(m.group(2)))
            continue
        if t.endswith(":") or t.startswith("Total "):
            name = re.sub(r"\s*\(Count:.*$", "", t.rstrip(":").strip())
            name = re.sub(r"^Total (Cores|MCs).*$", r"Total \1", name)
            if name in blocks:
                name = name + "#"          # later repeats are ignored
            blocks.setdefault(name, {})
    return blocks


def run_mcpat(xml_text, tag):
    WORK.mkdir(parents=True, exist_ok=True)
    xml = WORK / f"{tag}.xml"
    out = WORK / f"{tag}.txt"
    xml.write_text(xml_text)
    res = subprocess.run([str(MCPAT), "-infile", str(xml), "-print_level", "5"],
                         capture_output=True, text=True)
    out.write_text(res.stdout + res.stderr)
    if res.returncode or "Processor:" not in res.stdout:
        raise RuntimeError(f"McPAT failed on {xml}")
    return parse(res.stdout)


def power(b):
    return (b.get("Runtime Dynamic", 0.0) + b.get("Subthreshold Leakage", 0.0)
            + b.get("Gate Leakage", 0.0))


def leakage(b):
    return b.get("Subthreshold Leakage", 0.0) + b.get("Gate Leakage", 0.0)

# ── commands ────────────────────────────────────────────────────────────


def area(args):
    """Core and checker area at 22 nm (idle activity: area needs none)."""
    s = read_stats(next(RESULTS.glob("bwaves/lbase_base/simpoint_*/stats.txt")))
    mix = class_mix(s, None)
    rows = []
    core = run_mcpat(fill("core.xml", core_activity(s, mix)), "area_core")
    for blk in ("Processor", "Core", "Instruction Fetch Unit", "Renaming Unit",
                "Load Store Unit", "Memory Management Unit", "Execution Unit",
                "Register Files", "Instruction Scheduler", "Integer ALUs",
                "Floating Point Units (FPUs)", "Complex ALUs (Mul/Div)",
                "Results Broadcast Bus", "L2"):
        if blk in core:
            rows.append({"unit": "core", "width": 8, "block": blk,
                         "area_mm2": core[blk].get("Area", ""),
                         "leakage_w": leakage(core[blk])})
    for w in WIDTHS:
        chk = run_mcpat(fill("checker.xml",
                             checker_activity(s, mix, c(s, "commitStats0.numInsts")),
                             width=w), f"area_checker_w{w}")
        for blk in ("Execution Unit", "Register Files", "Instruction Scheduler",
                    "Integer ALUs", "Floating Point Units (FPUs)",
                    "Complex ALUs (Mul/Div)", "Results Broadcast Bus"):
            if blk in chk:
                rows.append({"unit": "diva_checker", "width": w, "block": blk,
                             "area_mm2": chk[blk].get("Area", ""),
                             "leakage_w": leakage(chk[blk])})
    with open(DIR / "area.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)
    core_area = core["Core"]["Area"]
    print(f"core {core_area:.2f} mm^2 (processor with L2 and MCs "
          f"{core['Processor']['Area']:.2f}), 22 nm")
    for r in rows:
        if r["unit"] == "diva_checker" and r["block"] == "Execution Unit":
            print(f"  DIVA checker W={r['width']}: {r['area_mm2']:.3f} mm^2 = "
                  f"{100 * r['area_mm2'] / core_area:.1f}% of the core")


def run_list(exps):
    runs = {}
    for e in exps:
        with open(RESULTS / f"{e}.csv") as f:
            for r in csv.DictReader(f):
                point = (f"simpoint_{r['interval']}" if r["interval"]
                         else "full")
                key = (r["bench"], r["config"], point)
                runs[key] = {"base": r["base"], "bench": r["bench"],
                             "config": r["config"], "point": point,
                             "weight": float(r["weight"] or 1.0)}
    return list(runs.values())


def sibling(run):
    """The idle-unit run with the same binary and SimPoint, if any."""
    cfg = run["config"]
    for suffix in (DIVA, re.compile(r"_(die|rftwin)$")):
        if suffix.search(cfg):
            sib = suffix.sub("_idlefu", cfg)
            p = RESULTS / run["bench"] / sib / run["point"] / "stats.txt"
            return read_stats(p) if p.exists() else None
    return None


def one_run(run):
    d = RESULTS / run["bench"] / run["config"] / run["point"]
    s = read_stats(d / "stats.txt")
    mix = class_mix(s, sibling(run))
    tag = f"{run['bench']}.{run['config']}.{run['point']}"
    b = run_mcpat(fill("core.xml", core_activity(s, mix)), tag)
    seconds = c(s, "numCycles") / CLOCK_HZ
    row = dict(run, cycles=c(s, "numCycles"), seconds=seconds,
               proc_dynamic_w=b["Processor"]["Runtime Dynamic"],
               proc_leakage_w=leakage(b["Processor"]),
               core_dynamic_w=b["Core"]["Runtime Dynamic"],
               exu_dynamic_w=b["Execution Unit"]["Runtime Dynamic"],
               energy_j=power(b["Processor"]) * seconds,
               checker_width=0, checked=0, checker_energy_j=0.0)
    m = DIVA.search(run["config"])
    if m:
        w = int(m.group(1))
        checked = c(s, "commit.divaChecked")
        k = run_mcpat(fill("checker.xml", checker_activity(s, mix, checked),
                           width=w), tag + ".checker")
        row.update(checker_width=w, checked=checked,
                   checker_energy_j=power(k["Execution Unit"]) * seconds)
    row["total_energy_j"] = row["energy_j"] + row["checker_energy_j"]
    (WORK / f"{tag}.txt").unlink()           # keep the XML, drop the report
    (WORK / f"{tag}.checker.txt").unlink(missing_ok=True)
    return row


def energy(args):
    runs = [r for r in run_list(args.experiments)
            if (RESULTS / r["bench"] / r["config"] / r["point"]
                / "stats.txt").exists()]
    out = DIR / "runs.csv"
    done = {}
    if out.exists():
        with open(out) as f:
            done = {(r["bench"], r["config"], r["point"]): r
                    for r in csv.DictReader(f)}
    todo = [r for r in runs if (r["bench"], r["config"], r["point"])
            not in done]
    print(f"{len(runs)} runs, {len(todo)} to do, {args.jobs} at a time")
    rows = list(done.values())
    with ThreadPoolExecutor(args.jobs) as pool:
        for i, row in enumerate(pool.map(one_run, todo), 1):
            rows.append(row)
            if i % 50 == 0 or i == len(todo):
                print(f"  {i}/{len(todo)}", flush=True)
                write(out, rows)
    write(out, rows)


def write(path, rows):
    fields = list(rows[0])
    with open(path, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=fields)
        wr.writeheader()
        wr.writerows(sorted(rows, key=lambda r: (r["bench"], r["config"],
                                                 r["point"])))


def summary(args):
    """SimPoint-weighted energy per (config, bench), against its base,
    without the SimPoints isca27_data.py excludes (as every other figure)."""
    from isca27_data import EXCLUDED
    with open(DIR / "runs.csv") as f:
        rows = [r for r in csv.DictReader(f)
                if (r["bench"], int(r["point"].split("_")[-1])
                    if r["point"].startswith("simpoint_") else None)
                not in EXCLUDED]
    by = defaultdict(list)
    for r in rows:
        by[(r["config"], r["bench"])].append(r)
    out = []
    for (cfg, bench), rs in sorted(by.items()):
        base = {r["point"]: r for r in by.get((rs[0]["base"], bench), [])}
        if not all(r["point"] in base for r in rs):
            continue
        w = sum(float(r["weight"]) for r in rs)
        mean = lambda rr, k: sum(float(r["weight"]) * float(r[k])
                                 for r in rr) / w
        bases = [base[r["point"]] for r in rs]
        e, e0 = mean(rs, "total_energy_j"), mean(bases, "total_energy_j")
        t, t0 = mean(rs, "cycles"), mean(bases, "cycles")
        out.append({"config": cfg, "bench": bench, "simpoints": len(rs),
                    "energy_j": e, "energy_overhead": e / e0 - 1,
                    "cycle_overhead": t / t0 - 1,
                    "checker_share": mean(rs, "checker_energy_j") / e,
                    "core_power_w": mean(rs, "energy_j") / mean(rs, "seconds")})
    with open(DIR / "summary.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(out[0]))
        wr.writeheader()
        wr.writerows(out)
    print(f"{len(out)} (config, benchmark) rows -> {DIR / 'summary.csv'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("command", choices=("area", "energy", "summary"))
    ap.add_argument("experiments", nargs="*", default=["E1", "E2", "E3", "EF"])
    ap.add_argument("-j", "--jobs", type=int, default=4,
                    help="McPAT processes at once (each takes ~1.5 GB)")
    args = ap.parse_args()
    {"area": area, "energy": energy, "summary": summary}[args.command](args)


if __name__ == "__main__":
    main()
