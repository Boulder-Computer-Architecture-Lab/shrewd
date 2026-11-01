#!/usr/bin/env python3
"""
Dynamic-weighted protection selection and SDC coverage from the QEMU
injection campaign (no new injections).

The campaign samples injection sites uniformly over static instructions, so
raw SDC counts over-weight cold code. Each injection here is weighted by
dyn(pc) / n_inj(pc), which estimates SDC probability under faults that
arrive uniformly per *dynamic* instruction.

Selections under the same budget (share of profiled dynamic instructions):
  S    impact:     P(SDC beyond SPEC tolerance)
  L    likelihood: P(SDC or crash or timeout), i.e. "AVF-style"
  R    random order (uninformed reference)
  OLD  the existing knapsack selection (raw SDC counts)
at four granularities:
  (none)  per mnemonic, what the existing binaries use (exact knapsack)
  _rm     per (routine, mnemonic), e.g. "fmul.d in LBM_performStreamCollide"
  _line   per (source line, mnemonic): what build_selection_binaries.py
          builds marker binaries for, since the -g assembly of any build
          marks each instruction's source line
  _pc     per static instruction of the campaign binary (analysis only:
          other builds have other PCs)
The finer ones are filled greedily by density (their items are small), with
estimates shrunk toward coarser ones: PC toward its (routine, mnemonic), that
and the line toward the mnemonic.

Selection uses one half of the injections. Coverage is measured on the other
half with one estimator for every granularity (per-PC rates, falling back to
the mnemonic's rate for PCs without test injections), with a bootstrap 95% CI,
so no selection is scored on the data it was fit to.

Outputs (in --out): summary.csv, mnemonics.csv, selections.csv, lines.csv
(held-out rates per source line and mnemonic, which run_coverage.py weights
gem5 commit logs with) and line_selections/<bench>/<selection>_<pct>pct.txt
(file, line, mnemonic triples for build_selection_binaries.py).

Usage:
    python3 selection_coverage.py [--injection-res DIR] [--out DIR]
"""
import argparse
import csv
import hashlib
import os
import random
import re
import subprocess
from collections import Counter, defaultdict
from pathlib import Path

BENCHMARKS = ["bwaves", "cactuBSSN", "lbm", "nab", "namd", "parest",
              "povray", "roms"]
BUDGETS = [0.05, 0.10, 0.20, 0.50]
KINDS = ("sdc", "vis", "crash")
CROSS = "/opt/riscv/bin/riscv64-unknown-linux-gnu"
# The binary the QEMU campaign injected into (its PCs are the profile's).
CAMPAIGN_BINARY = {
    "bwaves": "503.bwaves_r/exe/bwaves_r_base.riscv",
    "cactuBSSN": "507.cactuBSSN_r/exe/cactusBSSN_r_base.riscv",
    "lbm": "519.lbm_r/exe/lbm_r_base.riscv",
    "nab": "544.nab_r/exe/nab_r_base.riscv",
    "namd": "508.namd_r/exe/namd_r_base.riscv",
    "parest": "510.parest_r/exe/parest_r_base.riscv",
    "povray": "511.povray_r/exe/povray_r_base.riscv",
    "roms": "554.roms_r/exe/roms_r_base.riscv",
}
BASELINE_DIR = Path(__file__).resolve().parent / "baseline"


def source_file(path):
    """A source file's name as both builds see it: the SPEC build compiles
    Fortran from name.fppized.f, the protected-binary pipeline from name.F."""
    name = os.path.basename(path).replace(".fppized", "")
    stem, dot, ext = name.rpartition(".")
    return f"{stem}.{ext.lower()}" if dot else name


def source_lines(binary, pcs):
    """{pc: (file, line)} from the binary's line table (unknown: no entry)."""
    out = subprocess.run([f"{CROSS}-addr2line", "-e", str(binary)],
                         input="\n".join(pcs), capture_output=True,
                         text=True, check=True).stdout.splitlines()
    lines = {}
    for pc, where in zip(pcs, out):
        path, _, line = where.split(" (")[0].rpartition(":")
        if path != "??" and line.isdigit():
            lines[pc] = (source_file(path), int(line))
    return lines


def check_campaign_binary(binary, mnem_pc):
    """Share of profiled PCs whose objdump mnemonic matches the profile."""
    out = subprocess.run([f"{CROSS}-objdump", "-d", "--no-show-raw-insn",
                          str(binary)], capture_output=True, text=True,
                         check=True).stdout
    table = {}
    for line in out.splitlines():
        m = re.match(r"\s*([0-9a-f]+):\s+(\S+)", line)
        if m:
            table[hex(int(m.group(1), 16))] = OBJDUMP_ALIASES.get(
                m.group(2), m.group(2))
    return sum(table.get(pc) == m for pc, m in mnem_pc.items()) / len(mnem_pc)
VISIBLE = ("sdc", "crash", "timeout")
CRASH = ("crash", "timeout")


def mnemonic(disassembly):
    parts = disassembly.split()
    return parts[0] if parts else "?"


def asm_mnemonic(disassembly):
    """The mnemonic GCC's assembly spells this instruction with: QEMU shows
    li as addi from zero (patch_asm.py matches assembly mnemonics)."""
    parts = disassembly.replace(",", " ").split()
    if parts[:1] == ["addi"] and len(parts) > 2 and parts[2] in ("zero", "x0"):
        return "li"
    return parts[0] if parts else "?"


# objdump alias -> the QEMU mnemonic for the same encoding
OBJDUMP_ALIASES = {"li": "addi", "zext.b": "andi", "fmv.w.x": "fmv.s.x"}


def load_profile(root, bench):
    """Per-PC dynamic counts, mnemonics (QEMU's and the assembly's) and
    routines."""
    dyn_pc, mnem_pc, asm_pc, routine_pc = {}, {}, {}, {}
    with open(root / bench / f"{bench}_profile.csv") as f:
        rows = csv.reader(line for line in f if not line.startswith("#"))
        for row in rows:
            if len(row) < 6:
                continue
            dyn_pc[row[0]] = int(row[5])
            mnem_pc[row[0]] = mnemonic(row[1])
            asm_pc[row[0]] = asm_mnemonic(row[1])
            routine_pc[row[0]] = row[2]
    return dyn_pc, mnem_pc, asm_pc, routine_pc


def load_injections(root, bench):
    """Injections that were actually performed, plus the dropped count."""
    kept, dropped = [], 0
    with open(root / bench / "detailed_results.csv") as f:
        for row in csv.DictReader(f):
            if row["fault_injected"] != "True" or row["outcome"] == "error":
                dropped += 1
                continue
            kept.append(row)
    return kept, dropped


def load_old_selection(root, bench, budget):
    path = (root / bench / "aggregations" / "knapsack" /
            f"knapsack_mnemonic_selected_{int(budget * 100)}pct.csv")
    if not path.exists():
        return None
    with open(path) as f:
        return {row["mnemonic"] for row in csv.DictReader(f)}


def half_of(row, seed):
    """Deterministic train (0) / test (1) split of an injection."""
    key = (f"{seed}:{row['instruction_offset']}:{row['execution']}:"
           f"{row['bit']}:{row['phase']}")
    return hashlib.sha1(key.encode()).digest()[0] & 1


class Program:
    """The profiled static instructions and their groupings."""

    def __init__(self, dyn_pc, mnem_pc, asm_pc, routine_pc, line_pc):
        self.dyn_pc = dyn_pc
        self.mnem_pc = mnem_pc
        self.group = {
            "m": dict(mnem_pc),
            "rm": {pc: f"{routine_pc[pc]}|{mnem_pc[pc]}" for pc in mnem_pc},
            # PCs without line info get a key no build can match.
            "line": {pc: (f"{line_pc[pc][0]}:{line_pc[pc][1]}|{asm_pc[pc]}"
                          if pc in line_pc else f"?{pc}|{asm_pc[pc]}")
                     for pc in mnem_pc},
            "pc": {pc: pc for pc in mnem_pc},
        }
        self.dyn = {g: Counter() for g in self.group}
        self.pcs = {g: defaultdict(list) for g in self.group}
        for g, key_of in self.group.items():
            for pc, key in key_of.items():
                self.dyn[g][key] += dyn_pc[pc]
                self.pcs[g][key].append(pc)
        self.total_dyn = sum(dyn_pc.values())


def per_pc_counts(rows):
    n_pc = Counter()
    count = {k: Counter() for k in KINDS}
    for row in rows:
        pc = row["instruction_offset"]
        n_pc[pc] += 1
        count["sdc"][pc] += row["outcome"] == "sdc"
        count["vis"][pc] += row["outcome"] in VISIBLE
        count["crash"][pc] += row["outcome"] in CRASH
    return n_pc, count


def group_means(prog, n_pc, count, key_of):
    """Dynamic-weighted mean rate per group over its injected PCs."""
    weight, n_grp = Counter(), Counter()
    mass = {k: Counter() for k in KINDS}
    for pc, n in n_pc.items():
        if pc not in key_of:
            continue
        key = key_of[pc]
        weight[key] += prog.dyn_pc[pc]
        n_grp[key] += n
        for k in KINDS:
            mass[k][key] += prog.dyn_pc[pc] * count[k][pc] / n
    means = {k: {key: mass[k][key] / weight[key] for key in weight
                 if weight[key] > 0} for k in KINDS}
    return means, n_grp


class Estimates:
    """Outcome rates per mnemonic, (routine, mnemonic) and PC."""

    def __init__(self, prog, rows, alpha=2.0):
        n_pc, count = per_pc_counts(rows)
        self.m, _ = group_means(prog, n_pc, count, prog.group["m"])
        rm_raw, n_rm = group_means(prog, n_pc, count, prog.group["rm"])
        line_raw, n_line = group_means(prog, n_pc, count, prog.group["line"])
        self.line_raw = line_raw

        self.rm = {k: {} for k in KINDS}
        for key, pcs in prog.pcs["rm"].items():
            m = prog.mnem_pc[pcs[0]]
            for k in KINDS:
                prior = self.m[k].get(m, 0.0)
                if key in rm_raw[k]:
                    self.rm[k][key] = ((n_rm[key] * rm_raw[k][key] +
                                        alpha * prior) / (n_rm[key] + alpha))
                else:
                    self.rm[k][key] = prior

        self.line = {k: {} for k in KINDS}
        for key, pcs in prog.pcs["line"].items():
            m = prog.mnem_pc[pcs[0]]
            for k in KINDS:
                prior = self.m[k].get(m, 0.0)
                if key in line_raw[k]:
                    self.line[k][key] = ((n_line[key] * line_raw[k][key] +
                                          alpha * prior) /
                                         (n_line[key] + alpha))
                else:
                    self.line[k][key] = prior

        # Shrunk per-PC rates select; raw ones (the mnemonic's rate when a
        # PC has no injections) evaluate.
        self.pc = {k: {} for k in KINDS}
        self.raw = {k: {} for k in KINDS}
        for pc, m in prog.mnem_pc.items():
            n = n_pc.get(pc, 0)
            for k in KINDS:
                prior = self.rm[k][prog.group["rm"][pc]]
                self.pc[k][pc] = (count[k][pc] + alpha * prior) / (n + alpha)
                self.raw[k][pc] = (count[k][pc] / n if n
                                   else self.m[k].get(m, 0.0))
        self.n_pc = n_pc

    def rates(self, granularity, kind):
        return {"m": self.m, "rm": self.rm, "line": self.line,
                "pc": self.pc}[granularity][kind]


def greedy_fill(order, cost, capacity):
    chosen, used = set(), 0
    for item in order:
        if used + cost[item] <= capacity:
            chosen.add(item)
            used += cost[item]
    return chosen


def knapsack(items, value, cost, capacity, resolution=10000):
    """0/1 knapsack maximizing value within capacity.

    Costs are rounded up to 1/resolution of the capacity, so the selection
    never exceeds the budget. Greedy fill is not enough per mnemonic: a few
    huge mnemonics (lbm's fsub.d/fmul.d/fadd.d) decide the result.
    """
    unit = capacity / resolution if capacity > 0 else 1.0
    items = [i for i in items if value[i] > 0 and cost[i] <= capacity]
    weights = [max(1, int(-(-cost[i] // unit))) for i in items]
    best = [0.0] * (resolution + 1)
    keep = [bytearray(resolution + 1) for _ in items]
    for k, i in enumerate(items):
        w, v = weights[k], value[i]
        for c in range(resolution, w - 1, -1):
            if best[c - w] + v > best[c]:
                best[c] = best[c - w] + v
                keep[k][c] = 1
    chosen, c = set(), resolution
    for k in range(len(items) - 1, -1, -1):
        if keep[k][c]:
            chosen.add(items[k])
            c -= weights[k]
    return chosen


def coverage(pcs, dyn_pc, rate):
    """Share of the dynamic-weighted SDC mass on the selected PCs."""
    total = sum(dyn_pc[pc] * rate[pc] for pc in dyn_pc)
    if total == 0:
        return 0.0
    return sum(dyn_pc[pc] * rate[pc] for pc in pcs) / total


def percentile_ci(samples):
    samples = sorted(samples)
    n = len(samples)
    return samples[int(0.025 * (n - 1))], samples[int(0.975 * (n - 1))]


def jaccard(a, b):
    if a is None or b is None or not (a | b):
        return float("nan")
    return len(a & b) / len(a | b)


def top_shares(pcs, dyn_pc, key_of, n=3):
    """The selection's biggest groups by dynamic share, e.g. 'fmul.d:41%'."""
    by_key = Counter()
    for pc in pcs:
        by_key[key_of[pc]] += dyn_pc[pc]
    used = sum(by_key.values())
    return " ".join(f"{key}:{100 * d / used:.0f}%"
                    for key, d in by_key.most_common(n)) if used else ""


def select(prog, tr, granularity, budget, rng_key):
    """S, L and R selections (sets of group keys) at one granularity."""
    items = [key for key, d in prog.dyn[granularity].items()
             if d > 0 and not key.startswith("?")]
    cost = prog.dyn[granularity]
    capacity = budget * prog.total_dyn
    chosen = {}
    for name, kind in (("S", "sdc"), ("L", "vis")):
        rate = tr.rates(granularity, kind)
        if granularity == "m":
            known = [m for m in items if m in rate]
            value = {m: cost[m] * rate[m] for m in known}
            chosen[name] = knapsack(known, value, cost, capacity)
        else:
            order = sorted(items, key=lambda key: (-rate.get(key, 0.0), key))
            chosen[name] = greedy_fill(order, cost, capacity)
    order = sorted(items)
    random.Random(rng_key).shuffle(order)
    chosen["R"] = greedy_fill(order, cost, capacity)
    return chosen


def analyze(bench, args, rng):
    dyn_pc, mnem_pc, asm_pc, routine_pc = load_profile(args.injection_res,
                                                       bench)
    binary = BASELINE_DIR / CAMPAIGN_BINARY[bench]
    match = check_campaign_binary(binary, mnem_pc)
    if match < 0.98:
        raise SystemExit(f"{bench}: only {match:.1%} of profiled PCs match "
                         f"{binary}; is it the campaign's binary?")
    line_pc = source_lines(binary, list(mnem_pc))
    prog = Program(dyn_pc, mnem_pc, asm_pc, routine_pc, line_pc)
    rows, dropped = load_injections(args.injection_res, bench)

    train = [r for r in rows if half_of(r, args.seed) == 0]
    test = [r for r in rows if half_of(r, args.seed) == 1]
    tr = Estimates(prog, train, args.alpha)
    te = Estimates(prog, test, args.alpha)
    boot = [Estimates(prog, [rng.choice(test) for _ in test], args.alpha)
            for _ in range(args.bootstrap)]

    summary, selections_out, line_files = [], [], {}
    for budget in BUDGETS:
        pct = int(budget * 100)
        chosen = {}  # name -> (granularity, set of group keys)
        for granularity, suffix in (("m", ""), ("rm", "_rm"),
                                    ("line", "_line"), ("pc", "_pc")):
            for name, keys in select(prog, tr, granularity, budget,
                                     f"{args.seed}:{bench}:{granularity}"
                                     ).items():
                chosen[name + suffix] = (granularity, keys)
        old = load_old_selection(args.injection_res, bench, budget)
        if old is not None:
            chosen["OLD"] = ("m", old)

        for name, (granularity, keys) in chosen.items():
            pcs = {pc for key in keys for pc in prog.pcs[granularity][key]}
            used = sum(dyn_pc[pc] for pc in pcs)
            crash = tr.rates(granularity, "crash")
            sdc = tr.rates(granularity, "sdc")
            crashy = sum(prog.dyn[granularity][key] for key in keys
                         if crash.get(key, 0) > sdc.get(key, 0))
            cov = coverage(pcs, dyn_pc, te.raw["sdc"])
            lo, hi = (percentile_ci([coverage(pcs, dyn_pc, b.raw["sdc"])
                                     for b in boot])
                      if boot else (cov, cov))
            summary.append({
                "bench": bench, "budget_pct": pct,
                "selection": name, "granularity": granularity,
                "n_items": len(keys), "n_pcs": len(pcs),
                "dyn_share_used": used / prog.total_dyn,
                "coverage_test": cov, "ci_lo": lo, "ci_hi": hi,
                "crash_dominated_share": crashy / used if used else 0.0,
                "jaccard_vs_old": (jaccard(keys, old)
                                   if granularity == "m" else ""),
                "top_mnemonics": top_shares(pcs, dyn_pc, mnem_pc),
                "top_routines": top_shares(pcs, dyn_pc, routine_pc),
                "injections_used": len(rows),
                "injections_dropped": dropped,
            })
            selections_out.append({
                "bench": bench, "budget_pct": pct, "selection": name,
                "items": " ".join(sorted(keys)),
            })
            if granularity == "line":
                line_files[f"{name}_{pct}pct"] = keys

    mnemonic_rows = [{
        "bench": bench, "mnemonic": m, "dyn": prog.dyn["m"][m],
        "dyn_share": prog.dyn["m"][m] / prog.total_dyn,
        "p_sdc_train": tr.m["sdc"].get(m, ""),
        "p_visible_train": tr.m["vis"].get(m, ""),
        "p_crash_train": tr.m["crash"].get(m, ""),
        "p_sdc_test": te.m["sdc"].get(m, ""),
    } for m, _ in prog.dyn["m"].most_common()]

    line_rows = []
    for key, d in prog.dyn["line"].most_common():
        if key.startswith("?"):
            continue
        where, m = key.rsplit("|", 1)
        file, line = where.rsplit(":", 1)
        line_rows.append({
            "bench": bench, "file": file, "line": line, "mnemonic": m,
            "dyn": d,
            "p_sdc_test": te.line_raw["sdc"].get(
                key, te.m["sdc"].get(OBJDUMP_ALIASES.get(m, m), "")),
        })

    out_dir = args.out / "line_selections" / bench
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, keys in line_files.items():
        triples = sorted(k.rsplit("|", 1)[0].rsplit(":", 1) +
                         [k.rsplit("|", 1)[1]] for k in keys)
        (out_dir / f"{name}.txt").write_text(
            "# file\tline\tmnemonic\n" +
            "".join(f"{f}\t{n}\t{m}\n" for f, n, m in triples))

    print(f"{bench}: {len(rows)} injections ({dropped} dropped), "
          f"{len(prog.dyn['m'])} mnemonics, {len(prog.dyn['line'])} "
          f"(line, mnemonic) pairs, {len(dyn_pc)} PCs "
          f"({len(line_pc) / len(dyn_pc):.1%} with line info, "
          f"campaign binary match {match:.1%})")
    return summary, mnemonic_rows, selections_out, line_rows


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--injection-res", type=Path,
                        default=Path.home() / "Research" / "injection_res")
    parser.add_argument("--out", type=Path,
                        default=Path(__file__).resolve().parent /
                        "selection_coverage")
    parser.add_argument("--benchmarks", default=",".join(BENCHMARKS))
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--alpha", type=float, default=2.0,
                        help="Shrinkage strength, in pseudo-injections")
    parser.add_argument("--bootstrap", type=int, default=200,
                        help="Bootstrap resamples for CIs (0 disables)")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)
    outputs = {"summary.csv": [], "mnemonics.csv": [], "selections.csv": [],
               "lines.csv": []}
    for bench in args.benchmarks.split(","):
        for name, rows in zip(outputs, analyze(bench, args, rng)):
            outputs[name].extend(rows)

    for name, rows in outputs.items():
        with open(args.out / name, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    print(f"Wrote {', '.join(outputs)} and line_selections/ to {args.out}")


if __name__ == "__main__":
    main()
