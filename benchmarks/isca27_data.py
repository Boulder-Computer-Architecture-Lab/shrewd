#!/usr/bin/env python3
"""
Tidy per-(config, benchmark) table of the second ISCA'27 sweep, from the
per-run CSVs that `sweep.py collect` writes.

Config names (sweep.py): <key>_<mech>, <key>_all_<mech> (every ALU
instruction protected), <key>_avf<L>_<mech> (AVF-guided, L% of them),
<key>_alu_<mech> (only the marked ALU instructions) and
<key>_stuck<Class>_<mech> (a permanently faulty unit). Keys: lbase,
S<L>, R<L>, Sdup<L>, S<L>k<K> (coalesced markers), pad<N> (layout
controls) (SPEC); none, annotated, S (FT-GMRES).

Columns, each SimPoint-weighted over the SimPoints both runs have:
  vs_base          cycles over the family's unprotected run - 1 (the whole
                   cost: markers, duplicates and checking)
  vs_markers       cycles over the same binary's markers-only run - 1
                   (the checking alone; S/R binaries)
  marker_cost      the binary's markers-only run over the unprotected one - 1
  sdc_coverage     SDC risk on checked instructions / all (run_coverage.py)
  sdc_protected    the same over protected instructions (what a selection
                   covers whatever the mechanism; markers-only runs)
  protected_share  protected ALU instructions / ALU instructions
  avf              cycle-weighted ROB + IQ + LSQ AVF
  faults_committed, protected_faults_committed  summed over SimPoints
  sw_dup_pairs, sw_dup_detected, sw_dup_undetected  software duplication
                   under a stuck unit (ES): pairs checked, caught (one copy
                   corrupted), passed (both corrupted alike); summed
  crashed          SimPoints where the program crashed (a stuck unit's
                   corrupted results); they have no statistics, so every
                   other column covers the remaining SimPoints

Writes <results>/isca27_summary.csv.

Usage: python3 isca27_data.py [--results results_isca27]
"""
import argparse
import re
from pathlib import Path

import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENTS = ("E1", "E2", "E3", "EP", "ES", "EF", "EV", "EC", "EL")
# povray's SimPoint 82 executed different code under different mechanisms in
# the first sweep (its control flow depends on timing).
EXCLUDED = {("povray", 82)}
CONFIG = re.compile(r"(?P<key>lbase|none|annotated|S|S\d+k\d+|[SR]\d+|"
                    r"Sdup\d+|pad\d+)"
                    r"(?:_(?P<all>all)|_(?P<alu>alu)|_avf(?P<avf>\d+)|"
                    r"_stuck(?P<stuck>[A-Za-z]+))?_(?P<mech>.+)")


def load(results):
    frames = [pd.read_csv(results / f"{e}.csv") for e in EXPERIMENTS
              if (results / f"{e}.csv").exists()]
    if not frames:
        raise SystemExit(f"No <EXP>.csv in {results}: run sweep.py collect")
    runs = pd.concat(frames).drop_duplicates(["config", "bench", "interval"])
    runs["interval"] = runs["interval"].fillna(-1).astype(int)
    keep = [(b, i) not in EXCLUDED for b, i in zip(runs.bench, runs.interval)]
    return runs[keep]


def parse(config):
    m = CONFIG.fullmatch(config)
    if not m:
        raise ValueError(f"unknown config {config}")
    key = m["key"]
    sel = re.fullmatch(r"(Sdup|S|R)(\d+)(k\d+)?", key)
    return {"key": key, "mech": m["mech"],
            "kind": ("all" if m["all"] else "avf" if m["avf"] else
                     "alu" if m["alu"] else "stuck" if m["stuck"] else
                     "marked"),
            "sel": sel[1] + (sel[3] or "") if sel else key,
            "level": (int(sel[2]) if sel else int(m["avf"]) if m["avf"]
                      else 100 if m["all"] else None),
            "stuck": m["stuck"]}


def weighted(rows, column):
    return (rows.weight * rows[column]).sum() / rows.weight.sum()


def ratio(a, b, column="cycles"):
    """SimPoint-weighted a/b - 1 over the SimPoints both have."""
    common = sorted(set(a.interval) & set(b.interval))
    if not common:
        return float("nan")
    a = a.set_index("interval").loc[common]
    b = b.set_index("interval").loc[common]
    w = a.weight
    return (w * a[column]).sum() / (w * b[column]).sum() - 1


def summarize(runs):
    groups = {k: g for k, g in runs.groupby(["config", "bench"])}
    out = []
    for (config, bench), rows in groups.items():
        info = parse(config)
        crash = (rows.crashed.fillna(0).astype(bool) if "crashed" in rows
                 else pd.Series(False, index=rows.index))
        crashes = {"crashed": int(crash.sum()),
                   "crash": "; ".join(rows.crash[crash].dropna().unique())
                   if crash.any() else ""}
        rows = rows[~crash]
        if rows.empty:
            out.append({"config": config, "bench": bench, **info,
                        "simpoints": 0, **crashes})
            continue
        base = groups.get((rows.base.iloc[0], bench))
        markers = groups.get((f"{info['key']}_markers", bench))
        mass = (rows.weight * rows.get("sdc_mass", 0)).sum()
        domain = (rows.weight * rows.get("domain_committed", 0)).sum()
        row = {"config": config, "bench": bench, **info,
               "simpoints": len(rows), **crashes,
               "cycles": weighted(rows, "cycles"),
               "vs_base": ratio(rows, base) if base is not None else None,
               "vs_markers": (ratio(rows, markers)
                              if markers is not None and info["kind"] == "marked"
                              else None),
               "marker_cost": (ratio(markers, base)
                               if markers is not None and base is not None
                               else None)}
        if mass:
            row["sdc_coverage"] = (rows.weight * rows.sdc_covered).sum() / mass
            row["sdc_protected"] = (rows.weight * rows.sdc_protected).sum() / mass
        if domain:
            row["protected_share"] = (
                (rows.weight * rows.domain_protected).sum() / domain)
        if "avf_rob" in rows:
            avf = rows[["weight", "avf_rob", "avf_iq", "avf_lsq"]].fillna(0)
            row["avf"] = sum(weighted(avf, f"avf_{k}")
                             for k in ("rob", "iq", "lsq"))
        for col in ("faults_committed", "protected_faults_committed",
                    "stuck_fu_results", "sw_dup_pairs", "sw_dup_detected",
                    "sw_dup_undetected"):
            if col in rows:
                row[col] = rows[col].sum()
        if "stuck_fu_retire_cycle" in rows:
            row["retired"] = int((rows.stuck_fu_retire_cycle > 0).sum())
        if "output" in rows:
            row["output"] = rows.output.iloc[0]
        out.append(row)
    return pd.DataFrame(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--results", type=Path,
                        default=SCRIPT_DIR / "results_isca27")
    args = parser.parse_args()
    table = summarize(load(args.results))
    path = args.results / "isca27_summary.csv"
    table.to_csv(path, index=False)
    print(f"{len(table)} rows -> {path}")


if __name__ == "__main__":
    main()
