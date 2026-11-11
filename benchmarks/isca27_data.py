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
  page_share       protected partition: pages migrated / pages written
                   (per window, SimPoint-weighted)
  mirror_share     mirrored DRAM write bursts / the run's own write bursts
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
EXPERIMENTS = ("E1", "E2", "E3", "EP", "ES", "EF", "EV", "EC", "EL", "EX",
               "EA", "EAC", "EFM", "ER", "ED", "EW", "EWF")
# povray's SimPoint 82 executed different code under different mechanisms in
# the first sweep (its control flow depends on timing).
EXCLUDED = {("povray", 82)}
CONFIG = re.compile(r"(?P<key>lbase|none|annotated|S|S\d+k\d+|S\d+st|"
                    r"[SR]\d+|Sdup\d+|pad\d+)"
                    r"(?:_(?P<allmem>allmem)|_(?P<all>all)|_(?P<alu>alu)|"
                    r"_avf(?P<avf>\d+)|_stuck(?P<stuck>[A-Za-z]+))?"
                    r"_(?P<mech>.+)")


def load(results):
    frames = [pd.read_csv(results / f"{e}.csv") for e in EXPERIMENTS
              if (results / f"{e}.csv").exists()]
    if not frames:
        raise SystemExit(f"No <EXP>.csv in {results}: run sweep.py collect")
    # A run collected by several experiments appears once; columns only
    # some collections have (e.g. memory statistics) are merged.
    runs = pd.concat(frames).groupby(["config", "bench", "interval"],
                                     dropna=False, as_index=False).first()
    runs["interval"] = runs["interval"].fillna(-1).astype(int)
    keep = [(b, i) not in EXCLUDED for b, i in zip(runs.bench, runs.interval)]
    return runs[keep]


def parse(config):
    m = CONFIG.fullmatch(config)
    if not m:
        raise ValueError(f"unknown config {config}")
    key = m["key"]
    sel = re.fullmatch(r"(Sdup|S|R)(\d+)(k\d+|st)?", key)
    return {"key": key, "mech": m["mech"],
            "kind": ("allmem" if m["allmem"] else "all" if m["all"] else
                     "avf" if m["avf"] else "alu" if m["alu"] else
                     "stuck" if m["stuck"] else "marked"),
            "sel": sel[1] + (sel[3] or "") if sel else key,
            "level": (int(sel[2]) if sel else int(m["avf"]) if m["avf"]
                      else 100 if m["all"] or m["allmem"] else None),
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
    den = (w * b[column]).sum()
    # 0 for runs without the event (povray and FT-GMRES write no DRAM).
    return (w * a[column]).sum() / den - 1 if den else float("nan")


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
        # Runs collected before the memory statistics existed have none
        # (NaN), so these ratios need them in both runs.
        if base is not None and "dram_energy_pj" in rows and \
                "dram_energy_pj" in base and \
                rows.dram_energy_pj.notna().all() and \
                base.dram_energy_pj.notna().all():
            row["dram_energy_vs_base"] = ratio(rows, base, "dram_energy_pj")
        if base is not None and "mem_write_bursts" in rows and \
                "mem_write_bursts" in base and \
                rows.mem_write_bursts.notna().all() and \
                base.mem_write_bursts.notna().all():
            def writes(r):
                mirror = (r.mem_mirror_write_bursts.fillna(0)
                          if "mem_mirror_write_bursts" in r else 0)
                return r.assign(mem_writes=r.mem_write_bursts + mirror)
            row["mem_writes_vs_base"] = ratio(writes(rows), writes(base),
                                              "mem_writes")
        if "partition_written_pages" in rows and \
                (rows.partition_written_pages.fillna(0) > 0).all():
            # Pages the partition holds (each migrated once per window)
            # over the pages the window writes.
            row["page_share"] = weighted(
                rows.assign(s=rows.partition_migrations
                            / rows.partition_written_pages), "s")
        if "mem_write_bursts" in rows and rows.mem_write_bursts.notna().all() \
                and (rows.weight * rows.mem_write_bursts).sum() > 0:
            # Mirrored DRAM writes over the program's own write bursts.
            row["mirror_share"] = (
                (rows.weight * rows.mem_mirror_write_bursts.fillna(0)).sum()
                / (rows.weight * rows.mem_write_bursts).sum())
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
                    "sw_dup_undetected", "die_mem_twins", "partition_stores",
                    "partition_migrations", "partition_stall_cycles",
                    "partition_written_pages", "rftwin_deferred",
                    "rftwin_defer_cycles", "rftwin_mismatches",
                    "mem_protected_write_reqs", "mem_mirror_write_bursts",
                    "mem_write_bursts", "protection_toggles",
                    "mode_drains", "serialize_stall_cycles",
                    "st_protected_stores", "st_lines_marked", "st_stores",
                    "st_coalesced_stores", "st_write_throughs",
                    "st_drains_skipped", "st_drains_deferred",
                    "l2_writeclean",
                    "l2_writeback_dirty", "l2_accesses", "l1d_writebacks"):
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
