#!/usr/bin/env python3
import os
import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

SIMPOINT_SUMMARY = "top3_summary.txt"
RESULTS_DIR = "results"
DUPLICATE_DIR = "results_duplicate"
OUTPUT_PATH = "average_slowdown.png"

# ── 1. Parse top3_summary.txt for weights ──
def parse_simpoint_summary(summary_path):
    simpoints = {}
    current_bench = None
    if not os.path.exists(summary_path):
        print(f"ERROR: Cannot find {summary_path}")
        return simpoints
    with open(summary_path) as f:
        for line in f:
            line = line.strip()
            m = re.match(r"^Benchmark\s*:\s*(\S+)", line)
            if m:
                bench_raw = m.group(1)
                current_bench = re.sub(r"_baseline$", "", bench_raw)
                simpoints[current_bench] = []
                continue
            if current_bench is not None:
                m = re.match(r"^\d+\s+(\d+)\s+([\d.]+)\s+\d+\s*$", line)
                if m:
                    interval = int(m.group(1))
                    weight = float(m.group(2))
                    simpoints[current_bench].append((interval, weight))
    return simpoints

# ── 2. Read cycles and numInsts from stats.txt ──
def get_stats_from_file(stats_path):
    cycles, num_insts = None, None
    if not os.path.exists(stats_path) or os.path.getsize(stats_path) == 0:
        return cycles, num_insts
    with open(stats_path, "r") as f:
        for line in f:
            if "board.processor.switch.core.numCycles" in line:
                try: cycles = int(line.split()[1])
                except (IndexError, ValueError): pass
            elif "board.processor.switch.core.commitStats0.numInsts" in line \
                    and "NotNOP" not in line:
                try: num_insts = int(line.split()[1])
                except (IndexError, ValueError): pass
    return cycles, num_insts

# ── 3. Accumulate weighted CPI per (benchmark, variant) ──
simpoints_map = parse_simpoint_summary(SIMPOINT_SUMMARY)
weighted_cycles = {}
weighted_insts = {}

protection_pcts = [5, 10, 20, 50, 100]
hw_variants_list = [f"{p}pct" for p in protection_pcts]
sw_variants_list = [f"dup_{p}pct" for p in [5, 10, 20, 50]]
all_variants = ["baseline"] + hw_variants_list + sw_variants_list

for bench, sp_list in simpoints_map.items():
    for var in all_variants:
        tot_cycles = 0.0
        tot_insts = 0.0
        is_valid = True
        for interval, weight in sp_list:
            if var.startswith("dup_"):
                base_dir = DUPLICATE_DIR
                var_dir = var.replace("dup_", "")
            else:
                base_dir = RESULTS_DIR
                var_dir = var
            stats_path = os.path.join(base_dir, bench, var_dir,
                                      f"simpoint_{interval}", "stats.txt")
            cycles, num_insts = get_stats_from_file(stats_path)
            if cycles is None or num_insts is None:
                is_valid = False
                break
            tot_cycles += cycles * weight
            tot_insts += num_insts * weight
        if is_valid and tot_insts > 0:
            weighted_cycles[(bench, var)] = tot_cycles
            weighted_insts[(bench, var)] = tot_insts

# CPI = weighted_cycles / weighted_insts
cpi = {k: weighted_cycles[k] / weighted_insts[k] for k in weighted_cycles}

# ── 4. Identify benchmarks that have protection variants ──
sw_variants = {p: f"dup_{p}pct" for p in protection_pcts}
hw_variants = {p: f"{p}pct" for p in protection_pcts}

benchmarks = sorted({bm for (bm, var) in cpi if var != "baseline"
                     and (var in sw_variants.values() or var in hw_variants.values())})

# ── 5. Compute per-benchmark slowdown for each protection % ──
#    slowdown(%) = (variant_CPI - baseline_CPI) / baseline_CPI * 100
sw_slowdowns = {p: [] for p in protection_pcts}
hw_slowdowns = {p: [] for p in protection_pcts}

for bm in benchmarks:
    base = cpi.get((bm, "baseline"))
    if base is None:
        continue
    for p in protection_pcts:
        sw = cpi.get((bm, sw_variants[p]))
        hw = cpi.get((bm, hw_variants[p]))
        if sw is not None:
            sw_slowdowns[p].append((sw - base) / base * 100)
        if hw is not None:
            hw_slowdowns[p].append((hw - base) / base * 100)

# Average across benchmarks for each protection %
sw_avg = [np.mean(sw_slowdowns[p]) if sw_slowdowns[p] else np.nan for p in protection_pcts]
hw_avg = [np.mean(hw_slowdowns[p]) if hw_slowdowns[p] else np.nan for p in protection_pcts]

# ── 6. Plot ──
x = np.arange(len(protection_pcts))
width = 0.35

fig, ax = plt.subplots(figsize=(8, 5))
bars_sw = ax.bar(x - width / 2, sw_avg, width, label="Software Duplication", color="#4C72B0")
bars_hw = ax.bar(x + width / 2, hw_avg, width, label="Hardware Duplication", color="#DD8452")

ax.set_xlabel("Protection (%)")
ax.set_ylabel("Slowdown (%)")
ax.set_title("Performance Overhead vs. Protection Level")
ax.set_xticks(x)
ax.set_xticklabels([f"{p}%" for p in protection_pcts])
ax.legend()
ax.axhline(0, color="black", linewidth=0.8)

# Value labels on bars
for bar in list(bars_sw) + list(bars_hw):
    h = bar.get_height()
    if np.isnan(h):
        continue
    ax.annotate(f"{h:.1f}%",
                xy=(bar.get_x() + bar.get_width() / 2, h),
                xytext=(0, 3), textcoords="offset points",
                ha="center", va="bottom", fontsize=8)

fig.tight_layout()
fig.savefig(OUTPUT_PATH, dpi=150)
print(f"Plot saved to {OUTPUT_PATH}")

# Also print the table for reference
print(f"\n{'Protection':>12}  {'SW Slowdown':>12}  {'HW Slowdown':>12}  {'SW #bm':>6}  {'HW #bm':>6}")
for p, s, h in zip(protection_pcts, sw_avg, hw_avg):
    sn = len(sw_slowdowns[p])
    hn = len(hw_slowdowns[p])
    print(f"{p:>11}%  {s:>11.2f}%  {h:>11.2f}%  {sn:>6}  {hn:>6}")

print("\nPer-benchmark Slowdown:")
for bm in benchmarks:
    base = cpi.get((bm, "baseline"))
    if base is None:
        continue
    parts = []
    for p in protection_pcts:
        sw = cpi.get((bm, sw_variants[p]))
        hw = cpi.get((bm, hw_variants[p]))
        sw_s = f"{(sw-base)/base*100:.1f}" if sw else "N/A"
        hw_s = f"{(hw-base)/base*100:.1f}" if hw else "N/A"
        parts.append(f"{p}%: SW={sw_s}% HW={hw_s}%")
    print(f"  {bm}: {', '.join(parts)}")
