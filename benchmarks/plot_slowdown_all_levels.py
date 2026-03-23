#!/usr/bin/env python3
import os
import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

SIMPOINT_SUMMARY = "top3_summary.txt"
RESULTS_DIR = "results"

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
hw_variants = [f"{p}pct" for p in protection_pcts]
all_variants = ["baseline"] + hw_variants

for bench, sp_list in simpoints_map.items():
    for var in all_variants:
        tot_cycles = 0.0
        tot_insts = 0.0
        is_valid = True
        
        for interval, weight in sp_list:
            stats_path = os.path.join(RESULTS_DIR, bench, var, f"simpoint_{interval}", "stats.txt")
            
            cycles, num_insts = get_stats_from_file(stats_path)
            if cycles is None or num_insts is None:
                print(f"Missing or empty: {stats_path} -> Discarding variant {var} for {bench}")
                is_valid = False
                break
                
            tot_cycles += cycles * weight
            tot_insts += num_insts * weight
            
        if is_valid and tot_insts > 0:
            weighted_cycles[(bench, var)] = tot_cycles
            weighted_insts[(bench, var)] = tot_insts

# CPI = weighted_cycles / weighted_insts
cpi = {k: weighted_cycles[k] / weighted_insts[k] for k in weighted_cycles}

# ── 4. Prepare data for the grouped bar chart ──
# Only keep benchmarks that at least have a valid baseline run
benchmarks = sorted([bm for bm in simpoints_map.keys() if cpi.get((bm, "baseline")) is not None])

hw_plot_data = {p: [] for p in protection_pcts}
hw_plot_ratios = {p: [] for p in protection_pcts}

for bm in benchmarks:
    base = cpi.get((bm, "baseline"))
    for p in protection_pcts:
        var = f"{p}pct"
        val = cpi.get((bm, var))
        if val is not None:
            ratio = val / base
            hw_plot_data[p].append((ratio - 1) * 100)
            hw_plot_ratios[p].append(ratio)
        else:
            hw_plot_data[p].append(np.nan)

# ── 5. Calculate and append Geometric Mean ──
benchmarks.append("GeoMean")
for p in protection_pcts:
    ratios = hw_plot_ratios[p]
    if ratios: 
        # Calculate Geomean over whatever valid ratios exist for this protection level
        gmean_ratio = np.exp(np.mean(np.log(ratios)))
        hw_plot_data[p].append((gmean_ratio - 1) * 100)
    else:
        hw_plot_data[p].append(np.nan)

# ── 6. Plot the single grouped chart ──
x = np.arange(len(benchmarks))
n_bars = len(protection_pcts)
width = 0.8 / n_bars
# Calculate offsets to center the cluster of bars over the x-tick
offsets = np.linspace(-width * (n_bars - 1) / 2, width * (n_bars - 1) / 2, n_bars)

fig, ax = plt.subplots(figsize=(14, 6))

# A nice 5-color palette for the different HW percentages
colors = ["#4C72B0", "#DD8452", "#55A868", "#C44E52", "#8172B3"]

for i, p in enumerate(protection_pcts):
    ax.bar(x + offsets[i], hw_plot_data[p], width, label=f"{p}%", color=colors[i])

ax.set_xlabel("Benchmark")
ax.set_ylabel("Slowdown (%)")
ax.set_title("Hardware Duplication Performance Overhead")
ax.set_xticks(x)

# Bold the GeoMean label
labels = [lbl if lbl != "GeoMean" else "$\\bf{GeoMean}$" for lbl in benchmarks]
ax.set_xticklabels(labels, rotation=30, ha="right")

ax.legend(title="Protection Level")
ax.axhline(0, color="black", linewidth=0.8)

# Add a vertical dashed line to separate benchmarks from the GeoMean
ax.axvline(x[-2] + 0.5, color="gray", linestyle="--", linewidth=1.0)

fig.tight_layout()
out_file = "hw_slowdown_all_levels.png"
fig.savefig(out_file, dpi=150)
plt.close(fig)
print(f"Saved {out_file}")