#!/usr/bin/env python3
"""
Expected SDC coverage of gem5 runs, from their per-PC commit logs.

Each run directory holds <cpu>.shaft_pcs.csv (written by gem5 when
shaftCommitLog is on): per committed PC, how many instances committed, were
protected, and were actually checked by the protection mechanism. Each PC is
weighted by its held-out P(SDC):

    coverage = sum(checked * P_SDC) / sum(committed * P_SDC)

over the instructions the injection campaign covers (ALU instructions with a
destination register). A PC's rate is that of its (source line, mnemonic)
when the campaign measured it (select_window.py writes the rates by the
build's own line numbers), else that of its mnemonic; mnemonics come from
objdump (the alias names QEMU printed, e.g. mv, fmv.d) and source lines from
the binary's line table. sdc_protected weights the protected instead of the
checked instructions (the selection's coverage, whatever the mechanism).

Usage:
    python3 run_coverage.py --bench lbm RUN_DIR [RUN_DIR ...]
"""
import argparse
import csv
import re
import subprocess
import sys
from functools import lru_cache
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from selection_coverage import OBJDUMP_ALIASES, source_file  # noqa: E402

CROSS = "/opt/riscv/bin/riscv64-unknown-linux-gnu"
SELECTION_DIR = SCRIPT_DIR / "selection_coverage" / "window"
BUILD_LINES = SELECTION_DIR / "{bench}" / "lines.csv"


def pc_instructions(binary, pcs):
    """PC -> (mnemonic as QEMU names it, mnemonic as GCC writes it), for
    pcs. objdump's output is streamed: whole, it is ~150 MB for parest."""
    cmd = [f"{CROSS}-objdump", "-d", "--no-show-raw-insn", binary]
    table = {}
    with subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True) as proc:
        for line in proc.stdout:
            m = re.match(r"\s*([0-9a-f]+):\s+(\S+)", line)
            if m and int(m.group(1), 16) in pcs:
                table[int(m.group(1), 16)] = (
                    OBJDUMP_ALIASES.get(m.group(2), m.group(2)), m.group(2))
    if proc.returncode:
        raise subprocess.CalledProcessError(proc.returncode, cmd)
    return table


def pc_lines(binary, pcs):
    """PC -> (file, line) from the binary's line table."""
    out = subprocess.run([f"{CROSS}-addr2line", "-e", binary],
                         input="\n".join(hex(pc) for pc in pcs),
                         capture_output=True, text=True,
                         check=True).stdout.splitlines()
    lines = {}
    for pc, where in zip(pcs, out):
        path, _, line = where.split(" (")[0].rpartition(":")
        if path != "??" and line.isdigit():
            lines[pc] = (source_file(path), int(line))
    return lines


def run_binary(run_dir):
    """The simulated process's executable, resolved against its cwd."""
    config = (Path(run_dir) / "config.ini").read_text()
    exe = re.search(r"^executable=(.+)$", config, re.M)
    cwd = re.search(r"^cwd=(.+)$", config, re.M)
    if not exe:
        raise SystemExit(f"{run_dir}: no executable in config.ini")
    path = Path(exe.group(1))
    if not path.is_absolute() and cwd:
        path = Path(cwd.group(1)) / path
    return str(path)


@lru_cache(maxsize=None)
def rate_tables(bench):
    """Held-out P(SDC) per mnemonic and per (file, line, mnemonic)."""
    by_mnemonic, by_line = {}, {}
    with open(SELECTION_DIR / "mnemonics.csv") as f:
        for row in csv.DictReader(f):
            if row["bench"] == bench and row["p_sdc_test"] != "":
                by_mnemonic[row["mnemonic"]] = float(row["p_sdc_test"])
    path = Path(str(BUILD_LINES).format(bench=bench))
    if path.exists():
        with open(path) as f:
            for row in csv.DictReader(f):
                if row["p_sdc_test"] != "":
                    by_line[(row["file"], int(row["line"]),
                             row["mnemonic"])] = float(row["p_sdc_test"])
    return by_mnemonic, by_line


def read_log(run_dir):
    """(pc, committed, protected, checked) per PC of a run, or None."""
    logs = sorted(Path(run_dir).glob("*.shaft_pcs.csv"))
    if not logs:
        return None
    with open(logs[-1]) as f:
        return [(int(r["pc"], 16), int(r["committed"]), int(r["protected"]),
                 int(r["checked"])) for r in csv.DictReader(f)]


def pc_tables(binary, pcs):
    """Mnemonics and source lines of pcs in binary."""
    return pc_instructions(binary, set(pcs)), pc_lines(binary, sorted(pcs))


def coverage(run_dir, bench, binary=None, tables=None):
    """Commit-log totals of one run, weighted by held-out P(SDC). binary
    defaults to the run's executable; tables (pc_tables) may be shared by
    the runs of one binary."""
    rows = read_log(run_dir)
    if rows is None:
        return None
    insts, lines = tables or pc_tables(binary or run_binary(run_dir),
                                       [pc for pc, *_ in rows])
    by_mnemonic, by_line = rate_tables(bench)

    t = dict.fromkeys(("committed", "protected", "checked",
                       "domain_committed", "domain_protected",
                       "domain_checked"), 0)
    t["sdc_mass"] = t["sdc_covered"] = t["sdc_protected"] = 0.0
    for pc, n, prot, chk in rows:
        t["committed"] += n
        t["protected"] += prot
        t["checked"] += chk
        qemu_name, asm_name = insts.get(pc, (None, None))
        if qemu_name not in by_mnemonic:
            continue  # outside the campaign's fault model
        p = by_line.get(lines.get(pc, ("?", 0)) + (asm_name,),
                        by_mnemonic[qemu_name])
        t["domain_committed"] += n
        t["domain_protected"] += prot
        t["domain_checked"] += chk
        t["sdc_mass"] += n * p
        t["sdc_covered"] += chk * p
        t["sdc_protected"] += prot * p
    return t


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--bench", required=True)
    args = parser.parse_args()

    print("run,committed,protected_share,checked_share,domain_share,"
          "sdc_coverage")
    for run_dir in args.runs:
        t = coverage(run_dir, args.bench)
        if t is None:
            print(f"{run_dir},no shaft_pcs.csv")
            continue
        n = t["committed"]
        print(f"{run_dir},{n},{t['protected'] / n:.4f},"
              f"{t['checked'] / n:.4f},{t['domain_committed'] / n:.4f},"
              f"{t['sdc_covered'] / t['sdc_mass'] if t['sdc_mass'] else 0:.4f}")


if __name__ == "__main__":
    main()
