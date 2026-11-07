# ISCA'27 experiments (second sweep)

The experiments test the paper's four claims:

| # | Claim | Experiment |
|---|---|---|
| T1 | Selective protection guided by fault injection protects much more per instruction protected than an uninformed choice | **E1**: S vs R markers |
| T2 | Some programs have a critical part and a part that may fail (an iterative solver's outer loop vs its preconditioner; kernel vs user space), and the developer can say which | **EF**: FT-GMRES, annotation vs S |
| T3 | Putting the selectivity switch in hardware makes selective protection much cheaper than software duplication, and than protecting everything in hardware | **E2** (resource-sharing mechanisms), **E3** (a spatial checker) |
| T4 | Selectivity in hardware without program semantics is not enough | **E1**, **EF**: S vs AVF-guided protection (Walcott et al., ISCA'07) |

**Status (2026-11-03).**
- Everything is built and checked for the server run (§2).
- FT-GMRES (EF) is done locally (`results_isca27/ftgmres/`, `plots/ef_ftgmres.pdf`).
- Nothing is committed.
- The first sweep is archived:
  - `results_isca27_v1/` (results);
  - `selection_binaries_v1/` (binaries);
  - `ftgmres/campaign_v1/` (FT-GMRES campaign);
  - `archive_pre_isca27/sweep_v1/SWEEPS_v1.md` (its description).

---

## 1. What changed since the first sweep, and why

**1. The campaign's QEMU plugins miscount executions** (`Risc-V-SPEC2017/qemu_pin`).
- The profiler and the injector attach their per-instruction callback only to the *first* translated block that holds an instruction.
- QEMU often translates an instruction into several blocks, for example a loop whose back edge enters a block in the middle. Executions through the other blocks are not seen.
- **Effect on the profile:** its ALU instructions add up to 3–68% of all executed instructions, instead of about 60%.

  | bwaves | cactuBSSN | lbm | nab | namd | parest | povray |
  |---|---|---|---|---|---|---|
  | 7.4% | 3.2% | 68% | 43% | 7.6% | 8.5% | 10% |

- **Effect on the budgets.** Every "x% of dynamic instructions" was a share of wrong counts: bwaves S10 checked 50% of its window, cactuBSSN S50 12%. That caused E2's flat-then-spike curves and EB's dips.
- **Effect on the injections.** The injector's "k-th execution" is the k-th through that first block:
  - SPEC injections are biased toward those instances;
  - the 4,440 SPEC runs that never injected are ones where k was never reached.
- **Fixes:**
  - Fixed copies of the plugins are in `qemu_pin/`; the originals are unchanged.
  - FT-GMRES's campaign was rerun with them. It takes 15 minutes.
  - For SPEC, the counts now come from the gem5 windows (§5.1).
  - Rerunning the 155K-injection SPEC campaign is out of reach before the deadline. Its injection-instance bias is a stated limitation:
    - the outcome probabilities come from real injections into real instances;
    - those instances aren't uniform over time.

**2. L was not AVF.**
- L ranked instructions by P(any visible outcome), measured by injection, i.e. a program vulnerability factor. It uses program semantics as S does, so S vs L compared two software views.
- L is gone.
- The hardware-only baseline is now **AVF-guided protection**, after Walcott et al., ISCA'07. The core measures its own AVF and protects every instruction while the AVF is high, with no markers (§4.2).

**3. Selections didn't nest, and budgets didn't match.**
- The greedy fill skipped keys that didn't fit, so S20 wasn't a superset of S10.
- Selections are now a **threshold on the score**: every key above τ, with τ set by the budget. The levels nest.
- Levels are **10/25/50/75% of the window's ALU instructions**, measured, so the nominal level is the checked share.

**4. Software duplication didn't duplicate.**
- The old `dup` binaries:
  - wrote integer copies to `zero`;
  - replaced in-place FP ops (every accumulation, and bwaves' hot `fdiv.d ft10,ft10,fa3`) by `fmv.d rd,rd`;
  - compared each register with itself.
- That is why software duplication looked cheaper than hardware on bwaves.
- The new duplication recomputes into reserved registers and really compares (§5.2).

**5. DIVA's latency was a nominal 4 cycles for every instruction.** It now follows Austin's DIVA (§4.1).

**6. "Everything protected" wasn't the same work for every mechanism.**
- FuDup could check only ALU instructions (60–74% of the stream), while DIE and DIVA checked everything.
- Now every mechanism protects the same set: the ALU instructions with a destination register, which is the campaign's fault model and what markers can select.

**7. Mechanism classes.**
- *Resource-sharing* mechanisms reuse the core's resources: re-execution on the same unit, re-execution on a second idle unit, DIE twins.
- *Spatial* redundancy pays for dedicated hardware: the DIVA checker.
- The old "spatial" policy (borrow an idle unit, wait for one) is a resource-sharing mechanism, `idlefu`. When the units are busy, it costs what re-execution costs.

**8. Dropped:**
- the hybrid and opportunistic policies; E6 is internal only;
- the OLD type-level family, whose selections come from the same broken profile, and with it roms.

---

## 2. Running on the server

### Setup (once)

1. **On the server, move the first sweep aside.** The binary keys `lbase`, `S10` and `S50` exist in both sweeps, and the old checkpoints would be reused.

   ```
   cd ~/shaft/benchmarks
   mv checkpoints checkpoints_v1
   mv results_isca27 results_isca27_v1
   mv selection_binaries selection_binaries_v1
   ```

2. **Code:** commit and push from the laptop, then `git pull` on the server. The server needs:
   - gem5: `src/cpu/o3/`, `src/cpu/simple/`, `src/cpu/shaft_dup.hh` (new);
   - `benchmarks/`: `sweep.py`, `gem5_config.py`, `gem5_config_o3_only.py`, `make_checkpoints.py`, `run_coverage.py`.

   The selection and build scripts (`select_window.py`, `build_selection_binaries.py`, `ftgmres/`) run only on the laptop.

3. **Binaries** (untracked), from the laptop:

   ```
   rsync -a --exclude 'asm' --exclude 'work' \
         benchmarks/selection_binaries/ server:shaft/benchmarks/selection_binaries/
   ```

4. **Build gem5** on the server:

   ```
   cd ~/shaft && scons build/ALL/gem5.fast -j64
   ```

5. **Check the inputs.** Expect `1449 runs in 69 configs`, `91 checkpoint passes` and no `MISSING` lines. EF (FT-GMRES) runs on the laptop.

   ```
   cd benchmarks && python3 sweep.py list E1 E2 E3 EP
   ```

### Run (in tmux)

```
cd ~/shaft/benchmarks
G=$(realpath ../build/ALL/gem5.fast)
python3 sweep.py checkpoints --gem5 $G -j 100
python3 sweep.py run E1 E2 E3 EP --gem5 $G -j 150 --mem-gb 100
python3 sweep.py run E1 --gem5 $G -j 150 --mem-gb 100    # AVF-guided runs
```

- **Why the second `run`:** AVF-guided runs need the unprotected run's AVF trace at the same SimPoint. The first `run` starts the unprotected runs first and writes each schedule once its trace exists. The second `run` starts what is left.
- **Resuming:** every command skips finished work, so rerun it after any interruption.
- **Time:** about 360 core-hours of runs (at the first sweep's ~12 min per run), about 3 hours at 150 jobs, plus the checkpoint passes (a few hours; parest's are the deepest).

### Collect (on the laptop, which has the RISC-V binutils)

```
rsync -a server:shaft/benchmarks/results_isca27/ benchmarks/results_isca27/
cd benchmarks
python3 sweep.py collect E1 && python3 sweep.py collect E2 && \
  python3 sweep.py collect E3 && python3 sweep.py collect EP && \
  python3 sweep.py collect EF
python3 ftgmres/analyze.py --gem5 results_isca27/ftgmres/none_avf12_*/full
python3 isca27_data.py      # results_isca27/isca27_summary.csv
python3 plot_isca27.py      # plots/*.pdf, bars per level
```

---

## 3. Simulated system and measurement

The setup is unchanged from the first sweep:
- gem5 v25, RISC-V SE, 8-wide O3 at 3 GHz;
- ROB 192, IQ 64, LQ/SQ 32/32;
- functional units: 6 IntALU, 2 IntMultDiv, 4 FP_ALU, 2 FP_MultDiv (FMA 5 cycles; unpipelined divide 12 and square root 24), 4 SIMD, 4 load/store ports;
- 32 kB L1s with 1-cycle hits, 2 MB L2, DDR4-2400.

Each benchmark runs its 3 heaviest SimPoints:
- restore the checkpoint, taken 50M instructions before the interval;
- warm the caches in ATOMIC mode;
- switch to O3 and reset the stats;
- simulate 100M instructions.

Summaries weight the three SimPoints by their renormalized weights.

Markers and software-duplicate instructions are not counted, so every binary of a benchmark measures the same program instructions. All binaries of a benchmark share their code apart from the markers or the duplicates, since every one is compiled with t6/ft11 reserved.

FT-GMRES runs whole, in O3 from the first instruction.

---

## 4. Mechanisms

| Config | Class | How a protected instruction is checked |
|---|---|---|
| `base` | none | unprotected binary |
| `markers` | none | markers fetched, decoded and dropped at rename; nothing checked (the markers' own cost) |
| `temporal` | resource-sharing | re-executed on the **same** unit: +1 cycle and a second issue cycle on a pipelined unit; twice the latency on an unpipelined one |
| `idlefu` | resource-sharing | re-executed on a **second idle unit** of the same class in the same cycle; waits until one is free |
| `die` | resource-sharing | a twin instruction with its own IQ/ROB entry, the same source registers, and any free unit; compared at commit |
| `diva_w<W>[_x2]` | **spatial** | a dedicated in-order checker at commit, W checks started per cycle (§4.1) |
| `sw` | software | the compiled-in duplicate and check (§5.2) |

- `<binary>_all_<mech>` protects every ALU instruction with a destination register (`--shrewd-default-on --protect-domain-only`) of the unmarked binary. It carries no marker cost.
- `<binary>_avf<L>_<mech>` is AVF-guided protection (§4.2).
- `--stuck-fu FloatAdd` makes the first FP_ALU unit corrupt every result. The first mismatch a mechanism sees retires the unit.

### 4.1 The DIVA checker (Austin, MICRO'99; JILP'00 §3.3)

**The checker in Austin's paper:**
- It is as wide as the core.
- An instruction enters it once it has completed and is the oldest one not yet checked.
- Its *computation* pipeline re-executes the instruction on its own copy of the unit, and its latency is the unit's + 1 cycle for the compare.
- Its *communication* pipeline re-reads the registers, and a load's cache line, in 2 cycles, in parallel.
- Branches go through the computation pipeline: the target is recomputed and compared.
- Reported results:
  - checking everything costs 3% on average (14% worst case);
  - 0.1% with a dedicated cache port;
  - +0.7% / +0.8% at 2× / 4× checker latency.

**Our model** (`commit.cc`):
- When a checked instruction reaches the ROB head, its check starts at the later of its completion and the previous start + 1/W, in program order.
- It is done `max(unit latency + 1, 2, 1 + L1 hit for a load)` cycles later, times `_x2` if set.
- For example: integer ALU or branch 2 cycles, FP add 3, FMA 6, divide 13, square root 25.
- The head can't commit before then, and holds its ROB/LSQ entries meanwhile.
- The checker has its own ports, which is Austin's +R+M case.

### 4.2 AVF-guided protection (the hardware-only baseline, after Walcott et al.)

**Walcott et al.:**
- They estimate the AVF of the issue queue, the ROB (with the register file) and the LSQ from occupancy, by Little's law: AVF = ACE bandwidth × ACE residency / bits.
- Every 2M cycles they sum the three. Above a threshold, they turn on redundant multithreading for a fixed time.
- The program has no say.

**Our version, which is deliberately generous to the baseline:**
- **The AVF monitor**, every run, every 100K committed instructions:
  - AVF_X = Σ residency in X of the instructions that commit / (entries of X × cycles), for X = ROB, IQ and LSQ;
  - wrong-path instructions are un-ACE;
  - logged to `<cpu>.shaft_avf.csv`.
- **The schedule:** the AVF-guided run protects the intervals in which the *unprotected* run's summed AVF was highest, until they hold L% of the window's ALU instructions.
  - It is an oracle: it knows each interval's true AVF, at a 20× finer granularity than Walcott's, with free toggles.
  - Within a protected interval it protects every ALU instruction, so it spends exactly S<L>'s budget.
- **What E1 measures:** how much of the SDC risk each one covers. Any interval-level, semantics-free policy covers at most what this oracle covers at the same share.

---

## 5. Programs, selections and binaries

### 5.1 Selections (`select_window.py`)

1. **Keys** are (source file, line, mnemonic) of the build itself.
2. **Dynamic counts** come from the unprotected binary's own gem5 windows:
   - the per-PC commit logs of the first sweep's `lbase_base` runs, SimPoint-weighted;
   - mapped through that binary's line table.

   The campaign's profile is not used.
3. **Rates.** The campaign's injections are re-weighted by those counts, split evenly over a key's campaign PCs.
   - P(SDC) per key comes from the training half, shrunk toward the mnemonic's rate (α = 2).
   - Fortran lines are translated as before.
4. **S** ranks keys by training P(SDC). **R** ranks them randomly.
5. **Levels.** A level is the shortest prefix of the ranking holding 10/25/50/75% of the window's ALU instructions.
   - bwaves has a few huge keys: S10 = S25 (29%), and S75 = 93%.
6. **Held-out SDC coverage** of each selection uses the other half, also shrunk, with 95% bootstrap CIs (`selection_coverage/window/summary.csv`):

   | Benchmark | S10 | R10 | S25 | R25 | S50 | R50 | S75 | R75 |
   |---|---|---|---|---|---|---|---|---|
   | bwaves | 0.48 [0.17, 0.85] | 0.30 [0.01, 0.64] | 0.48 [0.17, 0.85] | 0.30 [0.01, 0.64] | 0.59 [0.23, 1.00] | 0.70 [0.11, 0.84] | 1.00 [0.99, 1.00] | 0.82 [0.40, 0.92] |
   | cactuBSSN | 0.63 [0.26, 0.80] | 0.12 [0.11, 0.13] | 0.75 [0.56, 0.88] | 0.26 [0.25, 0.27] | 1.00 [0.99, 1.00] | 0.54 [0.52, 0.55] | 1.00 [1.00, 1.00] | 0.77 [0.75, 0.78] |
   | lbm | 0.24 [0.21, 0.27] | 0.08 [0.06, 0.11] | 0.47 [0.43, 0.51] | 0.23 [0.20, 0.26] | 0.74 [0.69, 0.77] | 0.54 [0.51, 0.57] | 0.97 [0.95, 0.98] | 0.73 [0.69, 0.76] |
   | nab | 0.06 [0.00, 0.42] | 0.32 [0.03, 0.71] | 0.54 [0.00, 0.95] | 0.37 [0.08, 0.72] | 0.98 [0.20, 1.00] | 0.44 [0.19, 0.77] | 1.00 [0.71, 1.00] | 0.92 [0.50, 0.98] |
   | namd | 0.21 [0.19, 0.22] | 0.11 [0.10, 0.12] | 0.42 [0.40, 0.44] | 0.29 [0.28, 0.32] | 0.71 [0.67, 0.72] | 0.55 [0.53, 0.57] | 0.88 [0.86, 0.89] | 0.80 [0.78, 0.82] |
   | parest | 0.49 [0.41, 0.56] | 0.03 [0.02, 0.05] | 0.79 [0.64, 0.93] | 0.16 [0.11, 0.24] | 0.90 [0.82, 0.99] | 0.43 [0.41, 0.47] | 0.98 [0.93, 1.00] | 0.87 [0.81, 0.92] |
   | povray | 0.49 [0.19, 0.64] | 0.08 [0.04, 0.22] | 0.60 [0.28, 0.78] | 0.40 [0.28, 0.61] | 0.73 [0.45, 0.91] | 0.56 [0.43, 0.75] | 0.91 [0.79, 0.98] | 0.77 [0.66, 0.85] |

   - S beats R everywhere except where SDC risk sits in a few keys with a handful of injections. nab's comes from two keys, `eff.c:2100 fmul.d` and `eff.c:2083 add`, whose SDCs all landed in the held-out half. There no selection is reliable, and the CIs say so.
   - run_coverage.py scores gem5 runs with the same held-out rates (`selection_coverage/window/<b>/lines.csv`, `mnemonics.csv`).

### 5.2 Binaries (`build_selection_binaries.py`, `selection_binaries/build_all.sh`)

All binaries of a benchmark come from the same assembly, compiled once with the pipeline's flags plus `-ffixed-t6 -ffixed-ft11`.

| Key | File | What it is |
|---|---|---|
| `lbase` | `baseline/<exe>.baseline.riscv` | unpatched |
| `S<L>`, `R<L>` | `{S,R}_<L>pct/<exe>.secon_secoff.riscv` | every selected instruction inside `secon`/`secoff`; adjacent protected instructions share one pair, as in the paper |
| `Sdup<L>` | `S_<L>pct/<exe>.duplicate.riscv` | software duplication of S<L> |

**Software duplication.** For each selected instruction `op rd, srcs`:
- If rd is also a source, first copy it: `mv t6,rd` or `fmv.d ft11,rd`.
- Then:
  - integer destination: `op t6, srcs'` and `bne rd,t6,handler`;
  - FP destination: `op ft11, srcs'`, `feq.d t6,rd,ft11` and `beqz t6,handler`.
- That is 2–4 extra executed instructions.
- The handler is the next instruction, so fault-free runs never branch.

**Counting duplicates.** gem5 leaves every instruction on t6/ft11 out of the instruction count (`--dup-reg-no-count`), so no extra markers are needed.
- Library code, which isn't recompiled, uses t6/ft11 in at most 0.03% of any window (`_int_malloc`).
- **Check on lbm:** the baseline, Sdup75 and S75 binaries print identical output after 2 steps under QEMU, and the gem5 window of Sdup10 holds the same 2M program instructions as lbase.

### 5.3 FT-GMRES (`ftgmres/`)

- **Program:** unchanged. FGMRES(30) in FP64 (reliable) around sandboxed FP32 GMRES(10) inner solves (tolerant); the 64 × 64 Poisson problem.
- **Campaign rerun with the fixed plugins** (`ftgmres/campaign/`):
  - 9,000 injections, 3,000 per stratum, uniform over dynamic instructions within each;
  - 8,602 injected; 398 in libgcc's unwinder never fire.
- **The annotation's share of ALU instructions is 11.7%,** not the 2.9% the buggy profile suggested.
- **Held-out outcomes** (`ftgmres/analyze.py`):

  | Policy | Protected share | Harmful [95% CI] | SDC | No conv. | Crash |
  |---|---|---|---|---|---|
  | none | 0 | **7.06%** [6.44, 7.78] | 0.25% | 0.15% | 6.65% |
  | annotated (no profiling) | 11.7% | **0.55%** [0.13, 0.92] | 0 | 0 | 0.55% |
  | S (injection-guided) | 12.3% | **0.55%** [0.15, 0.94] | 0.01% | 0.01% | 0.54% |
  | R | 20.7% | 6.92% | 0.25% | 0.15% | 6.51% |
  | **AVF-guided** (hardware only) | 12.1% | **6.61%** [6.04, 7.22] | 0.24% | 0.14% | 6.23% |
  | full | 100% | 0 | 0 | 0 | 0 |

  - The developer's annotation removes 92% of harmful outcomes and every SDC without any profiling, as well as S does.
  - At the same share, hardware-only AVF guidance removes 6%. It protects the intervals in which the core is most occupied, which are not where the harmful faults are.
  - AVF guidance is scored on the per-PC protection its gem5 run measured. Its runs use the campaign's own binary, so the PCs match.

---

## 6. Experiments

Run counts: SPEC configs run on 7 benchmarks × 3 SimPoints (21 runs), and FT-GMRES configs once. `sweep.py list -v E1` prints the configs.

| Exp | Claim | Configs | Runs |
|---|---|---|---|
| E1 | T1, T4 | 25 | 525 |
| E2 | T3 | 24 | 504 |
| E3 | T3 (spatial) | 29 | 609 |
| EP | supporting | 6 | 126 |
| EF | T2, T4 | 34 | 34 |
| **Total** (shared configs once) | | **103** | **1,483** (91 checkpoint passes) |

### E1: which instructions to protect (T1, T4)

- **Configs:**
  - `S<L>_markers`, `R<L>_markers`: the selection's coverage, from protected instructions;
  - `S<L>_{temporal,die}`;
  - `lbase_avf<L>_{temporal,die}`;
  - `lbase_base` (also the AVF traces).
- **Metric:** SDC coverage at equal protected share (`sdc_protected` for markers runs, `sdc_coverage` otherwise), plus overhead at equal share.
- **Expected:**
  - S ≫ R, and S ≫ AVF-guided at every level;
  - AVF-guided ≈ R unless high-occupancy phases happen to hold SDC-prone code;
  - AVF-guided may cost less at equal share, since high-AVF intervals are often stall-bound and checking is hidden there. So report coverage per overhead too.

### E2: where selectivity pays (T3)

- **Configs:** `S<L>_{temporal,idlefu,die,markers}`, `Sdup<L>_sw`, `lbase_all_{temporal,idlefu,die}`.
- **Metrics:**
  - overhead vs `lbase` (SHAFT's total cost, markers included);
  - the marker share (`markers` runs);
  - the fraction of each mechanism's full-protection overhead SHAFT removes. That fraction compares a mechanism with itself, so it is safe across spheres of replication. Report its geomean over the mechanisms.
- **Expected:**
  - software duplication costs more than every hardware mechanism at the same selection: its extra instructions take fetch, IQ, ROB and FU slots, while markers are dropped at rename;
  - SHAFT at 10–50% costs a fraction of protecting everything;
  - SHAFT's benefit is largest for the mechanisms that strain the core most.
- **Known effect:** where the protected instructions saturate a unit class, every resource-sharing mechanism costs about the same. In bwaves' SimPoint 6 the protected `fdiv.d` keep the two unpipelined dividers busy at least 65M of 81M cycles.

### E3: a spatial checker made smaller (T3)

- **Configs:**
  - `S<L>_diva_w{1,2,4,8}`;
  - `lbase_all_diva_w{1,2,4,8}`;
  - latency ×2: `S25_diva_w*_x2` and `lbase_all_diva_w*_x2`.
- **Metric:** overhead vs checker width. Read off the iso-overhead width, for example the narrowest checker within 5%.
- **Expected:**
  - checking everything needs a checker close to the core's width;
  - S levels need 1–2.
  - The latency floor at large widths is the check latency for instructions on the critical path near commit, not the markers.

### EP: a permanent fault (supporting)

- **Configs:** `S50_stuckFloatAdd_{markers,temporal,idlefu,die,diva_w2}`.
- **Metric:** corrupted results committed.
- **Expected:**
  - `idlefu`, DIE and DIVA retire the unit at the first protected FP add on it;
  - `temporal` reruns on the same broken unit and never detects it.
- **Caveat:** a SimPoint with no protected FP add never detects the unit, as in cactuBSSN's SimPoint 95 in the first sweep.

### EF: FT-GMRES (T2, T4)

- **Configs:**
  - `none_base`;
  - `none_all_*`;
  - `{annotated,S}_{markers,temporal,idlefu,die,diva_w1,diva_w2}`;
  - `annotated_alu_*`;
  - `none_avf12_{temporal,idlefu,die,diva_w1,diva_w2}`;
  - `annotated_stuckFloatAdd_*`.
- **Two forms of the annotation.**
  - The annotation marks a whole region, so `annotated_*` has DIE and DIVA also check the region's loads, stores and branches.
  - `annotated_alu_*` protects only its ALU instructions. That is the same kind of instruction S, AVF-guided protection and "everything" protect, and the like-for-like comparison.
  - The difference between the two is the cost of also checking memory and control instructions.
- **Metrics:**
  - harmful outcomes per policy (§5.3, including AVF-guided);
  - overhead per policy as the geomean over the mechanisms. The campaign gives every mechanism the same protection credit, so per-mechanism bars would invite reading SoR differences that aren't measured.
- **Results (done locally, `plots/ef_ftgmres.pdf`).** Overhead vs unprotected (78M cycles):

  | Policy (ALU share) | same unit | idle unit | DIE | DIVA 1-wide | DIVA 2-wide | geomean |
  |---|---|---|---|---|---|---|
  | annotation, ALU only (11.7%) | 2.69% | 0.07% | 0.58% | 2.01% | 0.23% | 1.1% |
  | annotation, whole region | 2.76% | 0.08% | 0.59% | 5.39% | 1.51% | |
  | S (12.3%) | 2.66% | 0.70% | 1.72% | 2.30% | 0.35% | 1.5% |
  | AVF-guided (12.1%) | 9.34% | 0.22% | 0.73% | 2.05% | 0.23% | 2.5% |
  | everything (100%) | 24.9% | 0.90% | 5.64% | 18.6% | 2.16% | 10.0% |

  - Also checking the annotated region's loads, stores and branches costs DIVA 1-wide 3.4 extra points.
  - **Stuck FP adder, annotation protected:** idle unit, DIE and DIVA retire the unit and give the exact answer (+0.07–1.5%). Unchecked and same-unit runs commit 1.85M corrupted results and end at maxiter with a wrong answer (+174% / +248%).

### EC: coalesced markers (third batch, 2026-11-05)

- **Why:** at 10% the markers are half of SHAFT's cost (1.0 of 1.6–2.3% geomean, no bwaves or povray). Line-level regions are short: 0.75–1.9 markers per protected instruction. cactuBSSN S75 loses 23.6% to an L1I cliff from the markers alone (ANALYSIS_isca27.md §3.1).
- **Binaries:** `S<L>k2` (`build_coalesced_binaries.py` → `selection_binaries_coalesced/<b>/S_<L>pct_k2/`):
  - The S binaries already share one secon/secoff pair between *adjacent* protected instructions, as the paper's did (`patch_asm.optimize_merge_secon_secoff`).
  - `k2` also merges two regions separated by at most 2 straight-line instructions: no control flow and no label except debug labels, so the gap always runs with the first region and the second can only be entered through the gap.
  - The gap's ALU instructions are then checked too: more cost and more coverage.
  - Everything else is identical to the S binary. `--check` compares the code without markers; the only differences are assembler branch expansions (namd only: 12–74 sites, executed at most 19 times per million instructions).
- **Configs:** `S<L>k2_{markers,temporal,idlefu,die,diva_w1,diva_w2}` at 10/25/50/75%, all with `--protect-domain-only`, so a gap's loads and stores stay unchecked as in every other policy. 24 configs × 21 SimPoints = 504 runs; 28 checkpoint passes.
- **Compare with** `S<L>_*` (E2, E3) and `Sdup<L>_sw`. Check the committed op-class mix against `lbase_base` (window alignment), as for the S binaries.

### EL: code-layout controls (third batch)

- **Why:** an identical instruction stream moves by −4.8% to +15.5% on povray SimPoint 16 when only its addresses change (local test, ANALYSIS_isca27.md §3.1). Every marker or SW-duplicate overhead below that band is not attributable.
- **Binaries:** `pad{2,6,16,40}` (`build_layout_variants.py` → `selection_binaries_layout/<b>/pad<N>/`). The unprotected assembly is relinked with N unexecuted bytes ahead of the code.
- **Configs:** `pad<N>_base`, 4 × 21 = 84 runs; 28 checkpoint passes.
- **Metric:** the spread of `pad<N>_base` over `lbase_base` per benchmark, drawn as the noise line in `plots/a8_coalesced`.

### Third batch: commands

On the laptop (binaries are untracked; the server needs `benchmarks/sweep.py`):

```
rsync -a --exclude work benchmarks/selection_binaries_coalesced/ server:shaft/benchmarks/selection_binaries_coalesced/
rsync -a --exclude work benchmarks/selection_binaries_layout/ server:shaft/benchmarks/selection_binaries_layout/
```

On the server, in tmux. EP needs no new checkpoints, so it runs while they are made. Keep the two memory budgets under the machine's 120 GB:

```
cd ~/shaft/benchmarks && G=$(realpath ../build/ALL/gem5.fast)
python3 sweep.py list EP EC EL      # reclassifies EP's failed runs; expect no MISSING
python3 sweep.py checkpoints EC EL --gem5 $G -j 60 --mem-gb 70
python3 sweep.py run EP --gem5 $G -j 60 --mem-gb 40          # second window
python3 sweep.py run EP EC EL --gem5 $G -j 150 --mem-gb 100  # after both
```

Back on the laptop:

```
rsync -a server:shaft/benchmarks/results_isca27/ benchmarks/results_isca27/
cd benchmarks
python3 sweep.py collect EP && python3 sweep.py collect EC && python3 sweep.py collect EL
python3 isca27_data.py && python3 plot_isca27.py && python3 plot_isca27_analysis.py
```

---

### Fourth batch (2026-11-07): one interface, every structure

The bit now reaches structures beyond the units, rename and commit. All new mechanisms are new options with defaults off, so no existing config changes, and the guard EG checks that.

**New mechanisms (gem5):**
- **Memory twins** (CRAFT's checked stores and load-value queue): the existing DIE twin, on marked loads and stores. New stat `commit.dieMemTwins`.
- **RfTwin** (`--mechanism RfTwin`; SwapCodes): the DIE twin writes the result's check bits, and consumers wake only once the twin agrees (`iew.cc` writeback). Stats: `iew.rfTwin{DeferredWakeups,DeferCycles,Mismatches}`.
- **Protected partition** (`--partition Marked|All`; Intel partial mirroring / Virtualized ECC):
  - a committed protected store moves its 4 KiB page into the partition, stalling commit `--partition-migrate-cycles` (6000) once per page;
  - the memory controller then mirrors every write to the page (`src/mem/shaft_partition.hh`, `mem_ctrl.cc`).
  - Stats: `commit.partition{Stores,Migrations,StallCycles,WrittenPages}` and `mem_ctrl*.{protectedWriteReqs,mirrorWriteBursts}`. DRAM energy is read from the ranks.

**Binaries:** `S<L>st` (`build_store_extended_binaries.py` → `selection_binaries_store/`): the S regions, each grown to the store that writes its result (window 8, no control flow), so the marks include stores, a few loads and the pages written. `--check` verified that each differs from its S binary only in marker positions. Static counts are in `stores.json`.

**Experiments:**
- **EX:** `S{10,25,50}st` × {markers, die, rftwin, diva_w2, idlefu, mirror, die_mirror}. 21 runs per config; 21 checkpoint passes.
- **EA:** always-on references `lbase_allmem_{die,diva_w2}` (every instruction, memory and control included), `lbase_all_rftwin`, `lbase_mirrorall`.
- **EFM:** FT-GMRES under RfTwin and the partition.
- **ER:** `S{10,25,50,75}_rftwin` and the stuck FloatAdd.
- **EG:** `S25_{die,idlefu,diva_w2}`, rerun into `--results results_guard`; cycles must equal `results_isca27`'s.

**Commands:** in the session notes of 2026-11-07. In order: pull; checkpoints with the previous gem5 copy while the new one builds; EG; smoke runs on lbm and FT-GMRES; then everything.

## 7. Outputs

**Per run** (`results_isca27/<bench>/<config>/simpoint_<N>/` or `.../full/`):
- `stats.txt`;
- `*.shaft_pcs.csv`: committed, protected and checked per PC;
- `*.shaft_avf.csv`: per 100K-instruction interval, its cycles, instructions, ALU instructions, protected ALU instructions and ROB/IQ/LSQ AVF;
- `simout.txt`, `gem5.log`, `done.json`.

**Schedules:** `results_isca27/<bench>/avf_schedules/<point>_<L>pct.txt`.

**`sweep.py collect`** writes `<EXPS>.csv` (per run) and `<EXPS>_summary.csv` (per config and benchmark, SimPoint-weighted). The summary columns:
- `overhead`, against the family's base;
- `sdc_coverage`, over checked instructions;
- `sdc_protected`, over protected instructions;
- `protected_share` and `checked_share`, over all committed instructions;
- `domain_protected_share`, over ALU instructions;
- `avf`, the cycle-weighted ROB + IQ + LSQ AVF;
- `faults_committed`.

---

## 8. Caveats

- **SPEC injection instances** are biased by the plugin bug (§1); FT-GMRES's are not.
- **Coverage is window-local and held-out.** For nab and bwaves it is dominated by a few keys with few injections, so their CIs are wide.
- **povray**'s overheads below about ±6% are noise.
  - On the same binary, delaying a few ALU results changes how often its 32-entry load queue fills (−18 to −28%) and how its store-set predictor trains.
  - That is why the temporal S20 run was 5–7% *faster* than markers-only.
- **parest and povray** are front-end bound and layout-sensitive, which affects marker cost vs `lbase`.
- **povray SimPoint 82** executed different code under different mechanisms in the first sweep (its control flow depends on timing). Check it with `results_isca27_v1/alignment.csv` and drop it if it repeats.
- **Detection is an oracle.** The timing of checking is modeled; faults exist only in stuck-unit mode.
- **Not done:** area and energy (McPAT or synthesis) for E3's checker widths, and full-system kernel-only protection (T2's second example, argued in the text).
