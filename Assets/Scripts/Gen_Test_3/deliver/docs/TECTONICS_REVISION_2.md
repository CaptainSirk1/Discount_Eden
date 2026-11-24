# Discount Eden — Tectonics Revision 2: Material Balance and Mechanics

This revision replaces the mesh simulation's material accounting, adds a closed crustal material cycle, and replaces the
local extension constraints with a crust-strength solve plus emergent rigid continental blocks. The persistent
corner-table mesh and its topology operations are unchanged in structure.

**How things were verified** (labels used throughout):
- **[H] headless execution.** The real job structs, run serially under .NET 8 against stubbed `Unity.Jobs`,
  `Unity.Collections` and `Unity.Burst`, with the real `Unity.Mathematics` source. This is a *stubbed testing environment*.
- **[T] type-check.** Every deliverable file compiles against hand-written UnityEngine/UnityEditor stubs.
- **[R] reasoning / code inspection.**

**Not verified:**
- Nothing here ran in Unity.
- **Burst compilation, the job-safety system, and runtime performance in Unity are untested.** See *Risks for the first
  Unity run* below.

Default resolution for tests was ~5,000 vertices (2 CPU cores available); selected checks ran at ~20,000.

---

## 1. Why the previous simulation lost continents

I instrumented the original code without changing its behaviour ([H], `HeadlessHarness/BaselineProgram.cs.txt`). That
meant local barycentric volume before and after every split, collapse and flip, and a term-by-term decomposition of
`MeshIntegrateJob`.

| Original, 4 Gyr | seeded 5k | seeded 20k | uniform 5k | uniform 20k |
|---|---|---|---|---|
| total crust | 8.11 → 8.11 | 8.03 → 7.66 (−5%) | 12.74 → 10.76 (−16%) | 12.75 → 8.52 (−33%) |
| continental (t > 26 km) | 5.31 → 5.26 | 5.21 → 4.44 (−15%) | 0 → 3.73 | 0 → 2.97 |

(10⁹ km³.) Losses grow with resolution, which is consistent with what you saw at 20k.

### Numerical losses (mesh accounting, not geology)
1. **Excluded triangles.** Area lost in triangles marked "boundary or subducting" changed no thickness, so the *overriding*
   vertex's third of every shrinking trench triangle was deleted at continental thickness. Subduction collapses later
   handed that area back to neighbours at *their* thickness. Over 4 Gyr (seeded 5k) the two flows were −7.9 and +9.3:
   two large, opposite, untracked flows whose difference depended on geometry. At 20k: −6.0 vs +7.3, net loss.
2. **Rift splits discarded continental material.** A rift split replaced the new vertex's share of the stretched
   continental area with ~15 km "transitional" crust. The volume difference simply vanished.
3. **Stale and inconsistent areas.** `Merge` weighted by `areaFull` from *before* the move. Every topology op
   recomputed thickness from areas that the op itself had changed. Flips, splits and collapses each moved volume
   (flip −1.2, split −3.8, collisions +0.6 over 4 Gyr) with no ledger.
4. **Clamps.** `min(t, maxThickness)` deleted material. The ocean floor fill (+1.9) created material silently.

### Geological imbalance (the rules themselves)
5. **Erosion deleted volume** above 35 km instead of moving it.
6. **No constructive process.** Compression concentrates material but never creates it. Nothing returned subducted
   material to the crust, so once thick crust was gone it could not reform.
7. **Thickness was the only identity.** Stretched continental crust below 21 km became mechanically oceanic and was
   subducted (§8 of the brief). This is a structural problem; see §3.4.

### Mechanical bugs that made it worse
8. **Convection drive direction ([R], confirmed [H]).** `ConvectionDriveJob` used `away = p − c·cosθ`, which is tangent at
   the *cell*, not at the vertex. Its tangential part is `cosθ ×` the correct direction, so the drive **reversed beyond
   90°**: hot cells attracted far-away crust and cold cells repelled it. The equally large radial part was never removed
   from `driven`, and `MeshCoupleJob` measured tension from `driven[j] − driven[i]`. Positive radial components under hot
   cells therefore read as tension everywhere, which explains the planet-wide damage saturation in uniform mode.
   Under cold cells it read as compression (tension → 0, the stagnation you saw).
9. **Local constraints cannot resist uniform stretching ([R], confirmed [H]).** In a uniformly stretching field every
   vertex's extension constraint pulls it equally toward both neighbours, so the corrections cancel. Only the outer
   ~N rings feel N passes. A 2-Gyr craton over an upwelling thinned 35 → 28 km in 100 Myr with no damage at all.

## 2. Material accounting (new)

**Volume is the state.** Each vertex stores crust volume. Its dual area is the **cotangent (circumcentric) Voronoi
cell** of its planar one-ring, and `thickness = volume / area` is derived. Unlike ⅓-of-triangle areas, Voronoi cells
are continuous across Delaunay flips, so flips change no material and barely change thickness.

Every operation conserves volume by construction:

| operation | rule |
|---|---|
| motion | volume travels with the vertex; area change *is* strain (thickening/thinning) |
| flip | every vertex keeps its volume |
| plain split | new vertex draws from each neighbour the area that neighbour lost, at its thickness; across a crust boundary it copies the denser side and the draws are rescaled |
| rift split | new vertex is new mantle crust (`riftCrustThicknessKm`, age 0, undamaged) → ledger; neighbours **keep** their volume, so rift shoulders keep continental material |
| merge collapse | removed vertex's volume goes to the vertices whose cells grow, in proportion to area gained |
| slab collapse | as a merge, but the share gained by the overriding side is consumed → ledger |
| erosion | symmetric flux between neighbours (exactly conservative) |

**Ledger** (`MeshLedger.cs`). Each source and sink is written where it happens:
`crust = initial + rift + floor + magmatic − subducted − delaminated − erosion recycled + residual`.

**Measured residual [H]:**
- Sources and sinks off (Test A, 1 Gyr): 1.5×10⁻⁷ of 8.0, mesh validates.
- Every full run up to 20 Gyr: |residual| < 10⁻⁶ of ~10 (float rounding).
- An independent check (thickness × barycentric area) agrees within 0.1–0.7%. That gap is the difference between the
  two area definitions, not lost material.

## 3. The material cycle (new)

All processes are local or per convection cell; there is no global coverage target.

1. **Slab consumption.** A vertex going under its neighbours loses the volume of the area it loses
   (`t × lost area × subduction share`) while keeping its thickness. Remaining slab volume is consumed when the vertex
   collapses.
2. **Arc magmatism** (budgeted, §7.1, the volume-based option). Each cell's reservoir is charged with
   `consumed × magmaticReturnFraction × (1 + magmaticYouthBoost × globalHeat)`. It releases with time constant
   `magmaReleaseMyr` onto the overriding margins of that cell, weighted by overriding convergence (two rows wide).
   Magmatism on an oceanic overriding plate builds island arcs, so new continental nuclei need no existing continents
   (§7.2). The fraction never drops below `magmaticReturnFraction`, so a mature planet keeps a material cycle (§7.3).
3. **Felsic relamination.** Of the differentiated material in consumed slabs, only `felsicSubductionFraction` sinks. The
   rest is moved (a conservative transfer, held "in transit" in the ledger) onto overriding margins of the same cell:
   accretion at convergent margins.
4. **Delamination.** Crust thicker than `maxThicknessKm` loses its excess root at `delaminationRatePerMyr`. This
   replaces the deleting clamp and is the main sink for over-thick felsic crust.
5. **Erosion** is conservative thickness diffusion of crust above `erosionBaseKm`. Sediment carries its source's
   composition. Sediment on ocean floor is later subducted, so it is recycled through the same felsic rules.
6. **Floor fill.** Crust thinner than `minimumCrustKm` is topped up from the mantle and ledgered.

### 3.4 Continental identity: a minimal composition property (fallback, justified)
Thickness/age/history alone were insufficient. Measured with a conserved tracer, the original continental material was
**almost entirely subducted within 2 Gyr**, even though buoyant crust was never the subducting side. It leaked into thin,
subductable vertices by stretching and was consumed there.

Under volume conservation thickness is a strain variable: a stretched margin and a thickened ocean floor are
indistinguishable by thickness. No rule keyed to thickness can tell them apart.

So each vertex carries one extra conserved quantity, its **felsic (differentiated) volume**. It moves with the volume in
every operation and is created only by arcs (and the initial crust). Mechanics use
`crust type = max(CrustFrac(thickness), saturate(0.5 × felsic column / felsicBuoyantColumnKm))`.
Continental crust stays buoyant down to a 15 km felsic column, while thin felsic veneers (sediment, young arcs) stay
subductable — that is the recycling path. Elevation remains thickness-based, so a thinned continent is a submerged shelf.

## 4. Mechanics (revised)

- **Convection drive** (`MeshDriveJob`) uses `away = p·cosθ − c`, is faded smoothly to zero before
  `min(4·radius, π − 0.2)`, and is projected into the tangent plane. The point simulation's `ConvectionDriveJob` still
  has the bug; it is left unchanged so the point sim's behaviour does not change silently.
- **Strength solve** (`MeshSolverJobs.cs`). Velocity minimizes `Σ A|v − d|² + Σ k_ij((v_j − v_i)·ê_ij)²` by
  Jacobi-preconditioned CG, warm-started (40 iterations default; converged iterations are no-ops).
  - Edge stretch along the chord is exactly zero for rigid rotations, so coherent motion is free. [H] verified: a rigid
    rotation is kept at 100%.
  - `k = Ā·(l/|e|)²` with `l = plateCoherence × strength × (1 − damage) × lerp(oceanicCoherenceFactor, 1, crust type)`.
    Active trenches get k = 0.
- **Stress** is the spring force `l²·ε/velocityReference`, opening plus compression × `compressiveStressWeight`. It
  relaxes once a fracture lets blocks separate, which is the §10.2 behaviour.
- **Rigid continental blocks** (`MeshBlockJob`). Even fully converged, the spring network let 13% of the drive's strain
  through intact crust at l = 2 rad, with cost growing as l/L. So each step, union-find joins intact buoyant vertices
  (damage < `rigidBlockDamageLimit`) across non-trench edges. Each block gets its best-fit rotation and members are
  blended toward it. Nothing persists between steps; blocks split as soon as damage cuts them. This is the minimum
  rigid-body element I could justify, and it applies to continents only.
- **Damage**:
  - growth `rate × brittleness × (σ − Y_eff)/Y × (nucleation + D) × (1 − D)`, where brittleness is the thermal
    strength, so hot crust flows rather than cracks;
  - weakening `Y_eff = Y(1 − weakening·D)`;
  - healing only below `damageHealThreshold × intact Y`, independent of damage;
  - yield `Y = yield × (0.25 + x) × (l_intact/plateCoherence)²`, the same stiffness scaling as stress, so thin oceanic
    lithosphere fails and cratons fail only under large drag.
  The `(nucleation + D)` term is what turned diffuse crumbling into branching rifts in Test F.
- **Convective vigour** (explicit global rule, mantle energy only). With mean-reverting base temperatures and balanced
  feedback every cell drifted to ≈0 and the planet stopped. Cell temperatures are now
  `vigor × (raw − mean)/rms`, with vigour = `convectionVigor + convectionYouthVigor × globalHeat`. Feedback decides
  *which* cells are hot; it never adds material.
- **Insulation** scales with crust thickness (up to 3×), so assembling a thick continent heats the cell beneath it.
- `ageResistance` default 1.5 → 0.5 (§9.1: rigidity now comes from strength, not from slowing old crust).
- Velocity averaging across tangent planes: damping of a rigid rotation is O(L²) per pass (~2×10⁻⁴ at 20k) [R].
  Negligible; `plateRigidityPasses` remains available.

## 5. Test results [H] (5k vertices unless stated; figures in `figures/`, data in `results/`)

| Test | Result |
|---|---|
| **A** numerical conservation | sources/sinks off, 1 Gyr: residual 1.5×10⁻⁷ of 8.0; validator OK |
| **B** isolated processes (2 Gyr) | each process measurable in the ledger. No magmatism: felsic 6.45 → 4.71, continental area 28 → 20%. No erosion: thicker crust (mean continental 51 km), felsic 6.09. No delamination: roots to 201 km, continental area 31% |
| **C** seeded, 4 Gyr | continental volume 5.3 → 7.4, area 29 → 33%; interior tracers 93–98% still continental; continental damage 0.01–0.1 (fig. 1, 5) |
| **D** uniform, 4 Gyr | thick crust emerges within 0.5 Gyr (thickened primordial crust), felsic material grows 0 → 3.3 and keeps differentiating (fig. 1, 6) |
| **E** 20 Gyr | both modes reach steady state: seeded continental volume ~7.5, felsic ~6.9 from 4 Gyr on; uniform felsic plateaus ~5.3; speed sustained ~0.6–1.4 × velocityReference; no lock-up, no runaway (fig. 2) |
| **F** fracture | 2 Gyr craton over a fixed maximal upwelling: rigid for ~250 Myr, then a crack nucleates at the centre, branches into rifts and splits it into separately moving blocks by 600 Myr (fig. 7) |
| **G** convection | cells migrate (~0.3 rad/Gyr) and reorganize; continental clustering varies 0.1–0.3 with landmasses merging and splitting; **no clear periodic supercontinent cycle observed** |
| **H** sensitivity | **significant, see below** |
| **I** seeds | 3 seeds × 2 modes at 4 Gyr: continental area 32–34% in all six, felsic within ±2% (fig. 4) |

## 6. Unresolved problems (honest list)
1. **Timestep sensitivity.** At 2 Gyr, mean speed is 1.59 / 1.29 / 0.51 × velRef for dt = 0.9 / 1.8 (auto) / 3.6 Myr,
   and activity follows. Cause: the speed cap `maxStepFraction × L / dt` binds, and longer steps lower it. Use the
   automatic step. The proper fix is sub-stepping instead of capping.
2. **Resolution sensitivity.** 20k moves ~2× faster than 5k at 1 Gyr. Material totals agree within ~10%. All long tests
   were at 5k; behaviour at 20k beyond 1 Gyr is untested.
3. **Oceanic damage is pervasive and speckled**, not confined to narrow ridges and transforms (maps 5–6).
4. **Rift interiors** still thin somewhat diffusely before opening as ocean (fig. 7).
5. **No clear supercontinent cycle** in 20 Gyr; continents disperse and re-aggregate irregularly.
6. Relamination and magmatism are distributed **per convection cell**: material can jump from one trench to an arc
   elsewhere in the same cell.
7. Performance in Unity is unknown. The solve adds ~40 × 5 small jobs per step. Harness wall time at 5k was ~35 s per
   Gyr (serial, unoptimised .NET); 20k was ~6.5 min per Gyr.

## 7. Risks for the first Unity run (not testable here)
- Burst: the new jobs use `FixedList512Bytes`, `NativeArray<double>`, `math.inverse(float3x3)`, and `in NativeArray`
  parameters in static helpers. All should be supported, but none was Burst-compiled.
- Job safety: `MeshSpringSetupJob` writes per-corner arrays with `[NativeDisableParallelForRestriction]`. Each corner
  belongs to exactly one vertex's ring, so writes never collide [R]. Everything else writes only index `i`.
- Enable `validateEveryBatch` for the first runs.

## 8. Tuning guide (symptom → parameters)
| Symptom | Adjust |
|---|---|
| continents slowly shrink | ↑ `magmaticReturnFraction` or `magmaticFelsicFraction`; ↓ `felsicSubductionFraction`; ↓ `erosionDiffusivityKm2PerMyr` |
| continents grow without limit | the reverse; ↑ `delaminationRatePerMyr` |
| crust too thick / tall plateaus | ↓ `maxThicknessKm` (delamination onset), ↑ `delaminationRatePerMyr`, ↑ `gravitySpreading` |
| planet sluggish or locking up | ↑ `convectionVigor`; ↓ `oceanicCoherenceFactor`; ↓ `extensionYield` |
| everything deforms / continents smear | ↑ `continentalRigidity`, ↑ `plateCoherenceRad` (raise `strengthSolveIterations` with it), ↑ `rigidBlockDamageLimit` |
| continents never break | ↓ `extensionYield`; ↑ `convectionVigor` |
| broad diffuse damage instead of faults | ↓ `damageNucleation`, ↑ `damageWeakening`, ↑ `extensionDamageRate` |
| damage never heals | ↑ `damageHealThreshold` or `extensionDamageHealing` |
| supercontinent never breaks up | ↑ `insulationHeat`, ↑ `convectionDrift` |
| young-planet chaos too long or short | `planetaryCoolingTimescaleMyr`, `convectionYouthVigor`, `magmaticYouthBoost` |
| ledger residual > 1e-5 | a code path changes volume without booking it: report it (should never happen) |

New inspector diagnostics: material ledger, felsic budget, mechanics, and trend graphs. Debug views appended:
**Stress, Extension, Magmatism, Felsic, Blocks**.
