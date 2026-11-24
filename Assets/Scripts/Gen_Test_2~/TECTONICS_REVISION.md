# Discount Eden — Tectonics Revision: Algorithms, Parameters, Results

Files: `SphereMesh.cs`, `MeshStep.cs` (new), `MeshTectonicJobs.cs`, `PlanetTectonicsMesh.cs`,
`Editor/PlanetTectonicsMeshEditor.cs`. `TectonicTypes.cs` / `TectonicJobs.cs` are unchanged (the point sim is unaffected).

## Step pipeline

```
ConvectionDrive          mantle drive per vertex (unchanged, from the point sim)
MeshVelocity             TARGET velocity = lerp(drive, stiffness-weighted neighbours, couple*stiffness)
                         + gravitational spreading (stored separately); boundary rates
MeshCouple x N (>= 2)    pass 0 measures TENSION; every pass applies compression + extension limits
                         to the target field, relative to the neighbours' targets
MeshArea                 dual areas + "one-crust" triangle mask, using the final targets
MeshMove                 vel = lerp(vel, target + spreading, 1 - exp(-dt/velTau)); advect
MeshIntegrate            volume-conserving thickness, age, DAMAGE, faults, stiffness
MeshTopology             repair / flip / collisions / collapse + merge / split (rift or ordinary)
MeshFeedback             convection: insulation, surface cooling, slab cooling (area-weighted)
```

`MeshStep` builds every job from the state. The component schedules them; the headless test harness runs the same
structs serially, so the tests exercise the real pipeline.

**Why limits act on the target, not the velocity.** Earlier the limits removed a fraction of relative velocity *per step*,
so the same setting meant different stiffness at different timesteps (measured: plate speed 0.14 / 0.24 / 0.36 cm/yr at
dt 2.36 / 1.18 / 0.59). The target is timestep-independent, and inertia is applied afterwards, so "fraction of relative
motion removed" means the same thing at any dt.

---

## Stage 1 — Uniform initial crust and global cooling

**Initial crust** (`MeshStep.InitCrust`). `uniformInitialCrust`: every vertex gets `initialCrustThicknessKm ±
initialThicknessVariationKm` (per-vertex white noise, no large-scale pattern) and `initialCrustAgeMyr`. Off: the old
noise-seeded continents, kept for testing.

At 25 km, crust fraction cf = (25 − 7)/(35 − 7) = 0.64, so primordial crust is *buoyant* (cf ≥ 0.5) and cannot subduct
under other buoyant crust (`MeshLimits.Subducts`). Convergence therefore thickens it, and only crust thinned below ~21 km
by divergence becomes subductable. That is what lets uniform crust differentiate instead of being consumed.

**Global cooling.**
```
G(t)          = maxGlobalThermalAgeMyr * (1 - exp(-t / planetaryCoolingTimescaleMyr))   t = absolute sim time
thermalAge    = age + G(t)          (never stored; geological age is untouched and is what gets exported)
strength s    = thermalAge / (thermalAge + thermalStrengthRefMyr)                        0 = molten, -> 1 cold
```
G is evaluated per step from absolute time, so batch size and `totalTimeMyr` never change it.
`T/(T+ref)` has no hard saturation: a 2.5 Gyr craton (s ≈ 0.88 late) stays stronger than fresh sea floor (s ≈ 0.70 late).

Thermal strength `s` is used only for: the Thermal view, `MeshLimits.Stiffness`, the compression barrier, extension
resistance and damage yield. Density and drive resistance still use geological age, as before.

| Parameter | Unit | Default | Effect |
|---|---|---|---|
| uniformInitialCrust | bool | true | off = seeded continents |
| initialCrustThicknessKm | km | 25 | below ~21 km the start is subductable everywhere |
| initialCrustAgeMyr | Myr | 0 | older = stronger from the start |
| initialThicknessVariationKm | km | 0.5 | seeds where differentiation starts; larger = faster, patchier |
| enablePlanetaryCooling | bool | true | off: thermal age = geological age |
| planetaryCoolingTimescaleMyr | Myr | 1500 | larger = planet stays hot (weak, mobile) longer |
| maxGlobalThermalAgeMyr | Myr | 1000 | final "coolness" of brand-new crust; at 1000 new crust ends ~0.7 strength (grey) |
| thermalStrengthRefMyr | Myr | 400 | larger = everything weaker / hotter-looking at the same age |

**Thermal view** (`MeshDebugView.Thermal`): heat = 1 − s, mapped yellow-orange → magma → dark red-orange → brown-grey →
charcoal (darkest ≈ 20% grey, so it stays visible). The Age view is unchanged.

## Stage 2 — Extension resistance and damage

**Strength exponent** (`MeshLimits.StrengthExponent`): `x = s * (extensionAgeFactor + extensionThicknessFactor * cf)`.
0 for molten crust, ~3.5 for a cold craton with defaults.

**Extension limit** (in `MeshCoupleJob`, for every pair that isn't a crust boundary):
```
intact S  = extensionStiffness * (1 - exp(-x_pair))
limit f   = S * (1 - D) + min(fracturedCrustResistance, S) * D        D = max(damage_i, damage_j)
v_i      -= dir * openingRate * f * share_i                            share = stiffness_j / (s_i + s_j)
```
Only relative opening between neighbours is removed, so a continent moving as a whole is not slowed.

**Tension** (pass 0 of `MeshCoupleJob`): the largest *mantle-drive* opening strain rate toward any same-crust
neighbour, `dot(drive_j - drive_i, dir) / distance / velocityReference`. It is measured from the raw drive, before
coupling or any limit. So a strong barrier cannot hide the stress that will eventually break it. (An earlier version
measured the coupled target; locked plates then measured *less* tension and healed: wrong-way feedback.)

**Damage** (`MeshLimits.UpdateDamage`, in `MeshIntegrateJob`):
```
yieldEff = extensionYield * (0.25 + x * (1 - D))
tension > yieldEff :  D += extensionDamageRate * (tension - yieldEff) * (1 - D) * dt
otherwise          :  D *= exp(-extensionDamageHealing * dt)
```
Damage is bounded in [0, 1] by the `(1 - D)` factor. Hot or thin crust breaks easily; cold cratons need ~4× the tension.
Damage lowers the yield, which is the positive feedback meant to concentrate strain. It also weakens coupling stiffness:
`stiffness *= 1 - 0.5 * max(faultActivity, D)`.

Damage is per vertex, an approximation of directional fracture (no per-edge attributes were added).

| Parameter | Unit | Default | Effect |
|---|---|---|---|
| enableExtensionResistance | bool | true | off: no resistance, no damage, old continental rift rule |
| extensionStiffness | 0..1 | 0.9 | max fraction of relative opening removed by intact crust |
| extensionAgeFactor | – | 1.5 | strength from cooling |
| extensionThicknessFactor | – | 2.5 | extra strength for continental crust |
| extensionYield | tension | 0.2 | **main knob for fracture.** Lower = more fracturing |
| extensionDamageRate | 1/(Myr·tension) | 0.02 | how fast stressed crust breaks (~100-250 Myr) |
| extensionDamageHealing | 1/Myr | 0.003 | ~330 Myr healing time when unstressed |
| fracturedCrustResistance | 0..1 | 0.05 | resistance of fully broken crust |
| riftDamageThreshold | 0..1 | 0.5 | damage before continental splits make new crust (Stage 3) |
| constraintPasses | count | 6 | Jacobi passes for both limits (even). More = harder limits; each is one parallel ring pass |

**Compression limit** (unchanged in form, now in the same passes): exponential barrier
`1 - exp(-barrierStiffness * lerp(0.25, 1 + barrierAgeFactor, s) * x)` plus the gravity lock. Hot crust keeps a quarter of
the stiffness, so primordial crust thickens instead of locking. **Compression merge:** a converging buoyant edge
merges below `lerp(compressionMergeLength, minEdgeLength, s)` × target. Hot crust thickens by merging; cold crust is held by the barrier.

## Stage 3 — Rifting and topology

A split is a **rift split** (new crust) when the edge is opening and either:
- the crust is oceanic (cf < 0.5) and it opens faster than `riftOpenThreshold`, or an endpoint is an active divergent fault (ridges, unchanged); or
- the crust is continental and `max(damage) >= riftDamageThreshold` (fractured).

Intact continental crust that merely stretches is *refined* (interpolated), never turned into ocean floor.

| Operation | Damage | Other properties |
|---|---|---|
| Ordinary split | interpolated | interpolated |
| Boundary split (edge crosses crust types) | copied from denser side | copied from denser side |
| Rift split | 0.5 × parents' max (hot, still weak) | age 0, thin (`riftCrustBlend` rule); thermal state = G(t) automatically |
| Merge (collapse, same crust) | max of the two | **area-weighted** average (exact volume conservation) |
| Flip | untouched (vertex data) | untouched |
| Subduction | removed with the consumed vertex | its area goes to the slab behind it |

**Volume accounting fix found while testing:** at margin vertices, thickening now uses only the in-crust triangles' area
change, spread over the full dual area: `t *= 1 + (A_in_old - A_in_new) / A_full_new`. The old rule applied the in-crust
ratio to the whole vertex and created crust at every continental margin under compression.

## Stage 4 — Convection feedback

```
cov      = continental AREA fraction of the cell (thickness > 0.75 * continentThicknessKm)
cooling  = surfaceCooling * (1 - cov) * saturate((temp + 1) / 2)        hot cells lose heat fastest
feedback += dt * (insulationHeat * cov - cooling - subductionCool * consumedAreaFraction/dt - feedbackRelax * feedback)
baseTemp  = baseTemp * exp(-feedbackRelax * dt) + noise                  mean-reverting (was a persistent random walk)
temp      = clamp(baseTemp + feedback, -1.5, 1.5)
```
`(temp + 1) / 2` makes exposure cool warm cells strongly and cold cells barely, so exposed regions don't run away to
strongly negative. Making baseTemp mean-reverting means the initial random temperatures fade over ~200 Myr. The typical
size of the noise part is about tempNoise / sqrt(2 × feedbackRelax) ≈ 0.05, so long-term behaviour comes from feedback.

| Parameter | Default | Effect |
|---|---|---|
| insulationHeat | 0.012 /Myr | continental insulation warms cells (breakup driver) |
| surfaceCooling | 0.008 /Myr | exposed cells cool (assembly driver) |
| feedbackRelaxPerMyr | 0.005 /Myr | memory of the feedback and of baseTemp |
| temperatureNoise | 0.005 /sqrt(Myr) | randomness; keep well below the feedback terms |
| convectionDrift | 0.0002 rad/Myr | cells migrate toward trenches / continents |
| subductionCool | 0.5 | slabs cool cells |

## Stage 5 — Diagnostics

`MeshDiagnostics.Compute` (every `diagnosticsInterval` batches, main thread, O(V)) reports, all **area-weighted**: global
thermal age, continental surface %, total and continental crust volume (10⁹ km³), mean / max damage, continental mean
damage, % of continental area fractured, mean tension. The inspector also shows continental vertices subducted and the
existing topology counters.

**Damage view:** dimmed elevation, grey → yellow → red with damage, and a blue tint where tension exceeds yield.

---

## Test results (headless runtime tests, 3k–20k vertices, seed 12345)

Ran the real job structs serially in .NET against Unity.Mathematics with stand-ins for Burst/Collections.
**Not run in Unity**: the component, inspector, shader and rendering are checked by inspection only.

| Test | Result | Evidence |
|---|---|---|
| A differentiation | **Pass** | uniform 25 km → thickness p10/p90 18.6/35.4 km by 250 Myr; 4 Gyr run ends bimodal (7 km basins, 35-56 km highs) |
| B continental stability | **Pass** (seeded) | continents keep recognizable shapes for 3 Gyr while moving/rotating; continental area 28.7 → 30.3%; crust volume 8.11 → 8.17 (+0.7%) |
| C continental fracture | **Not achieved** | at default yield continents stay intact (<1% fractured). At yield 0.08, ~5% of continental area is damaged, in bands, but no clean rift splitting a continent was observed in 2 Gyr. Damage localizes mostly in oceans, as broad patches more than narrow lines |
| D convection feedback | **Pass** | cell temperature tracks continental cover; 35-42 sign reversals over 4 Gyr (14 before mean-reverting baseTemp) |
| E cooling | **Pass** | fresh crust (geological age < 5 Myr) heat 0.45 at 1 Gyr → 0.35 at 2 Gyr → 0.32 at 3 Gyr; at t = 0 heat = 1 by construction |
| F stability | **Mostly pass** | topology valid at every check, 0 inverted triangles, edge cap held (max 1.60), damage bounded. Timestep (dt 2.36/1.18/0.59): continental area 22.4/22.3/21.8%, volume identical to ±0.1%, damage ±7%; **plate speed still ±25%** |

**Performance.** 20k vertices: 68 ms/step serial single-threaded .NET (not Burst, not parallel); about 80% of that is the
parallel jobs, which split across cores in Unity. The 6 constraint passes are the largest new cost. If speed matters,
`constraintPasses = 2` gave almost identical results in the seeded tests.

## Known problems (honest list)

1. **Continental rifting (Test C) isn't emerging yet.** The tension measure is local mantle strain rate, which is smooth
   at the scale of a convection cell. Once a weak zone opens, it doesn't relieve stress elsewhere. A rift needs stress to
   concentrate at crack tips or transfer across a plate, which a purely local per-vertex measure can't do. The most
   promising next step is letting damage spread to neighbours along the tension axis (crack-tip propagation).
2. **Uniform start becomes nearly stagnant later** (speed ~0.03-0.06 cm/yr after 2 Gyr, damage near 1 everywhere).
   Primordial crust ages uniformly and drive resistance uses geological age.
3. **Uniform-start volume drifts about −5% per Gyr.** Rift splits through thick primordial crust take area from
   neighbours that don't re-thicken. A re-thickening fix was tried and removed: it ratcheted continents up to the
   75 km cap (measured: +56% volume in 3 Gyr).
4. **Plates are slow** at 3-5k vertices (0.1-0.4 cm/yr). Raise `convectionStrength` or the resolution; at 20k vertices
   speeds are ~0.7 cm/yr.
5. **Gravitational spreading is now ~10× weaker than before at the same `gravitySpreading` value.** The old code added it
   every step, which was a timestep-dependent bug. Raising it to 0.0004 to compensate made continents disappear in testing,
   so the default stays at 0.00004.
