# Installing Revision 2

## Files

Copy `Tectonics/` over your tectonic simulation folder. Put `Tectonics/Editor/PlanetTectonicsMeshEditor.cs` wherever
your existing mesh editor script lives (it must be in an `Editor` folder).

| File | Status |
|---|---|
| `SphereMesh.cs` | **replaced**: MeshParams, state buffers (volume, felsic, solver, blocks), Voronoi dual area |
| `MeshStep.cs` | **replaced**: `MeshSettings` (all tunables + defaults), `BuildParams`, job builders, `ScheduleStep`, `InitCrust`, `MeshRun` |
| `MeshTectonicJobs.cs` | **replaced**: drive, velocity, constraints, move, area, integrate, erosion, sync, relax, budget/convection |
| `PlanetTectonicsMesh.cs` | **replaced**: component now drives `MeshRun` (same code as the harness) |
| `PlanetTectonicsMeshEditor.cs` | **replaced**: ledger, mechanics and trend graphs |
| `TectonicTypes.cs` | **modified (additive)**: `PlanetPoint.felsic` and `PlanetPoint.damage` added; nothing else changed |
| `MeshLedger.cs` | **new**: ledger layout + `MeshDiagnostics` (moved here from MeshStep.cs) |
| `MeshTopologyJob.cs` | **new file** (topology moved out of MeshTectonicJobs.cs; volume-conservative) |
| `MeshPreviewJobs.cs` | **new file** (preview jobs + `MeshDebugView` moved out of MeshTectonicJobs.cs) |
| `MeshSolverJobs.cs` | **new**: crust-strength CG solve |
| `MeshBlockJob.cs` | **new**: emergent rigid continental blocks |

**Keep unchanged** (not part of the replacement set):
- `TectonicJobs.cs`, `PlanetTectonics.cs`, `PlanetTectonicsEditor.cs`: the original point simulation.
  - The mesh simulation **no longer depends on `TectonicJobs.cs`**.
  - Delete those three files only if nothing else uses `PlanetTectonics`.
  - Note that `TectonicJobs.ConvectionDriveJob` still contains the drive-direction bug (see docs); it is deliberately
    left alone so the point sim's behaviour does not change silently.
- `TectonicPreview.shader`, `TectonicWire.shader`: unchanged, still required.

**No duplicate type definitions** as long as the five replaced files are overwritten (checked by type-check).

**Your existing headless test project will not compile against the new code.** These signatures changed:
- `MeshStep.InitCrust`, `MeshStep.Area`
- `MeshDiagnostics.Compute`, which gained a `timeMyr` argument
- `MeshTectonicJobs` types

Replace it with `HeadlessHarness/`, which must stay **outside** `Assets`: its stubs define `Unity.Collections`,
`Unity.Jobs` and `UnityEngine` types.

## Serialized fields of `PlanetTectonicsMesh`

**Existing components keep their old serialized values**, and several defaults changed meaning. Right-click the component
→ **Reset**, then re-enter any custom values (or create a fresh component).

- **Removed:**
  - `oceanicCompression`: oceanic convergence now consumes slab volume explicitly.
  - `riftCrustBlend`: replaced by `riftCrustThicknessKm` (rift shoulders keep their material).
  - `erosionPerMyr`: replaced by `erosionDiffusivityKm2PerMyr` (transport, not deletion).
- **Meaning changed:**
  - `extensionYield`: now in units of spring stress, default 0.15.
  - `extensionDamageRate`: default 0.2, multiplied by (nucleation + damage).
  - `maxThicknessKm`: now also the delamination onset (no clamp).
  - `plateCoupling` and `extensionStiffness`: only used with the strength solve off.
  - `ageResistance`: default 0.5, was 1.5.
- **Added:**
  - initial crust: `initialFelsicFraction`
  - strength solve: `enableStrengthSolve`, `plateCoherenceRad`, `oceanicCoherenceFactor`, `strengthSolveIterations`,
    `strengthSolveTolerance`, `compressiveStressWeight`
  - rigid blocks: `continentalRigidity`, `rigidBlockDamageLimit`, `rigidBlockMinVertices`
  - damage: `damageNucleation`, `damageWeakening`, `damageHealThreshold`
  - material budget: `enableSubductionSink`, `enableRifting`, `riftCrustThicknessKm`, `minimumCrustKm`,
    `delaminationRatePerMyr`, `erosionDiffusivityKm2PerMyr`, `erosionBaseKm`, `erosionRecycleFraction`
  - magmatism and felsic: `enableMagmatism`, `magmaticReturnFraction`, `magmaticYouthBoost`, `magmaReleaseMyr`,
    `magmaticFelsicFraction`, `felsicSubductionFraction`, `felsicBuoyantColumnKm`
  - convection: `insulationOnset`, `convectionVigor`, `convectionYouthVigor`
  - diagnostics: `historyIntervalMyr`
- `MeshDebugView` gained values at the **end** (Stress, Extension, Magmatism, Felsic, Blocks), so saved view selections
  are unaffected.

## Recommended defaults
The defaults *are* `MeshSettings.D` in `MeshStep.cs`; the component's field initializers read from it. Key values:

| Group | Values |
|---|---|
| strength | `plateCoherenceRad 1`, `oceanicCoherenceFactor 0.3`, `strengthSolveIterations 40`, `continentalRigidity 0.95`, `rigidBlockDamageLimit 0.3` |
| damage | `extensionYield 0.15`, `extensionDamageRate 0.2`, `damageNucleation 0.1`, `damageWeakening 0.8`, `damageHealThreshold 0.5`, `extensionDamageHealing 0.005`, `compressiveStressWeight 0.5` |
| budget | `magmaticReturnFraction 0.15`, `magmaticYouthBoost 1`, `magmaticFelsicFraction 0.5`, `felsicSubductionFraction 0.15`, `felsicBuoyantColumnKm 15`, `delaminationRatePerMyr 0.02`, `maxThicknessKm 75`, `erosionDiffusivityKm2PerMyr 50`, `minimumCrustKm 3.5` |
| convection | `convectionVigor 0.45`, `convectionYouthVigor 0.25`, `insulationHeat 0.012`, `surfaceCooling 0.008` |
| time | `stepMyr 0` (automatic; manual steps change behaviour, see docs §6) |

For a first look use `targetVertexCount` 5,000–20,000 and keep `validateEveryBatch` on for the first runs.
