# Headless harness (outside Unity)

Runs the simulation's real job structs serially under .NET 8, through `MeshRun.ScheduleSteps`: the exact chain the
component schedules. `stubs/UnityStubs.cs` replaces `Unity.Jobs`, `Unity.Collections`, `Unity.Burst` and a little of
`UnityEngine`. It is a **stubbed environment**: no Burst, no job-safety checks, no parallelism.

**Keep this folder outside `Assets`.**

## Build
1. Clone `https://github.com/Unity-Technologies/Unity.Mathematics` (source only).
2. Build:
   `dotnet build harness -c Release -o bin -p:UnityMathematicsSrc=<path to Unity.Mathematics> -p:TectonicsDir=<path to Tectonics folder>`
   (`nuget.config` disables package sources; nothing is downloaded.)

## Run
`bin/Harness seeded|uniform n=5000 T=4000 every=250 seed=1 out=run.csv [snap=prefix snapEvery=500] [validate]`

- **Toggles:** `nosinks`, `nosources`, `noerosion`, `nomagma`, `nospread`.
- **Settings:** any `MeshSettings` field by name, e.g. `extensionYield=0.2`.
- **Scenarios:**
  - `scenario=fracture`: one craton over a fixed hot cell (Test F).
  - `scenario=solvertest`: synthetic check of the strength solve.
  - `staged`: stage-by-stage volume decomposition.

**Output:** console table plus CSV.
- ledger and residual
- continental, buoyant and felsic volumes and areas
- damage, stress, speeds, convection temperatures
- tracer distortion (interior shape retention)
- clustering, localization, landmass count

`scripts/maps.py` renders snapshot CSVs; `scripts/plots.py` makes the figures in `docs/figures`.
`BaselineProgram.cs.txt` is the instrumented driver used to measure the *original* code (it needs the original files
plus the `Instr` hooks described in it).
