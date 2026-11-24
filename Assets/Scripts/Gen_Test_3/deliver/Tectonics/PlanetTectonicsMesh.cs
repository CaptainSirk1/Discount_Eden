using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace PlanetGen.Tectonics.MeshSim
{
    /// <summary>
    /// Mesh-based tectonics: crust lives on the vertices of a closed triangle mesh that is rewritten
    /// (split / collapse / flip) as it moves, so holes are impossible and spacing has a hard cap.
    /// Crust VOLUME is the conserved state (thickness = volume / dual area); every change of the planet's total is a
    /// ledgered geological source or sink. See TECTONICS_REVISION_2.md.
    ///
    /// All stepping goes through <see cref="MeshRun"/>, the same code the headless harness runs. Inspector fields
    /// carry the names of <see cref="MeshSettings"/> and take their defaults from <see cref="MeshSettings.D"/>.
    /// </summary>
    [ExecuteAlways]
    public class PlanetTectonicsMesh : MonoBehaviour
    {
        static MeshSettings D => MeshSettings.D;

        [Header("Seed & Resolution")]
        public int seed = D.seed;
        [Tooltip("Approximate starting vertex count. Snapped to a geodesic sphere (10 n² + 2 vertices).")]
        [Range(500, 200000)] public int targetVertexCount = D.targetVertexCount;
        [Tooltip("Vertex slots reserved for growth, as a multiple of the starting count.")]
        [Range(1.2f, 4f)] public float capacityMultiplier = D.capacityMultiplier;
        [Tooltip("Random tangential offset of the starting vertices, x target edge length.")]
        [Range(0f, 0.4f)] public float meshJitter = D.meshJitter;
        [Range(3, 50)] public int convectionPointCount = D.convectionPointCount;
        [Range(1, 16)] public int convectionEvenness = D.convectionEvenness;
        public float planetRadiusKm = D.planetRadiusKm;

        [Header("Initial Crust")]
        [Tooltip("True: near-uniform primordial crust; continents must differentiate. False: noise-seeded continents (testing).")]
        public bool uniformInitialCrust = D.uniformInitialCrust;
        [Tooltip("Uniform mode: starting thickness, km.")]
        public float initialCrustThicknessKm = D.initialCrustThicknessKm;
        [Tooltip("Uniform mode: starting geological age, Myr.")]
        public float initialCrustAgeMyr = D.initialCrustAgeMyr;
        [Tooltip("Uniform mode: +- random thickness per vertex, km.")]
        [Range(0f, 5f)] public float initialThicknessVariationKm = D.initialThicknessVariationKm;
        [Tooltip("Uniform mode: differentiated (felsic) share of the primordial crust. 0 = mafic: all continental material must come from arcs.")]
        [Range(0f, 1f)] public float initialFelsicFraction = D.initialFelsicFraction;
        [Tooltip("Seeded mode only.")]
        [Range(0.05f, 0.7f)] public float continentCoverage = D.continentCoverage;
        [Tooltip("Seeded mode only.")]
        [Range(0.3f, 4f)] public float continentNoiseScale = D.continentNoiseScale;
        [Tooltip("Mature oceanic crust thickness, km (crust fraction 0).")]
        public float oceanThicknessKm = D.oceanThicknessKm;
        [Tooltip("Normal continental thickness, km (crust fraction 1).")]
        public float continentThicknessKm = D.continentThicknessKm;

        [Header("Planetary Cooling")]
        [Tooltip("Global thermal age G(t) = max * (1 - exp(-t / timescale)); thermal age = geological age + G.")]
        public bool enablePlanetaryCooling = D.enablePlanetaryCooling;
        public float planetaryCoolingTimescaleMyr = D.planetaryCoolingTimescaleMyr;
        public float maxGlobalThermalAgeMyr = D.maxGlobalThermalAgeMyr;
        [Tooltip("Thermal age at which crust is half as strong as it can get: strength = T / (T + ref).")]
        public float thermalStrengthRefMyr = D.thermalStrengthRefMyr;

        [Header("Time")]
        public float totalTimeMyr = D.totalTimeMyr;
        [Tooltip("0 = automatic: largest step that keeps vertices from moving more than maxStepFraction of an edge.")]
        public float stepMyr = D.stepMyr;
        [Range(0.05f, 0.5f)] public float maxStepFraction = D.maxStepFraction;

        [Header("Animation")]
        public float animMyrPerSecond = 100f;
        [Range(1, 64)] public int maxStepsPerFrame = 8;
        [Tooltip("Push inspector changes into the running simulation. Resolution, seed, step size and initial crust only take effect on restart.")]
        public bool liveTuning = true;
        public bool animateOnPlay = false;

        [Header("Preview Mesh")]
        public bool showPreviewMesh = true;
        public Material previewMaterial;
        public bool showWireframe = true;
        public Color wireColor = new Color(0f, 0f, 0f, 0.35f);
        public Material wireMaterial;

        [Header("Mesh Rules (x target edge length)")]
        [Range(1.2f, 2.5f)] public float maxEdgeLength = D.maxEdgeLength;
        [Range(0.2f, 0.8f)] public float minEdgeLength = D.minEdgeLength;
        [Range(0.3f, 0.79f)] public float compressionMergeLength = D.compressionMergeLength;
        [Range(0.05f, 0.5f)] public float sliverAltitude = D.sliverAltitude;
        [Tooltip("Tangential smoothing per step. Leave at 0 unless triangles look poor.")]
        [Range(0f, 0.3f)] public float relaxation = D.relaxation;
        [Range(8, 24)] public int maxValence = D.maxValence;

        [Header("Compression")]
        [Range(0.5f, 1.2f)] public float barrierStart = D.barrierStart;
        public float barrierStiffness = D.barrierStiffness;
        public float barrierAgeFactor = D.barrierAgeFactor;
        [Tooltip("Gravitational spreading: rad/Myr of outward flow per km of excess-thickness difference.")]
        public float gravitySpreading = D.gravitySpreading;
        [Range(0f, 1f)] public float gravityLock = D.gravityLock;
        public float gravityLockMarginKm = D.gravityLockMarginKm;
        [Tooltip("Gravity-lock thickness, and the thickness above which crustal roots delaminate.")]
        public float maxThicknessKm = D.maxThicknessKm;

        [Header("Crustal Strength (spring solve + rigid blocks)")]
        [Tooltip("Solve an edge-spring network for velocity each step, so strength acts across whole regions. Off: local constraint passes only (old behaviour).")]
        public bool enableStrengthSolve = D.enableStrengthSolve;
        [Tooltip("Rad. Length over which intact, cold continental crust resists strain in the spring network.")]
        public float plateCoherenceRad = D.plateCoherenceRad;
        [Tooltip("Coherence of oceanic lithosphere relative to continental (thin oceanic plates deform and break more easily).")]
        [Range(0.05f, 1f)] public float oceanicCoherenceFactor = D.oceanicCoherenceFactor;
        [Tooltip("CG iterations per step. Converged iterations are no-ops; raise for larger coherence or resolution.")]
        [Range(4, 200)] public int strengthSolveIterations = D.strengthSolveIterations;
        [Tooltip("Relative residual at which the solve stops.")]
        public float strengthSolveTolerance = D.strengthSolveTolerance;
        [Tooltip("Weight of compressive stress relative to tensile stress for damage (crust is stronger in compression).")]
        public float compressiveStressWeight = D.compressiveStressWeight;
        [Tooltip("0..1. How strongly intact continental crust is pulled to its block's rigid rotation.")]
        [Range(0f, 1f)] public float continentalRigidity = D.continentalRigidity;
        [Tooltip("Damage above which a vertex leaves its rigid block and deforms with the spring network.")]
        [Range(0.05f, 1f)] public float rigidBlockDamageLimit = D.rigidBlockDamageLimit;
        [Range(1, 200)] public int rigidBlockMinVertices = D.rigidBlockMinVertices;

        [Header("Extension Resistance & Damage")]
        public bool enableExtensionResistance = D.enableExtensionResistance;
        [Tooltip("Local constraint passes only (strength solve off): max fraction of opening removed per pass.")]
        [Range(0f, 1f)] public float extensionStiffness = D.extensionStiffness;
        [Tooltip("Strength exponent x = s * (ageFactor + thicknessFactor * crust type), s = thermal strength.")]
        public float extensionAgeFactor = D.extensionAgeFactor;
        public float extensionThicknessFactor = D.extensionThicknessFactor;
        [Tooltip("Residual resistance of fully damaged crust (0 = fractured crust separates freely).")]
        [Range(0f, 1f)] public float fracturedCrustResistance = D.fracturedCrustResistance;
        [Tooltip("Intact yield = yield * (0.25 + x) * (intact coherence / plateCoherence)^2, in units of the spring stress.")]
        public float extensionYield = D.extensionYield;
        [Tooltip("Damage growth per Myr per unit of relative over-stress.")]
        public float extensionDamageRate = D.extensionDamageRate;
        [Tooltip("Nucleation weight of intact crust: growth ~ (nucleation + D). Small = sharper, fewer faults.")]
        [Range(0.001f, 1f)] public float damageNucleation = D.damageNucleation;
        [Tooltip("How far full damage lowers the yield (localisation feedback).")]
        [Range(0f, 0.95f)] public float damageWeakening = D.damageWeakening;
        [Tooltip("Damage heals only while stress is below this fraction of the INTACT yield.")]
        [Range(0f, 1f)] public float damageHealThreshold = D.damageHealThreshold;
        [Tooltip("1/Myr exponential healing below the heal threshold.")]
        public float extensionDamageHealing = D.extensionDamageHealing;
        [Tooltip("Damage at which stretched buoyant edges split as new rift crust instead of being refined.")]
        [Range(0.05f, 1f)] public float riftDamageThreshold = D.riftDamageThreshold;

        [Header("Material Budget")]
        [Tooltip("Consumed slab volume leaves the crust. Off: conservative test mode.")]
        public bool enableSubductionSink = D.enableSubductionSink;
        [Tooltip("Rift / ridge splits create new crust. Off: every split is plain refinement.")]
        public bool enableRifting = D.enableRifting;
        public float riftCrustThicknessKm = D.riftCrustThicknessKm;
        [Tooltip("Thickness floor, km. Topping crust up to it is a ledgered mantle source. 0 = off.")]
        public float minimumCrustKm = D.minimumCrustKm;
        [Tooltip("1/Myr: share of a root thicker than maxThicknessKm that founders into the mantle per Myr.")]
        public float delaminationRatePerMyr = D.delaminationRatePerMyr;
        [Tooltip("Diffusivity, km²/Myr, of crust thicker than erosionBaseKm (conservative transport). 0 = off.")]
        public float erosionDiffusivityKm2PerMyr = D.erosionDiffusivityKm2PerMyr;
        public float erosionBaseKm = D.erosionBaseKm;
        [Tooltip("Share of sediment landing on oceanic crust that leaves the crust (subduction recycles the rest anyway).")]
        [Range(0f, 1f)] public float erosionRecycleFraction = D.erosionRecycleFraction;
        [Tooltip("Arc magmatism returns part of the subducted volume to the overriding margins of the same cell.")]
        public bool enableMagmatism = D.enableMagmatism;
        [Tooltip("Returned fraction of subducted volume on a mature planet.")]
        [Range(0f, 1f)] public float magmaticReturnFraction = D.magmaticReturnFraction;
        [Tooltip("Extra return fraction x global heat (1 young .. 0 fully cooled).")]
        public float magmaticYouthBoost = D.magmaticYouthBoost;
        [Tooltip("Myr. Release time of a cell's magma reservoir.")]
        public float magmaReleaseMyr = D.magmaReleaseMyr;
        [Tooltip("Differentiated (felsic) share of arc additions.")]
        [Range(0f, 1f)] public float magmaticFelsicFraction = D.magmaticFelsicFraction;
        [Tooltip("Share of the felsic part of consumed slabs that really sinks; the rest is relaminated onto the overriding margins.")]
        [Range(0f, 1f)] public float felsicSubductionFraction = D.felsicSubductionFraction;
        [Tooltip("Km of felsic column at which crust counts as buoyant (crust type 0.5) however thin it is. 0 = thickness only.")]
        public float felsicBuoyantColumnKm = D.felsicBuoyantColumnKm;

        [Header("Convection")]
        public float convectionStrength = D.convectionStrength;
        [Range(0.5f, 3f)] public float convectionRadiusScale = D.convectionRadiusScale;
        [Range(0.3f, 1.5f)] public float plumeThreshold = D.plumeThreshold;
        [Range(0.05f, 1f)] public float plumeRadius = D.plumeRadius;
        [Tooltip("Rad/Myr that cells migrate toward the trenches / continents that feed them.")]
        public float convectionDrift = D.convectionDrift;
        public float temperatureNoise = D.temperatureNoise;
        [Tooltip("Feedback heating per Myr at full insulation (scales with crust thickness).")]
        public float insulationHeat = D.insulationHeat;
        [Tooltip("Crust fraction at which crust starts to insulate.")]
        [Range(0f, 0.99f)] public float insulationOnset = D.insulationOnset;
        public float surfaceCooling = D.surfaceCooling;
        public float subductionCool = D.subductionCool;
        public float feedbackRelaxPerMyr = D.feedbackRelaxPerMyr;
        [Tooltip("RMS cell temperature of a fully cooled planet. The feedback decides which cells are hot; this decides how hard the mantle convects. 0 = feedback sets the scale (old behaviour).")]
        public float convectionVigor = D.convectionVigor;
        [Tooltip("Extra RMS temperature x global heat (young planets convect harder).")]
        public float convectionYouthVigor = D.convectionYouthVigor;

        [Header("Resistance & Coupling")]
        [Tooltip("Old crust divides the convective drive by up to (1 + this). Keep small: rigidity comes from the strength solve.")]
        public float ageResistance = D.ageResistance;
        public float ageSaturationMyr = D.ageSaturationMyr;
        public float velocityInertiaMyr = D.velocityInertiaMyr;
        [Tooltip("Neighbourhood coupling of the target velocity (strength solve off only).")]
        [Range(0f, 1f)] public float plateCoupling = D.plateCoupling;
        [Range(0, 16)] public int plateRigidityPasses = D.plateRigidityPasses;
        [Tooltip("Jacobi passes for the compression barrier and gravity lock (and extension, with the strength solve off).")]
        [Range(2, 16)] public int constraintPasses = D.constraintPasses;
        public float slabPull = D.slabPull;

        [Header("Subduction & Divergence")]
        public float subductThicknessDiffKm = D.subductThicknessDiffKm;
        public float subductDensityDiff = D.subductDensityDiff;
        [Tooltip("Opening speed (fraction of velocityReference) above which a split oceanic edge is a ridge.")]
        public float riftOpenThreshold = D.riftOpenThreshold;

        [Header("Terrain Accumulators")]
        [Range(0f, 1f)] public float overridingThickening = D.overridingThickening;
        public float orogenyGain = D.orogenyGain;
        public float orogenyDecayMyr = D.orogenyDecayMyr;
        public float arcGain = D.arcGain;
        public float arcDecayMyr = D.arcDecayMyr;

        [Header("Fault Classification")]
        public float velocityReference = D.velocityReference;
        public float faultThreshold = D.faultThreshold;
        public float faultMemoryMyr = D.faultMemoryMyr;
        public float transformDominance = D.transformDominance;

        [Header("Debug")]
        public MeshDebugView debugView = MeshDebugView.Elevation;
        public bool drawGizmos = true;
        public bool drawConvection = true;
        public bool drawFaultDirections = false;
        [Range(1, 50)] public int gizmoStride = 2;
        public bool hideBackside = true;
        [Range(1, 30)] public int gizmoSnapshotInterval = 4;
        [Tooltip("Run the full topology validator after every batch (slow). Logs the first problem and pauses.")]
        public bool validateEveryBatch = false;
        [Tooltip("Myr of flash in the TopologyEvents / Magmatism views.")]
        public float eventFlashMyr = D.eventFlashMyr;
        [Tooltip("Recompute the area-weighted diagnostics every N completed batches (they are not needed by the simulation).")]
        [Range(1, 100)] public int diagnosticsInterval = 10;
        [Tooltip("Simulated Myr between entries of the diagnostics history (trend graphs).")]
        public float historyIntervalMyr = 25f;

        /// <summary>The inspector fields as plain settings (what MeshRun and the harness consume).</summary>
        public MeshSettings Settings() => new MeshSettings
        {
            seed = seed, targetVertexCount = targetVertexCount, convectionPointCount = convectionPointCount, convectionEvenness = convectionEvenness,
            capacityMultiplier = capacityMultiplier, meshJitter = meshJitter, planetRadiusKm = planetRadiusKm,
            uniformInitialCrust = uniformInitialCrust, initialCrustThicknessKm = initialCrustThicknessKm, initialCrustAgeMyr = initialCrustAgeMyr,
            initialThicknessVariationKm = initialThicknessVariationKm, continentCoverage = continentCoverage, continentNoiseScale = continentNoiseScale,
            oceanThicknessKm = oceanThicknessKm, continentThicknessKm = continentThicknessKm,
            enablePlanetaryCooling = enablePlanetaryCooling, planetaryCoolingTimescaleMyr = planetaryCoolingTimescaleMyr,
            maxGlobalThermalAgeMyr = maxGlobalThermalAgeMyr, thermalStrengthRefMyr = thermalStrengthRefMyr,
            totalTimeMyr = totalTimeMyr, stepMyr = stepMyr, maxStepFraction = maxStepFraction,
            maxEdgeLength = maxEdgeLength, minEdgeLength = minEdgeLength, compressionMergeLength = compressionMergeLength,
            sliverAltitude = sliverAltitude, relaxation = relaxation, maxValence = maxValence,
            barrierStart = barrierStart, barrierStiffness = barrierStiffness, barrierAgeFactor = barrierAgeFactor, gravitySpreading = gravitySpreading,
            gravityLock = gravityLock, gravityLockMarginKm = gravityLockMarginKm, maxThicknessKm = maxThicknessKm,
            enableStrengthSolve = enableStrengthSolve, plateCoherenceRad = plateCoherenceRad, oceanicCoherenceFactor = oceanicCoherenceFactor,
            strengthSolveTolerance = strengthSolveTolerance, strengthSolveIterations = strengthSolveIterations,
            compressiveStressWeight = compressiveStressWeight, continentalRigidity = continentalRigidity,
            rigidBlockDamageLimit = rigidBlockDamageLimit, rigidBlockMinVertices = rigidBlockMinVertices,
            enableExtensionResistance = enableExtensionResistance, extensionStiffness = extensionStiffness, extensionAgeFactor = extensionAgeFactor,
            extensionThicknessFactor = extensionThicknessFactor, fracturedCrustResistance = fracturedCrustResistance,
            damageNucleation = damageNucleation, extensionYield = extensionYield, extensionDamageRate = extensionDamageRate,
            damageWeakening = damageWeakening, damageHealThreshold = damageHealThreshold, extensionDamageHealing = extensionDamageHealing,
            riftDamageThreshold = riftDamageThreshold,
            enableSubductionSink = enableSubductionSink, enableRifting = enableRifting, enableMagmatism = enableMagmatism,
            initialFelsicFraction = initialFelsicFraction, felsicBuoyantColumnKm = felsicBuoyantColumnKm, delaminationRatePerMyr = delaminationRatePerMyr,
            riftCrustThicknessKm = riftCrustThicknessKm, minimumCrustKm = minimumCrustKm, erosionDiffusivityKm2PerMyr = erosionDiffusivityKm2PerMyr,
            erosionBaseKm = erosionBaseKm, erosionRecycleFraction = erosionRecycleFraction,
            magmaticReturnFraction = magmaticReturnFraction, magmaticYouthBoost = magmaticYouthBoost, magmaReleaseMyr = magmaReleaseMyr,
            magmaticFelsicFraction = magmaticFelsicFraction, felsicSubductionFraction = felsicSubductionFraction,
            convectionStrength = convectionStrength, convectionRadiusScale = convectionRadiusScale, plumeThreshold = plumeThreshold,
            plumeRadius = plumeRadius, convectionDrift = convectionDrift, temperatureNoise = temperatureNoise,
            insulationHeat = insulationHeat, insulationOnset = insulationOnset, surfaceCooling = surfaceCooling, subductionCool = subductionCool,
            feedbackRelaxPerMyr = feedbackRelaxPerMyr, convectionVigor = convectionVigor, convectionYouthVigor = convectionYouthVigor,
            ageResistance = ageResistance, ageSaturationMyr = ageSaturationMyr, velocityInertiaMyr = velocityInertiaMyr,
            plateCoupling = plateCoupling, slabPull = slabPull, plateRigidityPasses = plateRigidityPasses, constraintPasses = constraintPasses,
            subductThicknessDiffKm = subductThicknessDiffKm, subductDensityDiff = subductDensityDiff, riftOpenThreshold = riftOpenThreshold,
            overridingThickening = overridingThickening, orogenyGain = orogenyGain, orogenyDecayMyr = orogenyDecayMyr,
            arcGain = arcGain, arcDecayMyr = arcDecayMyr,
            velocityReference = velocityReference, faultThreshold = faultThreshold, faultMemoryMyr = faultMemoryMyr,
            transformDominance = transformDominance, eventFlashMyr = eventFlashMyr,
        };

        // ---- results ----
        public PlanetPoint[] Points { get; private set; } = Array.Empty<PlanetPoint>();
        public Convection[] ConvectionPoints { get; private set; } = Array.Empty<Convection>();
        public float LastStepMyr => _run?.Dt ?? 0f;
        public float TargetEdge => _run?.L ?? 0f;

        // ---- status ----
        public bool HasSimulation => _run != null && _run.State != null;
        public bool IsAnimating => _playing;
        public int StepIndex => _run?.StepIndex ?? 0;
        public int TotalSteps => LastStepMyr > 0f ? Mathf.Max(1, Mathf.CeilToInt(totalTimeMyr / LastStepMyr)) : 0;
        public float CurrentTimeMyr => StepIndex * LastStepMyr;
        public float Progress => TotalSteps > 0 ? Mathf.Clamp01(StepIndex / (float)TotalSteps) : 0f;
        public bool IsFinished => HasSimulation && StepIndex >= TotalSteps;
        public float EffectiveMyrPerSecond { get; private set; }
        public int VertexCount { get; private set; }
        public int TriangleCount { get; private set; }
        public int VertexCapacity => _run?.State?.VCap ?? 0;
        public int TotalSplits { get; private set; }
        public int TotalCollapses { get; private set; }
        public int TotalFlips { get; private set; }
        public int TotalSubducted { get; private set; }
        public int TotalRifts { get; private set; }
        public int InvertedTriangles { get; private set; }
        public int CapacityHits { get; private set; }
        public float MinEdgeRatio { get; private set; }
        public float MaxEdgeRatio { get; private set; }
        public float SplitsPerMyr { get; private set; }
        public float CollapsesPerMyr { get; private set; }
        public float FlipsPerMyr { get; private set; }
        public string LastValidation { get; private set; } = "";
        /// <summary>Area-weighted diagnostics and the material ledger, refreshed every diagnosticsInterval batches.</summary>
        public MeshDiagnostics Diagnostics { get; private set; }
        /// <summary>Diagnostics sampled every historyIntervalMyr of simulated time, for trend graphs.</summary>
        public IReadOnlyList<MeshDiagnostics> History => _history;
        public int ContinentalSubducted { get; private set; }
        public float GlobalThermalAgeMyr => MeshStep.GlobalThermalAge(CurrentTimeMyr, enablePlanetaryCooling, planetaryCoolingTimescaleMyr, maxGlobalThermalAgeMyr);

        // ---- simulation state ----
        MeshRun _run;
        JobHandle _handle;
        bool _jobsInFlight, _previewPending, _playing, _finishPending, _previewDirty;
        int _batchesSinceSnapshot, _batchesSinceDiag;
        float _accum, _lastTick, _stepBudget = 1f;
        int _batchSteps, _rateSteps; float _rateTime;
        int _rateSplits0, _rateCollapses0, _rateFlips0, _rateStep0;
        float _lastHistoryMyr = -1e9f;
        readonly List<MeshDiagnostics> _history = new List<MeshDiagnostics>();
        System.Diagnostics.Stopwatch _runTimer;

        // ---- preview ----
        GameObject _previewGO; MeshRenderer _previewRenderer; Mesh _mesh;
        Material _autoMaterial, _autoWire;
        NativeArray<float3> _verts; NativeArray<Color32> _colors;
        NativeArray<int> _triIdx, _lineIdx;
        bool _linesUploaded;
        float _meshBoundsRadius;
        readonly Material[] _mats1 = new Material[1], _mats2 = new Material[2];

        PlanetPoint[] _live; int _liveCount;

        // ================================================================== lifecycle
        void OnEnable()
        {
            _lastTick = Time.realtimeSinceStartup;
#if UNITY_EDITOR
            UnityEditor.EditorApplication.update -= EditorTick;
            UnityEditor.EditorApplication.update += EditorTick;
#endif
            if (Application.isPlaying && animateOnPlay) StartAnimation();
        }

        void OnDisable()
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.update -= EditorTick;
#endif
            DisposeSimulation();
            DestroyPreview();
            SafeDestroy(_autoMaterial); _autoMaterial = null;
            SafeDestroy(_autoWire); _autoWire = null;
        }

        void OnValidate() { _previewDirty = true; }
        void Update() { if (Application.isPlaying) Tick(); }

#if UNITY_EDITOR
        void EditorTick() { if (!Application.isPlaying && this != null) Tick(); }
#endif

        // ================================================================== public controls
        [ContextMenu("Generate")]
        public void Generate() { BeginSimulation(); RunToEnd(); }

        [ContextMenu("Animate")]
        public void StartAnimation()
        {
            BeginSimulation();
            _playing = true; _accum = 0f; _lastTick = Time.realtimeSinceStartup;
        }

        public void Pause() { _playing = false; if (HasSimulation) PublishPoints(); }

        public void Resume()
        {
            if (!HasSimulation) { StartAnimation(); return; }
            if (IsFinished) return;
            _playing = true; _accum = 0f; _lastTick = Time.realtimeSinceStartup;
        }

        public void StepOnce()
        {
            if (!HasSimulation) BeginSimulation();
            _playing = false;
            CompleteJobs();
            if (liveTuning) RefreshTunables();
            _handle = SchedulePreview(_run.ScheduleSteps(1, default));
            _jobsInFlight = true;
            PublishPoints();
        }

        [ContextMenu("Finish Now")]
        public void FinishNow() { if (!HasSimulation) { Generate(); return; } RunToEnd(); }

        [ContextMenu("Clear")]
        public void Clear()
        {
            DisposeSimulation();
            DestroyPreview();
            Points = Array.Empty<PlanetPoint>();
            ConvectionPoints = Array.Empty<Convection>();
            _live = null; _liveCount = 0; VertexCount = 0; TriangleCount = 0;
            _history.Clear();
        }

        /// <summary>Runs the full topology validator now. Returns true if the mesh is valid.</summary>
        [ContextMenu("Validate Mesh")]
        public bool ValidateMesh()
        {
            if (!HasSimulation) { LastValidation = "No simulation"; return false; }
            CompleteJobs();
            string err = _run.State.Validate(0f, out int inverted);
            LastValidation = err == null
                ? $"OK at step {StepIndex}: {VertexCount} verts, {TriangleCount} tris, {inverted} inverted"
                : $"FAILED at step {StepIndex}:\n{err}";
            if (err == null) Debug.Log("[TectonicsMesh] " + LastValidation, this);
            else Debug.LogError("[TectonicsMesh] " + LastValidation, this);
            return err == null;
        }

        public void PublishPoints()
        {
            if (!HasSimulation) return;
            CompleteJobs();
            FillSnapshot();
            var arr = new PlanetPoint[_liveCount];
            Array.Copy(_live, arr, _liveCount);
            Points = arr;
        }

        /// <summary>Compact copy of the current mesh: unit-sphere vertices, CCW triangles, and per-vertex data
        /// in the same order as the vertices.</summary>
        public void ExportMesh(out Vector3[] vertices, out int[] triangles, out PlanetPoint[] points)
        {
            vertices = Array.Empty<Vector3>(); triangles = Array.Empty<int>(); points = Array.Empty<PlanetPoint>();
            if (!HasSimulation) return;
            CompleteJobs();
            var s = _run.State;
            var remap = new int[s.VCap];
            var vl = new List<Vector3>(VertexCount); var pl = new List<PlanetPoint>(VertexCount);
            for (int i = 0; i < s.VCap; i++)
            {
                remap[i] = -1;
                if (s.alive[i] == 0) continue;
                remap[i] = vl.Count;
                vl.Add(s.pos[i]);
                pl.Add(MakePoint(i));
            }
            var tl = new List<int>(TriangleCount * 3);
            for (int t = 0; t < s.TCap; t++)
            {
                if (s.triAlive[t] == 0) continue;
                tl.Add(remap[s.V[t * 3]]); tl.Add(remap[s.V[t * 3 + 1]]); tl.Add(remap[s.V[t * 3 + 2]]);
            }
            vertices = vl.ToArray(); triangles = tl.ToArray(); points = pl.ToArray();
        }

        // ================================================================== per-frame driver
        void Tick()
        {
            float now = Time.realtimeSinceStartup;
            float wall = Mathf.Clamp(now - _lastTick, 0f, 0.1f);
            _lastTick = now;
            if (!HasSimulation) return;

            if (_jobsInFlight && !_handle.IsCompleted)
            {
                _stepBudget = Mathf.Max(1f, _stepBudget * 0.7f);
                if (_playing) _rateTime += wall;
                return;
            }

            bool hadBatch = _jobsInFlight && _batchSteps > 0;
            bool uploaded = CompleteJobs();
            if (hadBatch)
            {
                _rateSteps += _batchSteps;
                _batchSteps = 0;
                _stepBudget = Mathf.Min(maxStepsPerFrame, _stepBudget + 0.25f);
            }
            if (_playing)
            {
                _rateTime += wall;
                if (_rateTime >= 0.5f)
                {
                    EffectiveMyrPerSecond = _rateSteps * LastStepMyr / _rateTime;
                    _rateSteps = 0; _rateTime = 0f;
                }
            }

            if (_finishPending)
            {
                _finishPending = false;
                PublishPoints();
                LogRun("finished");
            }

            SyncPreviewObject();

            if (_playing)
            {
                if (liveTuning) RefreshTunables();
                int remaining = TotalSteps - StepIndex;
                if (remaining <= 0) { _playing = false; _finishPending = true; }
                else
                {
                    int cap = Mathf.Clamp((int)_stepBudget, 1, maxStepsPerFrame);
                    _accum += wall * Mathf.Max(0f, animMyrPerSecond) / LastStepMyr;
                    int n = Mathf.Min((int)_accum, Mathf.Min(cap, remaining));
                    _accum = Mathf.Min(_accum - n, cap);
                    if (n > 0)
                    {
                        _handle = SchedulePreview(_run.ScheduleSteps(n, default));
                        _batchSteps = n;
                        _jobsInFlight = true;
                        JobHandle.ScheduleBatchedJobs();
                        if (StepIndex >= TotalSteps) { _playing = false; _finishPending = true; }
                    }
                }
            }
            if (!_jobsInFlight && _previewDirty)
            {
                if (liveTuning) RefreshTunables();
                _handle = SchedulePreview(default);
                _jobsInFlight = true;
                uploaded |= CompleteJobs();
            }

#if UNITY_EDITOR
            if (!Application.isPlaying && (uploaded || _playing)) UnityEditor.SceneView.RepaintAll();
#endif
        }

        // ================================================================== setup
        void BeginSimulation()
        {
            DisposeSimulation();
            _runTimer = System.Diagnostics.Stopwatch.StartNew();
            _run = new MeshRun();
            _run.Begin(Settings());

            _accum = 0f; _batchesSinceSnapshot = 0;
            _stepBudget = 1f; _rateSteps = 0; _rateTime = 0f; EffectiveMyrPerSecond = 0f;
            _rateSplits0 = _rateCollapses0 = _rateFlips0 = _rateStep0 = 0;
            SplitsPerMyr = CollapsesPerMyr = FlipsPerMyr = 0f;
            _batchSteps = 0; _playing = false; _finishPending = false;
            _history.Clear(); _lastHistoryMyr = -1e9f;
            ReadCounters();
            RefreshDiagnostics();
            _batchesSinceDiag = 0;
            ConvectionPoints = _run.State.conv.ToArray();
            _live = new PlanetPoint[_run.State.VCap];
            FillSnapshot();

            _previewDirty = true;
            SyncPreviewObject();
        }

        /// <summary>Pushes the inspector into the running simulation. Structural values (resolution, seed, step, cell
        /// count, initial crust) stay as they were at Begin: MeshRun keeps its own L, dt and cell count.</summary>
        void RefreshTunables()
        {
            if (!HasSimulation) return;
            var s = Settings();
            var old = _run.Settings;
            s.seed = old.seed; s.targetVertexCount = old.targetVertexCount; s.capacityMultiplier = old.capacityMultiplier;
            s.meshJitter = old.meshJitter; s.convectionPointCount = old.convectionPointCount; s.convectionEvenness = old.convectionEvenness;
            s.stepMyr = old.stepMyr;
            _run.Settings = s;
        }

        // ================================================================== job scheduling
        JobHandle SchedulePreview(JobHandle dependency)
        {
            _previewDirty = false;
            if (!HasSimulation || _mesh == null || !_verts.IsCreated) return dependency;
            var s = _run.State;
            _previewPending = true;
            var vj = new MeshPreviewVertexJob
            {
                M = _run.CurrentParams, view = (int)debugView, radius = planetRadiusKm,
                V = s.V, O = s.O, vc = s.vc, alive = s.alive, faultType = s.faultType, blockId = s.blockId, pos = s.pos, vel = s.vel,
                thickness = s.thickness, cfrac = s.cfrac, felsic = s.felsic, volume = s.volume, age = s.age,
                faultActivity = s.faultActivity, orogeny = s.orogeny, hotspot = s.hotspot,
                flashSplit = s.flashSplit, flashCollapse = s.flashCollapse, damage = s.damage, tension = s.tension,
                achieved = s.achieved, magma = s.magma,
                verts = _verts, colors = _colors
            }.Schedule(s.VCap + 1, 512, dependency);
            var ij = new MeshPreviewIndexJob
            {
                deadVertex = s.VCap, lines = showWireframe, V = s.V, O = s.O, triAlive = s.triAlive,
                triIndices = _triIdx, lineIndices = _lineIdx
            }.Schedule(s.TCap, 512, dependency);
            return JobHandle.CombineDependencies(vj, ij);
        }

        void RunToEnd()
        {
            CompleteJobs();
            _playing = false;
            if (liveTuning) RefreshTunables();
            // in chunks, so the diagnostics history gets filled on the way
            int chunk = Mathf.Max(1, Mathf.CeilToInt(Mathf.Max(historyIntervalMyr, LastStepMyr) / LastStepMyr));
            while (StepIndex < TotalSteps)
            {
                int n = Mathf.Min(chunk, TotalSteps - StepIndex);
                _run.ScheduleSteps(n, default).Complete();
                SampleHistory(true);
            }
            _handle = SchedulePreview(default);
            _jobsInFlight = true;
            JobHandle.ScheduleBatchedJobs();
            PublishPoints();
            _finishPending = false;
            LogRun("generated");
#if UNITY_EDITOR
            if (!Application.isPlaying) UnityEditor.SceneView.RepaintAll();
#endif
        }

        void ReadCounters()
        {
            var st = _run.State;
            var c = st.counters;
            VertexCount = c[MeshTopo.AliveV]; TriangleCount = c[MeshTopo.AliveT];
            TotalSplits = c[MeshTopo.Splits]; TotalCollapses = c[MeshTopo.Collapses]; TotalFlips = c[MeshTopo.Flips];
            TotalSubducted = c[MeshTopo.Subducted]; TotalRifts = c[MeshTopo.Rifts];
            InvertedTriangles = c[MeshTopo.Inverted]; CapacityHits = c[MeshTopo.CapHits];
            ContinentalSubducted = c[MeshTopo.ContinentalSubducted];
            MinEdgeRatio = st.stats[0]; MaxEdgeRatio = st.stats[1];

            int ds = StepIndex - _rateStep0;
            if (ds * LastStepMyr >= 20f)
            {
                float myr = ds * LastStepMyr;
                SplitsPerMyr = (TotalSplits - _rateSplits0) / myr;
                CollapsesPerMyr = (TotalCollapses - _rateCollapses0) / myr;
                FlipsPerMyr = (TotalFlips - _rateFlips0) / myr;
                _rateSplits0 = TotalSplits; _rateCollapses0 = TotalCollapses; _rateFlips0 = TotalFlips; _rateStep0 = StepIndex;
            }
        }

        void RefreshDiagnostics() { Diagnostics = _run.Diagnostics(); }

        void SampleHistory(bool force)
        {
            float interval = Mathf.Max(historyIntervalMyr, LastStepMyr);
            if (CurrentTimeMyr - _lastHistoryMyr < interval * (force ? 0.5f : 0.999f)) return;
            _lastHistoryMyr = CurrentTimeMyr;
            RefreshDiagnostics();
            _history.Add(Diagnostics);
        }

        bool CompleteJobs()
        {
            if (!_jobsInFlight) return false;
            _handle.Complete();
            _jobsInFlight = false;

            bool uploaded = false;
            if (_previewPending)
            {
                _previewPending = false;
                if (_mesh != null && _verts.IsCreated)
                {
                    const MeshUpdateFlags f = MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontValidateIndices;
                    _mesh.SetVertices(_verts, 0, _verts.Length, f);
                    _mesh.SetColors(_colors, 0, _colors.Length, f);
                    _mesh.SetIndices(_triIdx, MeshTopology.Triangles, 0, false);
                    if (showWireframe) { _mesh.SetIndices(_lineIdx, MeshTopology.Lines, 1, false); _linesUploaded = true; }
                    uploaded = true;
                }
            }

            if (HasSimulation)
            {
                ReadCounters();
                ConvectionPoints = _run.State.conv.ToArray();
                if (++_batchesSinceDiag >= diagnosticsInterval)
                {
                    _batchesSinceDiag = 0;
                    RefreshDiagnostics();
                }
                SampleHistory(false);
                if (drawGizmos && drawFaultDirections && ++_batchesSinceSnapshot >= gizmoSnapshotInterval)
                {
                    _batchesSinceSnapshot = 0;
                    FillSnapshot();
                }
                if (validateEveryBatch && StepIndex > 0 && !ValidateMesh()) _playing = false;
            }
            return uploaded;
        }

        void DisposeSimulation()
        {
            if (_jobsInFlight) { _handle.Complete(); _jobsInFlight = false; _previewPending = false; }
            _playing = false; _finishPending = false;
            _run?.Dispose();
            _run = null;
        }

        void LogRun(string verb)
        {
            long ms = _runTimer != null ? _runTimer.ElapsedMilliseconds : 0;
            var d = Diagnostics;
            Debug.Log($"[TectonicsMesh] {verb}: {StepIndex} steps @ {LastStepMyr:F2} Myr ({CurrentTimeMyr:F0} Myr), " +
                      $"{VertexCount} verts / {TriangleCount} tris, {TotalSplits} splits ({TotalRifts} rifts), " +
                      $"{TotalCollapses} collapses ({TotalSubducted} subducted), {TotalFlips} flips, " +
                      $"edges {MinEdgeRatio:F2}-{MaxEdgeRatio:F2} x target, {InvertedTriangles} inverted. " +
                      $"Crust {d.crustVolume / 1e9f:F3} (residual {d.residual / 1e9f:E2}) 10⁹ km³, continents {d.continentalAreaFrac * 100f:F1}%, " +
                      $"felsic {d.felsicVolume / 1e9f:F3}. {ms} ms", this);
        }

        // ================================================================== snapshot
        PlanetPoint MakePoint(int i)
        {
            var s = _run.State;
            float t = s.thickness[i];
            return new PlanetPoint
            {
                position = s.pos[i], velocity = s.vel[i],
                thickness = t, density = s.density[i], age = s.age[i], stiffness = s.stiffness[i],
                faultType = (FaultType)s.faultType[i], faultActivity = s.faultActivity[i], faultDir = s.faultDir[i],
                orogeny = s.orogeny[i], extension = s.extension[i], arc = s.arc[i],
                shear = s.shear[i], hotspot = s.hotspot[i], crowding = 1f,
                elevationKm = TMath.ElevationKm(t, s.age[i], _run.CurrentParams.P),
                felsic = s.volume[i] > 0f ? s.felsic[i] / s.volume[i] : 0f,
                damage = s.damage[i]
            };
        }

        void FillSnapshot()
        {
            if (!HasSimulation) return;
            var s = _run.State;
            if (_live == null || _live.Length != s.VCap) _live = new PlanetPoint[s.VCap];
            int n = 0;
            for (int i = 0; i < s.VCap; i++) if (s.alive[i] == 1) _live[n++] = MakePoint(i);
            _liveCount = n;
        }

        public List<FaultSample> ExtractFaultSamples(float minActivity = 0.12f)
        {
            var result = new List<FaultSample>();
            foreach (var p in Points)
            {
                if (p.faultType == FaultType.None || p.faultActivity < minActivity) continue;
                result.Add(new FaultSample { position = p.position, tangent = p.faultDir, type = p.faultType, activity = p.faultActivity, thickness = p.thickness });
            }
            return result;
        }

        // ================================================================== preview mesh
        void SyncPreviewObject()
        {
            bool want = showPreviewMesh && HasSimulation;
            if (!want) { if (_previewGO != null) DestroyPreview(); return; }
            var s = _run.State;

            if (_previewGO == null || _mesh == null || !_verts.IsCreated || _verts.Length != s.VCap + 1 || _triIdx.Length != s.TCap * 3)
            {
                BuildPreview(s.VCap, s.TCap);
                _previewDirty = true;
            }

            var face = previewMaterial != null ? previewMaterial : AutoMaterial();
            if (showWireframe)
            {
                var wire = wireMaterial != null ? wireMaterial : AutoWire();
                if (wireMaterial == null && wire.HasProperty("_Color") && wire.color != wireColor) wire.color = wireColor;
                if (_mats2[0] != face || _mats2[1] != wire || _previewRenderer.sharedMaterials.Length != 2)
                {
                    _mats2[0] = face; _mats2[1] = wire;
                    _previewRenderer.sharedMaterials = _mats2;
                }
                if (!_linesUploaded) _previewDirty = true;
            }
            else
            {
                if (_mats1[0] != face || _previewRenderer.sharedMaterials.Length != 1)
                {
                    _mats1[0] = face;
                    _previewRenderer.sharedMaterials = _mats1;
                }
                _linesUploaded = false;
            }

            if (!Mathf.Approximately(_meshBoundsRadius, planetRadiusKm))
            {
                _meshBoundsRadius = planetRadiusKm;
                _mesh.bounds = new Bounds(Vector3.zero, Vector3.one * (planetRadiusKm * 2.2f));
                _previewDirty = true;
            }
        }

        void BuildPreview(int vCap, int tCap)
        {
            DestroyPreview();
            _verts = new NativeArray<float3>(vCap + 1, Allocator.Persistent);
            _colors = new NativeArray<Color32>(vCap + 1, Allocator.Persistent);
            _triIdx = new NativeArray<int>(tCap * 3, Allocator.Persistent);
            _lineIdx = new NativeArray<int>(tCap * 6, Allocator.Persistent);
            for (int i = 0; i < _triIdx.Length; i++) _triIdx[i] = vCap;
            for (int i = 0; i < _lineIdx.Length; i++) _lineIdx[i] = vCap;

            _mesh = new Mesh { name = "Tectonics Mesh Preview", indexFormat = IndexFormat.UInt32, hideFlags = HideFlags.HideAndDontSave };
            _mesh.MarkDynamic();
            _mesh.SetVertices(_verts);
            _mesh.SetColors(_colors);
            _mesh.subMeshCount = 2;
            _mesh.SetIndices(_triIdx, MeshTopology.Triangles, 0, false);
            _mesh.SetIndices(_lineIdx, MeshTopology.Lines, 1, false);
            _linesUploaded = false;
            _meshBoundsRadius = planetRadiusKm;
            _mesh.bounds = new Bounds(Vector3.zero, Vector3.one * (planetRadiusKm * 2.2f));

            _previewGO = new GameObject("Tectonics Mesh Preview") { hideFlags = HideFlags.HideAndDontSave, layer = gameObject.layer };
            _previewGO.transform.SetParent(transform, false);
            _previewGO.AddComponent<MeshFilter>().sharedMesh = _mesh;
            _previewRenderer = _previewGO.AddComponent<MeshRenderer>();
            _previewRenderer.shadowCastingMode = ShadowCastingMode.Off;
            _previewRenderer.receiveShadows = false;
            _mats1[0] = null; _mats2[0] = null; _mats2[1] = null;
        }

        void DestroyPreview()
        {
            if (_jobsInFlight) { _handle.Complete(); _jobsInFlight = false; }
            _previewPending = false;
            SafeDestroy(_previewGO); _previewGO = null; _previewRenderer = null;
            SafeDestroy(_mesh); _mesh = null;
            if (_verts.IsCreated) _verts.Dispose();
            if (_colors.IsCreated) _colors.Dispose();
            if (_triIdx.IsCreated) _triIdx.Dispose();
            if (_lineIdx.IsCreated) _lineIdx.Dispose();
        }

        Material AutoMaterial()
        {
            if (_autoMaterial != null) return _autoMaterial;
            var sh = Shader.Find("PlanetGen/TectonicPreview");
            if (sh == null) sh = Shader.Find("Sprites/Default");
            _autoMaterial = new Material(sh) { name = "Tectonics Mesh Preview (auto)", hideFlags = HideFlags.HideAndDontSave };
            return _autoMaterial;
        }

        Material AutoWire()
        {
            if (_autoWire != null) return _autoWire;
            var sh = Shader.Find("PlanetGen/TectonicWire");
            if (sh == null) sh = Shader.Find("Hidden/Internal-Colored");
            _autoWire = new Material(sh) { name = "Tectonics Wire (auto)", hideFlags = HideFlags.HideAndDontSave, color = wireColor };
            return _autoWire;
        }

        static void SafeDestroy(UnityEngine.Object o)
        {
            if (o == null) return;
            if (Application.isPlaying) Destroy(o); else DestroyImmediate(o);
        }

        // ================================================================== gizmos
        static readonly Color[] FaultColors =
        {
            new Color(0.4f, 0.4f, 0.4f), Color.red, new Color(1f, 0.55f, 0f), Color.magenta, Color.cyan
        };

        void OnDrawGizmos()
        {
            if (!drawGizmos) return;
            Gizmos.matrix = transform.localToWorldMatrix;
            float r = planetRadiusKm;

            if (drawFaultDirections)
            {
                PlanetPoint[] pts = _liveCount > 0 && _live != null ? _live : Points;
                int count = pts == _live ? _liveCount : (Points?.Length ?? 0);
                Camera cam = Camera.current;
                for (int i = 0; i < count; i += gizmoStride)
                {
                    var p = pts[i];
                    if (p.faultType == FaultType.None || p.faultActivity <= faultThreshold) continue;
                    if (hideBackside && cam != null)
                    {
                        Vector3 wp = transform.TransformPoint(p.position * r);
                        Vector3 nrm = transform.TransformDirection(p.position);
                        Vector3 toCam = cam.orthographic ? -cam.transform.forward : cam.transform.position - wp;
                        if (Vector3.Dot(nrm, toCam) <= 0f) continue;
                    }
                    Gizmos.color = FaultColors[(int)p.faultType];
                    Gizmos.DrawRay(p.position * (r * 1.002f), p.faultDir * (TargetEdge * r * 0.8f));
                }
            }

            if (drawConvection && ConvectionPoints != null)
            {
                foreach (var c in ConvectionPoints)
                {
                    Gizmos.color = c.temp > 0 ? Color.red : Color.blue;
                    Gizmos.DrawWireSphere((Vector3)c.pos * r * 1.02f, r * 0.03f * Mathf.Max(0.3f, Mathf.Abs(c.temp)));
                }
            }
        }
    }
}
