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
    /// </summary>
    [ExecuteAlways]
    public class PlanetTectonicsMesh : MonoBehaviour
    {
        [Header("Seed & Resolution")]
        public int seed = 12345;
        [Tooltip("Approximate starting vertex count. Snapped to a geodesic sphere (10 n² + 2 vertices).")]
        [Range(500, 200000)] public int targetVertexCount = 20000;
        [Tooltip("Vertex slots reserved for growth, as a multiple of the starting count.")]
        [Range(1.2f, 4f)] public float capacityMultiplier = 2f;
        [Tooltip("Random tangential offset of the starting vertices, x target edge length.")]
        [Range(0f, 0.4f)] public float meshJitter = 0.15f;
        [Range(3, 50)] public int convectionPointCount = 14;
        [Range(1, 16)] public int convectionEvenness = 6;
        public float planetRadiusKm = 6371f;

        [Header("Initial Crust")]
        [Tooltip("True: start from near-uniform primordial crust and let continents emerge. " +
                 "False: noise-seeded continents (useful for testing continental behaviour in isolation).")]
        public bool uniformInitialCrust = true;
        [Tooltip("Uniform mode: starting thickness, km. Independent of oceanThicknessKm / continentThicknessKm. " +
                 "Above ~21 km (crust fraction 0.5) it is buoyant and thickens under convergence instead of subducting.")]
        public float initialCrustThicknessKm = 25f;
        [Tooltip("Uniform mode: starting geological age, Myr.")]
        public float initialCrustAgeMyr = 0f;
        [Tooltip("Uniform mode: +- random thickness per vertex, km. Subtle seed for where differentiation starts; no large-scale pattern.")]
        [Range(0f, 5f)] public float initialThicknessVariationKm = 0.5f;
        [Tooltip("Seeded mode only.")]
        [Range(0.05f, 0.7f)] public float continentCoverage = 0.3f;
        [Tooltip("Seeded mode only.")]
        [Range(0.3f, 4f)] public float continentNoiseScale = 1.3f;
        [Tooltip("Mature oceanic crust thickness, km (floor for thinning, thickness of new ridge crust).")]
        public float oceanThicknessKm = 7f;
        [Tooltip("Normal continental thickness, km (crust fraction 1).")]
        public float continentThicknessKm = 35f;

        [Header("Planetary Cooling")]
        [Tooltip("Global thermal age G(t) = max * (1 - exp(-t / timescale)), t = absolute simulated time. " +
                 "Thermal age = geological age + G. Off: thermal age = geological age.")]
        public bool enablePlanetaryCooling = true;
        [Tooltip("Myr. Time for the planet to get ~63% of the way to its final cooling state.")]
        public float planetaryCoolingTimescaleMyr = 1500f;
        [Tooltip("Myr. Thermal age that brand-new crust approaches on a fully cooled planet.")]
        public float maxGlobalThermalAgeMyr = 1000f;
        [Tooltip("Myr. Thermal age at which crust is half as strong as it can get: strength = T / (T + ref). " +
                 "Drives the Thermal view, stiffness, the compression barrier and extension resistance.")]
        public float thermalStrengthRefMyr = 400f;

        [Header("Time")]
        public float totalTimeMyr = 4000f;
        [Tooltip("0 = automatic: largest step that keeps vertices from moving more than maxStepFraction of an edge.")]
        public float stepMyr = 0f;
        [Tooltip("Max distance a vertex may move per step, x target edge length. Smaller = fewer inverted triangles.")]
        [Range(0.05f, 0.5f)] public float maxStepFraction = 0.2f;

        [Header("Animation")]
        public float animMyrPerSecond = 100f;
        [Range(1, 64)] public int maxStepsPerFrame = 8;
        [Tooltip("Push inspector changes into the running simulation. Resolution, seed, step size and initial crust only take effect on restart.")]
        public bool liveTuning = true;
        public bool animateOnPlay = false;

        [Header("Preview Mesh")]
        public bool showPreviewMesh = true;
        [Tooltip("Optional. Needs a shader that shows vertex colours; defaults to PlanetGen/TectonicPreview.")]
        public Material previewMaterial;
        [Tooltip("Draw every mesh edge on top of the surface.")]
        public bool showWireframe = true;
        public Color wireColor = new Color(0f, 0f, 0f, 0.35f);
        [Tooltip("Optional. Defaults to PlanetGen/TectonicWire.")]
        public Material wireMaterial;

        [Header("Mesh Rules (x target edge length)")]
        [Tooltip("Hard cap: any edge longer than this is split.")]
        [Range(1.2f, 2.5f)] public float maxEdgeLength = 1.6f;
        [Tooltip("Edges shorter than this are collapsed (subducted or merged).")]
        [Range(0.2f, 0.8f)] public float minEdgeLength = 0.45f;
        [Tooltip("Converging edges between buoyant crust (that can't subduct) merge below this length when the crust is hot, " +
                 "so resisted compression thickens it instead of packing the mesh. Cools toward minEdgeLength with thermal " +
                 "strength, so cold crust is held by the compression barrier instead. Must stay below maxEdgeLength / 2.")]
        [Range(0.3f, 0.79f)] public float compressionMergeLength = 0.7f;
        [Tooltip("A vertex this close to its opposite edge is treated as colliding with it.")]
        [Range(0.05f, 0.5f)] public float sliverAltitude = 0.2f;
        [Tooltip("Tangential smoothing per step. Moves mesh vertices without moving crust, which slowly transports material " +
                 "(measured: continents lose ~40% more area over 2 Gyr at 0.03). Leave at 0 unless triangles look poor.")]
        [Range(0f, 0.3f)] public float relaxation = 0f;
        [Range(8, 24)] public int maxValence = 12;

        [Header("Compression Limit")]
        [Tooltip("Edge length (x target) where resistance to further compression begins.")]
        [Range(0.5f, 1.2f)] public float barrierStart = 0.9f;
        [Tooltip("Exponential stiffness. Fraction of closing motion removed = 1 - exp(-k x).")]
        public float barrierStiffness = 10f;
        [Tooltip("How much old crust multiplies the barrier stiffness.")]
        public float barrierAgeFactor = 1.5f;
        [Tooltip("Fraction of area loss that thickens oceanic crust (continental = 1).")]
        [Range(0f, 1f)] public float oceanicCompression = 0f;
        [Tooltip("Orogenic collapse: rad/Myr of outward flow per km of excess-thickness difference to a neighbour. " +
                 "Spreads mountain belts instead of deleting them, so continental volume is conserved.")]
        public float gravitySpreading = 0.00004f;
        [Range(0f, 1f)] public float gravityLock = 1f;
        public float gravityLockMarginKm = 8f;

        [Header("Extension Resistance & Damage")]
        [Tooltip("Off: crust offers no resistance to being stretched and continental rifts use the old opening rule.")]
        public bool enableExtensionResistance = true;
        [Tooltip("Max fraction (0..1) of relative opening that intact, cold, thick crust removes per constraint pass.")]
        [Range(0f, 1f)] public float extensionStiffness = 0.9f;
        [Tooltip("Intact resistance = stiffness * (1 - exp(-s * (ageFactor + thicknessFactor * cf))), s = thermal strength 0..1.")]
        public float extensionAgeFactor = 1.5f;
        [Tooltip("See extensionAgeFactor. cf = crust fraction (0 oceanic, 1 continental).")]
        public float extensionThicknessFactor = 2.5f;
        [Tooltip("Base tension (attempted opening strain rate of the mantle drive, in velocityReference per radian) for damage. " +
                 "Effective yield = yield * (0.25 + x * (1 - damage)), x = the strength exponent above.")]
        public float extensionYield = 0.2f;
        [Tooltip("Damage per Myr per unit of tension above yield.")]
        public float extensionDamageRate = 0.02f;
        [Tooltip("1/Myr. Exponential healing of damage while tension is below yield.")]
        public float extensionDamageHealing = 0.003f;
        [Tooltip("Fraction of opening removed by fully damaged crust (0 = fractured crust separates freely).")]
        [Range(0f, 1f)] public float fracturedCrustResistance = 0.05f;
        [Tooltip("Damage at which stretched continental edges are split as new rift crust instead of being refined.")]
        [Range(0.05f, 1f)] public float riftDamageThreshold = 0.5f;

        [Header("Convection")]
        public float convectionStrength = 0.003f;
        [Range(0.5f, 3f)] public float convectionRadiusScale = 1.3f;
        [Range(0.3f, 1.5f)] public float plumeThreshold = 0.7f;
        [Range(0.05f, 1f)] public float plumeRadius = 0.3f;
        [Tooltip("Rad/Myr that cells migrate toward the trenches / continents that feed them.")]
        public float convectionDrift = 0.0002f;
        [Tooltip("Random walk of each cell's base temperature, per sqrt(Myr). Keep small next to the feedback terms.")]
        public float temperatureNoise = 0.005f;
        [Tooltip("Feedback heating per Myr at full continental (area) coverage of a cell.")]
        public float insulationHeat = 0.012f;
        [Tooltip("Feedback cooling per Myr of exposed (non-continental) area, scaled by (temp + 1) / 2 so hot cells cool fastest.")]
        public float surfaceCooling = 0.008f;
        public float subductionCool = 0.5f;
        public float feedbackRelaxPerMyr = 0.005f;

        [Header("Resistance & Coupling")]
        public float ageResistance = 1.5f;
        public float ageSaturationMyr = 300f;
        public float velocityInertiaMyr = 20f;
        [Range(0f, 1f)] public float plateCoupling = 0.9f;
        [Tooltip("Extra one-ring velocity averaging passes per step for continental crust (rounded up to even). " +
                 "More = more rigid continents but slower plates and some crust loss over long runs. 0 conserves crust best.")]
        [Range(0, 16)] public int plateRigidityPasses = 0;
        [Tooltip("Jacobi passes per step for the compression and extension limits (rounded up to even). A vertex limited " +
                 "from several directions at once leaks part of the motion each pass, so more passes = harder limits. " +
                 "Each pass is one parallel one-ring traversal.")]
        [Range(2, 16)] public int constraintPasses = 6;
        public float slabPull = 0.002f;

        [Header("Subduction")]
        public float subductThicknessDiffKm = 12f;
        public float subductDensityDiff = 0.2f;

        [Header("Divergence")]
        [Tooltip("Opening speed (fraction of velocityReference) above which a split edge makes fresh crust instead of interpolating.")]
        public float riftOpenThreshold = 0.1f;
        [Range(0f, 1f)] public float riftCrustBlend = 0.35f;

        [Header("Terrain Accumulators")]
        [Range(0f, 1f)] public float overridingThickening = 0.35f;
        public float orogenyGain = 0.01f;
        public float orogenyDecayMyr = 300f;
        [Tooltip("True volume loss above normal continental thickness. Most relief relaxation comes from gravitySpreading.")]
        public float erosionPerMyr = 0.0005f;
        public float maxThicknessKm = 75f;
        public float arcGain = 0.01f;
        public float arcDecayMyr = 150f;

        [Header("Fault Classification")]
        public float velocityReference = 0.003f;
        public float faultThreshold = 0.12f;
        public float faultMemoryMyr = 150f;
        public float transformDominance = 1.2f;

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
        [Tooltip("Seconds of flash in the TopologyEvents view, in simulated Myr.")]
        public float eventFlashMyr = 20f;
        [Tooltip("Recompute the area-weighted diagnostics every N completed batches (they are not needed by the simulation).")]
        [Range(1, 100)] public int diagnosticsInterval = 10;

        // ---- results ----
        public PlanetPoint[] Points { get; private set; } = Array.Empty<PlanetPoint>();
        public Convection[] ConvectionPoints { get; private set; } = Array.Empty<Convection>();
        public float LastStepMyr { get; private set; }
        public float TargetEdge { get; private set; }

        // ---- status ----
        public bool HasSimulation => _state != null;
        public bool IsAnimating => _playing;
        public int StepIndex => _stepIndex;
        public int TotalSteps => LastStepMyr > 0f ? Mathf.Max(1, Mathf.CeilToInt(totalTimeMyr / LastStepMyr)) : 0;
        public float CurrentTimeMyr => _stepIndex * LastStepMyr;
        public float Progress => TotalSteps > 0 ? Mathf.Clamp01(_stepIndex / (float)TotalSteps) : 0f;
        public bool IsFinished => _state != null && _stepIndex >= TotalSteps;
        public float EffectiveMyrPerSecond { get; private set; }
        public int VertexCount { get; private set; }
        public int TriangleCount { get; private set; }
        public int VertexCapacity => _state?.VCap ?? 0;
        public int TotalSplits { get; private set; }
        public int TotalCollapses { get; private set; }
        public int TotalFlips { get; private set; }
        public int TotalSubducted { get; private set; }
        public int TotalRifts { get; private set; }
        public int InvertedTriangles { get; private set; }
        public int CapacityHits { get; private set; }
        public float MinEdgeRatio { get; private set; }
        public float MaxEdgeRatio { get; private set; }
        /// <summary>Topology edits per simulated Myr over the last measurement window.</summary>
        public float SplitsPerMyr { get; private set; }
        public float CollapsesPerMyr { get; private set; }
        public float FlipsPerMyr { get; private set; }
        public string LastValidation { get; private set; } = "";
        /// <summary>Area-weighted diagnostics, refreshed every diagnosticsInterval batches.</summary>
        public MeshDiagnostics Diagnostics { get; private set; }
        public int ContinentalSubducted { get; private set; }
        public float GlobalThermalAgeMyr => MeshStep.GlobalThermalAge(CurrentTimeMyr, enablePlanetaryCooling, planetaryCoolingTimescaleMyr, maxGlobalThermalAgeMyr);

        // ---- simulation state ----
        SphereMeshState _state;
        MeshParams _M;
        JobHandle _handle;
        bool _jobsInFlight, _previewPending, _playing, _finishPending, _previewDirty;
        int _stepIndex, _batchesSinceSnapshot, _batchesSinceDiag;
        float _accum, _lastTick, _stepBudget = 1f;
        int _batchSteps, _rateSteps; float _rateTime;
        int _rateSplits0, _rateCollapses0, _rateFlips0, _rateStep0;
        System.Diagnostics.Stopwatch _runTimer;

        float _spacing, _dt, _L; int _convN; uint _seedHash;

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

        public void Pause() { _playing = false; if (_state != null) PublishPoints(); }

        public void Resume()
        {
            if (_state == null) { StartAnimation(); return; }
            if (IsFinished) return;
            _playing = true; _accum = 0f; _lastTick = Time.realtimeSinceStartup;
        }

        public void StepOnce()
        {
            if (_state == null) BeginSimulation();
            _playing = false;
            CompleteJobs();
            if (liveTuning) RefreshTunables();
            _handle = SchedulePreview(ScheduleSteps(1, default));
            _stepIndex++;
            _jobsInFlight = true;
            PublishPoints();
        }

        [ContextMenu("Finish Now")]
        public void FinishNow() { if (_state == null) { Generate(); return; } RunToEnd(); }

        [ContextMenu("Clear")]
        public void Clear()
        {
            DisposeSimulation();
            DestroyPreview();
            Points = Array.Empty<PlanetPoint>();
            ConvectionPoints = Array.Empty<Convection>();
            _live = null; _liveCount = 0; _stepIndex = 0; VertexCount = 0; TriangleCount = 0;
        }

        /// <summary>Runs the full topology validator now. Returns true if the mesh is valid.</summary>
        [ContextMenu("Validate Mesh")]
        public bool ValidateMesh()
        {
            if (_state == null) { LastValidation = "No simulation"; return false; }
            CompleteJobs();
            string err = _state.Validate(0f, out int inverted);
            LastValidation = err == null
                ? $"OK at step {_stepIndex}: {VertexCount} verts, {TriangleCount} tris, {inverted} inverted"
                : $"FAILED at step {_stepIndex}:\n{err}";
            if (err == null) Debug.Log("[TectonicsMesh] " + LastValidation, this);
            else Debug.LogError("[TectonicsMesh] " + LastValidation, this);
            return err == null;
        }

        public void PublishPoints()
        {
            if (_state == null) return;
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
            if (_state == null) return;
            CompleteJobs();
            var s = _state;
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
            if (_state == null) return;

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
                    EffectiveMyrPerSecond = _rateSteps * _dt / _rateTime;
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
                int remaining = TotalSteps - _stepIndex;
                if (remaining <= 0) { _playing = false; _finishPending = true; }
                else
                {
                    int cap = Mathf.Clamp((int)_stepBudget, 1, maxStepsPerFrame);
                    _accum += wall * Mathf.Max(0f, animMyrPerSecond) / _dt;
                    int n = Mathf.Min((int)_accum, Mathf.Min(cap, remaining));
                    _accum = Mathf.Min(_accum - n, cap);
                    if (n > 0)
                    {
                        _handle = SchedulePreview(ScheduleSteps(n, default));
                        _stepIndex += n;
                        _batchSteps = n;
                        _jobsInFlight = true;
                        JobHandle.ScheduleBatchedJobs();
                        if (_stepIndex >= TotalSteps) { _playing = false; _finishPending = true; }
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
            _convN = convectionPointCount;
            _seedHash = (uint)seed * 2654435761u + 12345u;

            int freq = Mathf.Max(2, Mathf.RoundToInt(Mathf.Sqrt((targetVertexCount - 2) / 10f)));
            var verts = new List<float3>(); var tris = new List<int>();
            SphereMeshState.BuildGeodesic(freq, verts, tris);
            int nv = verts.Count;

            // target edge = measured mean edge of the starting mesh
            double sum = 0; int ne = 0;
            for (int t = 0; t < tris.Count; t += 3)
                for (int k = 0; k < 3; k++) { sum += math.distance(verts[tris[t + k]], verts[tris[t + (k + 1) % 3]]); ne++; }
            _L = (float)(sum / ne);
            TargetEdge = _L;
            _spacing = math.sqrt(4f * math.PI / nv);

            _dt = stepMyr > 0f ? stepMyr : math.clamp(maxStepFraction * _L / (convectionStrength * 2f), 0.1f, 25f);
            LastStepMyr = _dt;

            int vCap = Mathf.Max(nv + 64, (int)(nv * capacityMultiplier));
            _state = new SphereMeshState(vCap, vCap * 2, _convN);

            var rng = new Unity.Mathematics.Random(_seedHash | 1u);
            for (int i = 0; i < nv; i++)
            {
                float3 p = verts[i];
                TMath.TangentFrame(p, out float3 t, out float3 bt);
                float a = rng.NextFloat(2f * math.PI);
                verts[i] = math.normalize(p + (t * math.cos(a) + bt * math.sin(a)) * (rng.NextFloat() * meshJitter * _L));
            }
            _state.Load(verts, tris);

            _stepIndex = 0;
            _M = BuildParams();
            var init = new CrustInit
            {
                uniform = uniformInitialCrust, initialThickness = initialCrustThicknessKm, initialAge = initialCrustAgeMyr,
                variation = initialThicknessVariationKm, coverage = continentCoverage, noiseScale = continentNoiseScale,
                convEvenness = convectionEvenness
            };
            MeshStep.InitCrust(_state, _M, nv, ref rng, init);

            // first topology pass: jitter makes the geodesic mesh non-Delaunay, so this mostly flips
            RunTopologyNow();
            for (int k = MeshTopo.Splits; k <= MeshTopo.Rifts; k++) _state.counters[k] = 0;
            _state.counters[MeshTopo.ConsumedThisStep] = 0;

            _stepIndex = 0; _accum = 0f; _batchesSinceSnapshot = 0;
            _stepBudget = 1f; _rateSteps = 0; _rateTime = 0f; EffectiveMyrPerSecond = 0f;
            _rateSplits0 = _rateCollapses0 = _rateFlips0 = _rateStep0 = 0;
            SplitsPerMyr = CollapsesPerMyr = FlipsPerMyr = 0f;
            _batchSteps = 0; _playing = false; _finishPending = false;
            ReadCounters();
            Diagnostics = MeshDiagnostics.Compute(_state, _M, planetRadiusKm);
            _batchesSinceDiag = 0;
            ConvectionPoints = _state.conv.ToArray();
            _live = new PlanetPoint[vCap];
            FillSnapshot();

            _previewDirty = true;
            SyncPreviewObject();
        }

        MeshParams BuildParams()
        {
            float cellAngle = math.acos(1f - 2f / _convN);
            float lockMargin = Mathf.Max(0.1f, gravityLockMarginKm);
            var P = new TectonicParams
            {
                dt = _dt, spacing = _spacing, interactR = _L, seed = _seedHash,
                maxSpeed = maxStepFraction * _L / _dt,

                convRadius = cellAngle * convectionRadiusScale, convStrength = convectionStrength,
                plumeThreshold = plumeThreshold, plumeRadius = plumeRadius, convDrift = convectionDrift,
                convMinSep = 0.5f * cellAngle, tempNoise = temperatureNoise,
                insulationHeat = insulationHeat, subductionCool = subductionCool, feedbackRelax = feedbackRelaxPerMyr,

                ageResist = ageResistance, ageRef = ageSaturationMyr, velTau = velocityInertiaMyr,
                couple = plateCoupling, slabPull = slabPull,

                subductThickness = subductThicknessDiffKm, subductDensity = subductDensityDiff,
                closeThreshold = 0.03f * velocityReference, openThreshold = riftOpenThreshold * velocityReference,
                riftBlend = riftCrustBlend,

                overThick = overridingThickening,
                orogenyGain = orogenyGain, orogenyDecay = orogenyDecayMyr, erosion = erosionPerMyr,
                maxThickness = maxThicknessKm, arcGain = arcGain, arcDecay = arcDecayMyr,

                faultThreshold = faultThreshold, faultMemory = faultMemoryMyr,
                velRef = velocityReference, transformDominance = transformDominance,
                oceanThick = oceanThicknessKm, contThick = continentThicknessKm,

                lockMargin = lockMargin, lockStart = maxThicknessKm - lockMargin, lockStrength = gravityLock,
            };
            float lo = Mathf.Min(minEdgeLength, maxEdgeLength * 0.45f);   // splits must never produce collapsible halves
            return new MeshParams
            {
                P = P,
                L = _L, Lmin = lo * _L, Lmax = maxEdgeLength * _L,
                Lmerge = Mathf.Clamp(compressionMergeLength, lo, maxEdgeLength * 0.49f) * _L,
                sliverH = sliverAltitude * _L,
                minOrient = 1e-3f * _L * _L,
                flipEps = Mathf.Max(1e-3f * _L * _L, 2e-7f),
                maxValence = maxValence,
                barrierRest = barrierStart * _L, barrierK = barrierStiffness, barrierAge = barrierAgeFactor,
                oceanCompress = oceanicCompression,
                riftOpen = riftOpenThreshold * velocityReference,
                relax = relaxation,
                flashDecay = Mathf.Max(0.1f, eventFlashMyr),
                spreading = gravitySpreading,

                globalThermal = GlobalThermalAgeMyr,
                thermalRef = Mathf.Max(1f, thermalStrengthRefMyr),

                extEnabled = enableExtensionResistance,
                extMax = extensionStiffness, extAge = extensionAgeFactor, extThick = extensionThicknessFactor,
                extYield = extensionYield, extRate = extensionDamageRate, extHeal = extensionDamageHealing,
                extFractured = fracturedCrustResistance, riftDamage = riftDamageThreshold,

                surfaceCool = surfaceCooling,
            };
        }

        void RefreshTunables() { _M = BuildParams(); }

        // ================================================================== job scheduling
        void RunTopologyNow() { MeshStep.Topology(_state, _M, false).Run(); }

        JobHandle ScheduleSteps(int count, JobHandle dependency)
        {
            var s = _state;
            var M = _M;
            float dt = _dt;
            int cap = s.VCap;
            bool relax = M.relax > 0f;
            JobHandle h = dependency;

            int passes = MeshStep.CouplePasses(constraintPasses, plateRigidityPasses);

            for (int k = 0; k < count; k++)
            {
                uint step = (uint)(_stepIndex + k);
                // global cooling is evaluated per step from absolute time, so batch size never changes it
                var Mk = M;
                Mk.globalThermal = MeshStep.GlobalThermalAge(step * dt, enablePlanetaryCooling, planetaryCoolingTimescaleMyr, maxGlobalThermalAgeMyr);

                h = MeshStep.Drive(s, Mk, dt).Schedule(cap, 256, h);
                h = MeshStep.Velocity(s, Mk, dt).Schedule(cap, 128, h);
                for (int pIdx = 0; pIdx < passes; pIdx++)
                    h = MeshStep.Couple(s, Mk, pIdx, plateRigidityPasses).Schedule(cap, 256, h);
                h = MeshStep.Area(s, Mk).Schedule(cap, 256, h);
                h = MeshStep.Move(s, Mk, dt).Schedule(cap, 256, h);
                h = MeshStep.Integrate(s, Mk, dt).Schedule(cap, 256, h);
                if (relax) h = MeshStep.Relax(s, Mk).Schedule(cap, 256, h);
                h = MeshStep.Topology(s, Mk, relax).Schedule(h);
                h = MeshStep.Feedback(s, Mk, dt, step).Schedule(h);
            }
            return h;
        }

        JobHandle SchedulePreview(JobHandle dependency)
        {
            _previewDirty = false;
            if (_state == null || _mesh == null || !_verts.IsCreated) return dependency;
            var s = _state;
            _previewPending = true;
            var vj = new MeshPreviewVertexJob
            {
                M = _M, view = (int)debugView, radius = planetRadiusKm,
                V = s.V, O = s.O, vc = s.vc, alive = s.alive, faultType = s.faultType, pos = s.pos, vel = s.vel,
                thickness = s.thickness, age = s.age, faultActivity = s.faultActivity, orogeny = s.orogeny,
                hotspot = s.hotspot, flashSplit = s.flashSplit, flashCollapse = s.flashCollapse,
                damage = s.damage, tension = s.tension,
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
            int n = Mathf.Max(0, TotalSteps - _stepIndex);
            _handle = SchedulePreview(ScheduleSteps(n, default));
            _stepIndex += n;
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
            var c = _state.counters;
            VertexCount = c[MeshTopo.AliveV]; TriangleCount = c[MeshTopo.AliveT];
            TotalSplits = c[MeshTopo.Splits]; TotalCollapses = c[MeshTopo.Collapses]; TotalFlips = c[MeshTopo.Flips];
            TotalSubducted = c[MeshTopo.Subducted]; TotalRifts = c[MeshTopo.Rifts];
            InvertedTriangles = c[MeshTopo.Inverted]; CapacityHits = c[MeshTopo.CapHits];
            ContinentalSubducted = c[MeshTopo.ContinentalSubducted];
            MinEdgeRatio = _state.stats[0]; MaxEdgeRatio = _state.stats[1];

            int ds = _stepIndex - _rateStep0;
            if (ds * _dt >= 20f)
            {
                float myr = ds * _dt;
                SplitsPerMyr = (TotalSplits - _rateSplits0) / myr;
                CollapsesPerMyr = (TotalCollapses - _rateCollapses0) / myr;
                FlipsPerMyr = (TotalFlips - _rateFlips0) / myr;
                _rateSplits0 = TotalSplits; _rateCollapses0 = TotalCollapses; _rateFlips0 = TotalFlips; _rateStep0 = _stepIndex;
            }
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

            if (_state != null)
            {
                ReadCounters();
                ConvectionPoints = _state.conv.ToArray();
                if (++_batchesSinceDiag >= diagnosticsInterval)
                {
                    _batchesSinceDiag = 0;
                    Diagnostics = MeshDiagnostics.Compute(_state, _M, planetRadiusKm);
                }
                if (drawGizmos && drawFaultDirections && ++_batchesSinceSnapshot >= gizmoSnapshotInterval)
                {
                    _batchesSinceSnapshot = 0;
                    FillSnapshot();
                }
                if (validateEveryBatch && _stepIndex > 0 && !ValidateMesh()) _playing = false;
            }
            return uploaded;
        }

        void DisposeSimulation()
        {
            if (_jobsInFlight) { _handle.Complete(); _jobsInFlight = false; _previewPending = false; }
            _playing = false; _finishPending = false;
            _state?.Dispose();
            _state = null;
        }

        void LogRun(string verb)
        {
            long ms = _runTimer != null ? _runTimer.ElapsedMilliseconds : 0;
            Debug.Log($"[TectonicsMesh] {verb}: {_stepIndex} steps @ {_dt:F2} Myr ({CurrentTimeMyr:F0} Myr), " +
                      $"{VertexCount} verts / {TriangleCount} tris, {TotalSplits} splits ({TotalRifts} rifts), " +
                      $"{TotalCollapses} collapses ({TotalSubducted} subducted), {TotalFlips} flips, " +
                      $"edges {MinEdgeRatio:F2}-{MaxEdgeRatio:F2} x target, {InvertedTriangles} inverted, {ms} ms", this);
        }

        // ================================================================== snapshot
        PlanetPoint MakePoint(int i)
        {
            var s = _state;
            float t = s.thickness[i];
            return new PlanetPoint
            {
                position = s.pos[i], velocity = s.vel[i],
                thickness = t, density = s.density[i], age = s.age[i], stiffness = s.stiffness[i],
                faultType = (FaultType)s.faultType[i], faultActivity = s.faultActivity[i], faultDir = s.faultDir[i],
                orogeny = s.orogeny[i], extension = s.extension[i], arc = s.arc[i],
                shear = s.shear[i], hotspot = s.hotspot[i], crowding = 1f,
                elevationKm = TMath.ElevationKm(t, s.age[i], _M.P)
            };
        }

        void FillSnapshot()
        {
            var s = _state;
            if (s == null) return;
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
            bool want = showPreviewMesh && _state != null;
            if (!want) { if (_previewGO != null) DestroyPreview(); return; }

            if (_previewGO == null || _mesh == null || !_verts.IsCreated || _verts.Length != _state.VCap + 1 || _triIdx.Length != _state.TCap * 3)
            {
                BuildPreview(_state.VCap, _state.TCap);
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
