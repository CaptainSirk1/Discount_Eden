using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace PlanetGen.Tectonics
{
    public enum TectonicDebugView { Elevation, Thickness, Age, FaultType, Orogeny, Hotspot, Velocity, Crowding }

    /// <summary>Owns all native buffers for one simulation run.</summary>
    public sealed class TectonicState : IDisposable
    {
        public readonly int Capacity, CellCount, ConvCount, LatticeCount;
        readonly List<Action> _dispose = new List<Action>();

        public NativeArray<float3> pos, vel, velNext, driven, sepDisp, compAxis, faultDir, slab;
        public NativeArray<float> thickness, age, stiffness, density, orogeny, extension, arc, shear, hotspot, faultActivity;
        public NativeArray<float> rClose, rOpen, rShear, rSub, rOver, rCrumple, crowding;
        public NativeArray<byte> alive, faultType, removeFlag;
        public NativeArray<int> spawnPartner, nearest, freeList, counters, cellStart, cellCursor, cellItems, cellOf;
        public NativeArray<float3> spawnPos;
        public NativeArray<int> spawnHash, spawnSlots;

        // remesh: target lattice + gather buffers
        public NativeArray<float3> lattice;
        public NativeArray<float3> tPos, tVel, tFaultDir, tSlab;
        public NativeArray<float> tThickness, tAge, tDensity, tStiffness, tOrogeny, tExtension, tArc, tShear, tHotspot, tFaultActivity;
        public NativeArray<byte> tFaultType;
        public NativeArray<Convection> conv;
        public NativeArray<float3> convSumCont, convSumSub;
        public NativeArray<int> convTotal, convCont, convSub;

        NativeArray<T> Alloc<T>(int n) where T : struct
        {
            var a = new NativeArray<T>(n, Allocator.Persistent, NativeArrayOptions.ClearMemory);
            _dispose.Add(a.Dispose);
            return a;
        }

        public TectonicState(int capacity, int cellCount, int convCount, int latticeCount)
        {
            Capacity = capacity; CellCount = cellCount; ConvCount = convCount; LatticeCount = latticeCount;
            pos = Alloc<float3>(capacity); vel = Alloc<float3>(capacity); velNext = Alloc<float3>(capacity);
            driven = Alloc<float3>(capacity); sepDisp = Alloc<float3>(capacity); compAxis = Alloc<float3>(capacity);
            faultDir = Alloc<float3>(capacity); slab = Alloc<float3>(capacity);
            thickness = Alloc<float>(capacity); age = Alloc<float>(capacity); stiffness = Alloc<float>(capacity);
            density = Alloc<float>(capacity); orogeny = Alloc<float>(capacity); extension = Alloc<float>(capacity);
            arc = Alloc<float>(capacity); shear = Alloc<float>(capacity); hotspot = Alloc<float>(capacity);
            faultActivity = Alloc<float>(capacity);
            rClose = Alloc<float>(capacity); rOpen = Alloc<float>(capacity); rShear = Alloc<float>(capacity);
            rSub = Alloc<float>(capacity); rOver = Alloc<float>(capacity); rCrumple = Alloc<float>(capacity);
            crowding = Alloc<float>(capacity);
            alive = Alloc<byte>(capacity); faultType = Alloc<byte>(capacity); removeFlag = Alloc<byte>(capacity);
            spawnPos = Alloc<float3>(capacity); spawnSlots = Alloc<int>(capacity);
            spawnHash = Alloc<int>(math.ceilpow2(capacity * 2));
            for (int k = 0; k < spawnHash.Length; k++) spawnHash[k] = -1;
            spawnPartner = Alloc<int>(capacity); nearest = Alloc<int>(capacity); freeList = Alloc<int>(capacity);
            counters = Alloc<int>(4);
            cellStart = Alloc<int>(cellCount + 1); cellCursor = Alloc<int>(cellCount);
            cellItems = Alloc<int>(capacity); cellOf = Alloc<int>(capacity);
            conv = Alloc<Convection>(convCount);
            convSumCont = Alloc<float3>(convCount); convSumSub = Alloc<float3>(convCount);
            convTotal = Alloc<int>(convCount); convCont = Alloc<int>(convCount); convSub = Alloc<int>(convCount);

            lattice = Alloc<float3>(latticeCount);
            tPos = Alloc<float3>(capacity); tVel = Alloc<float3>(capacity);
            tFaultDir = Alloc<float3>(capacity); tSlab = Alloc<float3>(capacity);
            tThickness = Alloc<float>(capacity); tAge = Alloc<float>(capacity); tDensity = Alloc<float>(capacity);
            tStiffness = Alloc<float>(capacity); tOrogeny = Alloc<float>(capacity); tExtension = Alloc<float>(capacity);
            tArc = Alloc<float>(capacity); tShear = Alloc<float>(capacity); tHotspot = Alloc<float>(capacity);
            tFaultActivity = Alloc<float>(capacity); tFaultType = Alloc<byte>(capacity);
        }

        public void Dispose()
        {
            foreach (var d in _dispose) d();
            _dispose.Clear();
        }
    }

    [ExecuteAlways]
    public class PlanetTectonics : MonoBehaviour
    {
        [Header("Seed & Resolution")]
        public int seed = 12345;
        [Range(2000, 200000)] public int meshPointCount = 20000;
        [Range(0f, 0.6f)] public float meshJitter = 0.35f;
        [Range(3, 50)] public int convectionPointCount = 14;
        [Tooltip("Mitchell best-candidate samples per convection point. 1 = pure random, higher = more even.")]
        [Range(1, 16)] public int convectionEvenness = 6;
        public float planetRadiusKm = 6371f;

        [Header("Initial Crust")]
        [Range(0.05f, 0.7f)] public float continentCoverage = 0.3f;
        [Range(0.3f, 4f)] public float continentNoiseScale = 1.3f;
        public float oceanThicknessKm = 7f;
        public float continentThicknessKm = 35f;

        [Header("Time")]
        [Tooltip("Can be raised mid-run to keep a finished animation going.")]
        public float totalTimeMyr = 4000f;
        [Tooltip("0 = automatic: largest step that keeps points from moving more than maxStepFraction of their spacing.")]
        public float stepMyr = 0f;
        [Range(0.1f, 0.8f)] public float maxStepFraction = 0.45f;

        [Header("Animation")]
        [Tooltip("Simulated Myr per real second while animating.")]
        public float animMyrPerSecond = 100f;
        [Tooltip("Hard cap on sim steps per frame, so a slow frame can't snowball.")]
        [Range(1, 64)] public int maxStepsPerFrame = 8;
        [Tooltip("Push inspector changes into the running simulation. Resolution, seed, step size, " +
                 "interaction radius and initial crust only take effect on restart.")]
        public bool liveTuning = true;
        public bool animateOnPlay = false;

        [Header("Preview Mesh")]
        [Tooltip("Burst-built quad mesh, much faster than gizmos and visible in the Game view.")]
        public bool showPreviewMesh = true;
        [Tooltip("Optional. Needs a shader that shows vertex colours; defaults to PlanetGen/TectonicPreview.")]
        public Material previewMaterial;
        [Range(0.2f, 1.5f)] public float previewPointSize = 0.65f;

        [Header("Convection")]
        [Tooltip("Peak surface drive in radians/Myr (0.0016 rad/Myr = 1 cm/yr).")]
        public float convectionStrength = 0.003f;
        [Range(0.5f, 3f)] public float convectionRadiusScale = 1.3f;
        [Range(0.3f, 1.5f)] public float plumeThreshold = 0.7f;
        [Range(0.05f, 1f)] public float plumeRadius = 0.3f;
        [Tooltip("Radians/Myr that cells migrate toward trenches / insulating continents.")]
        public float convectionDrift = 0.0004f;
        public float temperatureNoise = 0.01f;
        public float insulationHeat = 0.004f;
        public float subductionCool = 0.5f;
        public float feedbackRelaxPerMyr = 0.005f;

        [Header("Resistance & Coupling")]
        public float ageResistance = 1.5f;
        public float ageSaturationMyr = 300f;
        public float velocityInertiaMyr = 20f;
        [Range(0f, 1f)] public float plateCoupling = 0.9f;
        [Tooltip("Rad/Myr per unit density contrast; drives dense slabs toward the trench.")]
        public float slabPull = 0.002f;

        [Header("Interaction (multiples of mean point spacing)")]
        [Tooltip("Sets the grid cell size, so changes only apply on restart.")]
        [Range(1.6f, 3.5f)] public float interactionRadius = 2.2f;
        [Range(0.4f, 1.2f)] public float minPointDistance = 0.8f;
        [Range(0.8f, 2f)] public float collisionDistance = 1.3f;
        [Tooltip("1 = hard minimum distance (each overlap fully resolved per step).")]
        [Range(0f, 1f)] public float separationStrength = 0.8f;

        [Header("Gravity Cap (crust that can't thicken can't compress)")]
        [Tooltip("How completely convergence stalls once crust nears maxThicknessKm. 0 = original behaviour " +
                 "(points keep packing), 1 = colliding plates lock until erosion makes room.")]
        [Range(0f, 1f)] public float gravityLock = 1f;
        [Tooltip("km below maxThicknessKm over which the lock ramps in.")]
        public float gravityLockMarginKm = 8f;

        [Header("Subduction")]
        public float subductThicknessDiffKm = 12f;
        public float subductDensityDiff = 0.2f;
        [Range(0f, 2f)] public float subductionRate = 1f;
        [Tooltip("A subducting point closer than this (x spacing) to the overriding plate is consumed. Keeps trenches from over-packing.")]
        [Range(0.2f, 1f)] public float consumeDistance = 0.6f;

        [Header("Divergence")]
        [Range(1.1f, 2f)] public float gapDistance = 1.5f;
        [Range(0.3f, 1.2f)] public float holeClearance = 0.75f;
        [Tooltip("Fraction of velocityReference below which an opening is ignored.")]
        public float openThreshold = 0.05f;
        public float riftStiffnessFactor = 0.3f;
        [Range(0f, 1f)] public float riftCrustBlend = 0.35f;
        [Tooltip("km of continental thinning per Myr at reference spreading speed.")]
        public float extensionThinning = 0.25f;

        [Header("Void Filling")]
        [Tooltip("Spawn new crust wherever local density drops, not only between opening pairs. Fixes holes that grow past the interaction radius.")]
        public bool voidFilling = true;
        [Tooltip("Crowding (1 = even lattice) below which a point proposes new crust next to it.")]
        [Range(0.3f, 0.95f)] public float voidFillThreshold = 0.7f;
        [Tooltip("How one-sided a point's neighbourhood must be to count as a hole edge. Below this, a random direction is tried.")]
        [Range(0.05f, 0.6f)] public float voidDirectionBias = 0.2f;

        [Header("Remesh (safety net)")]
        [Tooltip("Resample every point onto the original lattice every N Myr. 0 = off. Guarantees even spacing " +
                 "but causes a brief visible shimmer; usually unnecessary with void filling and the gravity cap.")]
        public float remeshIntervalMyr = 0f;

        [Header("Crumple / Terrain Accumulators")]
        public float crumpleThickening = 0.35f;
        [Range(0f, 1f)] public float overridingThickening = 0.35f;
        public float orogenyGain = 0.01f;
        public float orogenyDecayMyr = 300f;
        public float erosionPerMyr = 0.002f;
        public float maxThicknessKm = 75f;
        public float arcGain = 0.01f;
        public float arcDecayMyr = 150f;

        [Header("Fault Classification")]
        public float velocityReference = 0.003f;
        public float faultThreshold = 0.12f;
        public float faultMemoryMyr = 150f;
        public float transformDominance = 1.2f;

        [Header("Debug")]
        public bool generateOnEnable = false;
        public bool drawGizmos = true;
        public TectonicDebugView debugView = TectonicDebugView.Elevation;
        [Range(1, 50)] public int gizmoStride = 2;
        [Range(0.1f, 3f)] public float gizmoPointSize = 1f;
        public bool drawConvection = true;
        public bool drawFaultDirections = true;
        public bool hideBackside = true;
        [Tooltip("While animating, refresh the gizmo point snapshot every N completed batches.")]
        [Range(1, 30)] public int gizmoSnapshotInterval = 4;

        // ---- results ----
        /// <summary>Final (or paused) result. Not refreshed every animation frame; call PublishPoints() if you need it mid-run.</summary>
        public PlanetPoint[] Points { get; private set; } = Array.Empty<PlanetPoint>();
        public Convection[] ConvectionPoints { get; private set; } = Array.Empty<Convection>();
        public float LastStepMyr { get; private set; }
        public float MeanSpacing { get; private set; }

        // ---- animation status (safe to read any time; cached at batch completion) ----
        public bool HasSimulation => _state != null;
        public bool IsAnimating => _playing;
        public int StepIndex => _stepIndex;
        public int TotalSteps => LastStepMyr > 0f ? Mathf.Max(1, Mathf.CeilToInt(totalTimeMyr / LastStepMyr)) : 0;
        public float CurrentTimeMyr => _stepIndex * LastStepMyr;
        public float Progress => TotalSteps > 0 ? Mathf.Clamp01(_stepIndex / (float)TotalSteps) : 0f;
        public bool IsFinished => _state != null && _stepIndex >= TotalSteps;
        public int AliveCount { get; private set; }
        public int SpawnedCount { get; private set; }
        public int SubductedCount { get; private set; }
        /// <summary>Measured simulation speed. Drops below animMyrPerSecond when the sim can't keep up.</summary>
        public float EffectiveMyrPerSecond { get; private set; }
        public int RemeshCount { get; private set; }

        // ---- simulation state (persists across frames) ----
        TectonicState _state;
        TectonicParams _P;
        JobHandle _handle;
        bool _jobsInFlight, _previewPending, _playing, _finishPending, _previewDirty;
        int _stepIndex, _batchesSinceSnapshot;
        float _accum, _lastTick;
        float _stepBudget = 1f;                 // adaptive steps-per-batch cap
        int _batchSteps, _rateSteps; float _rateTime;
        int _lastRemeshStep;
        System.Diagnostics.Stopwatch _runTimer;

        // structural values fixed for the lifetime of a run
        float _spacing, _dt, _interactR; int _gridDim, _capacity, _convN; uint _seedHash;

        // ---- preview ----
        GameObject _previewGO; MeshRenderer _previewRenderer; Mesh _mesh; Material _autoMaterial;
        NativeArray<float3> _verts; NativeArray<Color32> _colors;
        float _meshBoundsRadius;

        // ---- gizmo snapshot (reused, avoids GC churn while animating) ----
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
            else if (generateOnEnable && Points.Length == 0) Generate();
        }

        void OnDisable()
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.update -= EditorTick;
#endif
            DisposeSimulation();
            DestroyPreview();
            SafeDestroy(_autoMaterial); _autoMaterial = null;
        }

        void OnValidate() { _previewDirty = true; }

        void Update() { if (Application.isPlaying) Tick(); }

#if UNITY_EDITOR
        void EditorTick() { if (!Application.isPlaying && this != null) Tick(); }
#endif

        // ================================================================== public controls
        /// <summary>Runs the whole simulation immediately (original behaviour). State is kept so the preview can recolour.</summary>
        [ContextMenu("Generate")]
        public void Generate()
        {
            BeginSimulation();
            RunToEnd();
        }

        /// <summary>Starts a fresh run that advances a few steps per frame.</summary>
        [ContextMenu("Animate")]
        public void StartAnimation()
        {
            BeginSimulation();
            _playing = true;
            _accum = 0f;
            _lastTick = Time.realtimeSinceStartup;
        }

        public void Pause()
        {
            _playing = false;
            if (_state != null) PublishPoints();
        }

        public void Resume()
        {
            if (_state == null) { StartAnimation(); return; }
            if (IsFinished) return;
            _playing = true;
            _accum = 0f;
            _lastTick = Time.realtimeSinceStartup;
        }

        /// <summary>Advance exactly one step (pauses animation).</summary>
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

        /// <summary>Run all remaining steps right now.</summary>
        [ContextMenu("Finish Now")]
        public void FinishNow()
        {
            if (_state == null) { Generate(); return; }
            RunToEnd();
        }

        [ContextMenu("Clear")]
        public void Clear()
        {
            DisposeSimulation();
            DestroyPreview();
            Points = Array.Empty<PlanetPoint>();
            ConvectionPoints = Array.Empty<Convection>();
            _live = null; _liveCount = 0;
            _stepIndex = 0; AliveCount = 0;
        }

        /// <summary>Copies the current state into Points. Completes any in-flight jobs.</summary>
        public void PublishPoints()
        {
            if (_state == null) return;
            CompleteJobs();
            FillSnapshot();
            var arr = new PlanetPoint[_liveCount];
            Array.Copy(_live, arr, _liveCount);
            Points = arr;
        }

        // ================================================================== per-frame driver
        void Tick()
        {
            float now = Time.realtimeSinceStartup;
            float wall = Mathf.Clamp(now - _lastTick, 0f, 0.1f);
            _lastTick = now;
            if (_state == null) return;

            // Never block the frame. If last frame's batch is still running, let it keep going in the
            // background and shrink future batches; simulated time simply advances more slowly.
            if (_jobsInFlight && !_handle.IsCompleted)
            {
                _stepBudget = Mathf.Max(1f, _stepBudget * 0.7f);
                if (_playing) _rateTime += wall;
                return;
            }

            bool hadBatch = _jobsInFlight && _batchSteps > 0;
            bool uploaded = CompleteJobs();   // already finished: uploads mesh, caches stats, no wait
            if (hadBatch)
            {
                _rateSteps += _batchSteps;
                _batchSteps = 0;
                _stepBudget = Mathf.Min(maxStepsPerFrame, _stepBudget + 0.25f);   // finished in time: allow a bit more
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
                    _accum = Mathf.Min(_accum - n, cap);   // never bank a backlog

                    if (n > 0)
                    {
                        _handle = SchedulePreview(ScheduleSteps(n, default));
                        _stepIndex += n;
                        _batchSteps = n;
                        _jobsInFlight = true;
                        JobHandle.ScheduleBatchedJobs();   // runs on workers while the main thread renders
                        if (_stepIndex >= TotalSteps) { _playing = false; _finishPending = true; }
                    }
                }
            }
            if (!_jobsInFlight && _previewDirty)
            {
                // nothing stepped this frame (paused, finished, or slow playback): recolour / resize after inspector changes
                if (liveTuning) RefreshTunables();
                _handle = SchedulePreview(default);
                _jobsInFlight = true;
                uploaded |= CompleteJobs();
            }

#if UNITY_EDITOR
            if (!Application.isPlaying && (uploaded || _playing))
                UnityEditor.SceneView.RepaintAll();
#endif
        }

        // ================================================================== setup
        void BeginSimulation()
        {
            DisposeSimulation();
            _runTimer = System.Diagnostics.Stopwatch.StartNew();

            int M = meshPointCount;
            _convN = convectionPointCount;
            _spacing = math.sqrt(4f * math.PI / M);
            MeanSpacing = _spacing;

            _interactR = interactionRadius * _spacing;
            _gridDim = (int)math.ceil(2f / _interactR) + 1;
            int cells = _gridDim * _gridDim * _gridDim;
            _capacity = (int)(M * 1.5f);

            _dt = stepMyr > 0f ? stepMyr
                : math.clamp(maxStepFraction * _spacing / (convectionStrength * 2f), 0.25f, 25f);
            LastStepMyr = _dt;
            _seedHash = (uint)seed * 2654435761u + 12345u;

            _P = BuildParams();
            _state = new TectonicState(_capacity, cells, _convN, M);
            InitState(_state, _P, M, _convN);

            _stepIndex = 0; _accum = 0f; _batchesSinceSnapshot = 0;
            _stepBudget = 1f; _rateSteps = 0; _rateTime = 0f; EffectiveMyrPerSecond = 0f;
            _lastRemeshStep = 0; RemeshCount = 0;
            _batchSteps = 0;
            _playing = false; _finishPending = false;
            AliveCount = M; SpawnedCount = 0; SubductedCount = 0;
            ConvectionPoints = _state.conv.ToArray();
            _live = new PlanetPoint[_capacity];
            FillSnapshot();

            _previewDirty = true;
            SyncPreviewObject();
        }

        TectonicParams BuildParams()
        {
            float cellAngle = math.acos(1f - 2f / _convN);
            return new TectonicParams
            {
                // structural: fixed for the run
                dt = _dt, spacing = _spacing, interactR = _interactR,
                invCell = 1f / _interactR, gridDim = _gridDim, capacity = _capacity, seed = _seedHash,

                // everything below is live-tunable
                minDist = minPointDistance * _spacing, collDist = collisionDistance * _spacing,
                gapDist = gapDistance * _spacing, holeClear = holeClearance * _spacing,
                sepStrength = separationStrength,
                maxSpeed = maxStepFraction * _spacing / _dt,

                convRadius = cellAngle * convectionRadiusScale, convStrength = convectionStrength,
                plumeThreshold = plumeThreshold, plumeRadius = plumeRadius, convDrift = convectionDrift,
                convMinSep = 0.5f * cellAngle, tempNoise = temperatureNoise,
                insulationHeat = insulationHeat, subductionCool = subductionCool, feedbackRelax = feedbackRelaxPerMyr,

                ageResist = ageResistance, ageRef = ageSaturationMyr, velTau = velocityInertiaMyr,
                couple = plateCoupling, slabPull = slabPull,

                subductThickness = subductThicknessDiffKm, subductDensity = subductDensityDiff,
                subductRate = subductionRate,
                closeThreshold = 0.03f * velocityReference, openThreshold = openThreshold * velocityReference,
                riftStiffness = riftStiffnessFactor, riftBlend = riftCrustBlend, thinRate = extensionThinning,

                crumpleThick = crumpleThickening, overThick = overridingThickening,
                orogenyGain = orogenyGain, orogenyDecay = orogenyDecayMyr, erosion = erosionPerMyr,
                maxThickness = maxThicknessKm, arcGain = arcGain, arcDecay = arcDecayMyr,

                faultThreshold = faultThreshold, faultMemory = faultMemoryMyr,
                velRef = velocityReference, transformDominance = transformDominance,
                oceanThick = oceanThicknessKm, contThick = continentThicknessKm,

                lockMargin = Mathf.Max(0.1f, gravityLockMarginKm),
                lockStart = maxThicknessKm - Mathf.Max(0.1f, gravityLockMarginKm),
                lockStrength = gravityLock,
                consumeDist = consumeDistance * _spacing,

                expectNeighbors = Mathf.Max(1f, math.PI * (_interactR / _spacing) * (_interactR / _spacing) - 1f),
                voidFill = voidFilling ? voidFillThreshold : 0f,
                voidBias = voidDirectionBias,
                invSpawnCell = 1f / Mathf.Max(holeClearance * _spacing, 1e-5f)
            };
        }

        void RefreshTunables() { _P = BuildParams(); }

        void InitState(TectonicState s, in TectonicParams P, int M, int N)
        {
            float spacing = P.spacing;
            var rng = new Unity.Mathematics.Random(P.seed | 1u);

            // convection points: Mitchell best-candidate for even-ish random coverage
            for (int k = 0; k < N; k++)
            {
                float3 best = rng.NextFloat3Direction();
                float bestScore = float.MaxValue;
                for (int c = 0; c < convectionEvenness; c++)
                {
                    float3 cand = rng.NextFloat3Direction();
                    float maxDot = -1f;
                    for (int q = 0; q < k; q++) maxDot = math.max(maxDot, math.dot(cand, s.conv[q].pos));
                    if (maxDot < bestScore) { bestScore = maxDot; best = cand; }
                }
                float t = (rng.NextBool() ? 1f : -1f) * rng.NextFloat(0.3f, 1f);
                s.conv[k] = new Convection { pos = best, temp = t, baseTemp = t, feedback = 0f };
            }

            // mesh points: Fibonacci lattice (even area), jittered, renormalised
            float golden = math.PI * (3f - math.sqrt(5f));
            var noiseVals = new float[M];
            float3 offset = rng.NextFloat3(-100f, 100f);
            for (int i = 0; i < M; i++)
            {
                float y = 1f - 2f * (i + 0.5f) / M;
                float r = math.sqrt(math.max(0f, 1f - y * y));
                float phi = i * golden;
                float3 p = new float3(math.cos(phi) * r, y, math.sin(phi) * r);
                p += rng.NextFloat3Direction() * (rng.NextFloat() * meshJitter * spacing);
                p = math.normalize(p);
                s.pos[i] = p;

                float n = 0f, a = 1f, f = continentNoiseScale, norm = 0f;
                for (int o = 0; o < 4; o++)
                {
                    n += a * noise.snoise(p * f + offset);
                    norm += a; a *= 0.5f; f *= 2.1f;
                }
                noiseVals[i] = n / norm;
            }

            var sorted = (float[])noiseVals.Clone();
            Array.Sort(sorted);
            float thr = sorted[Mathf.Clamp((int)((1f - continentCoverage) * M), 0, M - 1)];

            for (int i = 0; i < M; i++)
            {
                float sm = math.smoothstep(thr - 0.04f, thr + 0.04f, noiseVals[i]);
                float contT = P.contThick * (0.9f + 0.3f * rng.NextFloat());
                float t = P.oceanThick + (contT - P.oceanThick) * sm;
                float age = sm > 0.5f ? rng.NextFloat(800f, 2500f) : rng.NextFloat(0f, 150f);
                s.thickness[i] = t; s.age[i] = age;
                s.density[i] = TMath.Density(t, age, P);
                s.stiffness[i] = TMath.Stiffness(t, age, 0f, P);
                s.alive[i] = 1;
                s.faultDir[i] = math.normalizesafe(math.cross(s.pos[i], new float3(0, 1, 0)), new float3(1, 0, 0));
            }

            for (int i = 0; i < M; i++) s.lattice[i] = s.pos[i];   // remesh target = the initial lattice

            int free = s.Capacity - M;
            for (int k = 0; k < free; k++) s.freeList[k] = s.Capacity - 1 - k;
            s.counters[0] = M; s.counters[1] = free;
        }

        // ================================================================== job scheduling
        /// <summary>Chains <paramref name="count"/> sim steps starting at the current step index. Does not advance _stepIndex.</summary>
        JobHandle ScheduleSteps(int count, JobHandle dependency)
        {
            var s = _state;
            var P = _P;
            float dt = _dt;
            int cap = s.Capacity;
            JobHandle h = dependency;

            for (int k = 0; k < count; k++)
            {
                uint step = (uint)(_stepIndex + k);

                var build = new BuildGridJob
                {
                    P = P, pos = s.pos, alive = s.alive, cellStart = s.cellStart, cursor = s.cellCursor,
                    cellItems = s.cellItems, cellOf = s.cellOf
                }.Schedule(h);

                var drive = new ConvectionDriveJob
                {
                    P = P, dt = dt, conv = s.conv, alive = s.alive, pos = s.pos, slab = s.slab,
                    density = s.density, age = s.age, driven = s.driven, nearest = s.nearest, hotspot = s.hotspot
                }.Schedule(cap, 256, h);

                var pre = JobHandle.CombineDependencies(build, drive);

                var inter = new InteractionJob
                {
                    P = P, dt = dt, step = step,
                    alive = s.alive, pos = s.pos, vel = s.vel, driven = s.driven,
                    thickness = s.thickness, density = s.density, stiffness = s.stiffness,
                    cellStart = s.cellStart, cellItems = s.cellItems,
                    velNext = s.velNext, sepDisp = s.sepDisp, compAxis = s.compAxis, slab = s.slab,
                    rClose = s.rClose, rOpen = s.rOpen, rShear = s.rShear, rSub = s.rSub, rOver = s.rOver,
                    rCrumple = s.rCrumple, crowding = s.crowding,
                    removeFlag = s.removeFlag, spawnPartner = s.spawnPartner, spawnPos = s.spawnPos
                }.Schedule(cap, 128, pre);

                var integ = new IntegrateJob
                {
                    P = P, dt = dt, alive = s.alive, velNext = s.velNext, sepDisp = s.sepDisp, compAxis = s.compAxis,
                    rClose = s.rClose, rOpen = s.rOpen, rShear = s.rShear, rSub = s.rSub, rOver = s.rOver,
                    rCrumple = s.rCrumple,
                    pos = s.pos, vel = s.vel, faultDir = s.faultDir,
                    thickness = s.thickness, age = s.age, density = s.density, stiffness = s.stiffness,
                    orogeny = s.orogeny, extension = s.extension, arc = s.arc, shear = s.shear,
                    faultActivity = s.faultActivity, faultType = s.faultType
                }.Schedule(cap, 256, inter);

                var fb = new ConvectionFeedbackJob
                {
                    P = P, dt = dt, step = step, conv = s.conv, alive = s.alive, removeFlag = s.removeFlag,
                    pos = s.pos, thickness = s.thickness, nearest = s.nearest,
                    sumCont = s.convSumCont, sumSub = s.convSumSub,
                    total = s.convTotal, contCnt = s.convCont, subCnt = s.convSub
                }.Schedule(integ);

                h = new StructureJob
                {
                    P = P, alive = s.alive, faultType = s.faultType,
                    removeFlag = s.removeFlag, spawnPartner = s.spawnPartner, spawnPos = s.spawnPos,
                    freeList = s.freeList, counters = s.counters,
                    spawnHash = s.spawnHash, spawnSlots = s.spawnSlots,
                    pos = s.pos, vel = s.vel, faultDir = s.faultDir, slab = s.slab,
                    thickness = s.thickness, age = s.age, density = s.density, stiffness = s.stiffness,
                    orogeny = s.orogeny, extension = s.extension, arc = s.arc, shear = s.shear,
                    hotspot = s.hotspot, faultActivity = s.faultActivity
                }.Schedule(fb);

                // optional remesh onto the original lattice
                if (remeshIntervalMyr > 0f)
                {
                    int every = Mathf.Max(1, Mathf.RoundToInt(remeshIntervalMyr / dt));
                    int done = (int)step + 1;
                    if (done - _lastRemeshStep >= every)
                    {
                        _lastRemeshStep = done;
                        RemeshCount++;
                        h = ScheduleRemesh(h);
                    }
                }
            }
            return h;
        }

        JobHandle ScheduleRemesh(JobHandle dep)
        {
            var s = _state;
            var P = _P;
            int M = s.LatticeCount;

            var grid = new BuildGridJob
            {
                P = P, pos = s.pos, alive = s.alive, cellStart = s.cellStart, cursor = s.cellCursor,
                cellItems = s.cellItems, cellOf = s.cellOf
            }.Schedule(dep);

            var gather = new RemeshGatherJob
            {
                P = P, lattice = s.lattice, cellStart = s.cellStart, cellItems = s.cellItems,
                pos = s.pos, vel = s.vel, faultDir = s.faultDir, slab = s.slab,
                thickness = s.thickness, age = s.age, density = s.density, stiffness = s.stiffness,
                orogeny = s.orogeny, extension = s.extension, arc = s.arc, shear = s.shear,
                hotspot = s.hotspot, faultActivity = s.faultActivity, faultType = s.faultType,
                oPos = s.tPos, oVel = s.tVel, oFaultDir = s.tFaultDir, oSlab = s.tSlab,
                oThickness = s.tThickness, oAge = s.tAge, oDensity = s.tDensity, oStiffness = s.tStiffness,
                oOrogeny = s.tOrogeny, oExtension = s.tExtension, oArc = s.tArc, oShear = s.tShear,
                oHotspot = s.tHotspot, oFaultActivity = s.tFaultActivity, oFaultType = s.tFaultType
            }.Schedule(M, 256, grid);

            var apply = new RemeshApplyJob
            {
                count = M,
                tPos = s.tPos, tVel = s.tVel, tFaultDir = s.tFaultDir, tSlab = s.tSlab,
                tThickness = s.tThickness, tAge = s.tAge, tDensity = s.tDensity, tStiffness = s.tStiffness,
                tOrogeny = s.tOrogeny, tExtension = s.tExtension, tArc = s.tArc, tShear = s.tShear,
                tHotspot = s.tHotspot, tFaultActivity = s.tFaultActivity, tFaultType = s.tFaultType,
                alive = s.alive, removeFlag = s.removeFlag, faultType = s.faultType, spawnPartner = s.spawnPartner,
                pos = s.pos, vel = s.vel, faultDir = s.faultDir, slab = s.slab,
                thickness = s.thickness, age = s.age, density = s.density, stiffness = s.stiffness,
                orogeny = s.orogeny, extension = s.extension, arc = s.arc, shear = s.shear,
                hotspot = s.hotspot, faultActivity = s.faultActivity
            }.Schedule(s.Capacity, 512, gather);

            return new RemeshFinalizeJob { count = M, freeList = s.freeList, counters = s.counters }.Schedule(apply);
        }

        JobHandle SchedulePreview(JobHandle dependency)
        {
            _previewDirty = false;
            if (_state == null || _mesh == null || !_verts.IsCreated) return dependency;
            var s = _state;
            _previewPending = true;
            return new PreviewMeshJob
            {
                P = _P, view = (int)debugView, radius = planetRadiusKm,
                halfSize = previewPointSize * _spacing * planetRadiusKm,
                alive = s.alive, faultType = s.faultType, pos = s.pos, vel = s.vel,
                thickness = s.thickness, age = s.age, faultActivity = s.faultActivity,
                orogeny = s.orogeny, hotspot = s.hotspot, crowding = s.crowding,
                verts = _verts, colors = _colors
            }.Schedule(s.Capacity, 512, dependency);
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
            PublishPoints();      // completes
            _finishPending = false;
            LogRun("generated");
#if UNITY_EDITOR
            if (!Application.isPlaying) UnityEditor.SceneView.RepaintAll();
#endif
        }

        /// <summary>Completes in-flight work and syncs managed-side caches. Returns true if the mesh was updated.</summary>
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
                    uploaded = true;
                }
            }

            if (_state != null)
            {
                AliveCount = _state.counters[0];
                SpawnedCount = _state.counters[2];
                SubductedCount = _state.counters[3];
                ConvectionPoints = _state.conv.ToArray();

                bool gizmosNeedPoints = drawGizmos && (!PreviewVisible || drawFaultDirections);
                if (gizmosNeedPoints && ++_batchesSinceSnapshot >= gizmoSnapshotInterval)
                {
                    _batchesSinceSnapshot = 0;
                    FillSnapshot();
                }
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
            Debug.Log($"[Tectonics] {verb}: {_stepIndex} steps @ {_dt:F2} Myr ({CurrentTimeMyr:F0} Myr), " +
                      $"{meshPointCount} -> {AliveCount} points (+{SpawnedCount} spawned, -{SubductedCount} subducted), " +
                      $"{ms} ms wall time");
        }

        // ================================================================== snapshot
        void FillSnapshot()
        {
            var s = _state;
            if (s == null) return;
            if (_live == null || _live.Length != s.Capacity) _live = new PlanetPoint[s.Capacity];
            int n = 0;
            for (int i = 0; i < s.Capacity; i++)
            {
                if (s.alive[i] == 0) continue;
                float t = s.thickness[i];
                _live[n++] = new PlanetPoint
                {
                    position = s.pos[i], velocity = s.vel[i],
                    thickness = t, density = s.density[i], age = s.age[i], stiffness = s.stiffness[i],
                    faultType = (FaultType)s.faultType[i], faultActivity = s.faultActivity[i],
                    faultDir = s.faultDir[i],
                    orogeny = s.orogeny[i], extension = s.extension[i], arc = s.arc[i],
                    shear = s.shear[i], hotspot = s.hotspot[i], crowding = s.crowding[i],
                    elevationKm = TMath.ElevationKm(t, s.age[i], _P)
                };
            }
            _liveCount = n;
        }

        public List<FaultSample> ExtractFaultSamples(float minActivity = 0.12f)
        {
            var result = new List<FaultSample>();
            foreach (var p in Points)
            {
                if (p.faultType == FaultType.None || p.faultActivity < minActivity) continue;
                result.Add(new FaultSample
                {
                    position = p.position, tangent = p.faultDir, type = p.faultType,
                    activity = p.faultActivity, thickness = p.thickness
                });
            }
            return result;
        }

        // ================================================================== preview mesh
        bool PreviewVisible => showPreviewMesh && _previewGO != null;

        void SyncPreviewObject()
        {
            bool want = showPreviewMesh && _state != null;
            if (!want) { if (_previewGO != null) DestroyPreview(); return; }

            int vc = _state.Capacity * 4;
            if (_previewGO == null || _mesh == null || !_verts.IsCreated || _verts.Length != vc)
            {
                BuildPreview(_state.Capacity);
                _previewDirty = true;
            }

            var mat = previewMaterial != null ? previewMaterial : AutoMaterial();
            if (_previewRenderer.sharedMaterial != mat) _previewRenderer.sharedMaterial = mat;

            if (!Mathf.Approximately(_meshBoundsRadius, planetRadiusKm))
            {
                _meshBoundsRadius = planetRadiusKm;
                _mesh.bounds = new Bounds(Vector3.zero, Vector3.one * (planetRadiusKm * 2.2f));
                _previewDirty = true;
            }
        }

        void BuildPreview(int capacity)
        {
            DestroyPreview();

            _verts = new NativeArray<float3>(capacity * 4, Allocator.Persistent);
            _colors = new NativeArray<Color32>(capacity * 4, Allocator.Persistent);

            _mesh = new Mesh { name = "Tectonics Preview", indexFormat = IndexFormat.UInt32, hideFlags = HideFlags.HideAndDontSave };
            _mesh.MarkDynamic();
            _mesh.SetVertices(_verts);
            _mesh.SetColors(_colors);
            var idx = new int[capacity * 6];
            for (int i = 0; i < capacity; i++)
            {
                int b = i * 4, o = i * 6;
                // clockwise when seen from outside the sphere, so backfaces cull
                idx[o] = b; idx[o + 1] = b + 2; idx[o + 2] = b + 1;
                idx[o + 3] = b; idx[o + 4] = b + 3; idx[o + 5] = b + 2;
            }
            _mesh.SetIndices(idx, MeshTopology.Triangles, 0, false);
            _meshBoundsRadius = planetRadiusKm;
            _mesh.bounds = new Bounds(Vector3.zero, Vector3.one * (planetRadiusKm * 2.2f));

            _previewGO = new GameObject("Tectonics Preview") { hideFlags = HideFlags.HideAndDontSave, layer = gameObject.layer };
            _previewGO.transform.SetParent(transform, false);
            _previewGO.AddComponent<MeshFilter>().sharedMesh = _mesh;
            _previewRenderer = _previewGO.AddComponent<MeshRenderer>();
            _previewRenderer.shadowCastingMode = ShadowCastingMode.Off;
            _previewRenderer.receiveShadows = false;
        }

        void DestroyPreview()
        {
            if (_jobsInFlight) { _handle.Complete(); _jobsInFlight = false; }  // preview job may be writing _verts
            _previewPending = false;
            SafeDestroy(_previewGO); _previewGO = null; _previewRenderer = null;
            SafeDestroy(_mesh); _mesh = null;
            if (_verts.IsCreated) _verts.Dispose();
            if (_colors.IsCreated) _colors.Dispose();
        }

        Material AutoMaterial()
        {
            if (_autoMaterial != null) return _autoMaterial;
            var sh = Shader.Find("PlanetGen/TectonicPreview");
            if (sh == null) sh = Shader.Find("Sprites/Default");   // vertex-coloured fallback
            _autoMaterial = new Material(sh) { name = "Tectonics Preview (auto)", hideFlags = HideFlags.HideAndDontSave };
            return _autoMaterial;
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

        Color ColorFor(in PlanetPoint p)
        {
            switch (debugView)
            {
                case TectonicDebugView.Thickness: return Color.Lerp(Color.blue, Color.white, p.thickness / maxThicknessKm);
                case TectonicDebugView.Age: return Color.Lerp(Color.yellow, new Color(0.2f, 0f, 0.4f), Mathf.Clamp01(p.age / 300f));
                case TectonicDebugView.FaultType:
                    return Color.Lerp(FaultColors[0], FaultColors[(int)p.faultType], Mathf.Clamp01(p.faultActivity * 3f));
                case TectonicDebugView.Orogeny: return Color.Lerp(Color.black, Color.white, Mathf.Clamp01(p.orogeny));
                case TectonicDebugView.Hotspot: return Color.Lerp(Color.black, Color.red, Mathf.Clamp01(p.hotspot * 0.5f));
                case TectonicDebugView.Velocity:
                {
                    Vector3 n = p.position;
                    Vector3 up = Mathf.Abs(n.y) < 0.99f ? Vector3.up : Vector3.right;
                    Vector3 t = Vector3.Cross(up, n).normalized;
                    Vector3 b = Vector3.Cross(n, t);
                    float h = Mathf.Atan2(Vector3.Dot(p.velocity, b), Vector3.Dot(p.velocity, t)) / (2f * Mathf.PI) + 1f;
                    float sp = Mathf.Clamp01(p.velocity.magnitude / velocityReference);
                    return Color.HSVToRGB(h % 1f, 0.85f, 0.25f + 0.75f * sp);
                }
                case TectonicDebugView.Crowding:
                {
                    float4 c = PreviewMeshJob.CrowdingColor(p.crowding);
                    return new Color(c.x, c.y, c.z);
                }
                default:
                    float e = p.elevationKm;
                    if (e < 0f) return Color.Lerp(new Color(0f, 0.05f, 0.35f), new Color(0.2f, 0.55f, 0.85f), Mathf.Clamp01((e + 5f) / 5f));
                    return Color.Lerp(new Color(0.25f, 0.6f, 0.2f), Color.white, Mathf.Clamp01(e / 5f));
            }
        }

        void OnDrawGizmos()
        {
            if (!drawGizmos) return;
            Gizmos.matrix = transform.localToWorldMatrix;
            float r = planetRadiusKm;

            // live snapshot while a simulation exists, published Points otherwise
            PlanetPoint[] pts = _liveCount > 0 && _live != null ? _live : Points;
            int count = pts == _live ? _liveCount : (Points?.Length ?? 0);

            bool drawCubes = !PreviewVisible;
            if (pts != null && (drawCubes || drawFaultDirections))
            {
                Camera cam = Camera.current;
                float size = MeanSpacing * r * 0.45f * gizmoPointSize;
                for (int i = 0; i < count; i += gizmoStride)
                {
                    var p = pts[i];
                    if (hideBackside && cam != null)
                    {
                        Vector3 worldPosition = transform.TransformPoint(p.position * r);
                        Vector3 normal = transform.TransformDirection(p.position);
                        Vector3 toCamera = cam.orthographic
                            ? -cam.transform.forward
                            : cam.transform.position - worldPosition;
                        if (Vector3.Dot(normal, toCamera) <= 0f) continue;
                    }
                    if (drawCubes)
                    {
                        Gizmos.color = ColorFor(p);
                        Gizmos.DrawCube(p.position * r, Vector3.one * size);
                    }
                    if (drawFaultDirections && p.faultType != FaultType.None && p.faultActivity > faultThreshold)
                    {
                        Gizmos.color = FaultColors[(int)p.faultType];
                        // lift slightly so rays aren't buried in the preview quads
                        Gizmos.DrawRay(p.position * (r * 1.002f), p.faultDir * (MeanSpacing * r * 0.8f));
                    }
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
