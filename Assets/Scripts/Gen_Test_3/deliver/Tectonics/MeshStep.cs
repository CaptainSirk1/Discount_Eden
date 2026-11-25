using System;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Mathematics;

namespace PlanetGen.Tectonics.MeshSim
{
    /// <summary>
    /// Every tunable of the mesh simulation, in inspector units. PlanetTectonicsMesh exposes the same names as
    /// serialized fields and initialises them from <see cref="D"/>, so the component, the harness and the docs share
    /// one set of defaults. Plain data (no UnityEngine dependency).
    /// </summary>
    [Serializable]
    public struct MeshSettings
    {
        // seed & resolution
        public int seed, targetVertexCount, convectionPointCount, convectionEvenness;
        public float capacityMultiplier, meshJitter, planetRadiusKm;
        // initial crust
        public bool uniformInitialCrust;
        public float initialCrustThicknessKm, initialCrustAgeMyr, initialThicknessVariationKm, continentCoverage, continentNoiseScale;
        public float oceanThicknessKm, continentThicknessKm;
        // planetary cooling
        public bool enablePlanetaryCooling;
        public float planetaryCoolingTimescaleMyr, maxGlobalThermalAgeMyr, thermalStrengthRefMyr;
        // time
        public float totalTimeMyr, stepMyr, maxStepFraction;
        // mesh rules (x target edge)
        public float maxEdgeLength, minEdgeLength, compressionMergeLength, sliverAltitude, relaxation;
        public int maxValence;
        // compression
        public float barrierStart, barrierStiffness, barrierAgeFactor, gravitySpreading, gravityLock, gravityLockMarginKm, maxThicknessKm;
        // crustal strength solve
        public bool enableStrengthSolve;
        public float plateCoherenceRad, oceanicCoherenceFactor, strengthSolveTolerance, compressiveStressWeight;
        public float continentalRigidity, rigidBlockDamageLimit;
        public int rigidBlockMinVertices;
        public int strengthSolveIterations;
        // extension & damage
        public bool enableExtensionResistance;
        public float extensionStiffness, extensionAgeFactor, extensionThicknessFactor, fracturedCrustResistance;
        public float damageNucleation;
        public float extensionYield, extensionDamageRate, damageWeakening, damageHealThreshold, extensionDamageHealing, riftDamageThreshold;
        // material budget
        public bool enableSubductionSink, enableRifting, enableMagmatism;
        public float initialFelsicFraction, felsicBuoyantColumnKm;
        public float delaminationRatePerMyr;
        public float riftCrustThicknessKm, minimumCrustKm, erosionDiffusivityKm2PerMyr, erosionBaseKm, erosionRecycleFraction;
        public float magmaticReturnFraction, magmaticYouthBoost, magmaReleaseMyr, magmaticFelsicFraction, felsicSubductionFraction;
        // convection
        public float convectionStrength, convectionRadiusScale, plumeThreshold, plumeRadius, convectionDrift, temperatureNoise;
        public float insulationHeat, insulationOnset, surfaceCooling, subductionCool, feedbackRelaxPerMyr, convectionVigor, convectionYouthVigor;
        // resistance & coupling
        public float ageResistance, ageSaturationMyr, velocityInertiaMyr, plateCoupling, slabPull;
        public int plateRigidityPasses, constraintPasses;
        // subduction / divergence
        public float subductThicknessDiffKm, subductDensityDiff, riftOpenThreshold;
        // accumulators
        public float overridingThickening, orogenyGain, orogenyDecayMyr, arcGain, arcDecayMyr;
        // faults
        public float velocityReference, faultThreshold, faultMemoryMyr, transformDominance;
        // debug
        public float eventFlashMyr;

        /// <summary>The recommended defaults (see TECTONICS_REVISION_2.md for the reasoning behind each).</summary>
        public static readonly MeshSettings D = new MeshSettings
        {
            seed = 12345, targetVertexCount = 20000, convectionPointCount = 14, convectionEvenness = 6,
            capacityMultiplier = 2f, meshJitter = 0.15f, planetRadiusKm = 10f,

            uniformInitialCrust = true, initialCrustThicknessKm = 25f, initialCrustAgeMyr = 0f, initialThicknessVariationKm = 0.5f,
            continentCoverage = 0.3f, continentNoiseScale = 1.3f, oceanThicknessKm = 7f, continentThicknessKm = 35f,

            enablePlanetaryCooling = true, planetaryCoolingTimescaleMyr = 1500f, maxGlobalThermalAgeMyr = 1000f, thermalStrengthRefMyr = 400f,

            totalTimeMyr = 4000f, stepMyr = 0f, maxStepFraction = 0.2f,

            maxEdgeLength = 1.6f, minEdgeLength = 0.45f, compressionMergeLength = 0.7f, sliverAltitude = 0.2f, relaxation = 0f, maxValence = 12,

            barrierStart = 0.9f, barrierStiffness = 10f, barrierAgeFactor = 1.5f, gravitySpreading = 0.00004f, gravityLock = 1f,
            gravityLockMarginKm = 8f, maxThicknessKm = 75f,

            enableStrengthSolve = true, plateCoherenceRad = 1f, oceanicCoherenceFactor = 0.3f, strengthSolveTolerance = 0.02f, strengthSolveIterations = 40,
            compressiveStressWeight = 0.5f, continentalRigidity = 0.95f, rigidBlockDamageLimit = 0.3f, rigidBlockMinVertices = 12,
            enableExtensionResistance = true, extensionStiffness = 0.9f, extensionAgeFactor = 1.5f, extensionThicknessFactor = 2.5f,
            fracturedCrustResistance = 0.05f,
            damageNucleation = 0.1f,
            extensionYield = 0.15f, extensionDamageRate = 0.2f, damageWeakening = 0.8f, damageHealThreshold = 0.5f,
            extensionDamageHealing = 0.005f, riftDamageThreshold = 0.5f,

            enableSubductionSink = true, enableRifting = true, enableMagmatism = true,
            initialFelsicFraction = 0f, felsicBuoyantColumnKm = 15f,
            delaminationRatePerMyr = 0.02f,
            riftCrustThicknessKm = 7f, minimumCrustKm = 3.5f, erosionDiffusivityKm2PerMyr = 50f, erosionBaseKm = 35f, erosionRecycleFraction = 0f,
            magmaticReturnFraction = 0.15f, magmaticYouthBoost = 1f, magmaReleaseMyr = 20f, magmaticFelsicFraction = 0.5f, felsicSubductionFraction = 0.15f,

            convectionStrength = 0.003f, convectionRadiusScale = 1.3f, plumeThreshold = 0.7f, plumeRadius = 0.3f, convectionDrift = 0.0002f,
            temperatureNoise = 0.005f, insulationHeat = 0.012f, insulationOnset = 0.5f, surfaceCooling = 0.008f, subductionCool = 0.5f,
            feedbackRelaxPerMyr = 0.005f, convectionVigor = 0.45f, convectionYouthVigor = 0.25f,

            ageResistance = 0.5f, ageSaturationMyr = 300f, velocityInertiaMyr = 20f, plateCoupling = 0.9f, slabPull = 0.002f,
            plateRigidityPasses = 0, constraintPasses = 6,

            subductThicknessDiffKm = 12f, subductDensityDiff = 0.2f, riftOpenThreshold = 0.1f,

            overridingThickening = 0.35f, orogenyGain = 0.01f, orogenyDecayMyr = 300f, arcGain = 0.01f, arcDecayMyr = 150f,

            velocityReference = 0.003f, faultThreshold = 0.12f, faultMemoryMyr = 150f, transformDominance = 1.2f,

            eventFlashMyr = 20f,
        };
    }

    /// <summary>Settings for the starting crust.</summary>
    public struct CrustInit
    {
        public bool uniform;            // true: near-uniform primordial crust; false: noise-seeded continents (testing)
        public float initialThickness;  // km, uniform mode
        public float initialAge;        // Myr, uniform mode
        public float variation;         // km, uniform mode: +- per-vertex random thickness
        public float coverage;          // seeded mode: continental fraction
        public float felsicFraction;    // uniform mode: differentiated fraction of the primordial crust
        public float noiseScale;        // seeded mode
        public int convEvenness;        // Mitchell best-candidate samples per convection cell
    }

    /// <summary>
    /// Builds every job of one simulation step from the state, so the Unity component (which schedules them)
    /// and the headless test harness (which runs them serially) always run the same pipeline.
    /// </summary>
    public static class MeshStep
    {
        /// <summary>The planet's global thermal age at absolute simulated time t: max * (1 - exp(-t / timescale)).
        /// Smooth and asymptotic, and independent of the run's total length.</summary>
        public static float GlobalThermalAge(float timeMyr, bool enabled, float timescaleMyr, float maxMyr)
            => enabled ? maxMyr * (1f - math.exp(-math.max(timeMyr, 0f) / math.max(timescaleMyr, 1e-3f))) : 0f;

        /// <summary>Constraint passes per step (even, for the velNext/velTmp ping-pong).</summary>
        public static int CouplePasses(int constraintPasses, int rigidityPasses)
            => (math.max(2, math.max(constraintPasses, rigidityPasses)) + 1) / 2 * 2;

        /// <summary>Converts inspector settings into job parameters for the step at absolute time timeMyr.
        /// L = target edge (unit chord), dt = step (Myr), convN = convection cells.</summary>
        public static MeshParams BuildParams(in MeshSettings S, float L, float dt, int convN, uint seedHash, float timeMyr)
        {
            float cellAngle = math.acos(1f - 2f / convN);
            float lockMargin = math.max(0.1f, S.gravityLockMarginKm);
            float spacing = L;
            var P = new TectonicParams
            {
                dt = dt, spacing = spacing, interactR = L, seed = seedHash,
                maxSpeed = S.maxStepFraction * L / dt,

                convRadius = cellAngle * S.convectionRadiusScale, convStrength = S.convectionStrength,
                plumeThreshold = S.plumeThreshold, plumeRadius = S.plumeRadius, convDrift = S.convectionDrift,
                convMinSep = 0.5f * cellAngle, tempNoise = S.temperatureNoise,
                insulationHeat = S.insulationHeat, subductionCool = S.subductionCool, feedbackRelax = S.feedbackRelaxPerMyr,

                ageResist = S.ageResistance, ageRef = S.ageSaturationMyr, velTau = S.velocityInertiaMyr,
                couple = S.plateCoupling, slabPull = S.slabPull,

                subductThickness = S.subductThicknessDiffKm, subductDensity = S.subductDensityDiff,
                closeThreshold = 0.03f * S.velocityReference, openThreshold = S.riftOpenThreshold * S.velocityReference,

                overThick = S.overridingThickening,
                orogenyGain = S.orogenyGain, orogenyDecay = S.orogenyDecayMyr,
                maxThickness = S.maxThicknessKm, arcGain = S.arcGain, arcDecay = S.arcDecayMyr,

                faultThreshold = S.faultThreshold, faultMemory = S.faultMemoryMyr,
                velRef = S.velocityReference, transformDominance = S.transformDominance,
                oceanThick = S.oceanThicknessKm, contThick = S.continentThicknessKm,

                lockMargin = lockMargin, lockStart = S.maxThicknessKm - lockMargin, lockStrength = S.gravityLock,
            };
            float lo = math.min(S.minEdgeLength, S.maxEdgeLength * 0.45f);   // splits must never produce collapsible halves
            float G = GlobalThermalAge(timeMyr, S.enablePlanetaryCooling, S.planetaryCoolingTimescaleMyr, S.maxGlobalThermalAgeMyr);
            float r2 = math.max(S.planetRadiusKm, 1f) * math.max(S.planetRadiusKm, 1f);
            // erosion: km^2/Myr -> unit-sphere area / Myr, capped for explicit-diffusion stability
            float erosionK = math.min(S.erosionDiffusivityKm2PerMyr / r2, 0.05f * L * L / math.max(dt, 1e-3f));
            return new MeshParams
            {
                P = P,
                L = L, Lmin = lo * L, Lmax = S.maxEdgeLength * L,
                Lmerge = math.clamp(S.compressionMergeLength, lo, S.maxEdgeLength * 0.49f) * L,
                sliverH = S.sliverAltitude * L,
                minOrient = 1e-3f * L * L,
                flipEps = math.max(1e-3f * L * L, 2e-7f),
                maxValence = S.maxValence,
                relax = S.relaxation,
                flashDecay = math.max(0.1f, S.eventFlashMyr),

                barrierRest = S.barrierStart * L, barrierK = S.barrierStiffness, barrierAge = S.barrierAgeFactor,
                spreading = S.gravitySpreading,

                globalThermal = G,
                thermalRef = math.max(1f, S.thermalStrengthRefMyr),
                globalHeat = S.enablePlanetaryCooling ? math.saturate(1f - G / math.max(S.maxGlobalThermalAgeMyr, 1e-3f)) : 0.5f,

                springOn = S.enableStrengthSolve && S.enableExtensionResistance,
                coherence = math.max(1e-4f, S.plateCoherenceRad),
                compStress = math.max(0f, S.compressiveStressWeight),
                oceanCoherence = math.saturate(S.oceanicCoherenceFactor),
                blockRigidity = math.saturate(S.continentalRigidity),
                blockDamage = math.max(1e-3f, S.rigidBlockDamageLimit),
                blockMinVertices = math.max(1, S.rigidBlockMinVertices),
                extEnabled = S.enableExtensionResistance,
                extMax = S.extensionStiffness, extAge = S.extensionAgeFactor, extThick = S.extensionThicknessFactor,
                extFractured = S.fracturedCrustResistance,

                dmgYield = S.extensionYield, dmgRate = S.extensionDamageRate, dmgWeaken = math.saturate(S.damageWeakening), dmgSeed = math.max(1e-3f, S.damageNucleation),
                dmgHealFrac = S.damageHealThreshold, dmgHeal = S.extensionDamageHealing,
                riftDamage = S.riftDamageThreshold, riftOpen = S.riftOpenThreshold * S.velocityReference,

                subductionSink = S.enableSubductionSink, rifting = S.enableRifting,
                riftThick = S.riftCrustThicknessKm, minCrust = S.minimumCrustKm, delamRate = math.max(0f, S.delaminationRatePerMyr),
                erosionK = erosionK, erosionBase = S.erosionBaseKm, erosionRecycle = math.saturate(S.erosionRecycleFraction),
                magmatism = S.enableMagmatism, magmaReturn = S.magmaticReturnFraction, magmaYouth = S.magmaticYouthBoost,
                magmaTau = math.max(S.magmaReleaseMyr, 0.1f), magmaFelsic = math.saturate(S.magmaticFelsicFraction),
                felsicSubduct = math.saturate(S.felsicSubductionFraction),

                felsicColumn = math.max(0f, S.felsicBuoyantColumnKm),
                surfaceCool = S.surfaceCooling, insulationOnset = math.saturate(S.insulationOnset),
                vigor = math.max(0f, S.convectionVigor + S.convectionYouthVigor *
                        (S.enablePlanetaryCooling ? math.saturate(1f - G / math.max(S.maxGlobalThermalAgeMyr, 1e-3f)) : 0.5f)),
            };
        }

        // ================================================================== job builders
        public static MeshDriveJob Drive(SphereMeshState s, in MeshParams M, float dt) => new MeshDriveJob
        {
            M = M, dt = dt, conv = s.conv, alive = s.alive, pos = s.pos, slab = s.slab,
            density = s.density, age = s.age, driven = s.driven, nearest = s.nearest, hotspot = s.hotspot
        };

        public static MeshVelocityJob Velocity(SphereMeshState s, in MeshParams M, float dt) => new MeshVelocityJob
        {
            M = M, dt = dt, V = s.V, O = s.O, vc = s.vc, alive = s.alive,
            pos = s.pos, vel = s.vel, driven = s.driven,
            thickness = s.thickness, cfrac = s.cfrac, density = s.density, stiffness = s.stiffness, age = s.age,
            velNext = s.velNext, compAxis = s.compAxis, slab = s.slab, spreadVel = s.spreadVel,
            rClose = s.rClose, rOpen = s.rOpen, rShear = s.rShear, rSub = s.rSub, rOver = s.rOver,
            rCrumple = s.rCrumple, subFrac = s.subFrac
        };

        public static MeshCoupleJob Couple(SphereMeshState s, in MeshParams M, int pass, int rigidityPasses)
        {
            bool ping = (pass & 1) == 0;
            return new MeshCoupleJob
            {
                M = M, average = pass < rigidityPasses, measure = pass == 0 && !M.springOn,
                V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos,
                velIn = ping ? s.velNext : s.velTmp, velOut = ping ? s.velTmp : s.velNext,
                thickness = s.thickness, cfrac = s.cfrac, density = s.density, stiffness = s.stiffness, age = s.age, damage = s.damage,
                tension = s.tension
            };
        }

        public static MeshBlockJob Blocks(SphereMeshState s, in MeshParams M) => new MeshBlockJob
        {
            M = M, V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos, area = s.area, cfrac = s.cfrac, damage = s.damage,
            kc = s.kc, velNext = s.velNext, parent = s.blockParent, blockId = s.blockId,
            blockL = s.blockL, blockI0 = s.blockI0, blockI1 = s.blockI1, blockI2 = s.blockI2, blockA = s.blockA
        };

        public static MeshMoveJob Move(SphereMeshState s, in MeshParams M, float dt) => new MeshMoveJob
        {
            dt = dt, velTau = M.P.velTau, maxSpeed = M.P.maxSpeed,
            alive = s.alive, spreadVel = s.spreadVel, velNext = s.velNext, pos = s.pos, vel = s.vel
        };

        public static MeshAreaJob Area(SphereMeshState s) => new MeshAreaJob
        { V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos, areaNew = s.areaNew };

        public static MeshIntegrateJob Integrate(SphereMeshState s, in MeshParams M, float dt) => new MeshIntegrateJob
        {
            M = M, dt = dt, V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos, vel = s.vel, compAxis = s.compAxis,
            areaNew = s.areaNew, rClose = s.rClose, rOpen = s.rOpen, rShear = s.rShear, rSub = s.rSub, rOver = s.rOver,
            rCrumple = s.rCrumple, subFrac = s.subFrac, tension = s.tension,
            volume = s.volume, felsic = s.felsic, area = s.area, thickness = s.thickness, cfrac = s.cfrac, age = s.age, density = s.density, stiffness = s.stiffness,
            orogeny = s.orogeny, extension = s.extension, arc = s.arc, shear = s.shear, faultActivity = s.faultActivity,
            damage = s.damage, flashSplit = s.flashSplit, flashCollapse = s.flashCollapse, magma = s.magma,
            achieved = s.achieved, arcW = s.arcW, sinkSub = s.sinkSub, srcFloor = s.srcFloor, sinkFelsic = s.sinkFelsic,
            sinkDelam = s.sinkDelam, sinkDelamFelsic = s.sinkDelamFelsic, accrete = s.accrete,
            faultType = s.faultType, faultDir = s.faultDir
        };

        public static MeshErosionJob Erosion(SphereMeshState s, in MeshParams M, float dt) => new MeshErosionJob
        {
            M = M, dt = dt, V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos,
            volume = s.volume, felsic = s.felsic, area = s.area, thickness = s.thickness, cfrac = s.cfrac,
            volumeTmp = s.volumeTmp, felsicTmp = s.felsicTmp, sinkErosion = s.sinkErosion, sinkErosionFelsic = s.sinkErosionFelsic,
            moveErosion = s.moveErosion
        };

        public static MeshErosionApplyJob ErosionApply(SphereMeshState s, in MeshParams M) => new MeshErosionApplyJob
        { M = M, cfrac = s.cfrac, alive = s.alive, volumeTmp = s.volumeTmp, felsicTmp = s.felsicTmp, area = s.area, volume = s.volume, felsic = s.felsic, thickness = s.thickness };

        public static MeshRelaxJob Relax(SphereMeshState s, in MeshParams M) => new MeshRelaxJob
        { relax = M.relax, V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos, posTmp = s.posTmp };

        public static MeshTopologyJob Topology(SphereMeshState s, in MeshParams M, bool relax) => new MeshTopologyJob
        {
            M = M, applyRelax = relax, flipPasses = 4, splitPasses = 4,
            V = s.V, O = s.O, vc = s.vc, freeV = s.freeV, freeT = s.freeT, counters = s.counters,
            nearest = s.nearest, consumedCell = s.consumedCell,
            triAlive = s.triAlive, alive = s.alive, faultType = s.faultType,
            pos = s.pos, posTmp = s.posTmp, vel = s.vel, velNext = s.velNext, faultDir = s.faultDir, slab = s.slab,
            consumedPos = s.consumedPos, consumedVol = s.consumedVol,
            volume = s.volume, felsic = s.felsic, area = s.area, thickness = s.thickness, cfrac = s.cfrac, age = s.age, density = s.density, stiffness = s.stiffness,
            orogeny = s.orogeny, extension = s.extension, arc = s.arc, shear = s.shear, hotspot = s.hotspot,
            faultActivity = s.faultActivity, damage = s.damage, magma = s.magma, tension = s.tension,
            flashSplit = s.flashSplit, flashCollapse = s.flashCollapse, stats = s.stats, ledger = s.ledger, convAccrete = s.convAccrete
        };

        public static MeshSyncJob Sync(SphereMeshState s, in MeshParams M) => new MeshSyncJob
        {
            M = M, V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos,
            volume = s.volume, felsic = s.felsic, age = s.age, faultActivity = s.faultActivity, damage = s.damage,
            area = s.area, thickness = s.thickness, cfrac = s.cfrac, density = s.density, stiffness = s.stiffness
        };

        public static MeshBudgetJob Budget(SphereMeshState s, in MeshParams M, float dt, uint step) => new MeshBudgetJob
        {
            M = M, dt = dt, step = step, conv = s.conv, alive = s.alive, pos = s.pos, area = s.area, arcW = s.arcW,
            nearest = s.nearest, consumedCell = s.consumedCell, consumedPos = s.consumedPos, consumedVol = s.consumedVol,
            counters = s.counters, volume = s.volume, felsic = s.felsic, thickness = s.thickness, cfrac = s.cfrac, density = s.density, age = s.age, magma = s.magma,
            sinkSub = s.sinkSub, srcFloor = s.srcFloor, sinkErosion = s.sinkErosion, moveErosion = s.moveErosion,
            sinkFelsic = s.sinkFelsic, sinkErosionFelsic = s.sinkErosionFelsic, sinkDelam = s.sinkDelam, sinkDelamFelsic = s.sinkDelamFelsic,
            accrete = s.accrete, ledger = s.ledger, pool = s.convPool, accretePool = s.convAccrete,
            sumCont = s.convSumCont, sumSub = s.convSumSub, cellArea = s.convArea, contArea = s.convContArea,
            cellArcW = s.convArcW, subCnt = s.convSub
        };

        /// <summary>One full simulation step, chained after <paramref name="dep"/>. M must already carry this step's
        /// globalThermal / globalHeat.</summary>
        public static JobHandle ScheduleStep(SphereMeshState s, in MeshParams M, float dt, uint step, int passes, int rigidityPasses, JobHandle dep)
            => ScheduleStep(s, M, dt, step, passes, rigidityPasses, 0, 0f, dep);

        public static JobHandle ScheduleStep(SphereMeshState s, in MeshParams M, float dt, uint step, int passes, int rigidityPasses,
            int solveIterations, float solveTolerance, JobHandle dep)
        {
            int cap = s.VCap;
            bool relax = M.relax > 0f;
            JobHandle h = dep;
            h = Drive(s, M, dt).Schedule(cap, 256, h);
            h = Velocity(s, M, dt).Schedule(cap, 128, h);
            if (M.springOn && solveIterations > 0)
            {
                h = MeshSpringSchedule.Schedule(s, M, solveIterations, solveTolerance, h);
                h = Blocks(s, M).Schedule(h);
            }
            for (int p = 0; p < passes; p++) h = Couple(s, M, p, rigidityPasses).Schedule(cap, 256, h);
            h = Move(s, M, dt).Schedule(cap, 256, h);
            h = Area(s).Schedule(cap, 256, h);
            h = Integrate(s, M, dt).Schedule(cap, 256, h);
            if (M.erosionK > 0f)
            {
                h = Erosion(s, M, dt).Schedule(cap, 256, h);
                h = ErosionApply(s, M).Schedule(cap, 512, h);
            }
            if (relax) h = Relax(s, M).Schedule(cap, 256, h);
            h = Topology(s, M, relax).Schedule(h);
            h = Sync(s, M).Schedule(cap, 256, h);
            h = Budget(s, M, dt, step).Schedule(h);
            return h;
        }

        // ================================================================== initial crust
        /// <summary>Convection cells plus starting crust for every live vertex (thickness, age; volume = thickness x area).</summary>
        public static void InitCrust(SphereMeshState s, in MeshParams M, ref Unity.Mathematics.Random rng, in CrustInit init)
        {
            var P = M.P;
            int N = s.ConvCount;
            for (int k = 0; k < N; k++)
            {
                float3 best = rng.NextFloat3Direction();
                float bestScore = float.MaxValue;
                for (int c = 0; c < init.convEvenness; c++)
                {
                    float3 cand = rng.NextFloat3Direction();
                    float maxDot = -1f;
                    for (int q = 0; q < k; q++) maxDot = math.max(maxDot, math.dot(cand, s.conv[q].pos));
                    if (maxDot < bestScore) { bestScore = maxDot; best = cand; }
                }
                float t = (rng.NextBool() ? 1f : -1f) * rng.NextFloat(0.3f, 1f);
                s.conv[k] = new Convection { pos = best, temp = t, baseTemp = t, feedback = 0f };
            }

            var live = new List<int>();
            for (int i = 0; i < s.VCap; i++) if (s.alive[i] == 1) live.Add(i);
            int count = live.Count;
            float thr = 0f;
            float[] noiseVals = null;
            if (!init.uniform)
            {
                noiseVals = new float[count];
                float3 offset = rng.NextFloat3(-100f, 100f);
                for (int n = 0; n < count; n++)
                {
                    float3 p = s.pos[live[n]];
                    float v = 0f, a = 1f, f = init.noiseScale, norm = 0f;
                    for (int o = 0; o < 4; o++) { v += a * noise.snoise(p * f + offset); norm += a; a *= 0.5f; f *= 2.1f; }
                    noiseVals[n] = v / norm;
                }
                var sorted = (float[])noiseVals.Clone();
                Array.Sort(sorted);
                thr = sorted[math.clamp((int)((1f - init.coverage) * count), 0, count - 1)];
            }

            for (int n = 0; n < count; n++)
            {
                int i = live[n];
                float t, age, fel;
                if (init.uniform)
                {
                    // primordial crust: one thickness everywhere plus subtle per-vertex noise (no large-scale pattern)
                    t = math.max(P.oceanThick * 0.5f, init.initialThickness + init.variation * rng.NextFloat(-1f, 1f));
                    age = math.max(0f, init.initialAge);
                    fel = math.saturate(init.felsicFraction);
                }
                else
                {
                    float sm = math.smoothstep(thr - 0.04f, thr + 0.04f, noiseVals[n]);
                    float contT = P.contThick * (0.9f + 0.3f * rng.NextFloat());
                    t = P.oceanThick + (contT - P.oceanThick) * sm;
                    age = sm > 0.5f ? rng.NextFloat(800f, 2500f) : rng.NextFloat(0f, 150f);
                    fel = sm;   // seeded continents are differentiated crust, seeded ocean floor is not
                }
                float A = MeshGeom.DualArea(s.V, s.O, s.vc, s.pos, i);
                s.area[i] = A; s.thickness[i] = t; s.volume[i] = t * A; s.felsic[i] = t * A * fel; s.age[i] = age;
                float cf = MeshLimits.CrustType(t, t * A, t * A * fel, M);
                s.cfrac[i] = cf;
                s.density[i] = MeshLimits.Density(cf, age);
                s.stiffness[i] = MeshLimits.Stiffness(cf, age, 0f, 0f, M);
                s.damage[i] = 0f;
                s.posTmp[i] = s.pos[i];
                s.faultDir[i] = math.normalizesafe(math.cross(s.pos[i], new float3(0, 1, 0)), new float3(1, 0, 0));
            }
        }
    }

    /// <summary>
    /// One simulation run: mesh, state and time stepping, with no UnityEngine dependency. PlanetTectonicsMesh
    /// schedules its steps on worker threads; the headless harness runs the same ScheduleSteps serially.
    /// </summary>
    public sealed class MeshRun : IDisposable
    {
        public SphereMeshState State { get; private set; }
        public MeshSettings Settings;
        public float Dt { get; private set; }
        public float L { get; private set; }
        public int ConvCount { get; private set; }
        public uint SeedHash { get; private set; }
        public int StepIndex;
        public float TimeMyr => StepIndex * Dt;

        public void Dispose() { State?.Dispose(); State = null; }

        public MeshParams ParamsAt(float timeMyr) => MeshStep.BuildParams(Settings, L, Dt, ConvCount, SeedHash, timeMyr);
        public MeshParams CurrentParams => ParamsAt(TimeMyr);

        public void Begin(in MeshSettings settings)
        {
            Dispose();
            Settings = settings;
            ConvCount = math.clamp(settings.convectionPointCount, 1, 120);
            SeedHash = (uint)settings.seed * 2654435761u + 12345u;

            int freq = Math.Max(2, (int)Math.Round(Math.Sqrt((settings.targetVertexCount - 2) / 10.0)));
            var verts = new List<float3>(); var tris = new List<int>();
            SphereMeshState.BuildGeodesic(freq, verts, tris);
            int nv = verts.Count;

            // target edge = measured mean edge of the starting mesh
            double sum = 0; int ne = 0;
            for (int t = 0; t < tris.Count; t += 3)
                for (int k = 0; k < 3; k++) { sum += math.distance(verts[tris[t + k]], verts[tris[t + (k + 1) % 3]]); ne++; }
            L = (float)(sum / ne);
            Dt = settings.stepMyr > 0f ? settings.stepMyr : math.clamp(settings.maxStepFraction * L / (settings.convectionStrength * 2f), 0.1f, 25f);

            int vCap = Math.Max(nv + 64, (int)(nv * settings.capacityMultiplier));
            State = new SphereMeshState(vCap, vCap * 2, ConvCount);
            var s = State;

            var rng = new Unity.Mathematics.Random(SeedHash | 1u);
            for (int i = 0; i < nv; i++)
            {
                float3 p = verts[i];
                TMath.TangentFrame(p, out float3 t, out float3 bt);
                float a = rng.NextFloat(2f * math.PI);
                verts[i] = math.normalize(p + (t * math.cos(a) + bt * math.sin(a)) * (rng.NextFloat() * settings.meshJitter * L));
            }
            s.Load(verts, tris);

            StepIndex = 0;
            var M = ParamsAt(0f);
            // first topology pass on the bare mesh: jitter makes the geodesic mesh non-Delaunay, so this mostly flips.
            // Run before the crust exists so it cannot move material around.
            MeshStep.Topology(s, M, false).Run();
            var init = new CrustInit
            {
                uniform = settings.uniformInitialCrust, initialThickness = settings.initialCrustThicknessKm,
                initialAge = settings.initialCrustAgeMyr, variation = settings.initialThicknessVariationKm,
                coverage = settings.continentCoverage, noiseScale = settings.continentNoiseScale,
                convEvenness = settings.convectionEvenness, felsicFraction = settings.initialFelsicFraction
            };
            MeshStep.InitCrust(s, M, ref rng, init);
            for (int k = MeshTopo.Splits; k < MeshTopo.Count; k++) if (k != MeshTopo.Inverted) s.counters[k] = 0;
            for (int k = 0; k < MeshLedger.Count; k++) s.ledger[k] = 0;
            s.ledger[MeshLedger.Initial] = s.TotalVolume();
            s.ledger[MeshLedger.FelsicInitial] = s.TotalFelsic();
        }

        /// <summary>Chains <paramref name="count"/> steps starting at StepIndex and advances StepIndex. Global cooling is
        /// evaluated per step from absolute time, so batch size never changes it.</summary>
        public JobHandle ScheduleSteps(int count, JobHandle dep)
        {
            var s = State;
            int passes = MeshStep.CouplePasses(Settings.constraintPasses, Settings.plateRigidityPasses);
            JobHandle h = dep;
            for (int k = 0; k < count; k++)
            {
                uint step = (uint)StepIndex;
                var M = ParamsAt(StepIndex * Dt);
                h = MeshStep.ScheduleStep(s, M, Dt, step, passes, Settings.plateRigidityPasses,
                    Settings.strengthSolveIterations, Settings.strengthSolveTolerance, h);
                StepIndex++;
            }
            return h;
        }

        public MeshDiagnostics Diagnostics() => MeshDiagnostics.Compute(State, CurrentParams, Settings.planetRadiusKm, TimeMyr);
    }
}
