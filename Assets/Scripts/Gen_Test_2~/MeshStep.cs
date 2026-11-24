using System;
using Unity.Mathematics;

namespace PlanetGen.Tectonics.MeshSim
{
    /// <summary>Settings for the starting crust.</summary>
    public struct CrustInit
    {
        public bool uniform;            // true: near-uniform primordial crust; false: noise-seeded continents (testing)
        public float initialThickness;  // km, uniform mode
        public float initialAge;        // Myr, uniform mode
        public float variation;         // km, uniform mode: +- per-vertex random thickness
        public float coverage;          // seeded mode: continental fraction
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

        /// <summary>Constraint passes per step (even, for the velNext/velTmp ping-pong): enough for both the requested
        /// constraint passes and the rigidity averaging passes.</summary>
        public static int CouplePasses(int constraintPasses, int rigidityPasses)
            => (math.max(2, math.max(constraintPasses, rigidityPasses)) + 1) / 2 * 2;

        public static ConvectionDriveJob Drive(SphereMeshState s, in MeshParams M, float dt) => new ConvectionDriveJob
        {
            P = M.P, dt = dt, conv = s.conv, alive = s.alive, pos = s.pos, slab = s.slab,
            density = s.density, age = s.age, driven = s.driven, nearest = s.nearest, hotspot = s.hotspot
        };

        public static MeshVelocityJob Velocity(SphereMeshState s, in MeshParams M, float dt) => new MeshVelocityJob
        {
            M = M, dt = dt, V = s.V, O = s.O, vc = s.vc, alive = s.alive,
            pos = s.pos, vel = s.vel, driven = s.driven,
            thickness = s.thickness, density = s.density, stiffness = s.stiffness, age = s.age,
            velNext = s.velNext, compAxis = s.compAxis, slab = s.slab, spreadVel = s.spreadVel,
            rClose = s.rClose, rOpen = s.rOpen, rShear = s.rShear, rSub = s.rSub, rOver = s.rOver,
            rCrumple = s.rCrumple
        };

        public static MeshAreaJob Area(SphereMeshState s, in MeshParams M) => new MeshAreaJob
        {
            M = M, V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos, velNext = s.velNext,
            thickness = s.thickness, density = s.density, areaOld = s.areaOld, areaFull = s.areaFull, areaMask = s.areaMask
        };

        public static MeshCoupleJob Couple(SphereMeshState s, in MeshParams M, int pass, int rigidityPasses)
        {
            bool ping = (pass & 1) == 0;
            return new MeshCoupleJob
            {
                M = M, average = pass < rigidityPasses, measure = pass == 0,
                V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos, driven = s.driven,
                velIn = ping ? s.velNext : s.velTmp, velOut = ping ? s.velTmp : s.velNext,
                thickness = s.thickness, density = s.density, stiffness = s.stiffness, age = s.age, damage = s.damage,
                tension = s.tension
            };
        }

        public static MeshMoveJob Move(SphereMeshState s, in MeshParams M, float dt) => new MeshMoveJob
        {
            dt = dt, velTau = M.P.velTau, maxSpeed = M.P.maxSpeed,
            alive = s.alive, spreadVel = s.spreadVel, velNext = s.velNext, pos = s.pos, vel = s.vel
        };

        public static MeshIntegrateJob Integrate(SphereMeshState s, in MeshParams M, float dt) => new MeshIntegrateJob
        {
            M = M, dt = dt, V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos, compAxis = s.compAxis,
            areaOld = s.areaOld, areaMask = s.areaMask, tension = s.tension,
            rClose = s.rClose, rOpen = s.rOpen, rShear = s.rShear, rSub = s.rSub, rOver = s.rOver, rCrumple = s.rCrumple,
            thickness = s.thickness, age = s.age, density = s.density, stiffness = s.stiffness,
            orogeny = s.orogeny, extension = s.extension, arc = s.arc, shear = s.shear,
            faultActivity = s.faultActivity, damage = s.damage, flashSplit = s.flashSplit, flashCollapse = s.flashCollapse,
            faultType = s.faultType, faultDir = s.faultDir
        };

        public static MeshRelaxJob Relax(SphereMeshState s, in MeshParams M) => new MeshRelaxJob
        { relax = M.relax, V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos, posTmp = s.posTmp };

        public static MeshTopologyJob Topology(SphereMeshState s, in MeshParams M, bool relax) => new MeshTopologyJob
        {
            M = M, applyRelax = relax, flipPasses = 4, splitPasses = 4,
            V = s.V, O = s.O, vc = s.vc, freeV = s.freeV, freeT = s.freeT, counters = s.counters,
            nearest = s.nearest, consumedCell = s.consumedCell,
            triAlive = s.triAlive, alive = s.alive, faultType = s.faultType,
            pos = s.pos, posTmp = s.posTmp, vel = s.vel, velNext = s.velNext, faultDir = s.faultDir, slab = s.slab,
            consumedPos = s.consumedPos,
            thickness = s.thickness, age = s.age, density = s.density, stiffness = s.stiffness,
            orogeny = s.orogeny, extension = s.extension, arc = s.arc, shear = s.shear, hotspot = s.hotspot,
            faultActivity = s.faultActivity, damage = s.damage,
            flashSplit = s.flashSplit, flashCollapse = s.flashCollapse, stats = s.stats, areaFull = s.areaFull
        };

        public static MeshFeedbackJob Feedback(SphereMeshState s, in MeshParams M, float dt, uint step) => new MeshFeedbackJob
        {
            P = M.P, dt = dt, surfaceCool = M.surfaceCool, step = step, conv = s.conv, alive = s.alive, pos = s.pos,
            thickness = s.thickness, areaFull = s.areaFull, nearest = s.nearest,
            consumedCell = s.consumedCell, consumedPos = s.consumedPos, counters = s.counters,
            sumCont = s.convSumCont, sumSub = s.convSumSub, cellArea = s.convArea, contArea = s.convContArea, subCnt = s.convSub
        };

        // ================================================================== initial crust
        /// <summary>Convection cells plus starting crust for the first <paramref name="count"/> vertices.</summary>
        public static void InitCrust(SphereMeshState s, in MeshParams M, int count, ref Unity.Mathematics.Random rng, in CrustInit init)
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

            float thr = 0f;
            float[] noiseVals = null;
            if (!init.uniform)
            {
                noiseVals = new float[count];
                float3 offset = rng.NextFloat3(-100f, 100f);
                for (int i = 0; i < count; i++)
                {
                    float3 p = s.pos[i];
                    float n = 0f, a = 1f, f = init.noiseScale, norm = 0f;
                    for (int o = 0; o < 4; o++) { n += a * noise.snoise(p * f + offset); norm += a; a *= 0.5f; f *= 2.1f; }
                    noiseVals[i] = n / norm;
                }
                var sorted = (float[])noiseVals.Clone();
                Array.Sort(sorted);
                thr = sorted[math.clamp((int)((1f - init.coverage) * count), 0, count - 1)];
            }

            for (int i = 0; i < count; i++)
            {
                float t, age;
                if (init.uniform)
                {
                    // primordial crust: one thickness everywhere plus subtle per-vertex noise (no large-scale pattern)
                    t = math.max(P.oceanThick * 0.5f, init.initialThickness + init.variation * rng.NextFloat(-1f, 1f));
                    age = math.max(0f, init.initialAge);
                }
                else
                {
                    float sm = math.smoothstep(thr - 0.04f, thr + 0.04f, noiseVals[i]);
                    float contT = P.contThick * (0.9f + 0.3f * rng.NextFloat());
                    t = P.oceanThick + (contT - P.oceanThick) * sm;
                    age = sm > 0.5f ? rng.NextFloat(800f, 2500f) : rng.NextFloat(0f, 150f);
                }
                s.thickness[i] = t; s.age[i] = age;
                s.density[i] = TMath.Density(t, age, P);
                s.stiffness[i] = MeshLimits.Stiffness(t, age, 0f, 0f, M);
                s.damage[i] = 0f;
                s.posTmp[i] = s.pos[i];
                s.faultDir[i] = math.normalizesafe(math.cross(s.pos[i], new float3(0, 1, 0)), new float3(1, 0, 0));
            }
        }
    }

    /// <summary>
    /// Area-weighted planet-wide measurements, for telling mechanical behaviour apart from numerical material loss.
    /// Uses each vertex's dual area (areaFull, one third of its incident triangles) from the last step. O(V); run
    /// periodically, not every step. Volumes are in km^3 (thickness km x area km^2).
    /// </summary>
    public struct MeshDiagnostics
    {
        public float globalThermalAge;     // Myr
        public float continentalAreaFrac;  // fraction of the surface with thickness > 0.75 x continental
        public float crustVolume;          // km^3, all crust
        public float continentalVolume;    // km^3, crust above the continental threshold
        public float meanDamage, maxDamage;        // area-weighted mean over the whole planet; max
        public float continentalMeanDamage;        // area-weighted mean over continental crust
        public float continentalFracturedFrac;     // fraction of continental area with damage >= riftDamage
        public float meanTension;                  // area-weighted

        public static MeshDiagnostics Compute(SphereMeshState s, in MeshParams M, float radiusKm)
        {
            var d = new MeshDiagnostics { globalThermalAge = M.globalThermal };
            float contLimit = 0.75f * M.P.contThick;
            double A = 0, Ac = 0, vol = 0, volC = 0, dmg = 0, dmgC = 0, fracC = 0, ten = 0;
            float maxD = 0f;
            for (int i = 0; i < s.VCap; i++)
            {
                if (s.alive[i] == 0) continue;
                double a = s.areaFull[i];
                float t = s.thickness[i], D = s.damage[i];
                A += a; vol += t * a; dmg += D * a; ten += s.tension[i] * a;
                maxD = math.max(maxD, D);
                if (t > contLimit)
                {
                    Ac += a; volC += t * a; dmgC += D * a;
                    if (D >= M.riftDamage) fracC += a;
                }
            }
            double r2 = (double)radiusKm * radiusKm;
            if (A > 0)
            {
                d.continentalAreaFrac = (float)(Ac / A);
                d.meanDamage = (float)(dmg / A);
                d.meanTension = (float)(ten / A);
            }
            if (Ac > 0) { d.continentalMeanDamage = (float)(dmgC / Ac); d.continentalFracturedFrac = (float)(fracC / Ac); }
            d.crustVolume = (float)(vol * r2);
            d.continentalVolume = (float)(volC * r2);
            d.maxDamage = maxD;
            return d;
        }
    }
}
