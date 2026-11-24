using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace PlanetGen.Tectonics.MeshSim
{
    // ---------------------------------------------------------------------------------
    // Step pipeline (chained by MeshStep.Schedule; the Unity component and the headless harness run the same chain):
    //   MeshDrive        convection drive in each vertex's tangent plane, nearest cell, plume exposure
    //   MeshVelocity     one-ring: target velocity = drive blended with neighbours; spreading; boundary rates
    //   MeshCouple x N   (>= 2) rigidity averaging, compression + extension limits; pass 0 measures stress
    //   MeshMove         velocity relaxes toward the limited target with inertia, then advects
    //   MeshArea         Voronoi dual areas of the moved vertices (areaNew)
    //   MeshIntegrate    volume is carried: thickness = volume / area; slab consumption; floor; accumulators; damage
    //   MeshErosion(+Apply)  conservative thickness diffusion of over-thick crust (optional)
    //   MeshRelax        optional tangential smoothing, written to posTmp
    //   MeshTopology     single-threaded: repair, flip, collisions, collapse, split. Volume-conservative.
    //   MeshSync         dual areas, thickness, density and stiffness of every vertex after topology
    //   MeshBudget       ledger reduction, convection feedback, arc magmatism from per-cell reservoirs
    // Parallel jobs write only index i. Only MeshTopology changes the topology.
    // ---------------------------------------------------------------------------------

    /// <summary>
    /// Surface drive from the convection cells. For vertex p and cell centre c (unit vectors, cos = p.c), the
    /// direction away from the cell IN p's TANGENT PLANE is  away = p (p.c) - c,  |away| = sin(theta).
    /// (The point simulation's ConvectionDriveJob uses p - c (p.c), which is tangent at c, not at p: its tangential
    /// part at p is cos(theta) times the correct direction, so the drive shrank with distance and REVERSED beyond
    /// 90 degrees, and it carried a radial component of the same size that leaked into the stress measurement.)
    /// Hot cells (temp > 0) push away, cold cells pull toward. The radial profile x exp(1 - x), x = theta / radius,
    /// is faded smoothly to zero before thetaMax = min(4 radius, pi - 0.2) so the field is continuous everywhere,
    /// including near the antipode where the direction is undefined.
    /// </summary>
    [BurstCompile]
    public struct MeshDriveJob : IJobParallelFor
    {
        public MeshParams M;
        public float dt;
        [ReadOnly] public NativeArray<Convection> conv;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos, slab;
        [ReadOnly] public NativeArray<float> density, age;
        [WriteOnly] public NativeArray<float3> driven;
        [WriteOnly] public NativeArray<int> nearest;
        public NativeArray<float> hotspot;

        public static float3 CellDrive(float3 p, float3 c, float temp, float radius, out float x)
        {
            float cs = math.clamp(math.dot(p, c), -1f, 1f);
            float theta = math.acos(cs);
            x = theta / radius;
            float thetaMax = math.min(4f * radius, math.PI - 0.2f);
            if (theta >= thetaMax) return float3.zero;
            float w = x * math.exp(1f - x) * (1f - math.smoothstep(0.75f * thetaMax, thetaMax, theta));
            float3 away = p * cs - c;
            float l = math.length(away);
            if (l < 1e-6f) return float3.zero;
            return away * (temp * w / l);
        }

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            var P = M.P;
            float3 p = pos[i];
            float3 f = float3.zero;
            float bestCos = -2f; int best = 0;
            float hs = hotspot[i];
            for (int k = 0; k < conv.Length; k++)
            {
                float3 c = conv[k].pos;
                float cs = math.dot(p, c);
                if (cs > bestCos) { bestCos = cs; best = k; }
                float T = conv[k].temp;
                f += CellDrive(p, c, T, P.convRadius, out float x);
                if (T > P.plumeThreshold && x < P.plumeRadius) hs += T * dt * (1f - x / P.plumeRadius);
            }
            hotspot[i] = hs;
            nearest[i] = best;
            float R = TMath.Resistance(density[i], age[i], P);
            float3 d = (f * P.convStrength + slab[i]) / R;
            driven[i] = d - p * math.dot(d, p);
        }
    }

    [BurstCompile]
    public struct MeshVelocityJob : IJobParallelFor
    {
        public MeshParams M;
        public float dt;

        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos, vel, driven;
        [ReadOnly] public NativeArray<float> thickness, cfrac, density, stiffness, age;

        [WriteOnly] public NativeArray<float3> velNext, compAxis, slab, spreadVel;
        [WriteOnly] public NativeArray<float> rClose, rOpen, rShear, rSub, rOver, rCrumple, subFrac;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            TectonicParams P = M.P;

            float3 pi = pos[i], vi = vel[i];
            float ti = thickness[i], cfi = cfrac[i], di = density[i], si = stiffness[i], ai = age[i];

            float3 coupledSum = vi * si;
            float wSum = si + 1e-4f;
            float closeSum = 0f, openSum = 0f, shearSum = 0f, subSum = 0f, overSum = 0f, crumpleSum = 0f, closeAll = 0f;
            int subN = 0, count = 0;
            float3 slabF = float3.zero, axis = float3.zero;
            float bestAbs = 0f;
            float3 spread = float3.zero;
            float excessI = math.max(ti - P.contThick, 0f);

            int c0 = vc[i], c = c0, guard = 0;
            do
            {
                int j = V[MeshTopo.Next(c)];
                float3 pj = pos[j];
                float3 delta = pj - pi;
                float3 dir = math.normalizesafe(delta - pi * math.dot(delta, pi));
                count++;

                // --- stiffness-weighted coupling to the one-ring (rigid-ish plates) ---
                float sj = stiffness[j];
                float tj = thickness[j], dj = density[j];
                bool subduct = MeshLimits.Subducts(ti, cfi, di, tj, cfrac[j], dj, M);
                float cw = sj * (MeshLimits.Boundary(ti, di, tj, dj, M) ? 0.1f : 1f);
                coupledSum += vel[j] * cw;
                wSum += cw;

                // --- relative motion across the edge ---
                float3 rel = vel[j] - vi;
                float along = math.dot(rel, dir);
                float3 shearV = rel - along * dir;
                shearV -= pi * math.dot(shearV, pi);
                float closing = math.max(-along, 0f), opening = math.max(along, 0f);
                closeSum += closing; openSum += opening; shearSum += math.length(shearV);
                if (math.abs(along) > bestAbs) { bestAbs = math.abs(along); axis = dir; }

                // --- subduction: the denser side goes under ---
                bool iSub = subduct && (di > dj || (di == dj && ai > age[j]));
                // --- gravitational spreading: over-thick crust flows toward thinner neighbours ---
                if (!subduct) spread += dir * (excessI - math.max(tj - P.contThick, 0f));

                if (closing > P.closeThreshold)
                {
                    closeAll += closing;
                    if (subduct)
                    {
                        if (iSub) { subSum += closing; subN++; slabF += dir * (P.slabPull * (di - dj)); }
                        else overSum += closing;
                    }
                    else crumpleSum += closing;
                }

                c = MeshTopo.Next(O[MeshTopo.Next(c)]);
            } while (c != c0 && ++guard < MeshTopo.MaxRing);

            // --- target velocity: convective drive blended toward the neighbourhood ---
            // MeshCoupleJob applies the compression and extension limits to this TARGET field (relative to the
            // neighbours' targets, so coherent motion is free), and MeshMoveJob relaxes the actual velocity toward
            // the limited target with inertia.
            float3 coupled = coupledSum / wSum;
            // with the strength solve on, coherence comes from the spring network; the target is the drive itself
            float cpl = M.springOn ? 0f : math.saturate(P.couple * si);
            float3 target = math.lerp(driven[i], coupled, cpl);
            target -= pi * math.dot(target, pi);
            velNext[i] = target;
            spreadVel[i] = count > 0 ? spread * (M.spreading / count) : float3.zero;

            float inv = count > 0 ? 1f / count : 0f;
            float vr = 1f / P.velRef;
            rClose[i] = closeSum * inv * vr;
            rOpen[i] = openSum * inv * vr;
            rShear[i] = shearSum * inv * vr;
            rCrumple[i] = crumpleSum * inv * vr;
            rOver[i] = overSum * inv * vr;
            rSub[i] = subN > 0 ? subSum / subN * vr : 0f;
            // share of this vertex's convergence that is it going under its neighbours (drives slab consumption)
            subFrac[i] = closeAll > 0f ? subSum / closeAll : 0f;
            compAxis[i] = axis;
            slab[i] = slabF;
        }
    }

    /// <summary>
    /// One Jacobi pass of the crust's mechanical constraints, run at least twice per step (ping-pong velNext/velTmp):
    ///   1. optional stiffness-weighted velocity averaging (continental rigidity, plateRigidityPasses)
    ///   2. compression limit: removes part of the closing motion toward each non-subducting neighbour
    ///   3. extension limit: removes part of the opening motion away from each same-crust neighbour
    /// Both limits act on velocity RELATIVE to the neighbour's current velocity, so coherent motion is free.
    ///
    /// STRESS (pass 0): the largest opening strain rate this vertex's TARGET velocity makes with a same-crust
    /// neighbour, in velocityReference per radian, before any limit acts. The target is the drive blended with the
    /// neighbourhood's actual motion (MeshVelocityJob), so it carries the crust's own response: inside an intact
    /// block that already moves coherently the coupled part of the target agrees and only the drive's share
    /// strains it, while next to a crack that is opening the actual motion diverges and the stress concentrates.
    /// That is what lets a fracture relieve its surroundings (raw mantle drive, used before, never relaxes).
    /// </summary>
    [BurstCompile]
    public struct MeshCoupleJob : IJobParallelFor
    {
        public MeshParams M;
        public bool average, measure;
        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos, velIn;
        [ReadOnly] public NativeArray<float> thickness, cfrac, density, stiffness, age, damage;
        [WriteOnly] public NativeArray<float3> velOut;
        [WriteOnly] public NativeArray<float> tension;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            float si = stiffness[i], ti = thickness[i], cfi = cfrac[i], di = density[i], ai = age[i], Di = damage[i];
            float3 vi = velIn[i], pi = pos[i];
            float3 v = vi;
            int c0 = vc[i], c = c0, n = 0;

            if (average)
            {
                float3 sum = vi * si; float w = si + 1e-4f;
                do
                {
                    int j = V[MeshTopo.Next(c)];
                    float wj = stiffness[j] * (MeshLimits.Boundary(ti, di, thickness[j], density[j], M) ? 0.1f : 1f);
                    sum += velIn[j] * wj; w += wj;
                    c = MeshTopo.Next(O[MeshTopo.Next(c)]);
                } while (c != c0 && ++n < MeshTopo.MaxRing);
                // extra rigidity is for continents: repeated averaging of the (divergent) mantle drive would
                // otherwise cancel most oceanic motion
                v = math.lerp(vi, sum / w, math.saturate(M.P.couple * si) * cfi);
            }

            float maxTension = 0f;
            c = c0; n = 0;
            do
            {
                int j = V[MeshTopo.Next(c)];
                float tj = thickness[j], cfj = cfrac[j], dj = density[j], sj = stiffness[j];
                float3 delta = pos[j] - pi;
                float d = math.length(delta);
                float3 dir = math.normalizesafe(delta - pi * math.dot(delta, pi));
                bool boundary = MeshLimits.Boundary(ti, di, tj, dj, M);
                float share = sj / (si + sj + 1e-5f);   // the softer side yields more

                if (measure && !boundary && d > 1e-7f)
                    maxTension = math.max(maxTension, math.dot(velIn[j] - vi, dir) / d / M.P.velRef);

                float rel = math.dot(v - velIn[j], dir);   // > 0: closing, < 0: opening
                if (rel > 0f)
                {
                    if (!MeshLimits.Subducts(ti, cfi, di, tj, cfj, dj, M))
                        v -= dir * (rel * MeshLimits.CompressionLock(d, ti, tj, ai, age[j], M) * share);
                }
                else if (M.extEnabled && !M.springOn && !boundary)
                    v -= dir * (rel * MeshLimits.ExtensionLock(cfi, cfj, ai, age[j], Di, damage[j], M) * share);

                c = MeshTopo.Next(O[MeshTopo.Next(c)]);
            } while (c != c0 && ++n < MeshTopo.MaxRing);

            v -= pi * math.dot(v, pi);
            velOut[i] = v;
            if (measure) tension[i] = maxTension;
        }
    }

    public static class MeshLimits
    {
        /// <summary>Crust-type boundary: enough thickness/density contrast that the two vertices are different crust.
        /// Used for sharp margins, weak coupling, stress measurement and slab consumption.</summary>
        public static bool Boundary(float ta, float da, float tb, float db, in MeshParams M)
            => math.abs(ta - tb) / M.P.subductThickness + math.abs(da - db) / M.P.subductDensity >= 1f;

        /// <summary>Mechanical crust type, 0 = oceanic .. 1 = continental:
        ///   max( CrustFrac(thickness),  saturate(0.5 * felsicColumn / felsicBuoyantColumn) )
        /// felsicColumn = felsic volume / volume x thickness (km of differentiated crust in the column).
        /// Thickness alone is a strain variable here (stretching thins crust without changing what it is), so a
        /// stretched continental margin would become mechanically oceanic and be subducted. The felsic column keeps
        /// differentiated crust buoyant down to felsicBuoyantColumn km, while a thin felsic veneer (sediment, a young
        /// arc on ocean floor) stays subductable, which is how felsic material is recycled.</summary>
        public static float CrustType(float thickness, float volume, float felsic, in MeshParams M)
        {
            float cf = TMath.CrustFrac(thickness, M.P);
            if (M.felsicColumn <= 0f) return cf;
            float col = thickness * math.saturate(felsic / math.max(volume, 1e-20f));
            return math.max(cf, math.saturate(0.5f * col / M.felsicColumn));
        }

        /// <summary>g/cm3-ish: oceanic crust densifies as it cools (age), continental-type crust is light.
        /// Same law as TMath.Density, but driven by the crust type instead of thickness alone.</summary>
        public static float Density(float cf, float age)
            => math.lerp(2.9f + 0.35f * math.saturate(age / 200f), 2.75f, cf);

        /// <summary>Can this pair subduct when it converges? Any two oceanic vertices can (the denser goes under),
        /// and so can a crust boundary whose denser side is oceanic. Buoyant crust (crust type >= 0.5) never goes
        /// under buoyant crust: it collides and thickens.</summary>
        public static bool Subducts(float ta, float cfa, float da, float tb, float cfb, float db, in MeshParams M)
        {
            if (cfa < 0.5f && cfb < 0.5f) return true;
            float cfDense = da >= db ? cfa : cfb;
            return cfDense < 0.5f && Boundary(ta, da, tb, db, M);
        }

        /// <summary>Thermal age = geological age + the planet's global thermal age. Never stored.</summary>
        public static float ThermalAge(float age, in MeshParams M) => age + M.globalThermal;

        /// <summary>0 = molten-hot, approaches 1 as crust cools: T / (T + ref).</summary>
        public static float ThermalStrength(float age, in MeshParams M)
        {
            float T = ThermalAge(age, M);
            return T / (T + M.thermalRef);
        }

        /// <summary>0..1 coupling stiffness. Thick and cool = stiff. Active faults and damage weaken it.</summary>
        public static float Stiffness(float cf, float age, float faultActivity, float damage, in MeshParams M)
        {
            float s = math.lerp(0.2f, 0.6f, cf) + 0.3f * ThermalStrength(age, M);
            return math.saturate(s * (1f - 0.5f * math.saturate(math.max(faultActivity, damage))));
        }

        /// <summary>Fraction of closing motion to remove between two non-subducting vertices d apart.
        /// Exponential barrier (1 - exp(-k x)); cool crust multiplies k (up to 1 + barrierAgeFactor), hot crust keeps
        /// a quarter of it, so primordial crust thickens instead of locking. Combined with the gravity cap.</summary>
        public static float CompressionLock(float d, float ta, float tb, float agea, float ageb, in MeshParams M)
        {
            float lockA = 0f;
            if (d < M.barrierRest)
            {
                float x = (M.barrierRest - d) / M.barrierRest;
                float s = ThermalStrength(0.5f * (agea + ageb), M);
                lockA = 1f - math.exp(-M.barrierK * math.lerp(0.25f, 1f + M.barrierAge, s) * x);
            }
            float head = math.max(math.saturate((ta - M.P.lockStart) / M.P.lockMargin), math.saturate((tb - M.P.lockStart) / M.P.lockMargin));
            return math.saturate(math.max(lockA, M.P.lockStrength * head));
        }

        /// <summary>Strength exponent x = s * (ageFactor + thicknessFactor * cf): s = thermal strength 0..1,
        /// cf = crust fraction. 0 for molten crust, ~3.5 for a cold craton with the default factors.</summary>
        public static float StrengthExponent(float cf, float age, in MeshParams M)
            => ThermalStrength(age, M) * (M.extAge + M.extThick * cf);

        /// <summary>Intact extension resistance, 0..extMax: extMax * (1 - exp(-x)).</summary>
        public static float IntactExtension(float cfa, float cfb, float agea, float ageb, in MeshParams M)
        {
            float x = 0.5f * (StrengthExponent(cfa, agea, M) + StrengthExponent(cfb, ageb, M));
            return M.extMax * (1f - math.exp(-x));
        }

        /// <summary>Fraction of opening motion to remove. Damage D (worse of the pair) blends intact resistance
        /// down to the fractured resistance: S (1 - D) + min(F, S) D.</summary>
        public static float ExtensionLock(float cfa, float cfb, float agea, float ageb, float Da, float Db, in MeshParams M)
        {
            float S = IntactExtension(cfa, cfb, agea, ageb, M);
            float D = math.max(Da, Db);
            return S * (1f - D) + math.min(M.extFractured, S) * D;
        }

        /// <summary>Stress intact crust withstands before it starts to break.
        /// Without the strength solve: yield * (0.25 + x)  (velRef / rad of attempted opening).
        /// With it, stress is a spring FORCE (l^2 x strain rate), and the force a lithosphere can carry scales with its
        /// stiffness the same way, so: yield * (0.25 + x) * (l_intact / plateCoherence)^2. Thin oceanic lithosphere
        /// then fails at modest force (ridges, transforms), cold continental keels only under large drag.</summary>
        public static float IntactYield(float cf, float age, in MeshParams M)
        {
            float y = M.dmgYield * (0.25f + StrengthExponent(cf, age, M));
            if (!M.springOn) return y;
            float S = 1f - math.exp(-StrengthExponent(cf, age, M));
            float rel = S * math.lerp(M.oceanCoherence, 1f, cf);
            return y * math.max(rel * rel, 1e-3f);
        }

        /// <summary>
        /// Damage after dt. Two thresholds, so growth and healing are not the same switch:
        ///   grow  while stress > Y (1 - weaken D)     Y = intact yield. Damage lowers its own threshold:
        ///                                              the positive feedback that localises a crack.
        ///         dD/dt = rate * brittle * (stress - Yeff) / Y * (seed + D) * (1 - D)
        ///         (seed + D): intact crust nucleates slowly, damaged crust grows fast, so the first weak spot
        ///         runs away and unloads its surroundings before a broadly over-stressed region fails everywhere
        ///         at once (observed with growth independent of D: a craton over an upwelling crumbled diffusely).
        ///         brittle = thermal strength: hot crust flows instead of cracking, so young crust deforms
        ///         without accumulating pervasive damage.
        ///   heal  while stress < healFrac * Y (the INTACT yield, not the damaged one, so a quiet crack heals
        ///         however damaged it is):  D *= exp(-heal dt)
        ///   hold  in between (an active fault stays weak while it is loaded).
        /// </summary>
        public static float UpdateDamage(float D, float stress, float cf, float age, float dt, in MeshParams M)
        {
            if (!M.extEnabled) return 0f;
            float Y = IntactYield(cf, age, M);
            float yEff = Y * (1f - M.dmgWeaken * D);
            if (stress > yEff)
            {
                float brittle = ThermalStrength(age, M);
                D += M.dmgRate * brittle * (stress - yEff) / math.max(Y, 1e-4f) * (M.dmgSeed + D) * (1f - D) * dt;
            }
            else if (stress < M.dmgHealFrac * Y) D *= math.exp(-M.dmgHeal * dt);
            return math.saturate(D);
        }
    }

    /// <summary>Velocity relaxes toward the limited target (plus gravitational spreading) over velocityInertiaMyr,
    /// then the vertex moves along the sphere. Its crust volume moves with it.</summary>
    [BurstCompile]
    public struct MeshMoveJob : IJobParallelFor
    {
        public float dt, velTau, maxSpeed;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> spreadVel;
        public NativeArray<float3> pos, vel, velNext;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            float3 p0 = pos[i];
            float resp = 1f - math.exp(-dt / velTau);
            float3 v = math.lerp(vel[i], velNext[i] + spreadVel[i], resp);
            v -= p0 * math.dot(v, p0);
            float sp = math.length(v);
            if (sp > maxSpeed) v *= maxSpeed / sp;
            velNext[i] = v;
            float3 disp = v * dt;
            disp -= p0 * math.dot(disp, p0);
            float3 p = math.normalize(p0 + disp);
            pos[i] = p;
            vel[i] = v - p * math.dot(v, p);
        }
    }

    /// <summary>Voronoi dual area of every vertex at its new position (topology unchanged since the last sync).</summary>
    [BurstCompile]
    public struct MeshAreaJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos;
        [WriteOnly] public NativeArray<float> areaNew;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            areaNew[i] = MeshGeom.DualArea(V, O, vc, pos, i);
        }
    }

    /// <summary>
    /// Material and accumulators after the move. Volume moves with its vertex, so deformation is conservative:
    ///   thickness = volume / area          (area lost thickens, area gained thins)
    /// Explicit, ledgered changes:
    ///   slab consumption   a vertex going under its neighbours loses the volume of the area it loses there:
    ///                      sink = t_old * max(areaOld - areaNew, 0) * subFrac   (its thickness is kept)
    ///   floor              crust thinner than minCrust is topped up from the mantle (age-0 material mixed in)
    /// Also: fault classification, damage, achieved extension (diagnostic), arc weight for magmatism.
    /// </summary>
    [BurstCompile]
    public struct MeshIntegrateJob : IJobParallelFor
    {
        public MeshParams M;
        public float dt;

        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos, vel, compAxis;
        [ReadOnly] public NativeArray<float> areaNew, rClose, rOpen, rShear, rSub, rOver, rCrumple, subFrac, tension;

        public NativeArray<float> volume, felsic, area, thickness, cfrac, age, density, stiffness;
        public NativeArray<float> orogeny, extension, arc, shear, faultActivity, damage, flashSplit, flashCollapse, magma;
        [WriteOnly] public NativeArray<float> achieved, arcW, sinkSub, srcFloor, sinkFelsic, sinkDelam, sinkDelamFelsic, accrete;
        public NativeArray<byte> faultType;
        public NativeArray<float3> faultDir;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            TectonicParams P = M.P;
            float3 pi = pos[i];

            // --- material: volume is the state ---
            float vol = volume[i];
            float a0 = math.max(area[i], 1e-12f), a1 = math.max(areaNew[i], 1e-12f);
            float tOld = vol / a0;
            float fel = felsic[i];
            float sink = 0f, sinkF = 0f;
            if (M.subductionSink && a1 < a0 && subFrac[i] > 0f)
            {
                sink = math.min(tOld * (a0 - a1) * subFrac[i], vol);
                sinkF = vol > 0f ? fel * (sink / vol) : 0f;
                vol -= sink; fel -= sinkF;
            }
            // buoyant felsic material in the slab mostly does not sink: it is scraped off / relaminated onto the
            // overriding plate (MeshBudgetJob places it). Only felsicSubduct of it really leaves the crust.
            float relam = sinkF * (1f - M.felsicSubduct);
            sinkSub[i] = sink - relam;
            sinkFelsic[i] = sinkF - relam;
            accrete[i] = relam;
            felsic[i] = math.max(fel, 0f);
            float fill = 0f;
            if (M.minCrust > 0f && vol / a1 < M.minCrust) { fill = M.minCrust * a1 - vol; }
            srcFloor[i] = fill;
            float ageMix = fill > 0f ? vol / (vol + fill) : 1f;
            vol += fill;
            // delamination: a crustal root thicker than maxThickness founders into the mantle (density inversion of
            // the eclogitised lower crust). Finite rate, so orogens can overshoot briefly and then relax.
            float delam = 0f, delamF = 0f;
            float excess = vol / a1 - P.maxThickness;
            if (M.delamRate > 0f && excess > 0f)
            {
                delam = excess * a1 * (1f - math.exp(-M.delamRate * dt));
                delamF = felsic[i] * (delam / vol);
                vol -= delam;
                felsic[i] = math.max(felsic[i] - delamF, 0f);
            }
            sinkDelam[i] = delam; sinkDelamFelsic[i] = delamF;
            float t = vol / a1;
            volume[i] = vol; area[i] = a1; thickness[i] = t;
            float cfStart = cfrac[i];
            float cfNow = MeshLimits.CrustType(t, vol, felsic[i], M);
            cfrac[i] = cfNow;
            if (a1 > a0) extension[i] = extension[i] + (a1 / a0 - 1f) * cfStart;

            float ag = age[i] * ageMix + dt;
            age[i] = ag;

            // --- accumulators ---
            float cl = rClose[i], op = rOpen[i], sh = rShear[i], crumple = rCrumple[i], over = rOver[i];
            float od = math.exp(-dt / P.orogenyDecay);
            orogeny[i] = orogeny[i] * od + (crumple + P.overThick * over) * dt * P.orogenyGain;
            arc[i] = arc[i] * math.exp(-dt / P.arcDecay) + over * dt * P.arcGain;
            shear[i] = shear[i] * od + sh * dt;

            // --- one-ring: achieved opening (diagnostic) and arc weight (overriding margin, two rows wide) ---
            float ach = 0f, overMax = 0f;
            float3 vi = vel[i];
            int c0 = vc[i], c = c0, guard = 0;
            do
            {
                int j = V[MeshTopo.Next(c)];
                float3 delta = pos[j] - pi;
                float d = math.length(delta);
                float3 dir = math.normalizesafe(delta - pi * math.dot(delta, pi));
                if (d > 1e-7f) ach = math.max(ach, math.dot(vel[j] - vi, dir) / d / P.velRef);
                overMax = math.max(overMax, rOver[j]);
                c = MeshTopo.Next(O[MeshTopo.Next(c)]);
            } while (c != c0 && ++guard < MeshTopo.MaxRing);
            achieved[i] = ach;
            arcW[i] = over + 0.5f * overMax;

            // --- fault classification with memory (fossil faults fade rather than vanish) ---
            float act = math.max(cl, math.max(op, sh));
            float decayed = faultActivity[i] * math.exp(-dt / P.faultMemory);
            if (act >= P.faultThreshold && act >= decayed)
            {
                faultActivity[i] = act;
                FaultType ft;
                if (sh > P.transformDominance * math.max(cl, op)) ft = FaultType.Transform;
                else if (op > cl) ft = FaultType.Divergent;
                else ft = (rSub[i] + over > crumple) ? FaultType.Subduction : FaultType.Collision;
                faultType[i] = (byte)ft;
                faultDir[i] = math.normalizesafe(math.cross(pi, compAxis[i]), faultDir[i]);
            }
            else
            {
                faultActivity[i] = decayed;
                if (decayed < P.faultThreshold * 0.25f) faultType[i] = (byte)FaultType.None;
            }

            float D = MeshLimits.UpdateDamage(damage[i], tension[i], cfNow, ag, dt, M);
            damage[i] = D;
            density[i] = MeshLimits.Density(cfNow, ag);
            stiffness[i] = MeshLimits.Stiffness(cfNow, ag, faultActivity[i], D, M);

            float fd = math.exp(-dt / M.flashDecay);
            flashSplit[i] = flashSplit[i] * fd;
            flashCollapse[i] = flashCollapse[i] * fd;
            magma[i] = magma[i] * fd;
        }
    }

    /// <summary>
    /// Erosion as conservative transport: crust thicker than erosionBase diffuses toward its neighbours.
    ///   flux i -> j = K w_ij (h_i - h_j) dt,  h = max(t - base, 0),  w_ij = (A_i + A_j) / (3 max(|e|^2, L^2))
    /// w is symmetric, so the fluxes cancel exactly (a regular mesh gives the continuum diffusivity K). Sediment
    /// arriving on oceanic crust (cf &lt; 0.5) can be partly recycled (erosionRecycle, ledgered); otherwise it stays as
    /// margin sediment and goes wherever that crust goes (much of it is later subducted and returned by arcs).
    /// Writes volumeTmp; MeshErosionApplyJob copies it back.
    /// </summary>
    [BurstCompile]
    public struct MeshErosionJob : IJobParallelFor
    {
        public MeshParams M;
        public float dt;
        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos;
        [ReadOnly] public NativeArray<float> volume, felsic, area, thickness, cfrac;
        [WriteOnly] public NativeArray<float> volumeTmp, felsicTmp, sinkErosion, sinkErosionFelsic, moveErosion;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            float hi = math.max(thickness[i] - M.erosionBase, 0f);
            float Ai = area[i], L2 = M.L * M.L;
            float fi = felsic[i] / math.max(volume[i], 1e-20f);
            float dv = 0f, outV = 0f, df = 0f;
            int c0 = vc[i], c = c0, n = 0;
            do
            {
                int j = V[MeshTopo.Next(c)];
                float hj = math.max(thickness[j] - M.erosionBase, 0f);
                if (hi > 0f || hj > 0f)
                {
                    float w = (Ai + area[j]) / (3f * math.max(math.lengthsq(pos[j] - pos[i]), L2));
                    float f = M.erosionK * w * (hj - hi) * dt;
                    dv += f;
                    if (f < 0f) outV -= f;
                    // sediment carries the composition of the side it comes from (upwind), so felsic is conserved too
                    df += f * (f > 0f ? felsic[j] / math.max(volume[j], 1e-20f) : fi);
                }
                c = MeshTopo.Next(O[MeshTopo.Next(c)]);
            } while (c != c0 && ++n < MeshTopo.MaxRing);
            float rec = 0f, recF = 0f;
            if (dv > 0f && cfrac[i] < 0.5f) { rec = dv * M.erosionRecycle; recF = math.max(df, 0f) * M.erosionRecycle; }
            float newVolume = math.max(volume[i] + dv - rec, 0f);
            volumeTmp[i] = newVolume;
            felsicTmp[i] = math.clamp(
                felsic[i] + df - recF, 0f, newVolume);
            sinkErosion[i] = rec;
            sinkErosionFelsic[i] = recF;
            moveErosion[i] = outV;
        }
    }

    [BurstCompile]
    public struct MeshErosionApplyJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> alive;
        public MeshParams M;
        [ReadOnly] public NativeArray<float> volumeTmp, felsicTmp, area;
        public NativeArray<float> volume, felsic, thickness, cfrac;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            float v = volumeTmp[i];
            volume[i] = v;
            felsic[i] = felsicTmp[i];
            float t = v / math.max(area[i], 1e-12f);
            thickness[i] = t;
            cfrac[i] = MeshLimits.CrustType(t, v, felsicTmp[i], M);
        }
    }

    /// <summary>After topology: dual area, thickness = volume / area, density and stiffness of every vertex.</summary>
    [BurstCompile]
    public struct MeshSyncJob : IJobParallelFor
    {
        public MeshParams M;
        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos;
        [ReadOnly] public NativeArray<float> volume, felsic, age, faultActivity, damage;
        [WriteOnly] public NativeArray<float> area, thickness, cfrac, density, stiffness;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            float a = MeshGeom.DualArea(V, O, vc, pos, i);
            float t = volume[i] / math.max(a, 1e-12f);
            float cf = MeshLimits.CrustType(t, volume[i], felsic[i], M);
            area[i] = a; thickness[i] = t; cfrac[i] = cf;
            density[i] = MeshLimits.Density(cf, age[i]);
            stiffness[i] = MeshLimits.Stiffness(cf, age[i], faultActivity[i], damage[i], M);
        }
    }

    /// <summary>Tangential Laplacian smoothing. Moves vertices (and the crust they carry), so keep it weak.</summary>
    [BurstCompile]
    public struct MeshRelaxJob : IJobParallelFor
    {
        public float relax;
        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos;
        [WriteOnly] public NativeArray<float3> posTmp;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            float3 p = pos[i], sum = float3.zero;
            int n = 0, c0 = vc[i], c = c0;
            do
            {
                sum += pos[V[MeshTopo.Next(c)]]; n++;
                c = MeshTopo.Next(O[MeshTopo.Next(c)]);
            } while (c != c0 && n < MeshTopo.MaxRing);
            float3 d = sum / n - p;
            d -= p * math.dot(d, p);
            posTmp[i] = math.normalize(p + d * relax);
        }
    }

    /// <summary>
    /// Once per step, single-threaded, O(V + cells):
    /// 1. LEDGER: reduces the per-vertex sources/sinks of this step into SphereMeshState.ledger (and zeroes them).
    /// 2. MAGMA RESERVOIRS: every cell's reservoir is charged with the slab volume consumed in it times
    ///      f = magmaReturn * (1 + magmaYouth * globalHeat)
    ///    (a young, hot mantle melts more of what sinks into it; f never drops below magmaReturn, so a mature planet
    ///    keeps an active material cycle). Each step a reservoir releases 1 - exp(-dt / magmaTau) of its content onto
    ///    the overriding margins of its cell, in proportion to arcW x area (arcW = overriding convergence, spread two
    ///    rows wide). Oceanic overriding plates (ocean-ocean subduction) therefore grow island arcs, which become new
    ///    buoyant nuclei: continental growth never requires existing continents. A cell with no active margin keeps
    ///    its reservoir until one appears.
    /// 3. CONVECTION FEEDBACK per cell, area-weighted:
    ///      feedback' = insulationHeat * cov
    ///                - surfaceCool * (1 - cov) * (temp + 1) / 2        exposed mantle loses heat; hotter cells lose more
    ///                - subductionCool * (consumed area fraction / Myr)  cold slabs
    ///                - feedbackRelax * feedback
    ///    cov = area-weighted insulation of the cell's crust: saturate((cf - insulationOnset) / (1 - insulationOnset)).
    ///    temp = clamp(baseTemp + feedback); baseTemp is a mean-reverting random walk (tempNoise).
    ///    Hot cells drift toward the crust that insulates them, cold cells toward the slabs that feed them.
    /// </summary>
    [BurstCompile]
    public struct MeshBudgetJob : IJob
    {
        public MeshParams M;
        public float dt;
        public uint step;

        public NativeArray<Convection> conv;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos;
        [ReadOnly] public NativeArray<float> area, arcW;
        [ReadOnly] public NativeArray<int> nearest, consumedCell;
        [ReadOnly] public NativeArray<float3> consumedPos;
        [ReadOnly] public NativeArray<float> consumedVol;
        public NativeArray<int> counters;
        public NativeArray<float> volume, felsic, thickness, cfrac, density, age, magma;
        public NativeArray<float> sinkSub, srcFloor, sinkErosion, moveErosion, sinkFelsic, sinkErosionFelsic, sinkDelam, sinkDelamFelsic, accrete;
        public NativeArray<double> ledger, pool, accretePool;

        public NativeArray<float3> sumCont, sumSub;
        public NativeArray<float> cellArea, contArea, cellArcW;
        public NativeArray<int> subCnt;

        public void Execute()
        {
            var P = M.P;
            int N = conv.Length;
            var cellSub = new FixedList512Bytes<float>();   // consumed volume per cell this step (<= 127 cells)
            for (int k = 0; k < N; k++)
            {
                sumCont[k] = float3.zero; sumSub[k] = float3.zero;
                cellArea[k] = 0f; contArea[k] = 0f; cellArcW[k] = 0f; subCnt[k] = 0;
                if (k < 127) cellSub.Add(0f);
            }

            // ---- 1. ledger + per-cell sums ----
            double dSub = 0, dFloor = 0, dEroRec = 0, dEroMove = 0, dFel = 0, dFelRec = 0, dDel = 0, dDelF = 0;
            float onset = math.min(M.insulationOnset, 0.99f);
            for (int i = 0; i < alive.Length; i++)
            {
                // every slot, alive or not: a vertex can book a source or sink in MeshIntegrate and be removed by
                // MeshTopology in the same step, and its booking must still reach the ledger
                int k = nearest[i];
                float sv = sinkSub[i];
                if (accrete[i] != 0f)
                {
                    if (k >= 0 && k < N) accretePool[k] = accretePool[k] + accrete[i];
                    accrete[i] = 0f;
                }
                if (sv != 0f || srcFloor[i] != 0f || sinkErosion[i] != 0f || moveErosion[i] != 0f || sinkDelam[i] != 0f)
                {
                    dDel += sinkDelam[i]; dDelF += sinkDelamFelsic[i]; sinkDelam[i] = 0f; sinkDelamFelsic[i] = 0f;
                    dSub += sv; dFloor += srcFloor[i]; dEroRec += sinkErosion[i]; dEroMove += moveErosion[i];
                    dFel += sinkFelsic[i]; dFelRec += sinkErosionFelsic[i];
                    sinkSub[i] = 0f; srcFloor[i] = 0f; sinkErosion[i] = 0f; moveErosion[i] = 0f; sinkFelsic[i] = 0f; sinkErosionFelsic[i] = 0f;
                    if (k >= 0 && k < 127) cellSub[k] = cellSub[k] + sv;
                }
                if (alive[i] == 0) continue;
                float A = area[i];
                cellArea[k] = cellArea[k] + A;
                // insulation: by crust type, and proportional to thickness (a 70 km pile insulates twice what
                // normal continental crust does), so assembling continents heats the cell beneath them
                float ins = math.saturate((TMath.CrustFrac(thickness[i], P) - onset) / (1f - onset))
                          * math.min(thickness[i] / math.max(P.contThick, 1f), 3f);
                if (ins > 0f) { contArea[k] = contArea[k] + A * ins; sumCont[k] = sumCont[k] + pos[i] * (A * ins); }
                float w = arcW[i] * A;
                cellArcW[k] = cellArcW[k] + w;
            }
            int consumed = counters[MeshTopo.ConsumedThisStep];
            for (int s = 0; s < consumed; s++)
            {
                int k = consumedCell[s];
                subCnt[k] = subCnt[k] + 1; sumSub[k] = sumSub[k] + consumedPos[s];
                if (k < 127) cellSub[k] = cellSub[k] + consumedVol[s];
            }
            counters[MeshTopo.ConsumedThisStep] = 0;
            ledger[MeshLedger.SubductedSqueeze] += dSub;
            ledger[MeshLedger.FloorFill] += dFloor;
            ledger[MeshLedger.ErosionRecycled] += dEroRec;
            ledger[MeshLedger.ErosionTransported] += dEroMove;
            ledger[MeshLedger.FelsicSubducted] += dFel;
            ledger[MeshLedger.FelsicRecycled] += dFelRec;
            ledger[MeshLedger.Delaminated] += dDel;
            ledger[MeshLedger.FelsicDelaminated] += dDelF;

            // ---- 2. magma reservoirs ----
            float f = M.magmatism ? M.magmaReturn * (1f + M.magmaYouth * M.globalHeat) : 0f;
            float release = 1f - math.exp(-dt / math.max(M.magmaTau, 1e-3f));
            var rel = new FixedList512Bytes<float>();   // per-cell magma released per unit of arcW x area
            var acc = new FixedList512Bytes<float>();   // per-cell relaminated felsic released per unit of arcW x area
            double charged = 0;
            for (int k = 0; k < N; k++)
            {
                float sub = k < 127 ? cellSub[k] : 0f;
                pool[k] = pool[k] + sub * f;
                charged += sub * f;
                float r = 0f;
                if (M.magmatism && cellArcW[k] > 1e-12f && pool[k] > 0.0)
                {
                    double out_ = pool[k] * release;
                    pool[k] = pool[k] - out_;
                    r = (float)(out_ / cellArcW[k]);
                }
                if (k < 127) rel.Add(r);
                float ra = 0f;
                if (cellArcW[k] > 1e-12f && accretePool[k] > 0.0)
                {
                    ra = (float)(accretePool[k] / cellArcW[k]);
                    ledger[MeshLedger.Relaminated] += accretePool[k];
                    accretePool[k] = 0.0;
                }
                if (k < 127) acc.Add(ra);
            }
            ledger[MeshLedger.MagmaCharged] += charged;
            double deposited = 0;
            {
                for (int i = 0; i < alive.Length; i++)
                {
                    if (alive[i] == 0 || arcW[i] <= 0f) continue;
                    int k = nearest[i];
                    if (k >= 127) continue;
                    float dep = M.magmatism ? rel[k] * arcW[i] * area[i] : 0f;
                    float accd = acc[k] * arcW[i] * area[i];
                    if (dep <= 0f && accd <= 0f) continue;
                    float v = volume[i];
                    age[i] = age[i] * (v / (v + dep));          // new magmatic material is age 0 (accreted keeps its age)
                    v += dep + accd;
                    volume[i] = v;
                    felsic[i] = felsic[i] + dep * M.magmaFelsic + accd;
                    float t = v / math.max(area[i], 1e-12f);
                    thickness[i] = t;
                    float cf = MeshLimits.CrustType(t, v, felsic[i], M);
                    cfrac[i] = cf;
                    density[i] = MeshLimits.Density(cf, age[i]);
                    magma[i] = magma[i] + dep / math.max(area[i], 1e-12f) / dt;   // km/Myr, decays in MeshIntegrate
                    deposited += dep;
                }
                // anything released but not deposited (cells >= 127) goes back to the reservoir it came from
            }
            ledger[MeshLedger.Magmatic] += deposited;
            ledger[MeshLedger.FelsicMagmatic] += deposited * M.magmaFelsic;

            // ---- 3. convection feedback ----
            float oceanArea = math.max(P.oceanThick, 1e-3f);
            for (int k = 0; k < N; k++)
            {
                Convection c = conv[k];
                float ca = math.max(cellArea[k], 1e-6f);
                float cov = contArea[k] / ca;
                float subVol = k < 127 ? cellSub[k] : 0f;
                float subRate = subVol / oceanArea / ca / dt;   // consumed area fraction per Myr (oceanic-thickness equivalent)
                float cooling = M.surfaceCool * math.saturate(1f - cov) * math.saturate((c.temp + 1f) * 0.5f);

                c.feedback += dt * (P.insulationHeat * cov - cooling - P.subductionCool * subRate - c.feedback * P.feedbackRelax);
                float n = TMath.Rand01((uint)k * 977u + step * 131u + P.seed) * 2f - 1f;
                c.baseTemp = math.clamp(c.baseTemp * math.exp(-P.feedbackRelax * dt) + n * P.tempNoise * math.sqrt(dt), -1f, 1f);
                c.temp = math.clamp(c.baseTemp + c.feedback, -1.5f, 1.5f);
                conv[k] = c;
            }

            // ---- convective vigour: the feedback decides WHICH cells are hot or cold; the planet's heat budget
            // decides how hard the mantle convects. Without this, balanced feedback lets every cell drift to zero
            // and the planet stops (observed). temp = vigor * (raw - mean) / rms, with vigor declining as the planet
            // cools. This is an explicit global rule about mantle energy, not about crust: it never adds material.
            if (M.vigor > 0f && N > 1)
            {
                float mean = 0f;
                for (int k = 0; k < N; k++) mean += conv[k].baseTemp + conv[k].feedback;
                mean /= N;
                float var_ = 0f;
                for (int k = 0; k < N; k++) { float x = conv[k].baseTemp + conv[k].feedback - mean; var_ += x * x; }
                float rms = math.sqrt(var_ / N);
                float scale = M.vigor / math.max(rms, 1e-3f);
                for (int k = 0; k < N; k++)
                {
                    var c = conv[k];
                    c.temp = math.clamp((c.baseTemp + c.feedback - mean) * scale, -1.5f, 1.5f);
                    conv[k] = c;
                }
            }

            for (int k = 0; k < N; k++)
            {
                Convection c = conv[k];

                float3 target = c.pos; bool has = false;
                if (c.temp < 0f && subCnt[k] > 0) { target = sumSub[k]; has = true; }
                else if (c.temp > 0f && contArea[k] > 0f) { target = sumCont[k]; has = true; }
                if (has)
                {
                    target = math.normalizesafe(target, c.pos);
                    float3 dir = target - c.pos * math.dot(target, c.pos);
                    c.pos = math.normalize(c.pos + dir * (P.convDrift * dt));
                }
                conv[k] = c;
            }

            for (int a = 0; a < N; a++)
            for (int b = a + 1; b < N; b++)
            {
                Convection ca = conv[a], cb = conv[b];
                float cs = math.clamp(math.dot(ca.pos, cb.pos), -1f, 1f);
                float theta = math.acos(cs);
                if (theta >= P.convMinSep) continue;
                float3 d = ca.pos - cb.pos * cs;
                float l = math.length(d);
                if (l < 1e-5f) continue;
                d /= l;
                float push = (P.convMinSep - theta) * 0.25f;
                ca.pos = math.normalize(ca.pos + d * push);
                cb.pos = math.normalize(cb.pos - d * push);
                conv[a] = ca; conv[b] = cb;
            }
        }
    }
}
