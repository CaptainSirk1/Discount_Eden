using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace PlanetGen.Tectonics.MeshSim
{
    // ---------------------------------------------------------------------------------
    // Step pipeline (chained, never completed mid-simulation):
    //   ConvectionDrive (reused from the point sim)
    //   -> MeshVelocity   (one-ring: target velocity = drive blended with neighbours; spreading; boundary rates)
    //   -> MeshCouple x N (>= 2: optional rigidity averaging, compression + extension limits, tension on pass 0)
    //   -> MeshArea       (dual areas + one-crust triangle mask, using the final velocities)
    //   -> MeshMove       (velocity relaxes toward the limited target with inertia, then advect)
    //   -> MeshIntegrate  (one-ring area change -> thickness, accumulators, fault classification)
    //   -> MeshRelax      (optional tangential smoothing, written to posTmp)
    //   -> MeshTopology   (single-threaded: repair, flip, vertex/edge collisions, collapse, split, flip)
    //   -> MeshFeedback   (convection insulation / slab cooling)
    // Parallel jobs only read the topology. Only MeshTopology changes it.
    // ---------------------------------------------------------------------------------

    [BurstCompile]
    public struct MeshVelocityJob : IJobParallelFor
    {
        public MeshParams M;
        public float dt;

        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos, vel, driven;
        [ReadOnly] public NativeArray<float> thickness, density, stiffness, age;

        [WriteOnly] public NativeArray<float3> velNext, compAxis, slab, spreadVel;
        [WriteOnly] public NativeArray<float> rClose, rOpen, rShear, rSub, rOver, rCrumple;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            TectonicParams P = M.P;

            float3 pi = pos[i], vi = vel[i];
            float ti = thickness[i], di = density[i], si = stiffness[i], ai = age[i];

            float3 coupledSum = vi * si;
            float wSum = si + 1e-4f;
            float closeSum = 0f, openSum = 0f, shearSum = 0f, subSum = 0f, overSum = 0f, crumpleSum = 0f;
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
                bool subduct = MeshLimits.Subducts(ti, di, tj, dj, M);
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
                // --- gravitational spreading: over-thick crust flows toward thinner neighbours (volume-conserving) ---
                if (!subduct) spread += dir * (excessI - math.max(tj - P.contThick, 0f));

                if (closing > P.closeThreshold)
                {
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
            // MeshCoupleJob then applies the compression and extension limits to this TARGET field (relative to the
            // neighbours' targets, so coherent motion is free), and MeshMoveJob relaxes the actual velocity toward
            // the limited target with inertia. Limiting the target rather than the per-step velocity keeps every
            // limit's meaning ("fraction of relative motion removed") independent of the timestep.
            float3 coupled = coupledSum / wSum;
            float cpl = math.saturate(P.couple * si);
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
            compAxis[i] = axis;
            slab[i] = slabF;
        }
    }

    /// <summary>
    /// Dual area of every vertex before it moves, with the FINAL velocities of this step known:
    ///   areaFull = 1/3 of all incident triangles (diagnostics, feedback weighting)
    ///   areaOld  = 1/3 of the incident triangles that lie inside one crust (areaMask marks them)
    /// A triangle is left out if any pair in it is a crust boundary or is converging and can subduct: area lost
    /// there is consumed crust and must not thicken anything. MeshIntegrateJob measures the same subset after the
    /// move, and the ratio is the volume-conserving thickness change.
    /// </summary>
    [BurstCompile]
    public struct MeshAreaJob : IJobParallelFor
    {
        public MeshParams M;
        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos, velNext;
        [ReadOnly] public NativeArray<float> thickness, density;
        [WriteOnly] public NativeArray<float> areaOld, areaFull;
        [WriteOnly] public NativeArray<uint> areaMask;

        bool Excluded(int a, int b)
        {
            if (MeshLimits.Boundary(thickness[a], density[a], thickness[b], density[b], M)) return true;
            if (!MeshLimits.Subducts(thickness[a], density[a], thickness[b], density[b], M)) return false;
            float3 d = math.normalizesafe(pos[b] - pos[a]);
            return math.dot(velNext[b] - velNext[a], d) < 0f;
        }

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            float3 pi = pos[i];
            float area = 0f, areaAll = 0f;
            uint mask = 0u;
            int c0 = vc[i], c = c0, guard = 0;
            do
            {
                int j = V[MeshTopo.Next(c)], jn = V[MeshTopo.Prev(c)];
                float triArea = 0.5f * math.length(math.cross(pos[j] - pi, pos[jn] - pi));
                areaAll += triArea;
                if (guard < 32 && !Excluded(i, j) && !Excluded(i, jn) && !Excluded(j, jn))
                {
                    area += triArea;
                    mask |= 1u << guard;
                }
                c = MeshTopo.Next(O[MeshTopo.Next(c)]);
            } while (c != c0 && ++guard < MeshTopo.MaxRing);
            areaOld[i] = area / 3f;
            areaFull[i] = areaAll / 3f;
            areaMask[i] = mask;
        }
    }

    /// <summary>
    /// One Jacobi pass of the crust's mechanical constraints, run at least twice per step (ping-pong velNext/velTmp):
    ///   1. optional stiffness-weighted velocity averaging (continental rigidity, plateRigidityPasses)
    ///   2. compression limit: removes part of the closing motion toward each non-subducting neighbour
    ///   3. extension limit: removes part of the opening motion away from each same-crust neighbour
    /// Both limits act on velocity RELATIVE to the neighbour's current velocity, so coherent motion is free.
    /// The first pass also measures tension: how fast the mantle drive is trying to pull this vertex away from its
    /// neighbours (strain rate of the convective drive, before coupling or any limit). That is what the crust
    /// resists, so a strong barrier can't hide the stress that will eventually break it.
    /// </summary>
    [BurstCompile]
    public struct MeshCoupleJob : IJobParallelFor
    {
        public MeshParams M;
        public bool average, measure;
        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos, velIn, driven;
        [ReadOnly] public NativeArray<float> thickness, density, stiffness, age, damage;
        [WriteOnly] public NativeArray<float3> velOut;
        [WriteOnly] public NativeArray<float> tension;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            float si = stiffness[i], ti = thickness[i], di = density[i], ai = age[i], Di = damage[i];
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
                float cf = TMath.CrustFrac(ti, M.P);
                v = math.lerp(vi, sum / w, math.saturate(M.P.couple * si) * cf);
            }

            float maxTension = 0f;
            float3 di3 = driven[i];
            c = c0; n = 0;
            do
            {
                int j = V[MeshTopo.Next(c)];
                float tj = thickness[j], dj = density[j], sj = stiffness[j];
                float3 delta = pos[j] - pi;
                float d = math.length(delta);
                float3 dir = math.normalizesafe(delta - pi * math.dot(delta, pi));
                bool boundary = MeshLimits.Boundary(ti, di, tj, dj, M);
                float share = sj / (si + sj + 1e-5f);   // the softer side yields more

                if (measure && !boundary && d > 1e-7f)
                    maxTension = math.max(maxTension, math.dot(driven[j] - di3, dir) / d / M.P.velRef);

                float rel = math.dot(v - velIn[j], dir);   // > 0: closing, < 0: opening
                if (rel > 0f)
                {
                    if (!MeshLimits.Subducts(ti, di, tj, dj, M))
                        v -= dir * (rel * MeshLimits.CompressionLock(d, ti, tj, ai, age[j], M) * share);
                }
                else if (M.extEnabled && !boundary)
                    v -= dir * (rel * MeshLimits.ExtensionLock(ti, tj, ai, age[j], Di, damage[j], M) * share);

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
        /// Used for sharp margins, weak coupling and area accounting.</summary>
        public static bool Boundary(float ta, float da, float tb, float db, in MeshParams M)
            => math.abs(ta - tb) / M.P.subductThickness + math.abs(da - db) / M.P.subductDensity >= 1f;

        /// <summary>Can this pair subduct when it converges? Any two oceanic vertices can (the denser goes under),
        /// and so can a crust boundary whose denser side is oceanic. Buoyant crust (cf >= 0.5, e.g. uniform 25 km
        /// primordial crust) never goes under buoyant crust: it collides and thickens.</summary>
        public static bool Subducts(float ta, float da, float tb, float db, in MeshParams M)
        {
            float cfa = TMath.CrustFrac(ta, M.P), cfb = TMath.CrustFrac(tb, M.P);
            if (cfa < 0.5f && cfb < 0.5f) return true;
            float cfDense = da >= db ? cfa : cfb;
            return cfDense < 0.5f && Boundary(ta, da, tb, db, M);
        }

        /// <summary>Thermal age = geological age + the planet's global thermal age. Never stored.</summary>
        public static float ThermalAge(float age, in MeshParams M) => age + M.globalThermal;

        /// <summary>0 = molten-hot, approaches 1 as crust cools. T / (T + ref): no hard saturation, so a
        /// 2 Gyr craton stays distinguishable from 300 Myr old sea floor.</summary>
        public static float ThermalStrength(float age, in MeshParams M)
        {
            float T = ThermalAge(age, M);
            return T / (T + M.thermalRef);
        }

        /// <summary>0..1 coupling stiffness. Thick and cool = stiff. Active faults and damage weaken it.</summary>
        public static float Stiffness(float thickness, float age, float faultActivity, float damage, in MeshParams M)
        {
            float cf = TMath.CrustFrac(thickness, M.P);
            float s = math.lerp(0.2f, 0.6f, cf) + 0.3f * ThermalStrength(age, M);
            return math.saturate(s * (1f - 0.5f * math.saturate(math.max(faultActivity, damage))));
        }

        /// <summary>Fraction of closing motion to remove between two non-subducting vertices d apart.
        /// Exponential barrier (1 - exp(-k x), so resistance ~ exp(k x) - 1). Cool crust multiplies k (up to
        /// 1 + barrierAgeFactor); hot crust keeps a quarter of it, so primordial crust thickens instead of locking.
        /// Combined with the gravity cap (crust that can't thicken any more can't be compressed any more).</summary>
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
        /// cf = crust fraction. 0 for molten crust, ~4 for a cold craton with the default factors.</summary>
        public static float StrengthExponent(float thickness, float age, in MeshParams M)
            => ThermalStrength(age, M) * (M.extAge + M.extThick * TMath.CrustFrac(thickness, M.P));

        /// <summary>Intact extension resistance, 0..extMax: extMax * (1 - exp(-x)). Hot crust barely resists;
        /// cool thick crust strongly does.</summary>
        public static float IntactExtension(float ta, float tb, float agea, float ageb, in MeshParams M)
        {
            float x = 0.5f * (StrengthExponent(ta, agea, M) + StrengthExponent(tb, ageb, M));
            return M.extMax * (1f - math.exp(-x));
        }

        /// <summary>Fraction of opening motion to remove. Damage D (worse of the pair) blends intact resistance
        /// down to the fractured resistance: S (1 - D) + min(F, S) D.</summary>
        public static float ExtensionLock(float ta, float tb, float agea, float ageb, float Da, float Db, in MeshParams M)
        {
            float S = IntactExtension(ta, tb, agea, ageb, M);
            float D = math.max(Da, Db);
            return S * (1f - D) + math.min(M.extFractured, S) * D;
        }

        /// <summary>Damage after dt. The yield scales with strength and drops as damage grows:
        ///   yieldEff = yield * (0.25 + x * (1 - D))
        /// so hot or thin crust breaks easily, cold cratons need far more tension, and a crack that has started
        /// keeps growing (the positive feedback that concentrates strain). Tension above yieldEff grows D toward 1
        /// (bounded by the (1 - D) factor); below it, D heals exponentially.</summary>
        public static float UpdateDamage(float D, float tension, float thickness, float age, float dt, in MeshParams M)
        {
            if (!M.extEnabled) return 0f;
            float yieldEff = M.extYield * (0.25f + StrengthExponent(thickness, age, M) * (1f - D));
            float excess = tension - yieldEff;
            if (excess > 0f) D += M.extRate * excess * (1f - D) * dt;
            else D *= math.exp(-M.extHeal * dt);
            return math.saturate(D);
        }
    }

    /// <summary>Velocity relaxes toward the limited target (plus gravitational spreading) over velocityInertiaMyr,
    /// then the vertex moves along the sphere.</summary>
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

    [BurstCompile]
    public struct MeshIntegrateJob : IJobParallelFor
    {
        public MeshParams M;
        public float dt;

        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos, compAxis;
        [ReadOnly] public NativeArray<float> areaOld, rClose, rOpen, rShear, rSub, rOver, rCrumple;
        [ReadOnly] public NativeArray<uint> areaMask;
        [ReadOnly] public NativeArray<float> tension;

        public NativeArray<float> thickness, age, density, stiffness, orogeny, extension, arc, shear, faultActivity, damage;
        public NativeArray<float> flashSplit, flashCollapse;
        public NativeArray<byte> faultType;
        public NativeArray<float3> faultDir;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            TectonicParams P = M.P;
            float3 pi = pos[i];

            // same triangle subset as MeshAreaJob (topology has not changed in between)
            uint mask = areaMask[i];
            float area = 0f, areaAll = 0f;
            int c0 = vc[i], c = c0, guard = 0;
            do
            {
                float triArea = 0.5f * math.length(math.cross(pos[V[MeshTopo.Next(c)]] - pi, pos[V[MeshTopo.Prev(c)]] - pi));
                areaAll += triArea;
                if (guard < 32 && (mask & (1u << guard)) != 0u) area += triArea;
                c = MeshTopo.Next(O[MeshTopo.Next(c)]);
            } while (c != c0 && ++guard < MeshTopo.MaxRing);
            area /= 3f; areaAll /= 3f;

            // --- crust volume is conserved: losing area thickens, gaining area thins ---
            // Only the in-crust triangles' area change counts, spread over the vertex's whole dual area: at a margin the
            // boundary triangles keep their thickness (their area change is consumed or created crust, not strain).
            float ratio = mask == 0u ? 1f : math.clamp(1f + (areaOld[i] - area) / math.max(areaAll, 1e-12f), 0.5f, 2f);
            float t = thickness[i];
            float cf = TMath.CrustFrac(t, P);
            if (ratio > 1f)   // buoyant crust (cf >= 0.5) conserves volume fully; oceanic crust mostly subducts instead
                t += t * (ratio - 1f) * math.lerp(M.oceanCompress, 1f, math.saturate(cf * 2f));
            else
            {
                extension[i] = extension[i] + (1f - ratio) * cf;
                t = math.max(P.oceanThick, t * ratio);   // new oceanic floor fills in at oceanic thickness
            }
            if (t > P.contThick) t = P.contThick + (t - P.contThick) * math.exp(-P.erosion * dt);
            t = math.min(t, P.maxThickness);
            thickness[i] = t;

            // area gained by oceanic crust is new seafloor (age 0): mean age is area-weighted
            float a = age[i];
            if (ratio < 1f && cf < 0.5f) a *= ratio;
            a += dt;
            age[i] = a;

            float cl = rClose[i], op = rOpen[i], sh = rShear[i], crumple = rCrumple[i], over = rOver[i];
            float od = math.exp(-dt / P.orogenyDecay);
            orogeny[i] = orogeny[i] * od + (crumple + P.overThick * over) * dt * P.orogenyGain;
            arc[i] = arc[i] * math.exp(-dt / P.arcDecay) + over * dt * P.arcGain;
            shear[i] = shear[i] * od + sh * dt;

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

            float D = MeshLimits.UpdateDamage(damage[i], tension[i], t, a, dt, M);
            damage[i] = D;

            density[i] = TMath.Density(t, a, P);
            stiffness[i] = MeshLimits.Stiffness(t, a, faultActivity[i], D, M);

            float fd = math.exp(-dt / M.flashDecay);
            flashSplit[i] = flashSplit[i] * fd;
            flashCollapse[i] = flashCollapse[i] * fd;
        }
    }

    /// <summary>Tangential Laplacian smoothing. Moves the mesh, not the crust, so keep it weak.</summary>
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

    // ---------------------------------------------------------------------------------
    // All topology changes. Single-threaded: there are few edits per step and serial order keeps
    // runs deterministic. Passes, in order:
    //   0. apply relaxation
    //   1. repair inverted triangles (forced collapse of their shortest edge)
    //   2. Delaunay flips
    //   3. vertex/edge collisions: a vertex about to cross its opposite edge -> split that edge at the
    //      foot point, then collapse the vertex into it (this is where most subduction happens)
    //   4. collapse edges shorter than Lmin (denser side consumed if the pair can subduct, else merge)
    //   5. split edges longer than Lmax (the hard spacing cap); opening edges make fresh crust
    //   6. Delaunay flips again, then stats
    // ---------------------------------------------------------------------------------
    [BurstCompile]
    public struct MeshTopologyJob : IJob
    {
        public MeshParams M;
        public bool applyRelax;
        public int flipPasses, splitPasses;

        public NativeArray<int> V, O, vc, freeV, freeT, counters, nearest, consumedCell;
        public NativeArray<byte> triAlive, alive, faultType;
        public NativeArray<float3> pos, posTmp, vel, velNext, faultDir, slab, consumedPos;
        public NativeArray<float> thickness, age, density, stiffness, orogeny, extension, arc, shear, hotspot, faultActivity, damage;
        public NativeArray<float> flashSplit, flashCollapse, stats, areaFull;

        int _splits, _collapses, _flips, _subducted, _capHits, _rifts;

        // ============================================================== execute
        public void Execute()
        {
            Begin();
            PassRepair();
            FlipPass();
            PassCollisions();
            PassCollapse();
            PassSplit();
            End();
        }

        public void Begin()
        {
            _splits = _collapses = _flips = _subducted = _capHits = _rifts = 0;
            if (applyRelax)
                for (int v = 0; v < alive.Length; v++) if (alive[v] == 1) pos[v] = posTmp[v];
        }

        /// <summary>1) inverted triangles: forced collapse of their shortest edge</summary>
        public void PassRepair()
        {
            int tCap = triAlive.Length;
            for (int t = 0; t < tCap; t++)
            {
                if (triAlive[t] == 0) continue;
                int b = t * 3;
                if (MeshTopo.OrientOK(pos[V[b]], pos[V[b + 1]], pos[V[b + 2]], 0f)) continue;
                int best = b; float bestL = float.MaxValue;
                for (int k = 0; k < 3; k++)
                {
                    float l = EdgeLen2(b + k);
                    if (l < bestL) { bestL = l; best = b + k; }
                }
                TryCollapseEdge(best, true);
            }
        }

        /// <summary>3) vertex/edge collisions</summary>
        public void PassCollisions()
        {
            int tCap = triAlive.Length;
            float lmin15 = 1.5f * M.Lmin;
            for (int t = 0; t < tCap; t++)
            {
                if (triAlive[t] == 0) continue;
                for (int k = 0; k < 3 && triAlive[t] == 1; k++)
                {
                    int c = t * 3 + k;
                    float3 p = pos[V[c]], q = pos[V[MeshTopo.Next(c)]], r = pos[V[MeshTopo.Prev(c)]];
                    float3 er = r - q;
                    float len2 = math.lengthsq(er);
                    if (len2 < lmin15 * lmin15) continue;   // short base: the collapse pass handles it
                    float u = math.dot(p - q, er) / len2;
                    if (u < 0.15f || u > 0.85f) continue;
                    float h = math.length(math.cross(er, p - q)) / math.sqrt(len2);
                    if (h > M.sliverH) continue;
                    int vp = V[c], vq = V[MeshTopo.Next(c)], vr = V[MeshTopo.Prev(c)];
                    bool consumeP = (CanSubduct(vp, vq) || CanSubduct(vp, vr))
                                    && density[vp] > math.lerp(density[vq], density[vr], u);
                    if (consumeP)
                    {
                        // dense crust running into a lighter edge goes under: consume it directly via its edge to
                        // q or r (TryCollapseEdge removes the denser vertex). Never create crust at the contact.
                        int first = u < 0.5f ? MeshTopo.Prev(c) : MeshTopo.Next(c);   // edge p-q is opposite Prev(c)
                        int second = u < 0.5f ? MeshTopo.Next(c) : MeshTopo.Prev(c);
                        if (TryCollapseEdge(first, false) || TryCollapseEdge(second, false)) break;
                    }

                    // otherwise put a vertex on the edge where p meets it and collapse p into it. If p is being
                    // consumed the new vertex carries p's own crust, so the edge's material isn't duplicated.
                    int m = TrySplit(c, u, false);
                    if (m < 0) continue;
                    if (consumeP) CopyCrust(m, vp);
                    // after the split, the edge p-m is opposite Next(c) in this triangle
                    TryCollapseEdge(MeshTopo.Next(c), false);
                    break;
                }
            }
        }

        /// <summary>4) collapse edges shorter than Lmin, and converging buoyant edges shorter than Lmerge
        /// (resisted compression thickens the crust by merging instead of packing the mesh)</summary>
        public void PassCollapse()
        {
            int tCap = triAlive.Length;
            float lmin2 = M.Lmin * M.Lmin;
            float lmerge2 = M.Lmerge * M.Lmerge;
            for (int t = 0; t < tCap; t++)
            {
                for (int k = 0; k < 3; k++)
                {
                    if (triAlive[t] == 0) break;
                    int c = t * 3 + k;
                    if (c > O[c]) continue;   // each edge once
                    float l2 = EdgeLen2(c);
                    // a refused collapse is retried without the geometry check once the edge is badly short;
                    // any triangle that inverts is repaired by the repair pass next step
                    if (l2 < lmin2) { if (!TryCollapseEdge(c, false) && l2 < 0.25f * lmin2) TryCollapseEdge(c, true); }
                    else if (l2 < lmerge2 && ConvergingBuoyant(c, l2)) TryCollapseEdge(c, false);
                }
            }
        }

        /// <summary>5) + 6) splits (the hard spacing cap) alternating with flips: a flip can create a long edge,
        /// so we finish on a split pass that finds nothing to do</summary>
        public void PassSplit()
        {
            int tCap = triAlive.Length;
            float lmax2 = M.Lmax * M.Lmax;
            for (int pass = 0; pass < splitPasses; pass++)
            {
                int before = _splits;
                for (int t = 0; t < tCap; t++)
                {
                    if (triAlive[t] == 0) continue;
                    for (int k = 0; k < 3; k++)
                    {
                        int c = t * 3 + k;
                        if (c > O[c]) continue;
                        if (EdgeLen2(c) > lmax2) TrySplit(c, 0.5f, true);
                    }
                }
                int flipsBefore = _flips;
                FlipPass();
                if (_splits == before && _flips == flipsBefore) break;
            }
        }

        /// <summary>stats + counters</summary>
        public void End()
        {
            int tCap = triAlive.Length;
            float minL = float.MaxValue, maxL = 0f;
            int inverted = 0;
            for (int t = 0; t < tCap; t++)
            {
                if (triAlive[t] == 0) continue;
                int b = t * 3;
                if (!MeshTopo.OrientOK(pos[V[b]], pos[V[b + 1]], pos[V[b + 2]], 0f)) inverted++;
                for (int k = 0; k < 3; k++)
                {
                    float l = EdgeLen2(b + k);
                    minL = math.min(minL, l); maxL = math.max(maxL, l);
                }
            }
            stats[0] = math.sqrt(minL) / M.L;
            stats[1] = math.sqrt(maxL) / M.L;

            counters[MeshTopo.Splits] += _splits;
            counters[MeshTopo.Collapses] += _collapses;
            counters[MeshTopo.Flips] += _flips;
            counters[MeshTopo.Subducted] += _subducted;
            counters[MeshTopo.CapHits] += _capHits;
            counters[MeshTopo.Rifts] += _rifts;
            counters[MeshTopo.Inverted] = inverted;
        }

        // ============================================================== helpers
        float EdgeLen2(int c) => math.lengthsq(pos[V[MeshTopo.Next(c)]] - pos[V[MeshTopo.Prev(c)]]);
        int Swing(int c) => MeshTopo.Next(O[MeshTopo.Next(c)]);

        void Link(int a, int b) { O[a] = b; O[b] = a; }

        int Valence(int v)
        {
            int c0 = vc[v], c = c0, n = 0;
            do { n++; c = Swing(c); } while (c != c0 && n < MeshTopo.MaxRing);
            return n;
        }

        bool HasNeighbor(int v, int w)
        {
            int c0 = vc[v], c = c0, n = 0;
            do { if (V[MeshTopo.Next(c)] == w) return true; c = Swing(c); } while (c != c0 && ++n < MeshTopo.MaxRing);
            return false;
        }

        int CornerOf(int tri, int v)
        {
            int b = tri * 3;
            if (V[b] == v) return b;
            if (V[b + 1] == v) return b + 1;
            return b + 2;
        }

        int AllocVertex()
        {
            int f = counters[MeshTopo.FreeV];
            if (f == 0) return -1;
            f--; counters[MeshTopo.FreeV] = f;
            int v = freeV[f];
            alive[v] = 1;
            counters[MeshTopo.AliveV] += 1;
            return v;
        }

        void FreeVertex(int v)
        {
            alive[v] = 0;
            int f = counters[MeshTopo.FreeV];
            freeV[f] = v; counters[MeshTopo.FreeV] = f + 1;
            counters[MeshTopo.AliveV] -= 1;
        }

        int AllocTri()
        {
            int f = counters[MeshTopo.FreeT];
            f--; counters[MeshTopo.FreeT] = f;
            int t = freeT[f];
            triAlive[t] = 1;
            counters[MeshTopo.AliveT] += 1;
            return t;
        }

        void FreeTri(int t)
        {
            triAlive[t] = 0;
            int b = t * 3;
            V[b] = V[b + 1] = V[b + 2] = -1;
            O[b] = O[b + 1] = O[b + 2] = -1;
            int f = counters[MeshTopo.FreeT];
            freeT[f] = t; counters[MeshTopo.FreeT] = f + 1;
            counters[MeshTopo.AliveT] -= 1;
        }

        /// <summary>Converging buoyant pair short enough to merge. The merge length shrinks from Lmerge (hot crust)
        /// toward Lmin as the crust cools: hot crust thickens freely, cold crust is held by the compression barrier.</summary>
        bool ConvergingBuoyant(int c, float l2)
        {
            int b = V[MeshTopo.Next(c)], d = V[MeshTopo.Prev(c)];
            if (CanSubduct(b, d)) return false;
            float sT = MeshLimits.ThermalStrength(0.5f * (age[b] + age[d]), M);
            float lm = math.lerp(M.Lmerge, M.Lmin, sT);
            if (l2 >= lm * lm) return false;
            float3 axis = math.normalizesafe(pos[d] - pos[b]);
            return math.dot(vel[d] - vel[b], axis) < -M.P.closeThreshold;
        }

        bool CanSubduct(int a, int b) => MeshLimits.Subducts(thickness[a], density[a], thickness[b], density[b], M);

        // ============================================================== flip
        public void FlipPass()
        {
            int tCap = triAlive.Length;
            for (int pass = 0; pass < flipPasses; pass++)
            {
                int before = _flips;
                for (int t = 0; t < tCap; t++)
                {
                    if (triAlive[t] == 0) continue;
                    for (int k = 0; k < 3; k++)
                    {
                        int c = t * 3 + k;
                        if (c > O[c]) continue;
                        if (ShouldFlip(c) && CanFlip(c)) { Flip(c); _flips++; }
                    }
                }
                if (_flips == before) break;
            }
        }

        /// <summary>On the sphere, Delaunay == convex hull: edge b-d is bad if e lies above the plane of (a, b, d).</summary>
        bool ShouldFlip(int c)
        {
            float3 a = pos[V[c]], b = pos[V[MeshTopo.Next(c)]], d = pos[V[MeshTopo.Prev(c)]], e = pos[V[O[c]]];
            float3 n = math.cross(b - a, d - a);
            float nl = math.length(n);
            if (nl < 1e-20f) return false;
            return math.dot(e - a, n) / nl > M.flipEps;
        }

        bool CanFlip(int c)
        {
            int o = O[c];
            int a = V[c], b = V[MeshTopo.Next(c)], d = V[MeshTopo.Prev(c)], e = V[o];
            if (a == e) return false;
            if (Valence(b) <= 3 || Valence(d) <= 3) return false;
            if (HasNeighbor(a, e)) return false;
            float3 pa = pos[a], pb = pos[b], pd = pos[d], pe = pos[e];
            return MeshTopo.OrientOK(pa, pb, pe, M.minOrient) && MeshTopo.OrientOK(pe, pd, pa, M.minOrient);
        }

        // T0 = (a, b, d), T1 = (e, d, b)  ->  T0 = (a, b, e), T1 = (e, d, a)
        void Flip(int c)
        {
            int o = O[c];
            int n0 = MeshTopo.Next(c), p0 = MeshTopo.Prev(c), n1 = MeshTopo.Next(o), p1 = MeshTopo.Prev(o);
            int a = V[c], b = V[n0], d = V[n1], e = V[o];
            int yBE = O[n1], yDA = O[n0];
            V[p0] = e; V[p1] = a;
            Link(c, yBE); Link(o, yDA); Link(n0, n1);
            vc[a] = c; vc[b] = n0; vc[d] = n1; vc[e] = o;
        }

        // ============================================================== split
        /// <summary>Inserts a vertex on the edge opposite c at parameter u from V[Next(c)]. Returns the new vertex or -1.</summary>
        int TrySplit(int c, float u, bool allowRift)
        {
            if (counters[MeshTopo.FreeV] < 1 || counters[MeshTopo.FreeT] < 2) { _capHits++; return -1; }
            int o = O[c];
            int n0 = MeshTopo.Next(c), p0 = MeshTopo.Prev(c), n1 = MeshTopo.Next(o), p1 = MeshTopo.Prev(o);
            int a = V[c], b = V[n0], d = V[p0], e = V[o];

            float3 pm = math.normalize(math.lerp(pos[b], pos[d], u));
            if (!MeshTopo.OrientOK(pos[a], pos[b], pm, M.minOrient) || !MeshTopo.OrientOK(pos[a], pm, pos[d], M.minOrient) ||
                !MeshTopo.OrientOK(pos[e], pos[d], pm, M.minOrient) || !MeshTopo.OrientOK(pos[e], pm, pos[b], M.minOrient))
                return -1;

            int m = AllocVertex();
            int T2 = AllocTri(), T3 = AllocTri();
            int yDA = O[n0], yBE = O[n1];
            int t20 = T2 * 3, t21 = t20 + 1, t22 = t20 + 2;
            int t30 = T3 * 3, t31 = t30 + 1, t32 = t30 + 2;

            // T0 = (a, b, m), T1 = (e, d, m), T2 = (a, m, d), T3 = (e, m, b)
            V[p0] = m; V[p1] = m;
            V[t20] = a; V[t21] = m; V[t22] = d;
            V[t30] = e; V[t31] = m; V[t32] = b;
            Link(c, t30); Link(n0, t22); Link(o, t20); Link(n1, t32);
            Link(t21, yDA); Link(t31, yBE);
            vc[m] = p0; vc[a] = c; vc[b] = n0; vc[d] = n1; vc[e] = o;

            InitSplitVertex(m, b, d, u, pm, allowRift);
            _splits++;
            return m;
        }

        /// <summary>
        /// New vertex on edge b-d. Two kinds of split:
        ///   rift split     -> genuinely new crust: age 0, thin, half the parents' damage (hot, still weak)
        ///   ordinary split -> mesh maintenance: interpolate everything (or copy the denser side across a crust boundary)
        /// A split is a rift when the edge is opening AND either the crust is oceanic (ridge: fast opening or an
        /// active divergent fault) or it is continental crust that has been fractured (damage >= riftDamage).
        /// Intact continental crust that merely stretches is refined, never turned into ocean floor here.
        /// </summary>
        bool InitSplitVertex(int m, int b, int d, float u, float3 pm, bool allowRift)
        {
            TectonicParams P = M.P;
            float3 axis = math.normalizesafe(pos[d] - pos[b]);
            float openRate = math.dot(vel[d] - vel[b], axis);
            bool divergentFault = (faultType[b] == (byte)FaultType.Divergent && faultActivity[b] > P.faultThreshold)
                               || (faultType[d] == (byte)FaultType.Divergent && faultActivity[d] > P.faultThreshold);
            float cfAvg = 0.5f * (TMath.CrustFrac(thickness[b], P) + TMath.CrustFrac(thickness[d], P));
            float dMax = math.max(damage[b], damage[d]);
            bool ridgeLike = openRate > M.riftOpen || divergentFault;
            bool rift;
            if (!allowRift) rift = false;
            else if (cfAvg < 0.5f || !M.extEnabled) rift = ridgeLike;          // oceanic ridges (and the old rule with extension off)
            else rift = openRate > 0f && dMax >= M.riftDamage;                  // continents only rift once fractured

            float3 v = math.lerp(vel[b], vel[d], u);
            v -= pm * math.dot(v, pm);
            pos[m] = pm; posTmp[m] = pm; vel[m] = v; velNext[m] = v;
            slab[m] = float3.zero;
            nearest[m] = u < 0.5f ? nearest[b] : nearest[d];
            float3 fd = u < 0.5f ? faultDir[b] : faultDir[d];
            faultDir[m] = math.normalizesafe(fd - pm * math.dot(fd, pm), new float3(1f, 0f, 0f));

            float srcT = math.lerp(thickness[b], thickness[d], u);
            float srcS = math.lerp(stiffness[b], stiffness[d], u);
            float act = math.max(faultActivity[b], faultActivity[d]);
            extension[m] = math.lerp(extension[b], extension[d], u);
            flashCollapse[m] = 0f;
            flashSplit[m] = 1f;

            if (rift)
            {
                // fresh crust. Thick/stiff surroundings leave transitional crust (failed-rift margins).
                // Thermal state comes for free: age 0 -> thermal age = the planet's global thermal age.
                float cf = TMath.CrustFrac(srcT, P);
                float nt = P.oceanThick + (srcT - P.oceanThick) * (P.riftBlend * cf * srcS);
                thickness[m] = nt; age[m] = 0f;
                orogeny[m] = 0f; arc[m] = 0f; shear[m] = 0f; hotspot[m] = 0f;
                faultType[m] = (byte)FaultType.Divergent;
                faultActivity[m] = act;
                damage[m] = 0.5f * dMax;
                _rifts++;
            }
            else if (MeshLimits.Boundary(thickness[b], density[b], thickness[d], density[d], M))
            {
                // the edge crosses a crust boundary: copy one side instead of blending, so margins stay sharp.
                // Copy the denser side: boundary edges mostly stretch at trenches, and copying the buoyant side
                // there would grow the overriding plate out over the slab.
                int src = density[b] >= density[d] ? b : d;
                thickness[m] = thickness[src]; age[m] = age[src];
                orogeny[m] = orogeny[src]; arc[m] = arc[src]; shear[m] = shear[src]; hotspot[m] = hotspot[src];
                extension[m] = extension[src];
                faultActivity[m] = faultActivity[src]; faultType[m] = faultType[src];
                damage[m] = damage[src];
            }
            else
            {
                // plain refinement: interpolate
                thickness[m] = srcT;
                age[m] = math.lerp(age[b], age[d], u);
                orogeny[m] = math.lerp(orogeny[b], orogeny[d], u);
                arc[m] = math.lerp(arc[b], arc[d], u);
                shear[m] = math.lerp(shear[b], shear[d], u);
                hotspot[m] = math.lerp(hotspot[b], hotspot[d], u);
                faultActivity[m] = math.lerp(faultActivity[b], faultActivity[d], u);
                faultType[m] = faultActivity[b] >= faultActivity[d] ? faultType[b] : faultType[d];
                damage[m] = math.lerp(damage[b], damage[d], u);
            }
            density[m] = TMath.Density(thickness[m], age[m], P);
            stiffness[m] = MeshLimits.Stiffness(thickness[m], age[m], faultActivity[m], damage[m], M);
            return rift;
        }

        // ============================================================== collapse
        /// <summary>Collapses the edge opposite c. Picks the survivor: a subducting pair loses its denser vertex
        /// (consumed); otherwise the softer vertex merges into the stiffer one.</summary>
        bool TryCollapseEdge(int c, bool force)
        {
            int o = O[c];
            int b = V[MeshTopo.Next(c)], d = V[MeshTopo.Prev(c)];
            bool sub = CanSubduct(b, d);

            bool removeD;
            float3 np;
            if (sub)
            {
                removeD = density[d] >= density[b];
                np = removeD ? pos[b] : pos[d];
            }
            else
            {
                float sb = stiffness[b], sd = stiffness[d];
                removeD = sd <= sb;
                np = math.normalize(math.lerp(pos[d], pos[b], sb / (sb + sd + 1e-6f)));
            }
            int cc = removeD ? c : o;
            int keep = removeD ? b : d, gone = removeD ? d : b;

            if (sub && !force)
            {
                // Consumed crust hands its area to its own plate (the slab behind it), not to the overriding
                // vertex: otherwise every subduction event would grow the overriding plate.
                int alt = BestSlabNeighbor(gone, keep);
                if (alt >= 0 && CanCollapse(alt, pos[V[MeshTopo.Next(alt)]], true))
                {
                    RecordConsumed(gone);
                    flashCollapse[keep] = 1f;
                    flashCollapse[V[MeshTopo.Next(alt)]] = 1f;
                    Collapse(alt, pos[V[MeshTopo.Next(alt)]]);
                    _collapses++;
                    return true;
                }
            }

            if (!CanCollapse(cc, np, !force))
            {
                // a merge that fails at the weighted point may still work with the survivor left in place
                if (sub || force) return false;
                np = pos[keep];
                if (!CanCollapse(cc, np, true)) return false;
            }

            if (sub) RecordConsumed(gone);
            else Merge(keep, gone);

            flashCollapse[keep] = 1f;
            Collapse(cc, np);
            _collapses++;
            return true;
        }

        void RecordConsumed(int gone)
        {
            int s = counters[MeshTopo.ConsumedThisStep];
            consumedCell[s] = nearest[gone]; consumedPos[s] = pos[gone];
            counters[MeshTopo.ConsumedThisStep] = s + 1;
            _subducted++;
            if (TMath.CrustFrac(thickness[gone], M.P) > 0.5f) counters[MeshTopo.ContinentalSubducted] += 1;
        }

        /// <summary>The corner whose collapse removes <paramref name="gone"/> into the same-crust neighbour farthest
        /// from <paramref name="over"/> (i.e. back along the subducting plate). -1 if there is none.</summary>
        int BestSlabNeighbor(int gone, int over)
        {
            float3 pg = pos[gone];
            float3 away = math.normalizesafe(pg - pos[over]);
            int best = -1; float bestScore = 0f;
            int c0 = vc[gone], cc = c0, n = 0;
            do
            {
                int x = V[MeshTopo.Next(cc)];
                if (x != over && !MeshLimits.Boundary(thickness[gone], density[gone], thickness[x], density[x], M))
                {
                    float score = math.dot(math.normalizesafe(pos[x] - pg), away);
                    if (score > bestScore) { bestScore = score; best = O[MeshTopo.Prev(cc)]; }   // removes gone, keeps x
                }
                cc = Swing(cc);
            } while (cc != c0 && ++n < MeshTopo.MaxRing);
            return best;
        }

        void CopyCrust(int dst, int src)
        {
            thickness[dst] = thickness[src]; age[dst] = age[src]; density[dst] = density[src]; stiffness[dst] = stiffness[src];
            orogeny[dst] = orogeny[src]; extension[dst] = extension[src]; arc[dst] = arc[src]; shear[dst] = shear[src];
            hotspot[dst] = hotspot[src]; faultActivity[dst] = faultActivity[src]; faultType[dst] = faultType[src];
            damage[dst] = damage[src];
            vel[dst] = vel[src]; velNext[dst] = velNext[src];
        }

        /// <summary>Merge g into k. Weighted by dual area, so thickness x area (crust volume) is conserved exactly;
        /// the survivor's dual area afterwards is close to the sum of both.</summary>
        void Merge(int k, int g)
        {
            TectonicParams P = M.P;
            float ak = math.max(areaFull[k], 1e-12f), ag = math.max(areaFull[g], 1e-12f);
            float w = ag / (ak + ag);
            float3 v = math.lerp(vel[k], vel[g], w);
            vel[k] = v; velNext[k] = v;
            thickness[k] = math.lerp(thickness[k], thickness[g], w);
            age[k] = math.lerp(age[k], age[g], w);
            orogeny[k] = math.lerp(orogeny[k], orogeny[g], w);
            extension[k] = math.lerp(extension[k], extension[g], w);
            arc[k] = math.lerp(arc[k], arc[g], w);
            shear[k] = math.lerp(shear[k], shear[g], w);
            hotspot[k] = math.lerp(hotspot[k], hotspot[g], w);
            if (faultActivity[g] > faultActivity[k]) { faultActivity[k] = faultActivity[g]; faultType[k] = faultType[g]; }
            damage[k] = math.max(damage[k], damage[g]);   // a fracture survives a merge
            areaFull[k] = ak + ag;
            density[k] = TMath.Density(thickness[k], age[k], P);
            stiffness[k] = MeshLimits.Stiffness(thickness[k], age[k], faultActivity[k], damage[k], M);
        }

        /// <summary>Link condition + valence + (optionally) no flipped or over-long triangles. Removes V[Prev(c)].</summary>
        bool CanCollapse(int c, float3 np, bool checkGeometry)
        {
            int o = O[c];
            int a = V[c], b = V[MeshTopo.Next(c)], d = V[MeshTopo.Prev(c)], e = V[o];
            if (a == e) return false;
            if (Valence(a) <= 3 || Valence(e) <= 3) return false;
            if (Valence(b) + Valence(d) - 4 > M.maxValence) return false;

            // link condition: the one-rings of b and d share only a and e
            var ringB = new FixedList512Bytes<int>();
            int c0 = vc[b], cc = c0, n = 0;
            do
            {
                if (ringB.Length < 120) ringB.Add(V[MeshTopo.Next(cc)]);
                cc = Swing(cc);
            } while (cc != c0 && ++n < MeshTopo.MaxRing);

            c0 = vc[d]; cc = c0; n = 0;
            do
            {
                int x = V[MeshTopo.Next(cc)];
                if (x != a && x != e && x != b)
                    for (int k = 0; k < ringB.Length; k++) if (ringB[k] == x) return false;
                cc = Swing(cc);
            } while (cc != c0 && ++n < MeshTopo.MaxRing);

            if (!checkGeometry) return true;

            int T0 = c / 3, T1 = o / 3;
            float lmax2 = M.Lmax * M.Lmax;
            if (!RingStaysValid(d, np, T0, T1, lmax2)) return false;
            if (math.any(np != pos[b]) && !RingStaysValid(b, np, T0, T1, lmax2)) return false;
            return true;
        }

        bool RingStaysValid(int v, float3 np, int T0, int T1, float lmax2)
        {
            int c0 = vc[v], c = c0, n = 0;
            do
            {
                int t = c / 3;
                if (t != T0 && t != T1)
                {
                    float3 q = pos[V[MeshTopo.Next(c)]], r = pos[V[MeshTopo.Prev(c)]];
                    if (!MeshTopo.OrientOK(np, q, r, M.minOrient)) return false;
                    if (math.lengthsq(q - np) > lmax2) return false;
                }
                c = Swing(c);
            } while (c != c0 && ++n < MeshTopo.MaxRing);
            return true;
        }

        // T0 = (a, b, d), T1 = (e, d, b). d is merged into b and both triangles disappear.
        void Collapse(int c, float3 np)
        {
            int o = O[c];
            int n0 = MeshTopo.Next(c), p0 = MeshTopo.Prev(c), n1 = MeshTopo.Next(o), p1 = MeshTopo.Prev(o);
            int a = V[c], b = V[n0], d = V[p0], e = V[o];
            int x0 = O[n0];   // across d-a
            int y0 = O[p0];   // across a-b
            int x1 = O[n1];   // across b-e
            int y1 = O[p1];   // across e-d

            int c0 = vc[d], cc = c0, n = 0;
            do { V[cc] = b; cc = Swing(cc); } while (cc != c0 && ++n < MeshTopo.MaxRing);

            Link(x0, y0);
            Link(x1, y1);
            FreeTri(c / 3);
            FreeTri(o / 3);
            vc[a] = CornerOf(x0 / 3, a);
            vc[e] = CornerOf(x1 / 3, e);
            vc[b] = CornerOf(y0 / 3, b);

            pos[b] = np; posTmp[b] = np;
            FreeVertex(d);
        }
    }

    /// <summary>
    /// Mantle feedback per convection cell (area-weighted, so mesh density doesn't bias it):
    ///   feedback' = insulationHeat * cov                          continents trap heat
    ///             - surfaceCooling * (1 - cov) * (temp + 1) / 2   exposed mantle loses heat; hotter cells lose more,
    ///                                                             so exposure never drives a cell far below zero
    ///             - subductionCool * consumedAreaFraction / dt    cold slabs
    ///             - feedbackRelax * feedback
    ///   temp = clamp(baseTemp + feedback). baseTemp starts random and relaxes toward 0 at feedbackRelax, driven by
    ///   tempNoise, so long-term behaviour comes from the feedback rather than the initial dice roll.
    /// </summary>
    [BurstCompile]
    public struct MeshFeedbackJob : IJob
    {
        public TectonicParams P;
        public float dt, surfaceCool;
        public uint step;

        public NativeArray<Convection> conv;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos;
        [ReadOnly] public NativeArray<float> thickness, areaFull;
        [ReadOnly] public NativeArray<int> nearest;
        [ReadOnly] public NativeArray<int> consumedCell;
        [ReadOnly] public NativeArray<float3> consumedPos;
        public NativeArray<int> counters;

        public NativeArray<float3> sumCont, sumSub;
        public NativeArray<float> cellArea, contArea;
        public NativeArray<int> subCnt;

        public void Execute()
        {
            int N = conv.Length;
            for (int k = 0; k < N; k++)
            {
                sumCont[k] = float3.zero; sumSub[k] = float3.zero;
                cellArea[k] = 0f; contArea[k] = 0f; subCnt[k] = 0;
            }

            float contLimit = P.contThick * 0.75f;
            for (int i = 0; i < alive.Length; i++)
            {
                if (alive[i] == 0) continue;
                int k = nearest[i];
                float A = areaFull[i];
                cellArea[k] = cellArea[k] + A;
                if (thickness[i] > contLimit) { contArea[k] = contArea[k] + A; sumCont[k] = sumCont[k] + pos[i] * A; }
            }
            int consumed = counters[MeshTopo.ConsumedThisStep];
            for (int s = 0; s < consumed; s++)
            {
                int k = consumedCell[s];
                subCnt[k] = subCnt[k] + 1; sumSub[k] = sumSub[k] + consumedPos[s];
            }
            counters[MeshTopo.ConsumedThisStep] = 0;
            float meanVertexArea = 4f * math.PI / math.max(counters[MeshTopo.AliveV], 1);

            for (int k = 0; k < N; k++)
            {
                Convection c = conv[k];
                float area = math.max(cellArea[k], 1e-6f);
                float cov = contArea[k] / area;
                float subRate = subCnt[k] * meanVertexArea / area / dt;
                float cooling = surfaceCool * (1f - cov) * math.saturate((c.temp + 1f) * 0.5f);

                c.feedback += dt * (P.insulationHeat * cov - cooling - P.subductionCool * subRate - c.feedback * P.feedbackRelax);
                float n = TMath.Rand01((uint)k * 977u + step * 131u + P.seed) * 2f - 1f;
                // baseTemp is mean-reverting (same rate as the feedback) so that over Gyr it is the noise, not the
                // random initial values, that it carries: typical size tempNoise / sqrt(2 * feedbackRelax)
                c.baseTemp = math.clamp(c.baseTemp * math.exp(-P.feedbackRelax * dt) + n * P.tempNoise * math.sqrt(dt), -1f, 1f);
                c.temp = math.clamp(c.baseTemp + c.feedback, -1.5f, 1.5f);

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

    // ---------------------------------------------------------------------------------
    // Live preview: the simulation mesh itself. One render vertex per vertex slot plus one spare
    // "dead" vertex at the origin that unused triangles and edges collapse onto.
    // ---------------------------------------------------------------------------------
    public enum MeshDebugView { Elevation, Thickness, Age, FaultType, Orogeny, Hotspot, Velocity, EdgeLength, Valence, TopologyEvents, Thermal, Damage }

    [BurstCompile]
    public struct MeshPreviewVertexJob : IJobParallelFor
    {
        public MeshParams M;
        public int view;
        public float radius;

        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive, faultType;
        [ReadOnly] public NativeArray<float3> pos, vel;
        [ReadOnly] public NativeArray<float> thickness, age, faultActivity, orogeny, hotspot, flashSplit, flashCollapse, damage, tension;

        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<float3> verts;
        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<UnityEngine.Color32> colors;

        public void Execute(int i)
        {
            if (i == alive.Length) { verts[i] = float3.zero; colors[i] = new UnityEngine.Color32(0, 0, 0, 0); return; }
            if (alive[i] == 0) { verts[i] = float3.zero; colors[i] = new UnityEngine.Color32(0, 0, 0, 0); return; }
            verts[i] = pos[i] * radius;
            float4 col = math.saturate(ColorFor(i));
            colors[i] = new UnityEngine.Color32((byte)(col.x * 255f + 0.5f), (byte)(col.y * 255f + 0.5f), (byte)(col.z * 255f + 0.5f), 255);
        }

        static float3 Hsv(float h, float s, float v)
        {
            float3 k = math.saturate(math.abs(math.frac(h + new float3(1f, 2f / 3f, 1f / 3f)) * 6f - 3f) - 1f);
            return v * math.lerp(new float3(1f), k, s);
        }

        static float4 FaultColor(int type)
        {
            switch (type)
            {
                case 1: return new float4(1f, 0f, 0f, 1f);
                case 2: return new float4(1f, 0.55f, 0f, 1f);
                case 3: return new float4(1f, 0f, 1f, 1f);
                case 4: return new float4(0f, 1f, 1f, 1f);
                default: return new float4(0.4f, 0.4f, 0.4f, 1f);
            }
        }

        float4 Elevation(int i)
        {
            float e = TMath.ElevationKm(thickness[i], age[i], M.P);
            if (e < 0f) return math.lerp(new float4(0f, 0.05f, 0.35f, 1f), new float4(0.2f, 0.55f, 0.85f, 1f), math.saturate((e + 5f) / 5f));
            return math.lerp(new float4(0.25f, 0.6f, 0.2f, 1f), new float4(1f), math.saturate(e / 5f));
        }

        /// <summary>Symbolic temperature ramp. heat 1 = molten, 0 = cold. Darkest stop stays at ~20% grey so cold
        /// crust is visible against the editor background and the wireframe.</summary>
        public static float4 ThermalColor(float heat)
        {
            float3 c0 = new float3(0.20f, 0.20f, 0.21f);   // cold: charcoal
            float3 c1 = new float3(0.36f, 0.29f, 0.24f);   // cool: brown-grey
            float3 c2 = new float3(0.58f, 0.13f, 0.03f);   // cooling: dark red-orange
            float3 c3 = new float3(1.00f, 0.42f, 0.04f);   // hot: magma orange
            float3 c4 = new float3(1.00f, 0.80f, 0.28f);   // extremely hot: yellow-orange
            float h = math.saturate(heat) * 4f;
            float3 c = h < 1f ? math.lerp(c0, c1, h) : h < 2f ? math.lerp(c1, c2, h - 1f) : h < 3f ? math.lerp(c2, c3, h - 2f) : math.lerp(c3, c4, h - 3f);
            return new float4(c, 1f);
        }

        float4 ColorFor(int i)
        {
            float4 white = 1f, black = new float4(0f, 0f, 0f, 1f);
            switch (view)
            {
                case 1: return math.lerp(new float4(0f, 0f, 1f, 1f), white, math.saturate(thickness[i] / M.P.maxThickness));
                case 2: return math.lerp(new float4(1f, 0.92f, 0.016f, 1f), new float4(0.2f, 0f, 0.4f, 1f), math.saturate(age[i] / 300f));
                case 3: return math.lerp(FaultColor(0), FaultColor(faultType[i]), math.saturate(faultActivity[i] * 3f));
                case 4: return math.lerp(black, white, math.saturate(orogeny[i]));
                case 5: return math.lerp(black, new float4(1f, 0f, 0f, 1f), math.saturate(hotspot[i] * 0.5f));
                case 6:
                {
                    float3 n = pos[i], v = vel[i];
                    TMath.TangentFrame(n, out float3 t, out float3 bt);
                    float h = math.atan2(math.dot(v, bt), math.dot(v, t)) / (2f * math.PI) + 1f;
                    float sp = math.saturate(math.length(v) / M.P.velRef);
                    return new float4(Hsv(math.frac(h), 0.85f, 0.25f + 0.75f * sp), 1f);
                }
                case 7: // EdgeLength: mean incident edge / target. Blue = short (dense), white = target, red = long.
                {
                    float sum = 0f; int n = 0;
                    int c0 = vc[i], c = c0;
                    do { sum += math.length(pos[V[MeshTopo.Next(c)]] - pos[i]); n++; c = MeshTopo.Next(O[MeshTopo.Next(c)]); }
                    while (c != c0 && n < MeshTopo.MaxRing);
                    float r = sum / n / M.L;
                    if (r < 1f) return math.lerp(new float4(0.05f, 0.2f, 1f, 1f), white, math.saturate((r - M.Lmin / M.L) / (1f - M.Lmin / M.L)));
                    return math.lerp(white, new float4(1f, 0.1f, 0.05f, 1f), math.saturate((r - 1f) / (M.Lmax / M.L - 1f)));
                }
                case 8: // Valence: 4- blue, 5 cyan, 6 grey, 7 yellow, 8+ red
                {
                    int n = 0, c0 = vc[i], c = c0;
                    do { n++; c = MeshTopo.Next(O[MeshTopo.Next(c)]); } while (c != c0 && n < MeshTopo.MaxRing);
                    if (n <= 4) return new float4(0.1f, 0.2f, 1f, 1f);
                    if (n == 5) return new float4(0.2f, 0.85f, 1f, 1f);
                    if (n == 6) return new float4(0.55f, 0.55f, 0.55f, 1f);
                    if (n == 7) return new float4(1f, 0.85f, 0.1f, 1f);
                    return new float4(1f, 0.1f, 0.05f, 1f);
                }
                case 9: // TopologyEvents: dimmed elevation, green = just split, orange = just collapsed into
                {
                    float4 b = Elevation(i) * 0.35f;
                    b += new float4(0.1f, 1f, 0.2f, 0f) * flashSplit[i];
                    b += new float4(1f, 0.45f, 0f, 0f) * flashCollapse[i];
                    b.w = 1f;
                    return b;
                }
                case 10: // Thermal: 1 - thermal strength of (age + global thermal age)
                    return ThermalColor(1f - MeshLimits.ThermalStrength(age[i], M));
                case 11: // Damage: dimmed elevation, yellow -> red with damage, blue tint where tension exceeds yield
                {
                    float4 b = Elevation(i) * 0.3f;
                    float D = damage[i];
                    float3 dc = D < 0.5f ? math.lerp(new float3(0.3f, 0.3f, 0.3f), new float3(1f, 0.85f, 0.1f), D * 2f)
                                         : math.lerp(new float3(1f, 0.85f, 0.1f), new float3(1f, 0.05f, 0.02f), D * 2f - 1f);
                    float3 rgb = math.lerp(b.xyz, dc, math.saturate(D * 4f));
                    if (tension[i] > M.extYield) rgb += new float3(0f, 0.1f, 0.35f) * math.saturate((tension[i] - M.extYield) / math.max(M.extYield, 1e-3f));
                    return new float4(rgb, 1f);
                }
                default: return Elevation(i);
            }
        }
    }

    [BurstCompile]
    public struct MeshPreviewIndexJob : IJobParallelFor
    {
        public int deadVertex;
        public bool lines;
        [ReadOnly] public NativeArray<int> V, O;
        [ReadOnly] public NativeArray<byte> triAlive;
        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<int> triIndices;    // 3 per triangle
        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<int> lineIndices;   // 6 per triangle

        public void Execute(int t)
        {
            int b = t * 3, l = t * 6;
            bool on = triAlive[t] == 1;
            for (int k = 0; k < 3; k++)
            {
                int c = b + k;
                triIndices[c] = on ? V[c] : deadVertex;
                if (!lines) continue;
                bool edge = on && c < O[c];   // each edge drawn once
                lineIndices[l + 2 * k] = edge ? V[MeshTopo.Next(c)] : deadVertex;
                lineIndices[l + 2 * k + 1] = edge ? V[MeshTopo.Prev(c)] : deadVertex;
            }
        }
    }
}
