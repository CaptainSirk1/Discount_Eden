using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace PlanetGen.Tectonics
{
    // ---------------------------------------------------------------------------------
    // Step pipeline (chained, never completed mid-simulation):
    //   BuildGrid  ||  ConvectionDrive      (parallel with each other)
    //   -> Interaction   (neighbour queries: coupling, gravity lock, faults, subduction, spawn proposals)
    //   -> Integrate     (move, thicken/thin, accumulate terrain data)
    //   -> ConvectionFeedback (continents insulate, slabs cool, cells drift)
    //   -> Structure     (apply removals, spawn new crust with spacing check)
    //   [-> BuildGrid -> RemeshGather -> RemeshApply -> RemeshFinalize]  (optional, every N steps)
    // Every parallel job writes only index i and reads neighbours from arrays it does not write.
    // ---------------------------------------------------------------------------------

    [BurstCompile]
    public struct BuildGridJob : IJob
    {
        public TectonicParams P;
        [ReadOnly] public NativeArray<float3> pos;
        [ReadOnly] public NativeArray<byte> alive;
        public NativeArray<int> cellStart;   // cells + 1
        public NativeArray<int> cursor;      // cells
        public NativeArray<int> cellItems;   // capacity
        public NativeArray<int> cellOf;      // capacity

        public void Execute()
        {
            int cells = cursor.Length;
            for (int c = 0; c <= cells; c++) cellStart[c] = 0;

            for (int i = 0; i < pos.Length; i++)
            {
                if (alive[i] == 0) continue;
                int c = TMath.Cell(pos[i], P);
                cellOf[i] = c;
                cellStart[c + 1] = cellStart[c + 1] + 1;
            }
            for (int c = 0; c < cells; c++) cellStart[c + 1] = cellStart[c + 1] + cellStart[c];
            for (int c = 0; c < cells; c++) cursor[c] = cellStart[c];

            for (int i = 0; i < pos.Length; i++)
            {
                if (alive[i] == 0) continue;
                int c = cellOf[i];
                int slot = cursor[c];
                cellItems[slot] = i;
                cursor[c] = slot + 1;
            }
        }
    }

    [BurstCompile]
    public struct ConvectionDriveJob : IJobParallelFor
    {
        public TectonicParams P;
        public float dt;
        [ReadOnly] public NativeArray<Convection> conv;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos;
        [ReadOnly] public NativeArray<float3> slab;
        [ReadOnly] public NativeArray<float> density;
        [ReadOnly] public NativeArray<float> age;
        public NativeArray<float3> driven;
        public NativeArray<int> nearest;
        public NativeArray<float> hotspot;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            float3 p = pos[i];
            float3 f = float3.zero;
            float bestCos = -2f; int best = 0;

            for (int k = 0; k < conv.Length; k++)
            {
                float3 c = conv[k].pos;
                float cs = math.dot(p, c);
                if (cs > bestCos) { bestCos = cs; best = k; }

                float theta = math.acos(math.clamp(cs, -1f, 1f));
                float x = theta / P.convRadius;
                if (x > 4f) continue;

                // surface flow profile: zero over the centre, peak near x = 1, long soft tail
                float w = x * math.exp(1f - x);
                float3 away = p - c * cs;
                float l = math.length(away);
                if (l < 1e-6f) continue;
                away /= l;

                float T = conv[k].temp;
                f += away * (T * w);

                if (T > P.plumeThreshold && x < P.plumeRadius)
                    hotspot[i] = hotspot[i] + T * dt * (1f - x / P.plumeRadius);
            }

            nearest[i] = best;
            float R = TMath.Resistance(density[i], age[i], P);
            driven[i] = (f * P.convStrength + slab[i]) / R;
        }
    }

    [BurstCompile]
    public struct InteractionJob : IJobParallelFor
    {
        public const int SpawnNone = -1;
        public const int SpawnVoid = -2;   // spawnPartner value for a density-based fill (no partner)

        public TectonicParams P;
        public float dt;
        public uint step;

        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos;
        [ReadOnly] public NativeArray<float3> vel;
        [ReadOnly] public NativeArray<float3> driven;
        [ReadOnly] public NativeArray<float> thickness;
        [ReadOnly] public NativeArray<float> density;
        [ReadOnly] public NativeArray<float> stiffness;
        [ReadOnly] public NativeArray<int> cellStart;
        [ReadOnly] public NativeArray<int> cellItems;

        public NativeArray<float3> velNext;
        public NativeArray<float3> sepDisp;
        public NativeArray<float3> compAxis;
        public NativeArray<float3> slab;
        public NativeArray<float> rClose;
        public NativeArray<float> rOpen;
        public NativeArray<float> rShear;
        public NativeArray<float> rSub;
        public NativeArray<float> rOver;
        public NativeArray<float> rCrumple;
        public NativeArray<float> crowding;
        public NativeArray<byte> removeFlag;
        public NativeArray<int> spawnPartner;
        public NativeArray<float3> spawnPos;

        float Headroom(float t) => math.saturate((t - P.lockStart) / P.lockMargin);

        public void Execute(int i)
        {
            removeFlag[i] = 0;
            spawnPartner[i] = SpawnNone;
            if (alive[i] == 0) { crowding[i] = 0f; return; }

            float3 pi = pos[i];
            float3 vi = vel[i];
            float ti = thickness[i], di = density[i], si = stiffness[i];
            int3 ci = (int3)math.floor((pi + 1f) * P.invCell);
            ci = math.clamp(ci, new int3(0), new int3(P.gridDim - 1));

            float3 coupledSum = vi * si;
            float wSum = si + 1e-4f;

            float closeSum = 0f, openSum = 0f, shearSum = 0f, wTot = 0f;
            float subSum = 0f, subW = 0f, overSum = 0f, crumpleSum = 0f;
            float3 sep = float3.zero, slabF = float3.zero;
            float bestAbs = 0f; float3 axis = float3.zero;
            float bestGap = 0f; int gapJ = -1;

            int count = 0; float3 dirSum = float3.zero;
            bool consumed = false;
            float headI = Headroom(ti);

            // gravity-lock constraints, resolved after the new velocity is known: (dir, vel_j . dir), amount
            var lockDir = new FixedList512Bytes<float4>();
            var lockAmt = new FixedList128Bytes<float>();

            for (int dz = -1; dz <= 1; dz++)
            for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++)
            {
                int3 c = ci + new int3(dx, dy, dz);
                if (math.any(c < 0) || math.any(c >= P.gridDim)) continue;
                int cell = c.x + P.gridDim * (c.y + P.gridDim * c.z);
                int end = cellStart[cell + 1];
                for (int s = cellStart[cell]; s < end; s++)
                {
                    int j = cellItems[s];
                    if (j == i) continue;
                    float3 delta = pos[j] - pi;
                    float d = math.length(delta);
                    if (d >= P.interactR || d < 1e-7f) continue;

                    float3 dir = delta / d;
                    dir -= pi * math.dot(dir, pi);
                    dir = math.normalizesafe(dir);

                    count++;
                    dirSum += dir;

                    float w = 1f - d / P.interactR; w *= w;
                    float sj = stiffness[j];

                    // --- stiffness-weighted velocity coupling (makes rigid-ish plates emerge) ---
                    float cw = w * sj;
                    coupledSum += vel[j] * cw;
                    wSum += cw;

                    // --- relative motion across the pair axis ---
                    float3 rel = vel[j] - vi;
                    float along = math.dot(rel, dir);
                    float3 shearV = rel - along * dir;
                    shearV -= pi * math.dot(shearV, pi);
                    float closing = math.max(-along, 0f);
                    float opening = math.max(along, 0f);
                    closeSum += closing * w;
                    openSum += opening * w;
                    shearSum += math.length(shearV) * w;
                    wTot += w;
                    if (math.abs(along) * w > bestAbs) { bestAbs = math.abs(along) * w; axis = dir; }

                    // --- subduction decision: thickness OR density contrast, denser goes under ---
                    float tj = thickness[j], dj = density[j];
                    float contrast = math.abs(ti - tj) / P.subductThickness + math.abs(di - dj) / P.subductDensity;
                    bool subduct = contrast >= 1f;
                    bool iSub = subduct && di > dj;

                    if (closing > P.closeThreshold && d < P.collDist)
                    {
                        if (subduct)
                        {
                            if (iSub)
                            {
                                subSum += closing * w; subW += w;
                                slabF += dir * (P.slabPull * (di - dj) * w);
                                // overridden: consume now instead of letting the trench pack up
                                if (d < P.consumeDist) consumed = true;
                            }
                            else overSum += closing * w;
                        }
                        else crumpleSum += closing * w;
                    }

                    if (!subduct)
                    {
                        // --- separation: softer point yields more. Subducting pairs pass under each other. ---
                        if (d < P.minDist)
                        {
                            float share = sj / (si + sj + 1e-5f);
                            sep -= dir * ((P.minDist - d) * share * P.sepStrength);
                        }

                        // --- gravity cap: crust that can't thicken any more can't be compressed any more ---
                        if (d < P.collDist && lockDir.Length < 30)
                        {
                            float lockA = P.lockStrength * math.max(headI, Headroom(tj));
                            if (lockA > 0f)
                            {
                                float share = sj / (si + sj + 1e-5f);
                                lockDir.Add(new float4(dir, math.dot(vel[j], dir)));
                                lockAmt.Add(lockA * share);
                            }
                        }
                    }

                    // --- rift gap candidate: opening pair with nothing between them ---
                    if (opening > P.openThreshold && j > i && d > bestGap)
                    {
                        float gd = P.gapDist * (1f + P.riftStiffness * 0.5f * (si + sj));
                        gd = math.min(gd, 0.95f * P.interactR);
                        if (d > gd)
                        {
                            float3 m = math.normalize(pi + pos[j]);
                            if (IsClear(i, j, m, ci)) { bestGap = d; gapJ = j; }
                        }
                    }
                }
            }

            // --- velocity: convective drive blended toward the stiffness-weighted neighbourhood, with inertia ---
            float3 coupled = coupledSum / wSum;
            float cpl = math.saturate(P.couple * si);
            float3 target = math.lerp(driven[i], coupled, cpl);
            float resp = 1f - math.exp(-dt / P.velTau);
            float3 vNew = math.lerp(vi, target, resp);

            // --- gravity cap: remove this point's share of the closing motion toward locked neighbours ---
            if (lockDir.Length > 0)
            {
                float3 corr = float3.zero; float n = 0f;
                for (int k = 0; k < lockDir.Length; k++)
                {
                    float4 e = lockDir[k];
                    float relClose = math.dot(vNew, e.xyz) - e.w;   // > 0: moving toward j
                    if (relClose > 0f) { corr += e.xyz * (relClose * lockAmt[k]); n += 1f; }
                }
                if (n > 0f) vNew -= corr / n;
            }

            vNew -= pi * math.dot(vNew, pi);
            float sp = math.length(vNew);
            if (sp > P.maxSpeed) vNew *= P.maxSpeed / sp;
            velNext[i] = vNew;

            float sl = math.length(sep);
            float maxSep = 0.5f * P.spacing;
            if (sl > maxSep) sep *= maxSep / sl;
            sepDisp[i] = sep;

            // --- rates (normalised by reference plate speed) ---
            float inv = wTot > 1e-6f ? 1f / wTot : 0f;
            float vr = 1f / P.velRef;
            rClose[i] = closeSum * inv * vr;
            rOpen[i] = openSum * inv * vr;
            rShear[i] = shearSum * inv * vr;
            rCrumple[i] = crumpleSum * inv * vr;
            rSub[i] = subW > 0f ? subSum / subW * vr : 0f;
            rOver[i] = overSum * inv * vr;
            compAxis[i] = axis;
            slab[i] = slabF;

            // --- removal: deterministic when overridden, plus the original area-conserving trickle ---
            bool remove = consumed;
            if (!remove && subW > 0f)
            {
                float meanClose = subSum / subW;
                float pRemove = P.subductRate * meanClose * dt / P.spacing;
                if (TMath.Rand01((uint)i * 747796405u + step * 2891336453u + P.seed) < pRemove)
                    remove = true;
            }
            removeFlag[i] = remove ? (byte)1 : (byte)0;

            // --- crowding: 1 on an even lattice ---
            float crowd = count / P.expectNeighbors;
            crowding[i] = crowd;

            // --- spawn proposal: rift gap first, then density-based void fill ---
            if (gapJ >= 0)
            {
                spawnPartner[i] = gapJ;
                spawnPos[i] = math.normalize(pi + pos[gapJ]);
            }
            else if (!remove && P.voidFill > 0f && crowd < P.voidFill)
            {
                float3 m = count > 0 ? dirSum / count : float3.zero;
                m -= pi * math.dot(m, pi);
                float ml = math.length(m);
                float3 away;
                if (ml > P.voidBias)
                {
                    away = -m / ml;   // hole edge: step into the empty side
                }
                else
                {
                    // uniformly sparse: try a random direction, the clearance test rejects bad picks
                    TMath.TangentFrame(pi, out float3 t, out float3 bt);
                    float a = TMath.Rand01((uint)i * 2246822519u + step * 3266489917u + (P.seed ^ 0x9e3779b9u)) * (2f * math.PI);
                    away = t * math.cos(a) + bt * math.sin(a);
                }
                float3 cand = math.normalize(pi + away * P.spacing);
                if (IsClear(i, -1, cand, ci))
                {
                    spawnPartner[i] = SpawnVoid;
                    spawnPos[i] = cand;
                }
            }
        }

        bool IsClear(int self, int other, float3 m, int3 ci)
        {
            float clear2 = P.holeClear * P.holeClear;
            for (int dz = -1; dz <= 1; dz++)
            for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++)
            {
                int3 c = ci + new int3(dx, dy, dz);
                if (math.any(c < 0) || math.any(c >= P.gridDim)) continue;
                int cell = c.x + P.gridDim * (c.y + P.gridDim * c.z);
                int end = cellStart[cell + 1];
                for (int s = cellStart[cell]; s < end; s++)
                {
                    int k = cellItems[s];
                    if (k == self || k == other) continue;
                    if (math.lengthsq(pos[k] - m) < clear2) return false;
                }
            }
            return true;
        }
    }

    [BurstCompile]
    public struct IntegrateJob : IJobParallelFor
    {
        public TectonicParams P;
        public float dt;

        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> velNext;
        [ReadOnly] public NativeArray<float3> sepDisp;
        [ReadOnly] public NativeArray<float3> compAxis;
        [ReadOnly] public NativeArray<float> rClose;
        [ReadOnly] public NativeArray<float> rOpen;
        [ReadOnly] public NativeArray<float> rShear;
        [ReadOnly] public NativeArray<float> rSub;
        [ReadOnly] public NativeArray<float> rOver;
        [ReadOnly] public NativeArray<float> rCrumple;

        public NativeArray<float3> pos;
        public NativeArray<float3> vel;
        public NativeArray<float3> faultDir;
        public NativeArray<float> thickness, age, density, stiffness;
        public NativeArray<float> orogeny, extension, arc, shear, faultActivity;
        public NativeArray<byte> faultType;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;

            float3 p0 = pos[i];
            float3 disp = velNext[i] * dt + sepDisp[i];
            disp -= p0 * math.dot(disp, p0);
            float3 p = math.normalize(p0 + disp);
            float3 v = velNext[i];
            v -= p * math.dot(v, p);
            pos[i] = p; vel[i] = v;

            float a = age[i] + dt;
            age[i] = a;

            float t = thickness[i];
            float cf = TMath.CrustFrac(t, P);
            float cl = rClose[i], op = rOpen[i], sh = rShear[i];
            float crumple = rCrumple[i];
            float over = rOver[i];

            // crumple zones thicken; overriding continental margins thicken less
            t += (crumple + P.overThick * over * cf) * P.crumpleThick * dt;

            // divergence stretches continental crust; oceanic is already at floor thickness
            if (op > 0f)
            {
                t = math.max(P.oceanThick, t - P.thinRate * op * cf * dt);
                extension[i] = extension[i] + op * cf * dt;
            }

            // erosion / isostatic relaxation of over-thick crust
            if (t > P.contThick) t = P.contThick + (t - P.contThick) * math.exp(-P.erosion * dt);
            t = math.min(t, P.maxThickness);
            thickness[i] = t;

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
                float3 along = math.cross(p, compAxis[i]);
                faultDir[i] = math.normalizesafe(along, faultDir[i]);
            }
            else
            {
                faultActivity[i] = decayed;
                if (decayed < P.faultThreshold * 0.25f) faultType[i] = (byte)FaultType.None;
            }

            density[i] = TMath.Density(t, a, P);
            stiffness[i] = TMath.Stiffness(t, a, faultActivity[i], P);
        }
    }

    [BurstCompile]
    public struct ConvectionFeedbackJob : IJob
    {
        public TectonicParams P;
        public float dt;
        public uint step;

        public NativeArray<Convection> conv;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<byte> removeFlag;
        [ReadOnly] public NativeArray<float3> pos;
        [ReadOnly] public NativeArray<float> thickness;
        [ReadOnly] public NativeArray<int> nearest;

        public NativeArray<float3> sumCont;
        public NativeArray<float3> sumSub;
        public NativeArray<int> total;
        public NativeArray<int> contCnt;
        public NativeArray<int> subCnt;

        public void Execute()
        {
            int N = conv.Length;
            for (int k = 0; k < N; k++)
            {
                sumCont[k] = float3.zero; sumSub[k] = float3.zero;
                total[k] = 0; contCnt[k] = 0; subCnt[k] = 0;
            }

            float contLimit = P.contThick * 0.75f;
            for (int i = 0; i < alive.Length; i++)
            {
                if (alive[i] == 0) continue;
                int k = nearest[i];
                total[k] = total[k] + 1;
                if (thickness[i] > contLimit) { contCnt[k] = contCnt[k] + 1; sumCont[k] = sumCont[k] + pos[i]; }
                if (removeFlag[i] == 1) { subCnt[k] = subCnt[k] + 1; sumSub[k] = sumSub[k] + pos[i]; }
            }

            for (int k = 0; k < N; k++)
            {
                Convection c = conv[k];
                float tot = math.max(total[k], 1);
                float cov = contCnt[k] / tot;
                float subRate = subCnt[k] / tot / dt;

                // continents insulate (warm), slabs cool; both relax away
                c.feedback += dt * (P.insulationHeat * cov - P.subductionCool * subRate - c.feedback * P.feedbackRelax);
                float n = TMath.Rand01((uint)k * 977u + step * 131u + P.seed) * 2f - 1f;
                c.baseTemp = math.clamp(c.baseTemp + n * P.tempNoise * math.sqrt(dt), -1f, 1f);
                c.temp = math.clamp(c.baseTemp + c.feedback, -1.5f, 1.5f);

                // cold cells migrate toward the trenches that feed them, hot cells toward the continents insulating them
                float3 target = c.pos; bool has = false;
                if (c.temp < 0f && subCnt[k] > 0) { target = sumSub[k]; has = true; }
                else if (c.temp > 0f && contCnt[k] > 0) { target = sumCont[k]; has = true; }
                if (has)
                {
                    target = math.normalizesafe(target, c.pos);
                    float3 dir = target - c.pos * math.dot(target, c.pos);
                    c.pos = math.normalize(c.pos + dir * (P.convDrift * dt));
                }
                conv[k] = c;
            }

            // keep cells from collapsing onto each other
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

    [BurstCompile]
    public struct StructureJob : IJob
    {
        public TectonicParams P;

        public NativeArray<byte> alive, faultType;
        [ReadOnly] public NativeArray<byte> removeFlag;
        [ReadOnly] public NativeArray<int> spawnPartner;
        [ReadOnly] public NativeArray<float3> spawnPos;
        public NativeArray<int> freeList, counters;
        public NativeArray<int> spawnHash;    // power-of-two size, all -1 between steps
        public NativeArray<int> spawnSlots;   // scratch: hash slots touched this step

        public NativeArray<float3> pos, vel, faultDir, slab;
        public NativeArray<float> thickness, age, density, stiffness;
        public NativeArray<float> orogeny, extension, arc, shear, hotspot, faultActivity;

        public void Execute()
        {
            int cap = alive.Length;
            int aliveCount = counters[0];
            int free = counters[1];
            int spawned = 0, removed = 0;

            // 1) eliminate subducted points
            for (int i = 0; i < cap; i++)
            {
                if (alive[i] == 1 && removeFlag[i] == 1)
                {
                    alive[i] = 0;
                    freeList[free++] = i;
                    aliveCount--; removed++;
                }
            }

            // 2) create new crust. A small spatial hash keeps this step's spawns at least holeClear apart,
            //    so many proposals per region are fine; the first valid one wins.
            //    removeFlag is intentionally left stale on reused slots; it guards against stale partners
            //    and is reset by the next InteractionJob.
            int mask = spawnHash.Length - 1;
            int touched = 0;
            float clear2 = P.holeClear * P.holeClear;

            for (int i = 0; i < cap; i++)
            {
                if (alive[i] == 0 || removeFlag[i] == 1) continue;
                int j = spawnPartner[i];
                if (j == InteractionJob.SpawnNone) continue;
                bool pair = j >= 0;
                if (pair && (alive[j] == 0 || removeFlag[j] == 1)) continue;
                if (free == 0) break;

                float3 m = spawnPos[i];
                int3 c = (int3)math.floor((m + 1f) * P.invSpawnCell);

                bool ok = true;
                for (int dz = -1; dz <= 1 && ok; dz++)
                for (int dy = -1; dy <= 1 && ok; dy++)
                for (int dx = -1; dx <= 1 && ok; dx++)
                {
                    int k = spawnHash[(int)(TMath.HashCell(c + new int3(dx, dy, dz)) & (uint)mask)];
                    if (k >= 0 && math.lengthsq(pos[k] - m) < clear2) ok = false;
                }
                if (!ok) continue;
                int own = (int)(TMath.HashCell(c) & (uint)mask);
                if (spawnHash[own] >= 0) continue;   // same cell already used (or a hash collision)

                int n = freeList[--free];
                spawnHash[own] = n;
                spawnSlots[touched++] = own;

                float srcT, srcS, srcExt, srcAct;
                float3 v;
                if (pair)
                {
                    srcT = 0.5f * (thickness[i] + thickness[j]);
                    srcS = 0.5f * (stiffness[i] + stiffness[j]);
                    srcExt = 0.5f * (extension[i] + extension[j]);
                    srcAct = math.max(faultActivity[i], faultActivity[j]);
                    v = 0.5f * (vel[i] + vel[j]);
                }
                else
                {
                    srcT = thickness[i]; srcS = stiffness[i]; srcExt = extension[i];
                    srcAct = faultActivity[i]; v = vel[i];
                }
                float cf = TMath.CrustFrac(srcT, P);

                // thick/stiff surroundings leave transitional crust (failed-rift margins); thin/young ones make clean oceanic crust
                float nt = P.oceanThick + (srcT - P.oceanThick) * (P.riftBlend * cf * srcS);
                v -= m * math.dot(v, m);

                alive[n] = 1; aliveCount++; spawned++;
                pos[n] = m; vel[n] = v; slab[n] = float3.zero;
                thickness[n] = nt; age[n] = 0f;
                orogeny[n] = 0f; arc[n] = 0f; shear[n] = 0f; hotspot[n] = 0f;
                extension[n] = srcExt;
                faultType[n] = (byte)FaultType.Divergent;
                faultActivity[n] = srcAct;
                faultDir[n] = faultDir[i];
                density[n] = TMath.Density(nt, 0f, P);
                stiffness[n] = TMath.Stiffness(nt, 0f, srcAct, P);
            }

            for (int s = 0; s < touched; s++) spawnHash[spawnSlots[s]] = -1;

            counters[0] = aliveCount;
            counters[1] = free;
            counters[2] = counters[2] + spawned;
            counters[3] = counters[3] + removed;
        }
    }

    // ---------------------------------------------------------------------------------
    // Optional remesh: resample everything onto the original lattice. Nearest-neighbour copy keeps
    // boundaries sharp; lattice points with nothing nearby become fresh oceanic crust.
    // Requires a BuildGridJob on the current positions first.
    // ---------------------------------------------------------------------------------
    [BurstCompile]
    public struct RemeshGatherJob : IJobParallelFor
    {
        public TectonicParams P;
        [ReadOnly] public NativeArray<float3> lattice;
        [ReadOnly] public NativeArray<int> cellStart, cellItems;
        [ReadOnly] public NativeArray<float3> pos, vel, faultDir, slab;
        [ReadOnly] public NativeArray<float> thickness, age, density, stiffness, orogeny, extension, arc, shear, hotspot, faultActivity;
        [ReadOnly] public NativeArray<byte> faultType;

        [WriteOnly] public NativeArray<float3> oPos, oVel, oFaultDir, oSlab;
        [WriteOnly] public NativeArray<float> oThickness, oAge, oDensity, oStiffness, oOrogeny, oExtension, oArc, oShear, oHotspot, oFaultActivity;
        [WriteOnly] public NativeArray<byte> oFaultType;

        public void Execute(int k)
        {
            float3 q = lattice[k];
            int3 ci = (int3)math.floor((q + 1f) * P.invCell);
            ci = math.clamp(ci, new int3(0), new int3(P.gridDim - 1));

            int best = -1; float bestD2 = float.MaxValue;
            for (int dz = -1; dz <= 1; dz++)
            for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++)
            {
                int3 c = ci + new int3(dx, dy, dz);
                if (math.any(c < 0) || math.any(c >= P.gridDim)) continue;
                int cell = c.x + P.gridDim * (c.y + P.gridDim * c.z);
                int end = cellStart[cell + 1];
                for (int s = cellStart[cell]; s < end; s++)
                {
                    int j = cellItems[s];
                    float d2 = math.lengthsq(pos[j] - q);
                    if (d2 < bestD2) { bestD2 = d2; best = j; }
                }
            }

            oPos[k] = q;
            bool fresh = best < 0 || bestD2 > P.gapDist * P.gapDist;
            float3 v = best >= 0 ? vel[best] : float3.zero;
            v -= q * math.dot(v, q);
            oVel[k] = v;
            oSlab[k] = float3.zero;

            if (fresh)
            {
                float t = P.oceanThick;
                oThickness[k] = t; oAge[k] = 0f;
                oDensity[k] = TMath.Density(t, 0f, P);
                float act = best >= 0 ? faultActivity[best] : 0f;
                oStiffness[k] = TMath.Stiffness(t, 0f, act, P);
                oOrogeny[k] = 0f; oExtension[k] = 0f; oArc[k] = 0f; oShear[k] = 0f; oHotspot[k] = 0f;
                oFaultActivity[k] = act;
                oFaultType[k] = (byte)FaultType.Divergent;
                float3 fd = best >= 0 ? faultDir[best] : math.cross(q, new float3(0f, 1f, 0f));
                oFaultDir[k] = math.normalizesafe(fd - q * math.dot(fd, q), new float3(1f, 0f, 0f));
                return;
            }

            float3 f = faultDir[best];
            oFaultDir[k] = math.normalizesafe(f - q * math.dot(f, q), f);
            oThickness[k] = thickness[best]; oAge[k] = age[best];
            oDensity[k] = density[best]; oStiffness[k] = stiffness[best];
            oOrogeny[k] = orogeny[best]; oExtension[k] = extension[best];
            oArc[k] = arc[best]; oShear[k] = shear[best]; oHotspot[k] = hotspot[best];
            oFaultActivity[k] = faultActivity[best]; oFaultType[k] = faultType[best];
        }
    }

    [BurstCompile]
    public struct RemeshApplyJob : IJobParallelFor
    {
        public int count;   // lattice size; slots [0, count) become alive
        [ReadOnly] public NativeArray<float3> tPos, tVel, tFaultDir, tSlab;
        [ReadOnly] public NativeArray<float> tThickness, tAge, tDensity, tStiffness, tOrogeny, tExtension, tArc, tShear, tHotspot, tFaultActivity;
        [ReadOnly] public NativeArray<byte> tFaultType;

        public NativeArray<byte> alive, removeFlag, faultType;
        public NativeArray<int> spawnPartner;
        public NativeArray<float3> pos, vel, faultDir, slab;
        public NativeArray<float> thickness, age, density, stiffness, orogeny, extension, arc, shear, hotspot, faultActivity;

        public void Execute(int i)
        {
            removeFlag[i] = 0;
            spawnPartner[i] = InteractionJob.SpawnNone;
            if (i >= count) { alive[i] = 0; return; }
            alive[i] = 1;
            pos[i] = tPos[i]; vel[i] = tVel[i]; faultDir[i] = tFaultDir[i]; slab[i] = tSlab[i];
            thickness[i] = tThickness[i]; age[i] = tAge[i]; density[i] = tDensity[i]; stiffness[i] = tStiffness[i];
            orogeny[i] = tOrogeny[i]; extension[i] = tExtension[i]; arc[i] = tArc[i]; shear[i] = tShear[i];
            hotspot[i] = tHotspot[i]; faultActivity[i] = tFaultActivity[i]; faultType[i] = tFaultType[i];
        }
    }

    [BurstCompile]
    public struct RemeshFinalizeJob : IJob
    {
        public int count;
        public NativeArray<int> freeList, counters;

        public void Execute()
        {
            int cap = freeList.Length;
            int free = cap - count;
            for (int k = 0; k < free; k++) freeList[k] = cap - 1 - k;
            counters[0] = count;
            counters[1] = free;
        }
    }

    // ---------------------------------------------------------------------------------
    // Live preview: one tangent quad per capacity slot, written straight from sim state.
    // Dead slots collapse to a degenerate quad at the origin so the index buffer never changes.
    // ---------------------------------------------------------------------------------
    [BurstCompile]
    public struct PreviewMeshJob : IJobParallelFor
    {
        public TectonicParams P;
        public int view;          // (int)TectonicDebugView
        public float radius;      // planet radius in mesh units
        public float halfSize;    // quad half-extent in mesh units

        [ReadOnly] public NativeArray<byte> alive, faultType;
        [ReadOnly] public NativeArray<float3> pos, vel;
        [ReadOnly] public NativeArray<float> thickness, age, faultActivity, orogeny, hotspot, crowding;

        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<float3> verts;
        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<UnityEngine.Color32> colors;

        public void Execute(int i)
        {
            int b = i * 4;
            if (alive[i] == 0)
            {
                var clear = new UnityEngine.Color32(0, 0, 0, 0);
                verts[b] = float3.zero; verts[b + 1] = float3.zero; verts[b + 2] = float3.zero; verts[b + 3] = float3.zero;
                colors[b] = clear; colors[b + 1] = clear; colors[b + 2] = clear; colors[b + 3] = clear;
                return;
            }

            float3 n = pos[i];
            TMath.TangentFrame(n, out float3 t, out float3 bt);
            float3 c = n * radius;
            float3 ts = t * halfSize, bs = bt * halfSize;
            verts[b]     = c - ts - bs;
            verts[b + 1] = c - ts + bs;
            verts[b + 2] = c + ts + bs;
            verts[b + 3] = c + ts - bs;

            float4 col = math.saturate(ColorFor(i, t, bt));
            var c32 = new UnityEngine.Color32(
                (byte)(col.x * 255f + 0.5f), (byte)(col.y * 255f + 0.5f), (byte)(col.z * 255f + 0.5f), 255);
            colors[b] = c32; colors[b + 1] = c32; colors[b + 2] = c32; colors[b + 3] = c32;
        }

        static float4 FaultColor(int type)
        {
            switch (type)
            {
                case 1: return new float4(1f, 0f, 0f, 1f);       // divergent
                case 2: return new float4(1f, 0.55f, 0f, 1f);    // subduction
                case 3: return new float4(1f, 0f, 1f, 1f);       // collision
                case 4: return new float4(0f, 1f, 1f, 1f);       // transform
                default: return new float4(0.4f, 0.4f, 0.4f, 1f);
            }
        }

        static float3 Hsv(float h, float s, float v)
        {
            float3 k = math.saturate(math.abs(math.frac(h + new float3(1f, 2f / 3f, 1f / 3f)) * 6f - 3f) - 1f);
            return v * math.lerp(new float3(1f), k, s);
        }

        /// <summary>Blue = sparse, white = even lattice, red = packed.</summary>
        public static float4 CrowdingColor(float c)
        {
            float4 white = 1f;
            if (c < 1f) return math.lerp(new float4(0.05f, 0.2f, 1f, 1f), white, math.saturate((c - 0.4f) / 0.6f));
            return math.lerp(white, new float4(1f, 0.1f, 0.05f, 1f), math.saturate((c - 1f) / 0.8f));
        }

        float4 ColorFor(int i, float3 t, float3 bt)
        {
            float4 white = 1f, black = new float4(0f, 0f, 0f, 1f);
            switch (view)
            {
                case 1: // Thickness
                    return math.lerp(new float4(0f, 0f, 1f, 1f), white, math.saturate(thickness[i] / P.maxThickness));
                case 2: // Age
                    return math.lerp(new float4(1f, 0.92f, 0.016f, 1f), new float4(0.2f, 0f, 0.4f, 1f), math.saturate(age[i] / 300f));
                case 3: // FaultType
                    return math.lerp(FaultColor(0), FaultColor(faultType[i]), math.saturate(faultActivity[i] * 3f));
                case 4: // Orogeny
                    return math.lerp(black, white, math.saturate(orogeny[i]));
                case 5: // Hotspot
                    return math.lerp(black, new float4(1f, 0f, 0f, 1f), math.saturate(hotspot[i] * 0.5f));
                case 6: // Velocity: hue = direction, brightness = speed. Plates show up as flat patches.
                {
                    float3 v = vel[i];
                    float h = math.atan2(math.dot(v, bt), math.dot(v, t)) / (2f * math.PI) + 1f;
                    float sp = math.saturate(math.length(v) / P.velRef);
                    return new float4(Hsv(math.frac(h), 0.85f, 0.25f + 0.75f * sp), 1f);
                }
                case 7: // Crowding
                    return CrowdingColor(crowding[i]);
                default: // Elevation
                {
                    float e = TMath.ElevationKm(thickness[i], age[i], P);
                    if (e < 0f)
                        return math.lerp(new float4(0f, 0.05f, 0.35f, 1f), new float4(0.2f, 0.55f, 0.85f, 1f), math.saturate((e + 5f) / 5f));
                    return math.lerp(new float4(0.25f, 0.6f, 0.2f, 1f), white, math.saturate(e / 5f));
                }
            }
        }
    }
}
