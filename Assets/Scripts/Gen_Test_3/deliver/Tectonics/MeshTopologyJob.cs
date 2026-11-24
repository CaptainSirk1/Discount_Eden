using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace PlanetGen.Tectonics.MeshSim
{
    // ---------------------------------------------------------------------------------
    // All topology changes. Single-threaded: there are few edits per step and serial order keeps
    // runs deterministic. Passes, in order:
    //   0. apply relaxation (refreshes every dual area)
    //   1. repair inverted triangles (forced collapse of their shortest edge)
    //   2. Delaunay flips
    //   3. vertex/edge collisions: a vertex about to cross its opposite edge
    //   4. collapse edges shorter than Lmin (denser side consumed if the pair can subduct, else merge)
    //   5. split edges longer than Lmax (the hard spacing cap); fractured / ridge edges make new crust
    //   6. Delaunay flips again, then stats
    //
    // MATERIAL RULES (volume is the state; every op below conserves it unless it says otherwise):
    //   flip            no vertex is created or removed, every vertex keeps its volume. Voronoi cell areas are
    //                   continuous across a Delaunay flip, so thickness barely changes.
    //   plain split     the new vertex m takes, from each neighbour whose cell shrinks, the volume of the area it
    //                   loses at that neighbour's thickness: neighbours keep their thickness, m gets the area-weighted
    //                   mix. Across a crust boundary m copies the denser side's thickness and the draws are scaled to
    //                   match, so margins stay sharp and volume is still exact.
    //   rift split      m is NEW crust from the mantle (riftThick, age 0, undamaged): ledger RiftCrust. Neighbours keep
    //                   their volume, so the continental material that was stretched over m's area stays in the rift
    //                   shoulders instead of being replaced by oceanic crust.
    //   merge collapse  the removed vertex's volume goes to the vertices whose cells grow, in proportion to the area
    //                   each one gains (so their thickness barely changes).
    //   slab collapse   as a merge, but the share of the area gained by the overriding plate (the other crust) is
    //                   consumed: ledger SubductedCollapse. The overriding plate keeps its own volume, so it neither
    //                   grows over the slab nor loses material.
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
        public NativeArray<float> volume, felsic, area, thickness, cfrac, age, density, stiffness, orogeny, extension, arc, shear, hotspot, faultActivity, damage;
        public NativeArray<float> flashSplit, flashCollapse, stats, consumedVol, magma, tension;
        public NativeArray<double> ledger, convAccrete;

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
            if (!applyRelax) return;
            for (int v = 0; v < alive.Length; v++) if (alive[v] == 1) pos[v] = posTmp[v];
            for (int v = 0; v < alive.Length; v++) if (alive[v] == 1) RefreshArea(v);
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
                        // q or r (TryCollapseEdge removes the denser vertex).
                        int first = u < 0.5f ? MeshTopo.Prev(c) : MeshTopo.Next(c);   // edge p-q is opposite Prev(c)
                        int second = u < 0.5f ? MeshTopo.Next(c) : MeshTopo.Prev(c);
                        if (TryCollapseEdge(first, false) || TryCollapseEdge(second, false)) break;
                    }

                    // otherwise put a vertex on the edge where p meets it (plain split: edge material) and collapse p
                    // into it. If p is denser and can subduct under it, that collapse consumes p.
                    int m = TrySplit(c, u, false);
                    if (m < 0) continue;
                    // after the split, the edge p-m is opposite Next(c) in this triangle
                    TryCollapseEdge(MeshTopo.Next(c), false);
                    break;
                }
            }
        }

        /// <summary>4) collapse edges shorter than Lmin, and converging buoyant edges shorter than Lmerge</summary>
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

        /// <summary>5) + 6) splits (the hard spacing cap) alternating with flips</summary>
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
            volume[v] = 0f; felsic[v] = 0f;
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

        /// <summary>Recomputes v's dual area and the derived thickness / density / stiffness (volume unchanged).</summary>
        void RefreshArea(int v)
        {
            float a = MeshGeom.DualArea(V, O, vc, pos, v);
            area[v] = a;
            float t = volume[v] / math.max(a, 1e-12f);
            thickness[v] = t;
            float cf = MeshLimits.CrustType(t, volume[v], felsic[v], M);
            cfrac[v] = cf;
            density[v] = MeshLimits.Density(cf, age[v]);
            stiffness[v] = MeshLimits.Stiffness(cf, age[v], faultActivity[v], damage[v], M);
        }

        static void AddUnique(ref FixedList512Bytes<int> set, int v)
        {
            for (int k = 0; k < set.Length; k++) if (set[k] == v) return;
            if (set.Length < 126) set.Add(v);
        }

        void AddRing(ref FixedList512Bytes<int> set, int v)
        {
            int c0 = vc[v], c = c0, n = 0;
            do { AddUnique(ref set, V[MeshTopo.Next(c)]); c = Swing(c); } while (c != c0 && ++n < MeshTopo.MaxRing);
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

        bool CanSubduct(int a, int b) => MeshLimits.Subducts(thickness[a], cfrac[a], density[a], thickness[b], cfrac[b], density[b], M);

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

        // T0 = (a, b, d), T1 = (e, d, b)  ->  T0 = (a, b, e), T1 = (e, d, a). Volumes unchanged.
        void Flip(int c)
        {
            int o = O[c];
            int n0 = MeshTopo.Next(c), p0 = MeshTopo.Prev(c), n1 = MeshTopo.Next(o), p1 = MeshTopo.Prev(o);
            int a = V[c], b = V[n0], d = V[n1], e = V[o];
            int yBE = O[n1], yDA = O[n0];
            V[p0] = e; V[p1] = a;
            Link(c, yBE); Link(o, yDA); Link(n0, n1);
            vc[a] = c; vc[b] = n0; vc[d] = n1; vc[e] = o;
            RefreshArea(a); RefreshArea(b); RefreshArea(d); RefreshArea(e);
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

            // cells of a, b, d, e before (they are the only ones the split changes)
            float4 before = new float4(area[a], area[b], area[d], area[e]);

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
            pos[m] = pm;

            bool rift = InitSplitVertex(m, b, d, u, pm, allowRift, out float copyThick);

            // ---- material ----
            float am = MeshGeom.DualArea(V, O, vc, pos, m);
            float4 after = new float4(MeshGeom.DualArea(V, O, vc, pos, a), MeshGeom.DualArea(V, O, vc, pos, b),
                                      MeshGeom.DualArea(V, O, vc, pos, d), MeshGeom.DualArea(V, O, vc, pos, e));
            float vm;
            if (rift)
            {
                // new crust from the mantle; neighbours keep their volume (shoulders keep their material)
                vm = M.riftThick * am;
                ledger[MeshLedger.RiftCrust] += vm;
                felsic[m] = 0f;   // new mantle-derived crust is not differentiated
                _rifts++;
            }
            else
            {
                // draw from every neighbour whose cell shrank, at its own thickness
                float4 vol4 = new float4(volume[a], volume[b], volume[d], volume[e]);
                float4 t4 = vol4 / math.max(before, 1e-12f);
                float4 g = math.max(before - after, 0f) * t4;
                float held = math.csum(g);
                float want = copyThick > 0f ? copyThick * am : held;
                if (held <= 1e-20f) { g = new float4(0f, 0.5f * want, 0.5f * want, 0f); held = math.csum(g); }
                float k = held > 0f ? math.clamp(want / held, 0.25f, 4f) : 0f;
                g = math.min(g * k, vol4 * 0.9f);
                vm = math.csum(g);
                volume[a] = vol4.x - g.x; volume[b] = vol4.y - g.y; volume[d] = vol4.z - g.z; volume[e] = vol4.w - g.w;
                // each draw carries its source's composition
                float4 fel4 = new float4(felsic[a], felsic[b], felsic[d], felsic[e]);
                float4 gf = g * (fel4 / math.max(vol4, 1e-20f));
                felsic[a] = fel4.x - gf.x; felsic[b] = fel4.y - gf.y; felsic[d] = fel4.z - gf.z; felsic[e] = fel4.w - gf.w;
                felsic[m] = math.csum(gf);
            }
            volume[m] = vm;
            RefreshArea(m); RefreshArea(a); RefreshArea(b); RefreshArea(d); RefreshArea(e);
            _splits++;
            return m;
        }

        /// <summary>
        /// Attributes of the new vertex on edge b-d. A split is a RIFT when the edge is opening AND either the crust is
        /// oceanic (ridge: fast opening or an active divergent fault) or it is buoyant crust that has been fractured
        /// (damage >= riftDamage). Intact buoyant crust that merely stretches is refined, never turned into ocean floor.
        /// copyThick > 0 asks the caller to give m that thickness (plain split across a crust boundary).
        /// </summary>
        bool InitSplitVertex(int m, int b, int d, float u, float3 pm, bool allowRift, out float copyThick)
        {
            TectonicParams P = M.P;
            copyThick = 0f;
            float3 axis = math.normalizesafe(pos[d] - pos[b]);
            float openRate = math.dot(vel[d] - vel[b], axis);
            bool divergentFault = (faultType[b] == (byte)FaultType.Divergent && faultActivity[b] > P.faultThreshold)
                               || (faultType[d] == (byte)FaultType.Divergent && faultActivity[d] > P.faultThreshold);
            float cfAvg = 0.5f * (cfrac[b] + cfrac[d]);
            float dMax = math.max(damage[b], damage[d]);
            bool ridgeLike = openRate > M.riftOpen || divergentFault;
            bool rift;
            if (!allowRift || !M.rifting) rift = false;
            else if (cfAvg < 0.5f || !M.extEnabled) rift = ridgeLike;          // oceanic ridges (and the old rule with extension off)
            else rift = openRate > 0f && dMax >= M.riftDamage;                  // buoyant crust only rifts once fractured

            float3 v = math.lerp(vel[b], vel[d], u);
            v -= pm * math.dot(v, pm);
            posTmp[m] = pm; vel[m] = v; velNext[m] = v;
            slab[m] = float3.zero;
            nearest[m] = u < 0.5f ? nearest[b] : nearest[d];
            float3 fd = u < 0.5f ? faultDir[b] : faultDir[d];
            faultDir[m] = math.normalizesafe(fd - pm * math.dot(fd, pm), new float3(1f, 0f, 0f));
            float act = math.max(faultActivity[b], faultActivity[d]);
            extension[m] = math.lerp(extension[b], extension[d], u);
            flashCollapse[m] = 0f;
            flashSplit[m] = 1f;
            magma[m] = 0f;
            tension[m] = math.lerp(tension[b], tension[d], u);

            if (rift)
            {
                // fresh crust: age 0, so its thermal state is exactly the planet's global thermal age
                age[m] = 0f;
                orogeny[m] = 0f; arc[m] = 0f; shear[m] = 0f; hotspot[m] = 0f;
                faultType[m] = (byte)FaultType.Divergent;
                faultActivity[m] = act;
                damage[m] = 0f;
            }
            else if (MeshLimits.Boundary(thickness[b], density[b], thickness[d], density[d], M))
            {
                // the edge crosses a crust boundary: copy one side instead of blending, so margins stay sharp.
                // Copy the denser side: boundary edges mostly stretch at trenches, and copying the buoyant side
                // there would grow the overriding plate out over the slab.
                int src = density[b] >= density[d] ? b : d;
                copyThick = thickness[src];
                age[m] = age[src];
                orogeny[m] = orogeny[src]; arc[m] = arc[src]; shear[m] = shear[src]; hotspot[m] = hotspot[src];
                extension[m] = extension[src];
                faultActivity[m] = faultActivity[src]; faultType[m] = faultType[src];
                damage[m] = damage[src];
            }
            else
            {
                age[m] = math.lerp(age[b], age[d], u);
                orogeny[m] = math.lerp(orogeny[b], orogeny[d], u);
                arc[m] = math.lerp(arc[b], arc[d], u);
                shear[m] = math.lerp(shear[b], shear[d], u);
                hotspot[m] = math.lerp(hotspot[b], hotspot[d], u);
                faultActivity[m] = math.lerp(faultActivity[b], faultActivity[d], u);
                faultType[m] = faultActivity[b] >= faultActivity[d] ? faultType[b] : faultType[d];
                damage[m] = math.lerp(damage[b], damage[d], u);
            }
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
                // the slab vertex collapses into its own plate (the slab behind it), not into the overriding vertex
                int alt = BestSlabNeighbor(gone, keep);
                if (alt >= 0 && CanCollapse(alt, pos[V[MeshTopo.Next(alt)]], true))
                {
                    flashCollapse[keep] = 1f;
                    DoCollapse(alt, pos[V[MeshTopo.Next(alt)]], keep, true);
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
            DoCollapse(cc, np, sub ? keep : -1, sub && !force);
            return true;
        }

        /// <summary>
        /// Collapses V[Prev(c)] (gone) into V[Next(c)] (survivor) at np and redistributes gone's volume to the vertices
        /// whose cells grow. A slab collapse (slabOver = the overriding vertex) consumes the share of the overriding
        /// side: everything gained by slabOver or by vertices of a different crust than the slab.
        /// </summary>
        void DoCollapse(int c, float3 np, int slabOver, bool slabCollapse)
        {
            int surv = V[MeshTopo.Next(c)], gone = V[MeshTopo.Prev(c)];
            var set = new FixedList512Bytes<int>();
            AddRing(ref set, gone);
            AddRing(ref set, surv);
            AddUnique(ref set, surv);
            // remove gone itself if it slipped in via surv's ring
            var S = new FixedList512Bytes<int>();
            for (int k = 0; k < set.Length; k++) if (set[k] != gone) S.Add(set[k]);
            var A0 = new FixedList512Bytes<float>();
            for (int k = 0; k < S.Length; k++) A0.Add(area[S[k]]);

            float vg = volume[gone];
            float fg = vg > 0f ? felsic[gone] / vg : 0f;
            float tg = thickness[gone], dg = density[gone], cfg = cfrac[gone];
            float ag = age[gone], Dg = damage[gone], og = orogeny[gone], eg = extension[gone], arg = arc[gone], sg = shear[gone], hg = hotspot[gone];
            float faG = faultActivity[gone]; byte ftG = faultType[gone];
            float3 velG = vel[gone];
            bool merge = !slabCollapse;
            if (merge)
            {
                // the survivor's motion is the area-weighted mean of the pair
                float w = area[gone] / math.max(area[gone] + area[surv], 1e-12f);
                float3 v = math.lerp(vel[surv], velG, w);
                vel[surv] = v; velNext[surv] = v;
            }
            bool recorded = slabCollapse && RecordConsumed(gone);

            Collapse(c, np);

            // ---- redistribute gone's volume by area gained ----
            var gain = new FixedList512Bytes<float>();
            float total = 0f, recv = 0f;
            for (int k = 0; k < S.Length; k++)
            {
                int v = S[k];
                float g = math.max(MeshGeom.DualArea(V, O, vc, pos, v) - A0[k], 0f);
                bool receives = true;
                if (slabCollapse && M.subductionSink)
                    receives = v != slabOver && !MeshLimits.Boundary(tg, dg, thickness[v], density[v], M);
                gain.Add(receives ? g : -g);   // negative = gained by the overriding side (consumed)
                total += g;
                if (receives) recv += g;
            }
            float consumed = 0f;
            if (total <= 1e-20f)
            {
                // degenerate ring: hand everything to the survivor (or consume it for a slab)
                if (slabCollapse && M.subductionSink) consumed = vg;
                else Receive(surv, vg, fg, ag, Dg, og, eg, arg, sg, hg, faG, ftG);
            }
            else
            {
                for (int k = 0; k < S.Length; k++)
                {
                    float g = gain[k];
                    if (g <= 0f) continue;
                    Receive(S[k], vg * g / total, fg, ag, Dg, og, eg, arg, sg, hg, faG, ftG);
                }
                consumed = vg * (1f - recv / total);
            }
            if (consumed > 0f)
            {
                // the felsic part mostly relaminates onto the overriding margins of this cell (see MeshIntegrateJob)
                float relam = consumed * fg * (1f - M.felsicSubduct);
                int cell = nearest[gone];
                if (cell >= 0 && cell < convAccrete.Length) convAccrete[cell] = convAccrete[cell] + relam;
                else relam = 0f;
                float sunk = consumed - relam;
                ledger[MeshLedger.SubductedCollapse] += sunk;
                ledger[MeshLedger.FelsicSubducted] += consumed * fg - relam;
                if (cfg >= 0.5f) ledger[MeshLedger.BuoyantSubducted] += sunk;
                if (recorded) consumedVol[counters[MeshTopo.ConsumedThisStep] - 1] = sunk;
            }
            for (int k = 0; k < S.Length; k++) if (alive[S[k]] == 1) RefreshArea(S[k]);
            flashCollapse[surv] = 1f;
            _collapses++;
        }

        /// <summary>Mixes received volume (and the attributes it carries) into v, weighted by volume.</summary>
        void Receive(int v, float dv, float fg, float ag, float Dg, float og, float eg, float arg, float sg, float hg, float faG, byte ftG)
        {
            if (dv <= 0f) return;
            float vv = volume[v];
            float w = dv / math.max(vv + dv, 1e-20f);
            volume[v] = vv + dv;
            felsic[v] = felsic[v] + dv * fg;
            age[v] = math.lerp(age[v], ag, w);
            damage[v] = math.lerp(damage[v], Dg, w);
            orogeny[v] = math.lerp(orogeny[v], og, w);
            extension[v] = math.lerp(extension[v], eg, w);
            arc[v] = math.lerp(arc[v], arg, w);
            shear[v] = math.lerp(shear[v], sg, w);
            hotspot[v] = math.lerp(hotspot[v], hg, w);
            if (faG * w > faultActivity[v]) { faultActivity[v] = faG; faultType[v] = ftG; }
        }

        bool RecordConsumed(int gone)
        {
            int s = counters[MeshTopo.ConsumedThisStep];
            if (s >= consumedCell.Length) return false;
            consumedCell[s] = nearest[gone]; consumedPos[s] = pos[gone]; consumedVol[s] = 0f;
            counters[MeshTopo.ConsumedThisStep] = s + 1;
            _subducted++;
            if (cfrac[gone] >= 0.5f) counters[MeshTopo.ContinentalSubducted] += 1;
            return true;
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

        // T0 = (a, b, d), T1 = (e, d, b). d is merged into b and both triangles disappear. Topology only.
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
}
