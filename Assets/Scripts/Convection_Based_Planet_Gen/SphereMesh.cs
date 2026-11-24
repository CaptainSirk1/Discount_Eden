using System;
using System.Collections.Generic;
using System.Text;
using Unity.Collections;
using Unity.Mathematics;

namespace PlanetGen.Tectonics.MeshSim
{
    // ---------------------------------------------------------------------------------
    // Corner-table triangle mesh on the unit sphere.
    //
    //   corner c belongs to triangle c / 3. V[c] = vertex at the corner, O[c] = the opposite corner
    //   in the neighbouring triangle (across the edge that does NOT touch V[c]).
    //   Triangles are counter-clockwise seen from outside. The edge opposite c runs V[Next(c)] -> V[Prev(c)].
    //   Around a vertex: Swing(c) = Next(O[Next(c)]) gives the next corner at the same vertex.
    //
    // The sphere is closed, so there are no boundary edges: every corner always has an opposite.
    // Free vertex / triangle slots are kept on stacks so capacity is fixed for the whole run.
    // ---------------------------------------------------------------------------------

    /// <summary>Everything the mesh jobs need, on top of the original TectonicParams.</summary>
    public struct MeshParams
    {
        public TectonicParams P;

        public float L;            // target edge length (unit sphere chord)
        public float Lmin, Lmax;   // collapse below / split above (absolute)
        public float sliverH;      // vertex closer than this to its opposite edge = vertex/edge collision
        public float minOrient;    // orientation test threshold
        public float flipEps;      // Delaunay test tolerance
        public int maxValence;

        public float barrierRest;   // edge length where compression resistance begins (absolute)
        public float barrierK;      // exponential stiffness of the barrier
        public float barrierAge;    // how strongly age compounds the barrier
        public float oceanCompress; // fraction of area loss that thickens oceanic crust (continental = 1)
        public float riftOpen;      // rad/Myr opening speed above which a split makes fresh crust
        public float relax;         // tangential smoothing weight per step
        public float flashDecay;    // Myr, for the TopologyEvents view
        public float spreading;     // rad/Myr per km of excess-thickness difference (orogenic collapse)
    }

    public static class MeshTopo
    {
        public const int MaxRing = 64;

        // counters[] layout
        public const int AliveV = 0, FreeV = 1, AliveT = 2, FreeT = 3,
            Splits = 4, Collapses = 5, Flips = 6, Subducted = 7, Inverted = 8, CapHits = 9,
            ConsumedThisStep = 10, Rifts = 11, ContinentalSubducted = 12, Count = 16;

        public static int Next(int c) => c % 3 == 2 ? c - 2 : c + 1;
        public static int Prev(int c) => c % 3 == 0 ? c + 2 : c - 1;

        public static bool OrientOK(float3 a, float3 b, float3 c, float minOrient)
            => math.dot(math.cross(b - a, c - a), a + b + c) > minOrient;
    }

    /// <summary>Owns all native buffers for one mesh simulation run.</summary>
    public sealed class SphereMeshState : IDisposable
    {
        public readonly int VCap, TCap, ConvCount;
        readonly List<Action> _dispose = new List<Action>();

        // topology
        public NativeArray<int> V, O, vc, freeV, freeT, counters, consumedCell;
        public NativeArray<byte> triAlive;
        public NativeArray<float3> consumedPos;
        public NativeArray<float> stats;   // [0] min edge / L, [1] max edge / L

        // vertex attributes
        public NativeArray<float3> pos, posTmp, vel, velNext, velTmp, driven, slab, compAxis, faultDir;
        public NativeArray<float> thickness, age, density, stiffness, orogeny, extension, arc, shear, hotspot, faultActivity;
        public NativeArray<float> areaOld, flashSplit, flashCollapse;
        public NativeArray<float> rClose, rOpen, rShear, rSub, rOver, rCrumple;
        public NativeArray<byte> alive, faultType;
        public NativeArray<int> nearest;
        public NativeArray<uint> areaMask;

        // convection
        public NativeArray<Convection> conv;
        public NativeArray<float3> convSumCont, convSumSub;
        public NativeArray<int> convTotal, convCont, convSub;

        NativeArray<T> Alloc<T>(int n) where T : struct
        {
            var a = new NativeArray<T>(n, Allocator.Persistent, NativeArrayOptions.ClearMemory);
            _dispose.Add(a.Dispose);
            return a;
        }

        public SphereMeshState(int vCap, int tCap, int convCount)
        {
            VCap = vCap; TCap = tCap; ConvCount = convCount;

            V = Alloc<int>(tCap * 3); O = Alloc<int>(tCap * 3);
            vc = Alloc<int>(vCap); freeV = Alloc<int>(vCap); freeT = Alloc<int>(tCap);
            counters = Alloc<int>(MeshTopo.Count);
            consumedCell = Alloc<int>(vCap); consumedPos = Alloc<float3>(vCap);
            triAlive = Alloc<byte>(tCap);
            stats = Alloc<float>(4);

            pos = Alloc<float3>(vCap); posTmp = Alloc<float3>(vCap); vel = Alloc<float3>(vCap);
            velNext = Alloc<float3>(vCap); velTmp = Alloc<float3>(vCap); driven = Alloc<float3>(vCap); slab = Alloc<float3>(vCap);
            compAxis = Alloc<float3>(vCap); faultDir = Alloc<float3>(vCap);
            thickness = Alloc<float>(vCap); age = Alloc<float>(vCap); density = Alloc<float>(vCap);
            stiffness = Alloc<float>(vCap); orogeny = Alloc<float>(vCap); extension = Alloc<float>(vCap);
            arc = Alloc<float>(vCap); shear = Alloc<float>(vCap); hotspot = Alloc<float>(vCap);
            faultActivity = Alloc<float>(vCap);
            areaOld = Alloc<float>(vCap); flashSplit = Alloc<float>(vCap); flashCollapse = Alloc<float>(vCap);
            rClose = Alloc<float>(vCap); rOpen = Alloc<float>(vCap); rShear = Alloc<float>(vCap);
            rSub = Alloc<float>(vCap); rOver = Alloc<float>(vCap); rCrumple = Alloc<float>(vCap);
            alive = Alloc<byte>(vCap); faultType = Alloc<byte>(vCap);
            nearest = Alloc<int>(vCap); areaMask = Alloc<uint>(vCap);

            conv = Alloc<Convection>(convCount);
            convSumCont = Alloc<float3>(convCount); convSumSub = Alloc<float3>(convCount);
            convTotal = Alloc<int>(convCount); convCont = Alloc<int>(convCount); convSub = Alloc<int>(convCount);
        }

        public void Dispose()
        {
            foreach (var d in _dispose) d();
            _dispose.Clear();
        }

        // ============================================================== initial mesh
        /// <summary>Class I geodesic sphere: each icosahedron face split into freq^2 triangles. 10*freq^2 + 2 vertices.</summary>
        public static void BuildGeodesic(int freq, List<float3> verts, List<int> tris)
        {
            verts.Clear(); tris.Clear();
            int n = Math.Max(1, freq);

            // icosahedron by brute-force convex hull of its 12 vertices (no hand-typed face table to get wrong)
            float phi = (1f + math.sqrt(5f)) * 0.5f;
            var ico = new List<float3>();
            for (int s1 = -1; s1 <= 1; s1 += 2)
            for (int s2 = -1; s2 <= 1; s2 += 2)
            {
                ico.Add(math.normalize(new float3(0, s1, s2 * phi)));
                ico.Add(math.normalize(new float3(s1, s2 * phi, 0)));
                ico.Add(math.normalize(new float3(s2 * phi, 0, s1)));
            }
            var faces = new List<int3>();
            for (int i = 0; i < 12; i++)
            for (int j = i + 1; j < 12; j++)
            for (int k = j + 1; k < 12; k++)
            {
                float3 nrm = math.cross(ico[j] - ico[i], ico[k] - ico[i]);
                bool pos = false, neg = false;
                for (int q = 0; q < 12; q++)
                {
                    if (q == i || q == j || q == k) continue;
                    float s = math.dot(ico[q] - ico[i], nrm);
                    if (s > 1e-5f) pos = true; else if (s < -1e-5f) neg = true;
                }
                if (pos && neg) continue;
                faces.Add(pos ? new int3(i, k, j) : new int3(i, j, k));   // outward = CCW
            }

            for (int i = 0; i < 12; i++) verts.Add(ico[i]);

            // shared edge vertices, created once per undirected edge (from lower index to higher)
            var edgeStart = new Dictionary<long, int>();
            int EdgeVertex(int a, int b, int k)   // k-th point from a toward b, 0 < k < n
            {
                int lo = math.min(a, b), hi = math.max(a, b);
                long key = (long)lo * 64 + hi;
                if (!edgeStart.TryGetValue(key, out int start))
                {
                    start = verts.Count;
                    for (int s = 1; s < n; s++) verts.Add(math.normalize(math.lerp(ico[lo], ico[hi], s / (float)n)));
                    edgeStart[key] = start;
                }
                int fromLo = a == lo ? k : n - k;
                return start + fromLo - 1;
            }

            var grid = new int[(n + 1) * (n + 1)];
            foreach (var f in faces)
            {
                int A = f.x, B = f.y, C = f.z;
                for (int i = 0; i <= n; i++)
                for (int j = 0; i + j <= n; j++)
                {
                    int id;
                    if (i == 0 && j == 0) id = A;
                    else if (i == n) id = B;
                    else if (j == n) id = C;
                    else if (j == 0) id = EdgeVertex(A, B, i);
                    else if (i == 0) id = EdgeVertex(A, C, j);
                    else if (i + j == n) id = EdgeVertex(B, C, j);
                    else
                    {
                        id = verts.Count;
                        float3 p = (ico[A] * (n - i - j) + ico[B] * i + ico[C] * j) / n;
                        verts.Add(math.normalize(p));
                    }
                    grid[i * (n + 1) + j] = id;
                }
                for (int i = 0; i < n; i++)
                for (int j = 0; i + j < n; j++)
                {
                    AddTri(verts, tris, grid[i * (n + 1) + j], grid[(i + 1) * (n + 1) + j], grid[i * (n + 1) + j + 1]);
                    if (i + j < n - 1)
                        AddTri(verts, tris, grid[(i + 1) * (n + 1) + j], grid[(i + 1) * (n + 1) + j + 1], grid[i * (n + 1) + j + 1]);
                }
            }
        }

        static void AddTri(List<float3> v, List<int> t, int a, int b, int c)
        {
            if (math.dot(math.cross(v[b] - v[a], v[c] - v[a]), v[a] + v[b] + v[c]) < 0f) { int s = b; b = c; c = s; }
            t.Add(a); t.Add(b); t.Add(c);
        }

        /// <summary>Loads a closed, consistently oriented triangle mesh. Builds opposites, free lists and counters.</summary>
        public void Load(List<float3> verts, List<int> tris)
        {
            int nv = verts.Count, nt = tris.Count / 3;
            if (nv > VCap || nt > TCap) throw new ArgumentException("mesh exceeds capacity");

            for (int i = 0; i < nv; i++) { pos[i] = verts[i]; alive[i] = 1; }
            var edgeCorner = new Dictionary<long, int>(nt * 3);
            for (int c = 0; c < nt * 3; c++) V[c] = tris[c];
            for (int t = 0; t < nt; t++) triAlive[t] = 1;
            for (int c = 0; c < nt * 3; c++)
            {
                long key = (long)V[MeshTopo.Next(c)] * VCap + V[MeshTopo.Prev(c)];
                edgeCorner[key] = c;
                vc[V[c]] = c;
            }
            for (int c = 0; c < nt * 3; c++)
            {
                long twin = (long)V[MeshTopo.Prev(c)] * VCap + V[MeshTopo.Next(c)];
                if (!edgeCorner.TryGetValue(twin, out int o)) throw new InvalidOperationException("mesh is not closed");
                O[c] = o;
            }

            int fv = VCap - nv, ft = TCap - nt;
            for (int k = 0; k < fv; k++) freeV[k] = VCap - 1 - k;   // pops lowest index first
            for (int k = 0; k < ft; k++) freeT[k] = TCap - 1 - k;
            for (int c = nt * 3; c < TCap * 3; c++) { V[c] = -1; O[c] = -1; }
            counters[MeshTopo.AliveV] = nv; counters[MeshTopo.FreeV] = fv;
            counters[MeshTopo.AliveT] = nt; counters[MeshTopo.FreeT] = ft;
        }

        // ============================================================== validation
        /// <summary>Full topology check. Slow; for debugging. Returns null when the mesh is valid.</summary>
        public string Validate(float minOrient, out int inverted)
        {
            var sb = new StringBuilder();
            int errors = 0; inverted = 0;
            void Err(string s) { if (errors++ < 12) sb.AppendLine(s); }

            int nF = 0, nV = 0;
            var used = new bool[VCap];
            for (int t = 0; t < TCap; t++)
            {
                if (triAlive[t] == 0) continue;
                nF++;
                for (int k = 0; k < 3; k++)
                {
                    int c = t * 3 + k;
                    int v = V[c];
                    if (v < 0 || v >= VCap || alive[v] == 0) { Err($"corner {c}: dead/invalid vertex {v}"); continue; }
                    used[v] = true;
                    int o = O[c];
                    if (o < 0 || o >= TCap * 3 || triAlive[o / 3] == 0) { Err($"corner {c}: bad opposite {o}"); continue; }
                    if (O[o] != c) Err($"corner {c}: O[O[c]] = {O[o]}");
                    if (V[MeshTopo.Next(c)] != V[MeshTopo.Prev(o)] || V[MeshTopo.Prev(c)] != V[MeshTopo.Next(o)])
                        Err($"corner {c}: shared edge vertices mismatch");
                    if (o / 3 == t) Err($"corner {c}: opposite in same triangle");
                }
                int a = V[t * 3], b = V[t * 3 + 1], cc = V[t * 3 + 2];
                if (a == b || b == cc || a == cc) Err($"tri {t}: repeated vertex");
                else if (a >= 0 && b >= 0 && cc >= 0 && !MeshTopo.OrientOK(pos[a], pos[b], pos[cc], 0f)) inverted++;
            }
            int valenceSum = 0;
            for (int v = 0; v < VCap; v++)
            {
                if (alive[v] == 0) continue;
                nV++;
                if (!used[v]) { Err($"vertex {v}: alive but unreferenced"); continue; }
                int c0 = vc[v];
                if (c0 < 0 || c0 >= TCap * 3 || triAlive[c0 / 3] == 0 || V[c0] != v) { Err($"vertex {v}: bad vc {c0}"); continue; }
                int c = c0, n = 0;
                do { n++; c = MeshTopo.Next(O[MeshTopo.Next(c)]); if (V[c] != v) { Err($"vertex {v}: ring left vertex"); break; } }
                while (c != c0 && n < 200);
                if (n >= 200) Err($"vertex {v}: ring did not close");
                if (n < 3) Err($"vertex {v}: valence {n}");
                valenceSum += n;
            }
            int nE = valenceSum / 2;
            if (nV - nE + nF != 2) Err($"Euler characteristic {nV - nE + nF} (V {nV}, E {nE}, F {nF})");
            if (nV != counters[MeshTopo.AliveV]) Err($"alive vertex counter {counters[MeshTopo.AliveV]} != {nV}");
            if (nF != counters[MeshTopo.AliveT]) Err($"alive triangle counter {counters[MeshTopo.AliveT]} != {nF}");
            if (counters[MeshTopo.AliveV] + counters[MeshTopo.FreeV] != VCap) Err("vertex free list out of sync");
            if (counters[MeshTopo.AliveT] + counters[MeshTopo.FreeT] != TCap) Err("triangle free list out of sync");

            if (errors == 0) return null;
            if (errors > 12) sb.AppendLine($"... {errors - 12} more");
            return sb.ToString();
        }
    }
}
