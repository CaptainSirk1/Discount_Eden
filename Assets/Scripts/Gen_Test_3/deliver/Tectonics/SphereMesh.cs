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
    //
    // MATERIAL STATE. Each vertex carries a crust VOLUME (km x unit-sphere area). Its dual AREA is the
    // cotangent (circumcentric) Voronoi cell of its planar one-ring triangles, and thickness = volume / area
    // is derived. Volume is what is conserved: motion, flips, splits and collapses only move it between
    // vertices; every change of the planet total is an explicit, ledgered source or sink (MeshLedger).
    // ---------------------------------------------------------------------------------

    /// <summary>Everything the mesh jobs need. Built by MeshStep.BuildParams from MeshSettings.
    /// Units: lengths are unit-sphere chords, speeds rad/Myr, times Myr, thickness km.</summary>
    public struct MeshParams
    {
        /// <summary>Shared crust constants and the tunables inherited from the point simulation (oceanThick,
        /// contThick, velRef, convection, coupling, subduction contrast, accumulators, fault classification).</summary>
        public TectonicParams P;

        // ---- mesh geometry ----
        public float L;            // target edge length
        public float Lmin, Lmax;   // collapse below / split above (absolute)
        public float Lmerge;       // converging buoyant edges merge below this (absolute, hot crust)
        public float sliverH;      // vertex closer than this to its opposite edge = vertex/edge collision
        public float minOrient;    // orientation test threshold
        public float flipEps;      // Delaunay test tolerance
        public int maxValence;
        public float relax;        // tangential smoothing weight per step
        public float flashDecay;   // Myr, for the TopologyEvents view

        // ---- compression ----
        public float barrierRest;   // edge length where compression resistance begins (absolute)
        public float barrierK;      // exponential stiffness of the barrier
        public float barrierAge;    // how strongly thermal strength compounds the barrier
        public float spreading;     // rad/Myr per km of excess-thickness difference (gravitational spreading)

        // ---- planetary cooling ----
        public float globalThermal; // Myr, the planet's global thermal age at the current step
        public float thermalRef;    // Myr, thermal age at which strength is half way: T / (T + ref)
        public float globalHeat;    // 0..1, 1 - globalThermal / maxGlobalThermal (1 = young planet)

        // ---- crustal strength (spring solve, MeshSolverJobs) ----
        public bool springOn;       // solve the edge-spring network for velocity (off: local constraint passes only)
        public float coherence;     // rad: coherence length of intact, cold continental crust in the spring network
        public float oceanCoherence;// coherence of oceanic crust relative to continental (0..1)
        public float compStress;    // weight of closing (compressive) stress relative to opening stress for damage
        public float blockRigidity; // 0..1 blend of intact continental crust toward its block's rigid rotation
        public float blockDamage;   // damage above which a vertex no longer belongs to a rigid block
        public float blockMinVertices;

        // ---- extension resistance ----
        public bool extEnabled;
        public float extMax;        // max fraction of opening removed by intact crust per pass
        public float extAge;        // thermal-strength factor of the strength exponent
        public float extThick;      // crust-fraction factor of the strength exponent
        public float extFractured;  // fraction of opening removed by fully damaged crust

        // ---- damage ----
        public float dmgYield;      // stress (velRef / rad) per unit of (0.25 + strength exponent) where damage starts
        public float dmgRate;       // 1/Myr damage growth per unit of relative over-stress
        public float dmgWeaken;     // 0..1: how far full damage lowers the yield (localisation feedback)
        public float dmgSeed;       // nucleation weight of intact crust in the damage growth (small = sharper faults)
        public float dmgHealFrac;   // heal only while stress < dmgHealFrac x INTACT yield
        public float dmgHeal;       // 1/Myr exponential healing rate below that
        public float riftDamage;    // damage needed before continental splits make new crust
        public float riftOpen;      // rad/Myr opening above which an oceanic split is a ridge

        // ---- crust budget ----
        public bool subductionSink; // consumed slab volume leaves the crust (off: conservative test mode)
        public bool rifting;        // rift splits create new crust (off: every split is plain refinement)
        public float riftThick;     // km, thickness of new rift / ridge crust
        public float minCrust;      // km, thickness floor; filling to it is a ledgered mantle source (0 = off)
        public float delamRate;     // 1/Myr: crust thicker than P.maxThickness loses this fraction of its excess root per Myr
        public float erosionK;      // unit-sphere area / Myr: thickness diffusivity of crust above erosionBase
        public float erosionBase;   // km, only crust thicker than this erodes
        public float erosionRecycle;// 0..1, fraction of sediment landing on oceanic crust that leaves the crust
        public bool magmatism;      // arc magmatism returns part of the subducted volume
        public float magmaReturn;   // fraction of subducted volume returned through arcs (mature planet)
        public float magmaYouth;    // extra fraction x globalHeat (young, hot mantle melts more)
        public float magmaTau;      // Myr, release time of a cell's magma reservoir
        public float magmaFelsic;   // 0..1, differentiated share of arc-magmatic additions
        public float felsicSubduct; // 0..1, share of the felsic part of consumed slab volume that really sinks;
                                    // the rest is relaminated onto the overriding margins of the same cell

        // ---- convection feedback ----
        public float felsicColumn;    // km of felsic column at which crust counts as buoyant (crust type 0.5); 0 = thickness only
        public float surfaceCool;     // exposed-mantle cooling per Myr
        public float insulationOnset; // crust fraction at which crust starts to insulate (1 = only full continents)
        public float vigor;           // target RMS cell temperature this step (0 = feedback sets the scale, as before)
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

    /// <summary>Geometry shared by every job that needs a vertex's dual area, so all of them agree exactly.</summary>
    public static class MeshGeom
    {
        /// <summary>Cotangent (circumcentric) Voronoi share of vertex p in planar triangle (p, q, r):
        /// (|pq|^2 cot(r) + |pr|^2 cot(q)) / 8. Signed: negative for an obtuse angle at p's far side, and the three
        /// shares of a triangle always sum to its area. Unlike 1/3 of the triangle area, the resulting cell areas
        /// do not jump when a Delaunay flip swaps a diagonal, so volume / area (thickness) stays continuous.</summary>
        public static float VoronoiShare(float3 p, float3 q, float3 r)
        {
            float3 rp = p - r, rq = q - r, qp = p - q, qr = r - q;
            float sr = math.length(math.cross(rp, rq)), sq = math.length(math.cross(qp, qr));
            float cotR = math.dot(rp, rq) / math.max(sr, 1e-20f);
            float cotQ = math.dot(qp, qr) / math.max(sq, 1e-20f);
            return (math.lengthsq(p - q) * cotR + math.lengthsq(p - r) * cotQ) * 0.125f;
        }

        /// <summary>Dual area of vertex v: Voronoi cell, floored at a quarter of the barycentric area so badly shaped
        /// (non-Delaunay, near-inverted) rings can never give a zero or negative area.</summary>
        public static float DualArea(NativeArray<int> V, NativeArray<int> O, NativeArray<int> vc, NativeArray<float3> pos, int v)
        {
            float3 p = pos[v];
            float vor = 0f, bary = 0f;
            int c0 = vc[v], c = c0, n = 0;
            do
            {
                float3 q = pos[V[MeshTopo.Next(c)]], r = pos[V[MeshTopo.Prev(c)]];
                vor += VoronoiShare(p, q, r);
                bary += math.length(math.cross(q - p, r - p)) * (0.5f / 3f);
                c = MeshTopo.Next(O[MeshTopo.Next(c)]);
            } while (c != c0 && ++n < MeshTopo.MaxRing);
            return math.max(vor, 0.25f * bary);
        }
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
        public NativeArray<float> consumedVol;
        public NativeArray<float> stats;   // [0] min edge / L, [1] max edge / L

        // material
        public NativeArray<float> volume, area, areaNew, thickness, volumeTmp;
        /// <summary>Differentiated (felsic, continental-type) part of each vertex's volume, 0..volume. Carried with the
        /// volume by every operation; created only by arc magmatism (and the initial crust).</summary>
        public NativeArray<float> felsic, felsicTmp;
        /// <summary>Derived mechanical crust type 0..1 (MeshLimits.CrustType), kept in sync with thickness.</summary>
        public NativeArray<float> cfrac;
        // vertex attributes
        public NativeArray<float3> pos, posTmp, vel, velNext, velTmp, spreadVel, driven, slab, compAxis, faultDir;
        public NativeArray<float> age, density, stiffness, orogeny, extension, arc, shear, hotspot, faultActivity;
        public NativeArray<float> flashSplit, flashCollapse, damage, tension, achieved, magma, arcW, subFrac;
        public NativeArray<float> rClose, rOpen, rShear, rSub, rOver, rCrumple;
        // per-step ledger contributions written by parallel jobs, reduced (and zeroed) by MeshBudgetJob
        public NativeArray<float> sinkSub, srcFloor, sinkErosion, moveErosion, sinkFelsic, sinkErosionFelsic, sinkDelam, sinkDelamFelsic, accrete;
        public NativeArray<byte> alive, faultType;
        public NativeArray<int> nearest;

        // convection + magma reservoirs (one per cell)
        public NativeArray<Convection> conv;
        public NativeArray<float3> convSumCont, convSumSub;
        public NativeArray<float> convArea, convContArea, convArcW;
        public NativeArray<int> convSub;
        public NativeArray<double> convPool, convAccrete;

        // spring solver (MeshSolverJobs) work buffers
        public NativeArray<float3> sb, sr, sz, sp, sAp, sx;
        public NativeArray<float> sdiag, spart, kc, lc;
        public NativeArray<double> springScalars;
        // rigid continental blocks (MeshBlockJob)
        public NativeArray<int> blockParent, blockId;
        public NativeArray<float3> blockL, blockI0, blockI1, blockI2;
        public NativeArray<float> blockA;

        /// <summary>Cumulative material ledger, see MeshLedger for the layout.</summary>
        public NativeArray<double> ledger;

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
            consumedCell = Alloc<int>(vCap); consumedPos = Alloc<float3>(vCap); consumedVol = Alloc<float>(vCap);
            triAlive = Alloc<byte>(tCap);
            stats = Alloc<float>(4);

            volume = Alloc<float>(vCap); area = Alloc<float>(vCap); areaNew = Alloc<float>(vCap);
            thickness = Alloc<float>(vCap); volumeTmp = Alloc<float>(vCap);
            felsic = Alloc<float>(vCap); felsicTmp = Alloc<float>(vCap); cfrac = Alloc<float>(vCap);

            pos = Alloc<float3>(vCap); posTmp = Alloc<float3>(vCap); vel = Alloc<float3>(vCap);
            velNext = Alloc<float3>(vCap); velTmp = Alloc<float3>(vCap); spreadVel = Alloc<float3>(vCap);
            driven = Alloc<float3>(vCap); slab = Alloc<float3>(vCap); compAxis = Alloc<float3>(vCap); faultDir = Alloc<float3>(vCap);
            age = Alloc<float>(vCap); density = Alloc<float>(vCap); stiffness = Alloc<float>(vCap);
            orogeny = Alloc<float>(vCap); extension = Alloc<float>(vCap); arc = Alloc<float>(vCap);
            shear = Alloc<float>(vCap); hotspot = Alloc<float>(vCap); faultActivity = Alloc<float>(vCap);
            flashSplit = Alloc<float>(vCap); flashCollapse = Alloc<float>(vCap);
            damage = Alloc<float>(vCap); tension = Alloc<float>(vCap); achieved = Alloc<float>(vCap);
            magma = Alloc<float>(vCap); arcW = Alloc<float>(vCap); subFrac = Alloc<float>(vCap);
            rClose = Alloc<float>(vCap); rOpen = Alloc<float>(vCap); rShear = Alloc<float>(vCap);
            rSub = Alloc<float>(vCap); rOver = Alloc<float>(vCap); rCrumple = Alloc<float>(vCap);
            sinkSub = Alloc<float>(vCap); srcFloor = Alloc<float>(vCap); sinkErosion = Alloc<float>(vCap); moveErosion = Alloc<float>(vCap); sinkFelsic = Alloc<float>(vCap); sinkErosionFelsic = Alloc<float>(vCap); sinkDelam = Alloc<float>(vCap); sinkDelamFelsic = Alloc<float>(vCap); accrete = Alloc<float>(vCap);
            alive = Alloc<byte>(vCap); faultType = Alloc<byte>(vCap);
            nearest = Alloc<int>(vCap);

            conv = Alloc<Convection>(convCount);
            convSumCont = Alloc<float3>(convCount); convSumSub = Alloc<float3>(convCount);
            convArea = Alloc<float>(convCount); convContArea = Alloc<float>(convCount); convArcW = Alloc<float>(convCount);
            convSub = Alloc<int>(convCount); convPool = Alloc<double>(convCount); convAccrete = Alloc<double>(convCount);
            ledger = Alloc<double>(MeshLedger.Count);
            sb = Alloc<float3>(vCap); sr = Alloc<float3>(vCap); sz = Alloc<float3>(vCap); sp = Alloc<float3>(vCap);
            sAp = Alloc<float3>(vCap); sx = Alloc<float3>(vCap);
            sdiag = Alloc<float>(vCap); spart = Alloc<float>(vCap);
            kc = Alloc<float>(tCap * 3); lc = Alloc<float>(tCap * 3);
            springScalars = Alloc<double>(MeshSpring.Count);
            blockParent = Alloc<int>(vCap); blockId = Alloc<int>(vCap);
            blockL = Alloc<float3>(vCap); blockI0 = Alloc<float3>(vCap); blockI1 = Alloc<float3>(vCap); blockI2 = Alloc<float3>(vCap);
            blockA = Alloc<float>(vCap);
        }

        public void Dispose()
        {
            foreach (var d in _dispose) d();
            _dispose.Clear();
        }

        /// <summary>Planet total crust volume (unit-sphere km). O(V).</summary>
        public double TotalVolume()
        {
            double v = 0;
            for (int i = 0; i < VCap; i++) if (alive[i] == 1) v += volume[i];
            return v;
        }

        /// <summary>Crust in transit: relaminated felsic material waiting in a cell's accretion buffer (unit-sphere km).
        /// It is still crust, so the ledger balance is TotalVolume + InTransit.</summary>
        public double InTransit()
        {
            double v = 0;
            for (int k = 0; k < ConvCount; k++) v += convAccrete[k];
            for (int i = 0; i < VCap; i++) v += accrete[i];
            return v;
        }

        /// <summary>Planet total differentiated (felsic) volume (unit-sphere km). O(V).</summary>
        public double TotalFelsic()
        {
            double v = 0;
            for (int i = 0; i < VCap; i++) if (alive[i] == 1) v += felsic[i];
            return v;
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
                if (!(volume[v] >= 0f) || !(thickness[v] > 0f)) Err($"vertex {v}: bad material volume {volume[v]} thickness {thickness[v]}");
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
