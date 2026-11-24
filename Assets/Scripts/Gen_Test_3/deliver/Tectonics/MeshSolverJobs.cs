using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace PlanetGen.Tectonics.MeshSim
{
    // ---------------------------------------------------------------------------------
    // Crustal strength as an edge-spring network, solved for velocity each step.
    //
    //   minimise   sum_i A_i |v_i - d_i|^2  +  sum_edges k_ij ((v_j - v_i) . e_ij)^2        v_i tangent at p_i
    //
    // d_i is the target velocity (convective drive + slab pull, MeshVelocityJob), A_i the dual area (basal traction)
    // and e_ij the unit CHORD between the two vertices. (v_j - v_i) . e_ij is the rate the edge stretches; it is
    // exactly zero for every rigid rotation of the sphere, so coherent plate motion costs nothing and only strain is
    // resisted. The normal equations
    //
    //   (A + K) v = A d,     (K v)_i = P_i sum_j k_ij ((v_i - v_j) . e_ij) e_ij,  P_i = tangent projection
    //
    // are symmetric positive definite and solved with Jacobi-preconditioned conjugate gradients, warm-started from the
    // previous velocity. Every CG iteration is one parallel one-ring sweep plus two serial O(V) dot products.
    //
    // Why not the local constraint passes alone: under a uniform stretch every vertex is pulled equally toward both
    // neighbours, so local corrections cancel and homogeneous extension of a continent interior is never resisted
    // (measured: an intact 2 Gyr craton over an upwelling thinned 35 -> 28 km in 100 Myr with almost no damage).
    // The solve transmits force across the whole stiff region.
    //
    // STIFFNESS. k_ij = Abar_ij (l_ij / |e|)^2 : a region of intact crust behaves rigidly over about l_ij.
    //   l_ij = plateCoherence * S_eff * lerp(oceanicCoherence, 1, min crust type of the pair)
    //   S_eff = S (1 - D) + min(F, S) D,  S = 1 - exp(-x),  x = strength exponent
    //   (MeshLimits.IntactExtension without extMax), D = worse damage of the pair, F = fracturedCrustResistance.
    //   Converging pairs that can subduct (active trenches, judged from last step's motion) get k = 0.
    //
    // STRESS. The spring force per unit length across an edge, in units of velocityReference x radians:
    //   stress_ij = l_ij^2 eps_ij / velRef,   eps_ij = (v_j - v_i) . e_ij / |e|   (opening rate, 1/Myr)
    // In a stiff block it is the basal drag the block transmits through that edge (about g R^2 / 2 for a block of
    // radius R over a drive gradient g), independent of mesh resolution and of plateCoherence. Once a weak zone fails
    // and the halves separate, each half carries only its own drag and the stress in the intact interiors drops.
    // MeshSpringFinishJob stores the largest opening stress of each vertex in `tension`.
    //
    // The stiffness of every edge is evaluated once per step (MeshSpringSetupJob, per corner) so that a CG iteration
    // is a plain sparse multiply.
    // ---------------------------------------------------------------------------------

    public static class MeshSpring
    {
        public const int AlphaNum = 0, AlphaDen = 1, Rz = 2, RzNew = 3, Rz0 = 4, Beta = 5, Count = 8;

        /// <summary>Coherence length l_ij (rad) of the edge between i and j.</summary>
        public static float Coherence(float cfa, float cfb, float agea, float ageb, float Da, float Db, in MeshParams M)
        {
            float x = 0.5f * (MeshLimits.StrengthExponent(cfa, agea, M) + MeshLimits.StrengthExponent(cfb, ageb, M));
            float S = 1f - math.exp(-x);
            float D = math.max(Da, Db);
            float Se = S * (1f - D) + math.min(M.extFractured, S) * D;
            // oceanic lithosphere is far thinner than continental keels: scale its coherence down by crust type
            float cf = math.min(cfa, cfb);
            return M.coherence * Se * math.lerp(M.oceanCoherence, 1f, cf);
        }

        /// <summary>Edge stiffness, symmetric in i and j. 0 for an active trench or with the solver off.</summary>
        public static float Stiffness(int i, int j, float3 pi, float3 pj, float ai, float aj,
            in NativeArray<float> thickness, in NativeArray<float> cfrac, in NativeArray<float> density, in NativeArray<float> age,
            in NativeArray<float> damage, in NativeArray<float3> vel, in MeshParams M, out float l)
        {
            l = 0f;
            float ti = thickness[i], tj = thickness[j];
            if (MeshLimits.Subducts(ti, cfrac[i], density[i], tj, cfrac[j], density[j], M))
            {
                float3 e = math.normalizesafe(pj - pi);
                if (math.dot(vel[j] - vel[i], e) < 0f) return 0f;   // converging trench: the slab goes under freely
            }
            l = Coherence(cfrac[i], cfrac[j], age[i], age[j], damage[i], damage[j], M);
            float e2 = math.max(math.lengthsq(pj - pi), 1e-12f);
            return 0.5f * (ai + aj) * l * l / e2;
        }
    }

    /// <summary>Right-hand side, Jacobi diagonal and the start of CG: b = A d, x = warm start, Ax.</summary>
    [BurstCompile]
    public struct MeshSpringSetupJob : IJobParallelFor
    {
        public MeshParams M;
        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos, vel, velNext;
        [ReadOnly] public NativeArray<float> area, thickness, cfrac, density, age, damage;
        [WriteOnly] public NativeArray<float3> sb, sr, sz, sp;
        [WriteOnly] public NativeArray<float> sdiag, spart;
        /// <summary>Per corner c: stiffness and coherence of the edge V[c] -> V[Next(c)]. Each corner belongs to
        /// exactly one vertex's ring, so the parallel writes never collide.</summary>
        [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<float> kc, lc;

        public void Execute(int i)
        {
            if (alive[i] == 0) { spart[i] = 0f; return; }
            float3 pi = pos[i];
            float ai = area[i];
            float3 x = vel[i] - pi * math.dot(vel[i], pi);   // warm start: last step's velocity
            float3 b = velNext[i] * ai;
            float3 Ax = x * ai;
            float diag = ai;
            int c0 = vc[i], c = c0, n = 0;
            do
            {
                int j = V[MeshTopo.Next(c)];
                float3 pj = pos[j];
                float k = MeshSpring.Stiffness(i, j, pi, pj, ai, area[j], thickness, cfrac, density, age, damage, vel, M, out float l);
                kc[c] = k; lc[c] = l;
                if (k > 0f)
                {
                    float3 e = math.normalizesafe(pj - pi);
                    float3 xj = vel[j] - pj * math.dot(vel[j], pj);
                    Ax += e * (k * math.dot(x - xj, e));
                    float3 et = e - pi * math.dot(e, pi);
                    diag += k * math.lengthsq(et);
                }
                c = MeshTopo.Next(O[MeshTopo.Next(c)]);
            } while (c != c0 && ++n < MeshTopo.MaxRing);
            Ax -= pi * math.dot(Ax, pi);
            b -= pi * math.dot(b, pi);
            float3 r = b - Ax;
            float3 z = r / diag;
            sb[i] = b; sr[i] = r; sz[i] = z; sp[i] = z; sdiag[i] = diag;
            spart[i] = math.dot(r, z);
        }
    }

    /// <summary>Ap = (A + K) p, and the per-vertex terms of p . Ap.</summary>
    [BurstCompile]
    public struct MeshSpringApplyJob : IJobParallelFor
    {
        public MeshParams M;
        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos, sp;
        [ReadOnly] public NativeArray<float> area, kc;
        [WriteOnly] public NativeArray<float3> sAp;
        [WriteOnly] public NativeArray<float> spart;

        public void Execute(int i)
        {
            if (alive[i] == 0) { spart[i] = 0f; return; }
            float3 pi = pos[i], p = sp[i];
            float ai = area[i];
            float3 Ap = p * ai;
            int c0 = vc[i], c = c0, n = 0;
            do
            {
                float k = kc[c];
                if (k > 0f)
                {
                    int j = V[MeshTopo.Next(c)];
                    float3 e = math.normalizesafe(pos[j] - pi);
                    Ap += e * (k * math.dot(p - sp[j], e));
                }
                c = MeshTopo.Next(O[MeshTopo.Next(c)]);
            } while (c != c0 && ++n < MeshTopo.MaxRing);
            Ap -= pi * math.dot(Ap, pi);
            sAp[i] = Ap;
            spart[i] = math.dot(p, Ap);
        }
    }

    /// <summary>Serial reduction of the per-vertex partial sums into one of the CG scalars.</summary>
    [BurstCompile]
    public struct MeshSpringReduceJob : IJob
    {
        public int slot;
        [ReadOnly] public NativeArray<float> spart;
        public NativeArray<double> scalars;

        public void Execute()
        {
            double s = 0;
            for (int i = 0; i < spart.Length; i++) s += spart[i];
            scalars[slot] = s;
            if (slot == MeshSpring.Rz && scalars[MeshSpring.Rz0] <= 0.0) scalars[MeshSpring.Rz0] = s;   // first residual
        }
    }

    /// <summary>x += alpha p, r -= alpha Ap, z = r / diag; per-vertex r . z for the next beta.
    /// alpha = 0 once the residual has fallen below tolerance, so extra iterations are no-ops.</summary>
    [BurstCompile]
    public struct MeshSpringUpdateJob : IJobParallelFor
    {
        public float tolerance;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<double> scalars;
        [ReadOnly] public NativeArray<float3> sp, sAp;
        [ReadOnly] public NativeArray<float> sdiag;
        public NativeArray<float3> sx, sr;
        [WriteOnly] public NativeArray<float3> sz;
        [WriteOnly] public NativeArray<float> spart;

        public void Execute(int i)
        {
            if (alive[i] == 0) { spart[i] = 0f; return; }
            double rz = scalars[MeshSpring.Rz], pAp = scalars[MeshSpring.AlphaDen], rz0 = scalars[MeshSpring.Rz0];
            float alpha = (pAp > 1e-30 && rz > tolerance * tolerance * rz0) ? (float)(rz / pAp) : 0f;
            float3 x = sx[i] + sp[i] * alpha;
            float3 r = sr[i] - sAp[i] * alpha;
            float3 z = r / sdiag[i];
            sx[i] = x; sr[i] = r; sz[i] = z;
            spart[i] = math.dot(r, z);
        }
    }

    /// <summary>beta = rz_new / rz, p = z + beta p. Then rz = rz_new (done by the next reduce).</summary>
    [BurstCompile]
    public struct MeshSpringDirectionJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<double> scalars;
        [ReadOnly] public NativeArray<float3> sz;
        public NativeArray<float3> sp;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            double rz = scalars[MeshSpring.Rz], rzNew = scalars[MeshSpring.RzNew];
            float beta = rz > 1e-30 ? (float)(rzNew / rz) : 0f;
            sp[i] = sz[i] + sp[i] * beta;
        }
    }

    /// <summary>Moves rz_new into rz between iterations (serial, trivial).</summary>
    [BurstCompile]
    public struct MeshSpringShiftJob : IJob
    {
        public NativeArray<double> scalars;
        public void Execute() { scalars[MeshSpring.Rz] = scalars[MeshSpring.RzNew]; }
    }

    /// <summary>Writes the solved velocity into velNext and the spring stress into tension.</summary>
    [BurstCompile]
    public struct MeshSpringFinishJob : IJobParallelFor
    {
        public MeshParams M;
        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos, sx;
        [ReadOnly] public NativeArray<float> thickness, density, kc, lc;
        [WriteOnly] public NativeArray<float3> velNext;
        [WriteOnly] public NativeArray<float> tension;

        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            float3 pi = pos[i], xi = sx[i];
            float best = 0f;
            float norm = 1f / M.P.velRef;
            int c0 = vc[i], c = c0, n = 0;
            do
            {
                int j = V[MeshTopo.Next(c)];
                float3 pj = pos[j];
                float k = kc[c], l = lc[c];
                if (k > 0f && !MeshLimits.Boundary(thickness[i], density[i], thickness[j], density[j], M))
                {
                    float3 d = pj - pi;
                    float len = math.max(math.length(d), 1e-9f);
                    float eps = math.dot(sx[j] - xi, d / len) / len;   // opening strain rate, 1/Myr
                    // opening counts fully, closing at compStress (crust is stronger in compression)
                    best = math.max(best, l * l * (eps > 0f ? eps : -eps * M.compStress) * norm);
                }
                c = MeshTopo.Next(O[MeshTopo.Next(c)]);
            } while (c != c0 && ++n < MeshTopo.MaxRing);
            velNext[i] = xi - pi * math.dot(xi, pi);
            tension[i] = best;
        }
    }

    public static class MeshSpringSchedule
    {
        /// <summary>Chains the CG solve after dep. Result in velNext (target velocities with crustal strength applied)
        /// and tension (spring stress). iterations = fixed count; converged iterations do nothing.</summary>
        public static JobHandle Schedule(SphereMeshState s, in MeshParams M, int iterations, float tolerance, JobHandle dep)
        {
            int cap = s.VCap;
            var h = new MeshSpringClearJob { scalars = s.springScalars }.Schedule(dep);
            h = new MeshSpringSetupJob
            {
                M = M, V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos, vel = s.vel, velNext = s.velNext,
                area = s.area, thickness = s.thickness, cfrac = s.cfrac, density = s.density, age = s.age, damage = s.damage,
                sb = s.sb, sr = s.sr, sz = s.sz, sp = s.sp, sdiag = s.sdiag, spart = s.spart, kc = s.kc, lc = s.lc
            }.Schedule(cap, 128, h);
            h = new MeshSpringInitXJob { alive = s.alive, pos = s.pos, vel = s.vel, sx = s.sx }.Schedule(cap, 512, h);
            h = new MeshSpringReduceJob { slot = MeshSpring.Rz, spart = s.spart, scalars = s.springScalars }.Schedule(h);
            for (int it = 0; it < iterations; it++)
            {
                h = new MeshSpringApplyJob
                {
                    M = M, V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos, sp = s.sp,
                    area = s.area, kc = s.kc, sAp = s.sAp, spart = s.spart
                }.Schedule(cap, 128, h);
                h = new MeshSpringReduceJob { slot = MeshSpring.AlphaDen, spart = s.spart, scalars = s.springScalars }.Schedule(h);
                h = new MeshSpringUpdateJob
                {
                    tolerance = tolerance, alive = s.alive, scalars = s.springScalars, sp = s.sp, sAp = s.sAp, sdiag = s.sdiag,
                    sx = s.sx, sr = s.sr, sz = s.sz, spart = s.spart
                }.Schedule(cap, 512, h);
                h = new MeshSpringReduceJob { slot = MeshSpring.RzNew, spart = s.spart, scalars = s.springScalars }.Schedule(h);
                h = new MeshSpringDirectionJob { alive = s.alive, scalars = s.springScalars, sz = s.sz, sp = s.sp }.Schedule(cap, 512, h);
                h = new MeshSpringShiftJob { scalars = s.springScalars }.Schedule(h);
            }
            h = new MeshSpringFinishJob
            {
                M = M, V = s.V, O = s.O, vc = s.vc, alive = s.alive, pos = s.pos, sx = s.sx,
                thickness = s.thickness, density = s.density, kc = s.kc, lc = s.lc,
                velNext = s.velNext, tension = s.tension
            }.Schedule(cap, 128, h);
            return h;
        }
    }

    [BurstCompile]
    public struct MeshSpringClearJob : IJob
    {
        public NativeArray<double> scalars;
        public void Execute() { for (int k = 0; k < scalars.Length; k++) scalars[k] = 0; }
    }

    [BurstCompile]
    public struct MeshSpringInitXJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos, vel;
        [WriteOnly] public NativeArray<float3> sx;
        public void Execute(int i)
        {
            if (alive[i] == 0) return;
            float3 p = pos[i], v = vel[i];
            sx[i] = v - p * math.dot(v, p);
        }
    }
}
