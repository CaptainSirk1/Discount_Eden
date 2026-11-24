using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace PlanetGen.Tectonics.MeshSim
{
    /// <summary>
    /// Emergent rigid continental blocks. Single-threaded, O(V + E) per step.
    ///
    /// The spring solve (MeshSolverJobs) resists strain but cannot make a large interior rigid at an affordable
    /// iteration count: measured on uniformly strong crust, a coherence length of 2 rad still lets through 13 % of the
    /// drive's strain after full convergence, and the cost grows with coherence / edge length. So intact continental
    /// crust is additionally made to move as rigid blocks:
    ///
    ///   1. union-find joins neighbouring vertices that are both INTACT BUOYANT crust (crust type >= 0.5, damage below
    ///      blockDamageLimit) across an edge the spring solve kept stiff (not an active trench).
    ///   2. each block of at least blockMinVertices gets the area-weighted best-fit rigid rotation of its solved
    ///      velocities:  omega = I^-1 sum_i A_i (p_i x v_i),   I = sum_i A_i (1 - p_i p_i^T)
    ///   3. each member is blended toward omega x p_i by blockRigidity x (1 - D_i / blockDamageLimit).
    ///
    /// Nothing is stored between steps: blocks are recomputed from the crust every step, so a block splits as soon as
    /// a damaged zone cuts it, and two continents that collide only fuse once the suture has healed. Strain therefore
    /// concentrates in damaged zones and in oceanic crust, which keep the spring-network behaviour. Velocities only:
    /// no material is moved here.
    /// </summary>
    [BurstCompile]
    public struct MeshBlockJob : IJob
    {
        public MeshParams M;
        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive;
        [ReadOnly] public NativeArray<float3> pos;
        [ReadOnly] public NativeArray<float> area, cfrac, damage, kc;
        public NativeArray<float3> velNext;
        public NativeArray<int> parent, blockId;
        public NativeArray<float3> blockL;      // per root: sum A (p x v)
        public NativeArray<float3> blockI0, blockI1, blockI2;   // per root: rows of sum A (1 - p p^T)
        public NativeArray<float> blockA;       // per root: member count (as float)

        int Find(int x)
        {
            int r = x;
            while (parent[r] != r) r = parent[r];
            while (parent[x] != r) { int nx = parent[x]; parent[x] = r; x = nx; }
            return r;
        }

        bool Eligible(int i) => alive[i] == 1 && cfrac[i] >= 0.5f && damage[i] < M.blockDamage;

        public void Execute()
        {
            int n = alive.Length;
            for (int i = 0; i < n; i++)
            {
                parent[i] = i; blockId[i] = -1;
                blockL[i] = float3.zero; blockI0[i] = float3.zero; blockI1[i] = float3.zero; blockI2[i] = float3.zero; blockA[i] = 0f;
            }
            if (M.blockRigidity <= 0f) return;

            // 1. connectivity of intact buoyant crust
            for (int i = 0; i < n; i++)
            {
                if (!Eligible(i)) continue;
                int c0 = vc[i], c = c0, g = 0;
                do
                {
                    int j = V[MeshTopo.Next(c)];
                    if (j > i && kc[c] > 0f && Eligible(j))
                    {
                        int a = Find(i), b = Find(j);
                        if (a != b) parent[math.max(a, b)] = math.min(a, b);
                    }
                    c = MeshTopo.Next(O[MeshTopo.Next(c)]);
                } while (c != c0 && ++g < MeshTopo.MaxRing);
            }

            // 2. per-block angular momentum and inertia
            for (int i = 0; i < n; i++)
            {
                if (!Eligible(i)) continue;
                int r = Find(i);
                blockId[i] = r;
                float3 p = pos[i]; float A = area[i];
                blockL[r] = blockL[r] + math.cross(p, velNext[i]) * A;
                blockI0[r] = blockI0[r] + (new float3(1f, 0f, 0f) - p * p.x) * A;
                blockI1[r] = blockI1[r] + (new float3(0f, 1f, 0f) - p * p.y) * A;
                blockI2[r] = blockI2[r] + (new float3(0f, 0f, 1f) - p * p.z) * A;
                blockA[r] = blockA[r] + 1f;
            }

            // 3. rigid rotation per block, then blend members toward it
            for (int i = 0; i < n; i++)
            {
                int r = blockId[i];
                if (r < 0) continue;
                if (blockA[r] < M.blockMinVertices) { blockId[i] = -1; continue; }
                var I = new float3x3(blockI0[r], blockI1[r], blockI2[r]);   // symmetric, so row/column order is moot
                if (math.abs(math.determinant(I)) < 1e-18f) continue;
                float3 omega = math.mul(math.inverse(I), blockL[r]);
                float3 p = pos[i];
                float3 rigid = math.cross(omega, p);
                float w = M.blockRigidity * math.saturate(1f - damage[i] / math.max(M.blockDamage, 1e-4f));
                float3 v = math.lerp(velNext[i], rigid, w);
                velNext[i] = v - p * math.dot(v, p);
            }
        }
    }
}
