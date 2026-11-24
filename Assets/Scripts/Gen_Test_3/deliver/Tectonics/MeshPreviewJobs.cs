using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace PlanetGen.Tectonics.MeshSim
{
    // ---------------------------------------------------------------------------------
    // Live preview: the simulation mesh itself. One render vertex per vertex slot plus one spare
    // "dead" vertex at the origin that unused triangles and edges collapse onto.
    // New views are appended at the end so existing serialized values keep their meaning.
    // ---------------------------------------------------------------------------------
    public enum MeshDebugView
    {
        Elevation, Thickness, Age, FaultType, Orogeny, Hotspot, Velocity, EdgeLength, Valence, TopologyEvents, Thermal, Damage,
        /// <summary>Stress: attempted opening of the target velocity field relative to the crust's intact yield.</summary>
        Stress,
        /// <summary>Extension: achieved opening strain rate (after the limits), velRef per radian.</summary>
        Extension,
        /// <summary>Magmatism: recent arc-magmatic growth rate.</summary>
        Magmatism,
        /// <summary>Felsic: differentiated share of the crust (continental material), independent of thickness.</summary>
        Felsic,
        /// <summary>Blocks: rigid continental blocks of the last step, one colour each (grey = not in a block).</summary>
        Blocks
    }

    [BurstCompile]
    public struct MeshPreviewVertexJob : IJobParallelFor
    {
        public MeshParams M;
        public int view;
        public float radius;

        [ReadOnly] public NativeArray<int> V, O, vc;
        [ReadOnly] public NativeArray<byte> alive, faultType;
        [ReadOnly] public NativeArray<int> blockId;
        [ReadOnly] public NativeArray<float3> pos, vel;
        [ReadOnly] public NativeArray<float> thickness, cfrac, felsic, volume, age, faultActivity, orogeny, hotspot, flashSplit, flashCollapse, damage, tension, achieved, magma;

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

        /// <summary>Dimmed elevation underlay with a 0..1 heat overlay: grey -> yellow -> red.</summary>
        float4 Overlay(int i, float x)
        {
            float4 b = Elevation(i) * 0.3f;
            float3 dc = x < 0.5f ? math.lerp(new float3(0.3f, 0.3f, 0.3f), new float3(1f, 0.85f, 0.1f), x * 2f)
                                 : math.lerp(new float3(1f, 0.85f, 0.1f), new float3(1f, 0.05f, 0.02f), x * 2f - 1f);
            return new float4(math.lerp(b.xyz, dc, math.saturate(x * 4f)), 1f);
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
                case 11: // Damage, with a blue tint where stress exceeds the damaged yield
                {
                    float4 c = Overlay(i, damage[i]);
                    float Y = MeshLimits.IntactYield(cfrac[i], age[i], M) * (1f - M.dmgWeaken * damage[i]);
                    if (tension[i] > Y) c.xyz += new float3(0f, 0.1f, 0.35f) * math.saturate((tension[i] - Y) / math.max(Y, 1e-3f));
                    return c;
                }
                case 12: // Stress / intact yield: 0.5 = half way to breaking, 1 = breaking intact crust
                    return Overlay(i, 0.5f * tension[i] / math.max(MeshLimits.IntactYield(cfrac[i], age[i], M), 1e-4f));
                case 13: // achieved extension, 1 = velocityReference per radian
                    return Overlay(i, achieved[i]);
                case 14: // magmatic growth: 1 = 0.05 km/Myr
                    return Overlay(i, magma[i] / 0.05f);
                case 15: // felsic share: ocean floor dark blue -> continental material sand
                    return new float4(math.lerp(new float3(0.05f, 0.1f, 0.35f), new float3(0.9f, 0.75f, 0.45f), math.saturate(felsic[i] / math.max(volume[i], 1e-20f))), 1f);
                case 16: // rigid blocks
                {
                    int b = blockId[i];
                    if (b < 0) return Elevation(i) * 0.35f + new float4(0f, 0f, 0f, 0.65f);
                    float h = TMath.Rand01((uint)b * 2654435761u);
                    return new float4(Hsv(h, 0.6f, 0.9f), 1f);
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
