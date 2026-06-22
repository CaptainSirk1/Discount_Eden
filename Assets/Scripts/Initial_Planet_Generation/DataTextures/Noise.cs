using UnityEngine;

// All noise math lives here. Never call this directly — go through NoiseSettings.Sample().
// Modding Tip:
//   To add a new noise type, add it to NoiseSettings.NoiseType enum,
//   add a case in Sample() below, and implement the math here.
public static class Noise
{
    // --- Public Interface ---

    public static float Sample(Vector3 point, NoiseSettings settings, int seed)
    {
        Initialize(seed);

        float value = settings.noiseType switch
        {
            NoiseSettings.NoiseType.RidgedMultifractal => SampleRidged(point, settings),
            _ => SampleFBm(point, settings)
        };

        return value * settings.strength;
    }

    public static Vector3 SampleWarp(Vector3 point, NoiseSettings settings, int seed)
    {
        Initialize(seed);

        // Three independent FBm samples offset to avoid correlation
        float wx = FBm(point.x + 1.3f, point.y + 2.1f, point.z + 0.7f, settings);
        float wy = FBm(point.x + 4.7f, point.y + 0.9f, point.z + 3.2f, settings);
        float wz = FBm(point.x + 8.3f, point.y + 5.6f, point.z + 1.8f, settings);

        return new Vector3(wx, wy, wz) * settings.strength;
    }

    // --- Noise Types ---

    static float SampleFBm(Vector3 point, NoiseSettings settings)
    {
        return FBm(
            point.x * settings.scale,
            point.y * settings.scale,
            point.z * settings.scale,
            settings);
    }

    static float SampleRidged(Vector3 point, NoiseSettings settings)
    {
        float value = 0f;
        float amplitude = 1f;
        float frequency = settings.scale;
        float maxValue = 0f;

        for (int i = 0; i < settings.octaves; i++)
        {
            float noise = Simplex(
                point.x * frequency,
                point.y * frequency,
                point.z * frequency);

            // Invert and square for sharp ridges
            noise = 1f - Mathf.Abs(noise);
            noise *= noise;

            value += noise * amplitude;
            maxValue += amplitude;
            amplitude *= settings.persistence;
            frequency *= settings.lacunarity;
        }

        return maxValue > 0 ? value / maxValue : 0f;
    }

    static float FBm(float x, float y, float z, NoiseSettings settings)
    {
        float value = 0f;
        float amplitude = 1f;
        float frequency = 1f;
        float maxValue = 0f;

        for (int i = 0; i < settings.octaves; i++)
        {
            value += Simplex(x * frequency, y * frequency, z * frequency) * amplitude;
            maxValue += amplitude;
            amplitude *= settings.persistence;
            frequency *= settings.lacunarity;
        }

        return maxValue > 0 ? value / maxValue : 0f;
    }

    // --- Simplex Math ---
    // Based on Stefan Gustavson's public domain implementation.

    static int[] perm = new int[512];
    static bool initialized = false;
    static int currentSeed = -1;

    static void Initialize(int seed)
    {
        if (initialized && currentSeed == seed) return;

        int[] p = new int[256];
        for (int i = 0; i < 256; i++) p[i] = i;

        System.Random rng = new System.Random(seed);
        for (int i = 255; i > 0; i--)
        {
            int j = rng.Next(i + 1);
            int temp = p[i];
            p[i] = p[j];
            p[j] = temp;
        }

        for (int i = 0; i < 512; i++)
            perm[i] = p[i & 255];

        initialized = true;
        currentSeed = seed;
    }

    static readonly int[][] grad3 = {
        new[]{1,1,0}, new[]{-1,1,0}, new[]{1,-1,0}, new[]{-1,-1,0},
        new[]{1,0,1}, new[]{-1,0,1}, new[]{1,0,-1}, new[]{-1,0,-1},
        new[]{0,1,1}, new[]{0,-1,1}, new[]{0,1,-1}, new[]{0,-1,-1}
    };

    static float Dot(int[] g, float x, float y, float z)
        => g[0] * x + g[1] * y + g[2] * z;

    static int FastFloor(float x)
        => x > 0 ? (int)x : (int)x - 1;

    static float Simplex(float x, float y, float z)
    {
        const float F3 = 1f / 3f;
        const float G3 = 1f / 6f;

        float s = (x + y + z) * F3;
        int i = FastFloor(x + s);
        int j = FastFloor(y + s);
        int k = FastFloor(z + s);

        float t = (i + j + k) * G3;
        float x0 = x - (i - t);
        float y0 = y - (j - t);
        float z0 = z - (k - t);

        int i1, j1, k1, i2, j2, k2;
        if (x0 >= y0)
        {
            if (y0 >= z0)      { i1=1; j1=0; k1=0; i2=1; j2=1; k2=0; }
            else if (x0 >= z0) { i1=1; j1=0; k1=0; i2=1; j2=0; k2=1; }
            else               { i1=0; j1=0; k1=1; i2=1; j2=0; k2=1; }
        }
        else
        {
            if (y0 < z0)       { i1=0; j1=0; k1=1; i2=0; j2=1; k2=1; }
            else if (x0 < z0)  { i1=0; j1=1; k1=0; i2=0; j2=1; k2=1; }
            else               { i1=0; j1=1; k1=0; i2=1; j2=1; k2=0; }
        }

        float x1 = x0 - i1 + G3, y1 = y0 - j1 + G3, z1 = z0 - k1 + G3;
        float x2 = x0 - i2 + 2f*G3, y2 = y0 - j2 + 2f*G3, z2 = z0 - k2 + 2f*G3;
        float x3 = x0 - 1f + 3f*G3, y3 = y0 - 1f + 3f*G3, z3 = z0 - 1f + 3f*G3;

        int ii = i & 255, jj = j & 255, kk = k & 255;
        int gi0 = perm[ii +    perm[jj +    perm[kk   ]]] % 12;
        int gi1 = perm[ii+i1 + perm[jj+j1 + perm[kk+k1]]] % 12;
        int gi2 = perm[ii+i2 + perm[jj+j2 + perm[kk+k2]]] % 12;
        int gi3 = perm[ii+1  + perm[jj+1  + perm[kk+1 ]]] % 12;

        float t0 = 0.6f - x0*x0 - y0*y0 - z0*z0;
        float n0 = t0 < 0 ? 0f : (t0*t0)*(t0*t0) * Dot(grad3[gi0], x0, y0, z0);

        float t1 = 0.6f - x1*x1 - y1*y1 - z1*z1;
        float n1 = t1 < 0 ? 0f : (t1*t1)*(t1*t1) * Dot(grad3[gi1], x1, y1, z1);

        float t2 = 0.6f - x2*x2 - y2*y2 - z2*z2;
        float n2 = t2 < 0 ? 0f : (t2*t2)*(t2*t2) * Dot(grad3[gi2], x2, y2, z2);

        float t3 = 0.6f - x3*x3 - y3*y3 - z3*z3;
        float n3 = t3 < 0 ? 0f : (t3*t3)*(t3*t3) * Dot(grad3[gi3], x3, y3, z3);

        return 32f * (n0 + n1 + n2 + n3);
    }
}