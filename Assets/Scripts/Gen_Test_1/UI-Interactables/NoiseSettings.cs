using UnityEngine;

// Configurable noise asset. Create one per LOD level or noise use case.
// Assign in TerrainSettings or directly on LOD components.
// All noise sampling goes through this object — callers never touch SimplexNoise directly.
[CreateAssetMenu(menuName = "Planet/Noise Settings")]
public class NoiseSettings : ScriptableObject
{
    public enum NoiseType { Simplex, RidgedMultifractal }

    [Header("Additive Layer")]
    [Tooltip("Optional custom noise layer. Can chain multiple NoiseSettings together for complex results")]
    public NoiseSettings additiveLayer;

    [Range(0f, 1f)] public float additiveStrength = 0.5f;

    [Header("Noise Type")]
    public NoiseType noiseType = NoiseType.RidgedMultifractal;

    [Header("Fractal Parameters")]
    [Range(1, 8)]   public int octaves = 4;
    [Range(0.1f, 8f)] public float scale = 2f;
    [Range(1f, 4f)] public float lacunarity = 2f;
    [Range(0f, 1f)] public float persistence = 0.5f;

    [Header("Output")]
    [Range(0f, 1f)] public float strength = 1f;

    // Returns a scalar height value — used for terrain noise and mountain noise
    public float Sample(Vector3 point, int seed, int depth = 0)
    {
        // Depth limit prevents infinite loops from circular asset references
        if (depth > 8) return 0f;

        float value = Noise.Sample(point, this, seed);

        if (additiveLayer != null)
            value += additiveLayer.Sample(point, seed, depth + 1) * additiveStrength;

        return value;
    }

    // Returns a 3D warp vector — used for domain warping before cell lookups
    public Vector3 SampleWarp(Vector3 point, int seed, int depth = 0)
    {
        if (depth > 8) return Vector3.zero;

        Vector3 value = Noise.SampleWarp(point, this, seed);

        if (additiveLayer != null)
            value += additiveLayer.SampleWarp(point, seed, depth + 1) * additiveStrength;

        return value;
    }
}