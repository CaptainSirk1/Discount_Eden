using System;
using UnityEngine;

[CreateAssetMenu]
public class TerrainSettings : ScriptableObject
{
    // Adjustable Terrain Presets that will change how the planet looks given the same seed

    [Tooltip("Radius of the planet in world units.")]
    [Range(1f, 100f)] public float planetRadius = 10f;

    [Header("Global Terrain Presets")]
    [Tooltip("The unique seed used to generate the Voronoi distribution and base landmasses.")]
    public int worldSeed = 0;


    [Tooltip("Number of voronoi sections the world is divided into. 'Tectonic resolution,' as it were. Also affects the width of mountain ranges.")]
    [Range(10, 300)] public int cellCount = 100;

    [Tooltip("Number of 'Continent Center' nodes. These nodes will generate first. Additional nodes will have settings weighted towards similarity with nearest nodes.")]
    [Range(4, 150)] public int continentCenters = 10;

    [Tooltip("Increases the average movement strength of tectonic cells. Higher values mean more mountainous planets.")]
    [Range(0f, 1f)] public float strengthMultiplier = 0.7f;

    [Tooltip("The total height range of the planetary heightmap.")]
    [Range(5, 20)] public int heightRange = 10;

    [Tooltip("Determines the strength of noise used to break up inorganic voronoi divisions")]
    [Range(0f, 1f)] public float continentEdgeNoise = 0.8f;

    [Tooltip("Determines how likely neighboring cells are to share a similar elevation")]
    [Range(0.1f, 0.9f)]
    public float elevationCoherence = 0.9f;

    [Header("Noise Settings")]
    [Tooltip("Domain warp applied to voronoi cell lookups. Controls how organic cell boundaries appear.")]
    public NoiseSettings warpNoise;

    [Tooltip("Broad terrain variation across the whole planet surface.")]
    public NoiseSettings terrainNoise;

    [Tooltip("Mountain and ridge noise applied in boundary zones.")]
    public NoiseSettings mountainNoise;

    [Header("Climate Inputs")]
    [Tooltip("The maximum possible depth of the oceans if all water is liquid.")]
    [Range(0.1f, 1f)] public float globalMoisture = 0.5f;

    [Tooltip("The degree to which mountainous terrain blocks moisture propagation")]
    [Range(1f, 2f)] public float shieldMultiplier = 1.2f;


    private void OnValidate()
    {
        continentCenters = Mathf.Clamp(continentCenters, 1, cellCount);
    }    
}