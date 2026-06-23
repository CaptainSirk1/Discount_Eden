using UnityEngine;
using System.Collections.Generic;

public class HeightMap
{
    GenerateTectonics tectonics;
    TerrainSettings settings;
    TectonicInfluenceMap influence;
    
    public Texture2D height {get; private set;}
    public Color[] heightData {get; private set;} // Optimization #4

    public HeightMap(GenerateTectonics tectonics, TerrainSettings settings, TectonicInfluenceMap influence)
    {
        this.tectonics = tectonics;
        this.settings = settings;
        this.influence = influence;
    }
    public void Generate()
    {
        height = GenerateHeight();
    }

    Texture2D GenerateHeight()
    {
        Texture2D tex = new Texture2D(UVHelper.MAP_WIDTH, UVHelper.MAP_HEIGHT, TextureFormat.RGB24, false);
        heightData = new Color[UVHelper.MAP_WIDTH * UVHelper.MAP_HEIGHT];

        // Pre-compute trigonometric values
        float[] cosLong = new float[UVHelper.MAP_WIDTH];
        float[] sinLong = new float[UVHelper.MAP_WIDTH];
        for (int x = 0; x < UVHelper.MAP_WIDTH; x++)
        {
            float u = x / (float)(UVHelper.MAP_WIDTH - 1);
            float longitude = (u - 0.5f) * 2f * Mathf.PI;
            cosLong[x] = Mathf.Cos(longitude);
            sinLong[x] = Mathf.Sin(longitude);
        }

        float[] cosLat = new float[UVHelper.MAP_HEIGHT];
        float[] sinLat = new float[UVHelper.MAP_HEIGHT];
        for (int y = 0; y < UVHelper.MAP_HEIGHT; y++)
        {
            float v = y / (float)(UVHelper.MAP_HEIGHT - 1);
            float latitude = (0.5f - v) * Mathf.PI;
            cosLat[y] = Mathf.Cos(latitude);
            sinLat[y] = Mathf.Sin(latitude);
        }

        List<int> nearbyScratchpad = new List<int>();

        for (int y = 0; y < UVHelper.MAP_HEIGHT; y++)
        {
            float cLat = cosLat[y];
            float sLat = sinLat[y];

            for (int x = 0; x < UVHelper.MAP_WIDTH; x++)
            {
                float faultDepression = 0;

                // Guard — if no mountain noise is assigned, skip noise sampling entirely
                bool hasMountainNoise = settings.mountainNoise != null;
                bool hasTerrainNoise = settings.terrainNoise != null;

                // Fast sphere point construction
                Vector3 spherePoint = new Vector3(cLat * cosLong[x], sLat, cLat * sinLong[x]);
                Vector3 warpedPoint = UVHelper.WarpSpherePoint(spherePoint, settings.warpNoise, settings.worldSeed);

                // Configure Depth and Radius to determine how wide and deep divergent fault lines run.
                float faultRadius = 0.04f;
                float faultDepth = 0.2f;
                if (influence.splineGrid.GetNearestPoint(spherePoint, out _, out float faultStrength, out float dSq))
                {
                    float dist = Mathf.Sqrt(dSq);
                    float falloff = Mathf.Clamp01(1f - dist / faultRadius);
                    falloff *= falloff;
                    faultDepression = falloff * faultStrength * faultDepth;
                }

                float elevation = tectonics.GetBlendedElevation(warpedPoint, nearbyScratchpad);
                float baseColor = (elevation + 1f) / 4f;

                float terrainColor = 0f;
                if (hasTerrainNoise)
                {
                    float terrainNoise = settings.terrainNoise.Sample(spherePoint, settings.worldSeed);
                    terrainColor = terrainNoise * 0.1f;
                }

                float mountainColor = 0f;
                if (hasMountainNoise)
                {
                    // Optimization #4: Use raw array access instead of GetPixelBilinear
                    Color influenceValue = influence.influenceData[y * UVHelper.MAP_WIDTH + x];
                    float convergent = influenceValue.r;
                    float divergent = influenceValue.b;

                    // Scale noise opacity by influence channel values
                    float noiseOpacity = 0.05f
                                    + convergent * 0.95f   // up to 1.0 in convergent zones
                                    + divergent * 0.25f;   // up to 0.3 in divergent zones

                    float mountainNoise = settings.mountainNoise.Sample(spherePoint, settings.worldSeed);
                    mountainColor = mountainNoise * noiseOpacity;
                }

                float finalColor = Mathf.Clamp01(baseColor + terrainColor + mountainColor - faultDepression);

                heightData[y * UVHelper.MAP_WIDTH + x] = new Color(finalColor, finalColor, finalColor);   
            }
        }
        tex.SetPixels(heightData);
        tex.Apply();
        return tex;
    }
}