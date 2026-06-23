using UnityEngine;

public class HeightMap
{
    GenerateTectonics tectonics;
    TerrainSettings settings;
    TectonicInfluenceMap influence;
    
    public Texture2D height {get; private set;}

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
        Color[] pixels = new Color[UVHelper.MAP_WIDTH * UVHelper.MAP_HEIGHT];

        for (int y = 0; y < UVHelper.MAP_HEIGHT; y++)
        {
            for (int x = 0; x < UVHelper.MAP_WIDTH; x++)
            {
                float u = x / (float)(UVHelper.MAP_WIDTH - 1);
                float v = y / (float)(UVHelper.MAP_HEIGHT - 1);

                float faultDepression = 0;

                // Guard — if no mountain noise is assigned, skip noise sampling entirely
                bool hasMountainNoise = settings.mountainNoise != null;
                bool hasTerrainNoise = settings.terrainNoise != null;

                Vector3 spherePoint = UVHelper.UVToSphere(u, v);
                Vector3 warpedPoint = UVHelper.WarpSpherePoint(spherePoint, settings.warpNoise, settings.worldSeed);

                // Configure Depth and Radius to determine how wide and deep divergent fault lines run.
                float faultRadius = 0.05f;
                float faultDepth = 0.4f;
                if (influence.splineGrid.GetNearestPoint(spherePoint, out _, out float faultStrength, out float dSq))
                {
                    float dist = Mathf.Sqrt(dSq);
                    float falloff = Mathf.Clamp01(1f - dist / faultRadius);
                    falloff *= falloff;
                    faultDepression = falloff * faultStrength * faultDepth;
                }

                float elevation = tectonics.GetBlendedElevation(warpedPoint);
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
                    // Sample influence map at this pixel position
                    Vector2 influenceUV = UVHelper.SphereToUV(spherePoint);
                    Color influenceValue = influence.tectonicInfluence.GetPixelBilinear(influenceUV.x, influenceUV.y);
                    float convergent = influenceValue.r;
                    float divergent = influenceValue.b;

                    // Scale noise opacity by influence channel values
                    // Base presence everywhere, stronger in boundary zones
                    float noiseOpacity = 0.05f
                                    + convergent * 0.95f   // up to 1.0 in convergent zones
                                    + divergent * 0.25f;   // up to 0.3 in divergent zones

                    float mountainNoise = settings.mountainNoise.Sample(spherePoint, settings.worldSeed);
                    mountainColor = mountainNoise * noiseOpacity;
                }

                float finalColor = Mathf.Clamp01(baseColor + terrainColor + mountainColor - faultDepression);

                pixels[y * UVHelper.MAP_WIDTH + x] = new Color(finalColor, finalColor, finalColor);   
            }
        }
        tex.SetPixels(pixels);
        tex.Apply();
        return tex;
    }
}