using UnityEngine;
using System.Collections.Generic;

public class MoistureMap
{
    const int MAP_WIDTH = UVHelper.MAP_WIDTH;
    const int MAP_HEIGHT = UVHelper.MAP_HEIGHT;

    ClimateSettings cSettings;
    TerrainSettings tSettings;
    GenerateTectonics tectonics;
    HeightMap height;
    TectonicInfluenceMap influence;

    public Texture2D moisture {get; private set;}

    public MoistureMap( ClimateSettings cSettings, 
                        TerrainSettings tSettings, 
                        HeightMap height, 
                        TectonicInfluenceMap influence, 
                        GenerateTectonics tectonics)
    {
        this.cSettings = cSettings;
        this.tSettings = tSettings;
        this.height = height;
        this.influence = influence;
        this.tectonics = tectonics;

    }

    public void Generate()
    {
        Queue<int> frontier = new Queue<int>();
        HashSet<int> assigned = new HashSet<int>();
        AssignOceanMoisture(frontier, assigned);
        AssignDerivedMoisture(frontier, assigned);
        moisture = GenerateMoistureMap();
    }

    void AssignOceanMoisture(Queue<int> frontier, HashSet<int> assigned)
    {
        float oceanThreshold = cSettings.globalMoisture * 2f - 1f;
        for (int i = 0; i < tectonics.cells.Length; i++)
        {
            if (tectonics.cells[i].baseElevation < oceanThreshold)
            {
                tectonics.cells[i].moistureLevel = 1f;
                frontier.Enqueue(i);
                assigned.Add(i);
            }
        }
    }

    void AssignDerivedMoisture(Queue<int> frontier, HashSet<int> assigned)
    {
        while (frontier.Count > 0)
        {
            int currentIdx = frontier.Dequeue();
            TectonicCell current = tectonics.cells[currentIdx];
            Vector3 direction = WindTempMap.CalculateWindVector3D(current.position);
            int nextIdx = tectonics.GetCellInDirection(current.position, direction);

            if (nextIdx == -1 || nextIdx == currentIdx || assigned.Contains(nextIdx)) continue;

            Vector3 boundarySample = ((current.position + tectonics.cells[nextIdx].position) * 0.5f).normalized;
            Vector2 uv = UVHelper.SphereToUV(boundarySample);
            Color influenceColor = influence.tectonicInfluence.GetPixelBilinear(uv.x, uv.y);
            float moistureShield = influenceColor.r + influenceColor.b;

            float newMoisture = Mathf.Clamp01(current.moistureLevel - (0.01f + moistureShield * tSettings.shieldMultiplier));
            tectonics.cells[nextIdx].moistureLevel = newMoisture;

            if (newMoisture > 0f)
            {
                assigned.Add(nextIdx);
                frontier.Enqueue(nextIdx);
            }
        }
    }

    Texture2D GenerateMoistureMap()
    {
        Texture2D tex = new Texture2D(MAP_WIDTH, MAP_HEIGHT, TextureFormat.RGB24, false);
        Color[] pixels = new Color[MAP_WIDTH * MAP_HEIGHT];

        Color dryColor = new Color(0.6f, 0.55f, 0.32f); // Desert tan
        Color wetColor = new Color(0.1f, 0.55f, 0.05f); // Deep green

        for (int y = 0; y < MAP_HEIGHT; y++)
        {
            for (int x = 0; x < MAP_WIDTH; x++)
            {
                float u = x / (float)(MAP_WIDTH - 1);
                float v = y / (float)(MAP_HEIGHT - 1);

                Vector3 spherePoint = UVHelper.UVToSphere(u, v);

                Vector2 heightUV = UVHelper.SphereToUV(spherePoint);
                Color heightSample = height.height.GetPixelBilinear(heightUV.x, heightUV.y);
                float elevation = heightSample.r;

                // Convert texture space elevation back to -1/1 elevation space
                // Inverse of (elevation + 1f) / 4f
                float elevationRaw = (elevation * 4f) - 1f;

                // Now compare directly against oceanThreshold in -1/1 space
                float oceanThreshold = cSettings.globalMoisture * 2f - 1f;

                Color pixelColor;
                if (elevationRaw <= oceanThreshold)
                {
                    // Ocean — depth based on how far below threshold
                    float depth = Mathf.Clamp01((oceanThreshold - elevationRaw) / 2f);
                    pixelColor = Color.Lerp(new Color(0.2f, 0.4f, 0.8f), new Color(0f, 0.1f, 0.4f), depth);
                }
                else
                {
                    // Land — elevation above sea level in -1/1 space
                    float blendedMoisture = tectonics.GetBlendedMoisture(spherePoint);
                    float elevationAboveSeaLevel = Mathf.Max(0, elevationRaw - oceanThreshold);
                    float moistureValue = Mathf.Clamp01(blendedMoisture - elevationAboveSeaLevel * 0.3f);
                    pixelColor = Color.Lerp(dryColor, wetColor, moistureValue);
                }

                pixels[y * MAP_WIDTH + x] = pixelColor;
            }
        }
        tex.SetPixels(pixels);
        tex.Apply();
        return tex;
    }
}