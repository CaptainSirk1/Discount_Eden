using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using PlanetGeneration;

public class HeightMap
{
    GenerateTectonics tectonics;
    TerrainSettings settings;
    TectonicInfluenceMap influence;
    
    public Texture2D height {get; private set;}
    public Color[] heightData {get; private set;} 

    public HeightMap(GenerateTectonics tectonics, TerrainSettings settings, TectonicInfluenceMap influence)
    {
        this.tectonics = tectonics;
        this.settings = settings;
        this.influence = influence;
    }
    public void Generate()
    {
        int width = UVHelper.MAP_WIDTH;
        int height = UVHelper.MAP_HEIGHT;
        int totalPixels = width * height;

        NativeArray<Color32> hData = new NativeArray<Color32>(totalPixels, Allocator.TempJob);
        NativeArray<Color32> iData = new NativeArray<Color32>(totalPixels, Allocator.TempJob);
        
        // Convert influence data to Color32
        Color32[] infl32 = new Color32[totalPixels];
        for(int i=0; i<totalPixels; i++) infl32[i] = influence.influenceData[i];
        iData.CopyFrom(infl32);

        tectonics.GetJobData(out NativeArray<TectonicCell> cells, out NativeArray<int> offsets, out NativeArray<int> indices);
        influence.GetJobData(out NativeArray<Vector3> fPoints, out NativeArray<float> fStrengths);

        var job = new HeightMapJob
        {
            width = width,
            height = height,
            worldSeed = settings.worldSeed,
            trenchDepthMultiplier = settings.trenchDepthMultiplier,
            warpNoise = NoiseData.FromSettings(settings.warpNoise, settings.worldSeed),
            terrainNoise = NoiseData.FromSettings(settings.terrainNoise, settings.worldSeed),
            mountainNoise = NoiseData.FromSettings(settings.mountainNoise, settings.worldSeed),
            cells = cells,
            bucketOffsets = offsets,
            bucketIndices = indices,
            bucketSize = 8,
            influenceData = iData,
            heightData = hData
        };

        JobHandle handle = job.Schedule(totalPixels, 64);
        handle.Complete();

        this.height = new Texture2D(width, height, TextureFormat.RGB24, false);
        heightData = new Color[totalPixels];
        Color32[] h32 = new Color32[totalPixels];
        hData.CopyTo(h32);
        for (int i = 0; i < totalPixels; i++) heightData[i] = h32[i];

        this.height.SetPixels32(h32);
        this.height.Apply();

        hData.Dispose();
        iData.Dispose();
        cells.Dispose();
        offsets.Dispose();
        indices.Dispose();
        fPoints.Dispose();
        fStrengths.Dispose();
    }
}