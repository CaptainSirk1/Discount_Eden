using UnityEngine;

public class PlanetGenerator : MonoBehaviour
{
    // Add new map types as they're created.
    public enum MapType { Temperature, Wind, TectonicInfluence, HeightMap, MoistureMap }
    public MapType displayMap;

    // Controls how many vertices run along the edge of each face on the cube sphere.
    [Range(2, 256)]
    public int resolution = 10;
    public bool autoUpdate = true;

    public TerrainSettings terrainSettings;
    public ClimateSettings climateSettings;
    public ClimateState runtimeClimate;

    [HideInInspector]
    public bool terrainSettingsFoldout;
    [HideInInspector]
    public bool climateSettingsFoldout;

    TerrainShapeGenerator shapeGen;
    GenerateTectonics tectonics;

    WindTempMap windTempGen;
    TectonicInfluenceMap influenceGen;
    HeightMap heightGen;
    MoistureMap moistureGen;

    [SerializeField]
    MeshFilter[] meshFilters;
    CubeSphereFace[] terrainFaces;

    MeshRenderer[] meshRenderers;
    Material[] faceMaterials;

    void Initialize()
    {
        shapeGen = new TerrainShapeGenerator(terrainSettings); // Converts a unit sphere point into a world-space vertex position.

        if (meshFilters == null || meshFilters.Length == 0)
        {
            meshFilters = new MeshFilter[6];
        }
        // There will always be 6 cube sphere faces. 
        terrainFaces = new CubeSphereFace[6];
        
        // The number of renderers and materials may need to increase if additional textures are added.
        meshRenderers = new MeshRenderer[6];
        faceMaterials = new Material[6];

        Vector3[] directions = {Vector3.up, Vector3.down, Vector3.left, Vector3.right, Vector3.forward, Vector3.back };

        for (int i = 0; i < 6; i++)
        {
            if (meshFilters[i] == null)
            {
                GameObject meshObj = new GameObject("mesh");
                meshObj.transform.parent = transform;
                meshObj.AddComponent<MeshRenderer>().sharedMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                meshFilters[i] = meshObj.AddComponent<MeshFilter>();
                meshFilters[i].sharedMesh = new Mesh();
            }
            
            terrainFaces[i] = new CubeSphereFace(shapeGen, meshFilters[i].sharedMesh, resolution, directions[i]);

            // Cache renderer and material so GenerateColors doesn't have to search for them
            meshRenderers[i] = meshFilters[i].GetComponent<MeshRenderer>();
            faceMaterials[i] = meshRenderers[i].sharedMaterial;
        }
    }
    public void GeneratePlanet()
    {
        tectonics = new GenerateTectonics(terrainSettings);
        tectonics.Generate();

        Initialize();
        GenerateMesh();
        GenerateColors();
    }

    public void OnShapeSettingsUpdated()
    {
        if (autoUpdate)
        {
            GenerateMesh();
        }
    }

    public void OnColorSettingsUpdated()
    {
        // 1. Refresh the runtime state from the settings asset
        if (climateSettings != null)
        {
            runtimeClimate = new ClimateState(climateSettings);
        }
        
        if (autoUpdate)
        {
            GenerateColors();
        }
    }

    void GenerateMesh()
    {
        foreach (CubeSphereFace face in terrainFaces)
        {
            face.ConstructMesh();
        }
    }
    
    void GenerateColors()
    {
        if (runtimeClimate == null && climateSettings != null)
            runtimeClimate = new ClimateState(climateSettings);

        if (runtimeClimate == null) return;

        // Generate all maps — only runs when GeneratePlanet() is called
        windTempGen = new WindTempMap(runtimeClimate);
        windTempGen.Generate();

        influenceGen = new TectonicInfluenceMap(tectonics, terrainSettings);
        influenceGen.Generate();

        heightGen = new HeightMap(tectonics, terrainSettings, influenceGen);
        heightGen.Generate();

        moistureGen = new MoistureMap(climateSettings, terrainSettings, heightGen, influenceGen, tectonics);
        moistureGen.Generate();

        // Apply whichever map is selected
        ApplyDisplayMap();
    }

    public void ApplyDisplayMap()
    {
        // Guard against maps not being generated yet
        if (windTempGen == null || influenceGen == null || heightGen == null || moistureGen == null) return;

        Texture2D textureToDisplay = displayMap switch
        {
            MapType.Temperature      => windTempGen.temperatureMap,
            MapType.Wind             => windTempGen.windMap,
            MapType.TectonicInfluence => influenceGen.tectonicInfluence,
            MapType.HeightMap        => heightGen.height,
            MapType.MoistureMap      => moistureGen.moisture,
            _                        => moistureGen.moisture

        };

        // Use cached materials — no GetComponent call needed
        foreach (Material mat in faceMaterials)
        {
            if (mat == null) continue;
            mat.SetTexture("_BaseMap", textureToDisplay);
            mat.color = Color.white;
        }
    }
}