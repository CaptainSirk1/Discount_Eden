using UnityEngine;

public class TerrainShapeGenerator
{
    public TerrainSettings settings { get; private set; }

    public TerrainShapeGenerator(TerrainSettings settings)
    {
        this.settings = settings;
    }

    public Vector3 CalculatePointOnPlanet(Vector3 pointOnUnitSphere)
    {
        return pointOnUnitSphere * settings.planetRadius;
    }
}