using UnityEngine;

public class TerrainShapeGenerator
{
    TerrainSettings settings;

    public TerrainShapeGenerator(TerrainSettings settings)
    {
        this.settings = settings;
    }

    public Vector3 CalculatePointOnPlanet(Vector3 pointOnUnitSphere)
    {
        return pointOnUnitSphere * settings.planetRadius;
    }
}