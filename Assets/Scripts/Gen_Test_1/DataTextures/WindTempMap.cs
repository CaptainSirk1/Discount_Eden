using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using PlanetGeneration;

// Generation of a temperature and wind map.
// The data produced in these maps will inform data calculations for the moisture map.
// Every time the input data changes, new maps are generated.

public class WindTempMap
{
    const int MAP_WIDTH = UVHelper.MAP_WIDTH;
    const int MAP_HEIGHT = UVHelper.MAP_HEIGHT;

    ClimateState climate;

    public Texture2D temperatureMap { get; private set; }
    public Texture2D windMap { get; private set; }

    public WindTempMap(ClimateState climate)
    {
        this.climate = climate;
    }

    public void Generate()
    {
        int totalPixels = MAP_WIDTH * MAP_HEIGHT;
        NativeArray<Color32> tempPixels = new NativeArray<Color32>(totalPixels, Allocator.TempJob);
        NativeArray<Color32> windPixels = new NativeArray<Color32>(totalPixels, Allocator.TempJob);

        var job = new WindTempMapJob
        {
            width = MAP_WIDTH,
            height = MAP_HEIGHT,
            averageTemp = climate.averageTemp,
            tempPixels = tempPixels,
            windPixels = windPixels
        };

        JobHandle handle = job.Schedule(totalPixels, 64);
        handle.Complete();

        temperatureMap = new Texture2D(MAP_WIDTH, MAP_HEIGHT, TextureFormat.RGB24, false);
        windMap = new Texture2D(MAP_WIDTH, MAP_HEIGHT, TextureFormat.RGB24, false);

        Color32[] tempArr = new Color32[totalPixels];
        Color32[] windArr = new Color32[totalPixels];
        tempPixels.CopyTo(tempArr);
        windPixels.CopyTo(windArr);

        temperatureMap.SetPixels32(tempArr);
        temperatureMap.Apply();
        windMap.SetPixels32(windArr);
        windMap.Apply();

        tempPixels.Dispose();
        windPixels.Dispose();
    }


    // Returns wind as a 3D sphere-surface tangent vector for cell propagation.
    // East vector is derived from the sphere point so it's always tangent to the surface,
    // correctly handling pole distortion that the UV-space version ignores.
    public static Vector3 CalculateWindVector3D(Vector3 pointOnUnitSphere)
    {
        float latitude = Mathf.Asin(Mathf.Clamp(pointOnUnitSphere.y, -1f, 1f));
        float zonalStrength = -Mathf.Sin(3f * latitude);
        float meridionalStrength = Mathf.Sin(6f * latitude) * 0.2f;

        // East direction — always tangent to sphere surface, accounts for pole distortion
        Vector3 east = Vector3.Cross(Vector3.up, pointOnUnitSphere).normalized;
        // North direction — perpendicular to east and sphere normal
        Vector3 north = Vector3.Cross(pointOnUnitSphere, east).normalized;

        return (east * zonalStrength + north * meridionalStrength).normalized;
    }

    // --- Sampling ---
    // These are what ClimateSimulator calls when it needs atmospheric
    // data at a given cell position.

    public float SampleTemperature(Vector3 pointOnUnitSphere)
    {
        // Keep in mind that the blue channel on the temperature map is purely visual.
        // The red is the only channel that needs to be sampled to extract temperature data.
        Vector2 uv = UVHelper.SphereToUV(pointOnUnitSphere);
        Color pixel = temperatureMap.GetPixelBilinear(uv.x, uv.y);
        return pixel.r;
    }

    public Vector2 SampleWind(Vector3 pointOnUnitSphere)
    {
        Vector2 uv = UVHelper.SphereToUV(pointOnUnitSphere);
        Color pixel = windMap.GetPixelBilinear(uv.x, uv.y);

        // Decode from 0/1 storage back to -1/+1 direction vector
        float x = pixel.r * 2f - 1f;
        float y = pixel.g * 2f - 1f;
        return new Vector2(x, y);
    }
}