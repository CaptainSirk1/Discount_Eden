using UnityEngine;

// Generation of a temperature and wind map.
// The data produced in these maps will inform data calculations for the moisture map.
// Every time the input data changes, new maps are generated.

// If one wanted to make the axial tilt or rotation speed of the planet, it might be worth tying those new variables into this script somehow.
// At the moment, they're pretty much set in stone, with the planet being locked into an Earthlike tilt and rotation speed.
public class WindTempMap
{

    const int MAP_WIDTH = UVHelper.MAP_WIDTH;
    const int MAP_HEIGHT = UVHelper.MAP_HEIGHT;

    // The class will need to read averageTemp from the climate state to shift the temeprature map gradient.
    // This is the only piece of data passed into this class.
    ClimateState climate;

    // Create 2 separate texture maps. One for Temperature. One for Wind Direction.
    public Texture2D temperatureMap { get; private set; }
    public Texture2D windMap { get; private set; }


    public WindTempMap(ClimateState climate)
    {
        // the climate passed in from the ClimateState script will be ever changing.
        // This reads whatever the current climate state happens to be and assigns it to a given texture map upon generation.
        this.climate = climate;
    }

    public void Generate()
    {
        // Calls the functions to generate each map and stores the result.
        temperatureMap = GenerateTemperatureMap();
        windMap = GenerateWindMap();
    }

   
    // --- Temperature Map ---

    // Iterates over every uv coordinate in a Texture2d and calls helper functions to assign temperature color data
    Texture2D GenerateTemperatureMap()
    {   // Possible Texture2D formats are R8, RG16, RGB24, and RGBA32.
        // Unity can get kinda weird below RGB24.
        Texture2D tex = new Texture2D(MAP_WIDTH, MAP_HEIGHT, TextureFormat.RGB24, false);
        Color[] pixels = new Color[MAP_WIDTH * MAP_HEIGHT];

        for (int y = 0; y < MAP_HEIGHT; y++)
        {
            for (int x = 0; x < MAP_WIDTH; x++)
            {
                float u = x / (float)(MAP_WIDTH - 1);
                float v = y / (float)(MAP_HEIGHT - 1);

                Vector3 spherePoint = UVHelper.UVToSphere(u, v);
                float temp = CalculateTemperature(spherePoint);

                // RGB24 texture — temperature stored in red channel as 0 to 1, blue channel as 1 to 0.
                // Lower temp = more blue. Higher temp = more red.
                pixels[y * MAP_WIDTH + x] = new Color(temp, 0, 1f - temp);
            }
        }

        tex.SetPixels(pixels);
        tex.Apply();
        return tex;
    }

    // Returns a 'temp' value that can be turned into color data.
    float CalculateTemperature(Vector3 pointOnUnitSphere)
    {
        // Latitude ranges from -PI/2 to PI/2
        // Cosine of latitude gives 1.0 at equator, 0.0 at poles
        float latitude = Mathf.Asin(Mathf.Clamp(pointOnUnitSphere.y, -1f, 1f));
        float latitudeTemp = Mathf.Cos(latitude);

        // averageTemp slider shifts the whole gradient hotter or colder.
        // At averageTemp = 1.0 the equator reaches maximum temperature.
        // Divide by 2 because averageTemp ranges 0-2 and we want 0-1 output.
        float globalShift = climate.averageTemp / 2f;

        // Blend between the latitude gradient and the global shift.
        // This keeps the pole/equator contrast while honoring the global setting.
        return Mathf.Clamp01(Mathf.Lerp(latitudeTemp * globalShift, globalShift, 0.3f));
    }

    // --- Wind Map ---

    // Similar to GenerateTemperatureMap(), but set up for multiple variables.
    Texture2D GenerateWindMap()
    {
        // Note: Apparently Unity doesn't like RG16 much. It swizzles the color values.
        // Must use RGB24 even though only red and green are used.
        // 0.5 in each channel = no wind. Above 0.5 = positive direction.
        Texture2D tex = new Texture2D(MAP_WIDTH, MAP_HEIGHT, TextureFormat.RGB24, false);
        Color[] pixels = new Color[MAP_WIDTH * MAP_HEIGHT];

        for (int y = 0; y < MAP_HEIGHT; y++)
        {
            for (int x = 0; x < MAP_WIDTH; x++)
            {
                float u = x / (float)(MAP_WIDTH - 1);
                float v = y / (float)(MAP_HEIGHT - 1);

                // UVHelper is a separate script dedicated to generating and reading spherical texture data. See script for details.
                Vector3 spherePoint = UVHelper.UVToSphere(u, v);
                Vector2 wind = CalculateWindVector(spherePoint);

                // Remap from (-1, +1) range to (0, 1) range for texture storage.
                // 0.5 = no wind, 0.0 = full negative, 1.0 = full positive.
                float r = wind.x * 0.5f + 0.5f;
                float g = wind.y * 0.5f + 0.5f;

                pixels[y * MAP_WIDTH + x] = new Color(r, g, 0);
            }
        }

        tex.SetPixels(pixels);
        tex.Apply();
        return tex;
    }

    public static Vector2 CalculateWindVector(Vector3 pointOnUnitSphere)
    {
        float latitude = Mathf.Asin(Mathf.Clamp(pointOnUnitSphere.y, -1f, 1f));

        // Three convection cell bands per hemisphere, producing alternating east/west flow. 
        // Sin(3 * lat) crosses zero at the band boundaries.
        // Negative sign makes trade winds blow eastward in the texture's coordinate frame (west-to-east in the tropics as on Earth).
        float zonalStrength = -Mathf.Sin(3f * latitude);

        // A small northward/southward component at band boundaries adds realism — air doesn't only move east/west.
        // Changing the 0.2f value would increase or decrease the North/South influence.
        // Changing the 6f value would change how many times the wind switches direction across the sphere.
        // This would likely have a noticeable effect on biome distribution and variety at the expense of Earthlike realism.
        float meridionalStrength = Mathf.Sin(6f * latitude) * 0.2f;

        return new Vector2(zonalStrength, meridionalStrength);
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