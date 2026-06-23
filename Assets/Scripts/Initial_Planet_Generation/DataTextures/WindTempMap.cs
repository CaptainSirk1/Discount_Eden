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
        // 1. Pre-compute trigonometric values for all X and Y coordinates.
        // This moves expensive Sin/Cos calls out of the nested pixel loop.
        float[] cosLong = new float[MAP_WIDTH];
        float[] sinLong = new float[MAP_WIDTH];
        for (int x = 0; x < MAP_WIDTH; x++)
        {
            float u = x / (float)(MAP_WIDTH - 1);
            float longitude = (u - 0.5f) * 2f * Mathf.PI;
            cosLong[x] = Mathf.Cos(longitude);
            sinLong[x] = Mathf.Sin(longitude);
        }

        float[] cosLat = new float[MAP_HEIGHT];
        float[] sinLat = new float[MAP_HEIGHT];
        float[] latitudes = new float[MAP_HEIGHT];
        for (int y = 0; y < MAP_HEIGHT; y++)
        {
            float v = y / (float)(MAP_HEIGHT - 1);
            latitudes[y] = (0.5f - v) * Mathf.PI;
            cosLat[y] = Mathf.Cos(latitudes[y]);
            sinLat[y] = Mathf.Sin(latitudes[y]);
        }

        temperatureMap = new Texture2D(MAP_WIDTH, MAP_HEIGHT, TextureFormat.RGB24, false);
        windMap = new Texture2D(MAP_WIDTH, MAP_HEIGHT, TextureFormat.RGB24, false);
        Color[] tempPixels = new Color[MAP_WIDTH * MAP_HEIGHT];
        Color[] windPixels = new Color[MAP_WIDTH * MAP_HEIGHT];

        // 2. Combined single-pass loop over all pixels
        for (int y = 0; y < MAP_HEIGHT; y++)
        {
            float latitude = latitudes[y];
            float cLat = cosLat[y];
            float sLat = sinLat[y];

            // Re-use latitude-based values for temperature and wind
            float latitudeTemp = cLat; // Cos(latitude)
            float globalShift = climate.averageTemp / 2f;
            float tempValue = Mathf.Clamp01(Mathf.Lerp(latitudeTemp * globalShift, globalShift, 0.3f));
            Color tempColor = new Color(tempValue, 0, 1f - tempValue);

            // Wind zonal/meridional flow also only depends on latitude
            float zonalStrength = -Mathf.Sin(3f * latitude);
            float meridionalStrength = Mathf.Sin(6f * latitude) * 0.2f;
            float windR = zonalStrength * 0.5f + 0.5f;
            float windG = meridionalStrength * 0.5f + 0.5f;
            Color windColor = new Color(windR, windG, 0);

            for (int x = 0; x < MAP_WIDTH; x++)
            {
                int index = y * MAP_WIDTH + x;
                tempPixels[index] = tempColor;
                windPixels[index] = windColor;
            }
        }

        temperatureMap.SetPixels(tempPixels);
        temperatureMap.Apply();
        windMap.SetPixels(windPixels);
        windMap.Apply();
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