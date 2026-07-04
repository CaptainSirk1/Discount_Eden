using UnityEngine;

// Generates the planet surface color texture from climate and terrain data.
//
// INPUTS:
//   HeightMap         — elevation data for ocean/land threshold and ice elevation
//   MoistureMap       — blended moisture per cell for land color darkening
//   WindTempMap       — temperature for ice zone determination
//   TerrainSettings   — globalMoisture for ocean threshold, base terrain color
//   ClimateSettings   — averageTemp for global ice threshold
//
// OUTPUTS:
//   texture — 512x256 RGB surface color map.
//             Combines ocean depth, land color with moisture darkening, and ice zones.
//             Fed to the planet material alongside the normal map.
//
// PLANNED ADDITIONS (in order of priority):
//   - Atmospheric tint pass — global color shift based on atmospheric composition.
//       Methane shifts blue-purple, CO2+dust shifts orange, water vapor softens and warms.
//       Applied as a final overlay rather than baked into per-pixel color.
//   - Volcanic zone coloring — dark basalt and sulphur yellow in high blue-channel influence areas.
//       Requires volcanic hotspot data in influence map green channel.
//   - Vegetation layer — per-species opacity pass driven by moisture/temperature/elevation bell curves.
//       Each species has a color, optimal range, and priority rating.
//       Higher priority species paint over lower priority ones at the same location.
//       Designed so new species can be added to a library without touching this class.
//   - Topsoil base color — player-adjustable RGB offset applied before vegetation.
//       Trace elements shift it darker independently of moisture darkening.
//   - Cloud layer — separate pass or shader-driven, not baked into surface texture.
public class PlanetTexture
{
    HeightMap heightMap;
    MoistureMap moistureMap;
    WindTempMap windTempMap;
    TectonicInfluenceMap influenceMap;
    TerrainSettings tSettings;
    ClimateSettings cSettings;

    public Texture2D texture { get; private set; }

    // Base land color — slightly brownish grey for a lifeless planet.
    // Eventually exposed as a player-adjustable topsoil base color.
    static readonly Color baseLandColor = new Color(0.38f, 0.35f, 0.32f);

    // How much moisture darkens the land color.
    // 0 = no darkening, 1 = fully black at maximum moisture.
    const float moistureDarkenStrength = 0.3f;

    // Ocean colors — shallow to deep gradient.
    static readonly Color shallowOcean = new Color(0.20f, 0.35f, 0.45f);
    static readonly Color deepOcean    = new Color(0.04f, 0.08f, 0.18f);

    // Ice color — blue-white, slightly off-white to avoid pure white blowout.
    static readonly Color iceColor = new Color(0.88f, 0.92f, 0.98f);

    // Temperature below which ice forms at sea level.
    // Normalized 0-1 where 0.5 = freezing point.
    const float iceTemperatureThreshold = 0.35f;

    // Elevation above sea level at which ice begins forming regardless of temperature.
    // In heightmap texture space (0-1). Higher elevations get ice even in warm climates.
    const float iceElevationThreshold = 0.72f;

    // How sharply ice blends in at the temperature/elevation thresholds.
    // Higher = crisper ice boundary, lower = more gradual transition.
    const float iceBlendSharpness = 4f;

    public PlanetTexture(
        HeightMap heightMap,
        MoistureMap moistureMap,
        WindTempMap windTempMap,
        TectonicInfluenceMap influenceMap,
        TerrainSettings tSettings,
        ClimateSettings cSettings)
    {
        this.heightMap    = heightMap;
        this.moistureMap  = moistureMap;
        this.windTempMap  = windTempMap;
        this.influenceMap = influenceMap;
        this.tSettings    = tSettings;
        this.cSettings    = cSettings;
    }

    public void Generate()
    {
        texture = GenerateTexture();
    }

    Texture2D GenerateTexture()
    {
        int width  = UVHelper.MAP_WIDTH;
        int height = UVHelper.MAP_HEIGHT;

        Texture2D tex = new Texture2D(width, height, TextureFormat.RGB24, false);
        Color[] pixels = new Color[width * height];

        // Ocean threshold in raw -1/1 elevation space.
        float oceanThreshold = tSettings.globalMoisture * 2f - 1f;

        // Pre-compute trig for sphere point construction.
        float[] cosLong = new float[width];
        float[] sinLong = new float[width];
        for (int x = 0; x < width; x++)
        {
            float u = x / (float)(width - 1);
            float longitude = (u - 0.5f) * 2f * Mathf.PI;
            cosLong[x] = Mathf.Cos(longitude);
            sinLong[x] = Mathf.Sin(longitude);
        }

        float[] cosLat = new float[height];
        float[] sinLat = new float[height];
        for (int y = 0; y < height; y++)
        {
            float v = y / (float)(height - 1);
            float latitude = (0.5f - v) * Mathf.PI;
            cosLat[y] = Mathf.Cos(latitude);
            sinLat[y] = Mathf.Sin(latitude);
        }

        for (int y = 0; y < height; y++)
        {
            float cLat = cosLat[y];
            float sLat = sinLat[y];

            for (int x = 0; x < width; x++)
            {
                int idx = y * width + x;

                // Read heightmap — red channel stores greyscale elevation.
                float elevationTex = heightMap.heightData[idx].r;

                // Convert texture elevation back to -1/1 space.
                // Inverse of (elevation + 1f) / 4f used in HeightMap.
                float elevationRaw = elevationTex * 4f - 1f;

                Color pixelColor;

                if (elevationRaw <= oceanThreshold)
                {
                    // --- Ocean ---
                    // Depth gradient from shallow coastal to deep ocean.
                    float depth = Mathf.Clamp01((oceanThreshold - elevationRaw) / 2f);
                    pixelColor = Color.Lerp(shallowOcean, deepOcean, depth);

                    // Ice over ocean in cold zones.
                    Vector3 spherePoint = new Vector3(cLat * cosLong[x], sLat, cLat * sinLong[x]);
                    float temperature = windTempMap.SampleTemperature(spherePoint);
                    float iceBlend = Mathf.Clamp01((iceTemperatureThreshold - temperature) * iceBlendSharpness);
                    pixelColor = Color.Lerp(pixelColor, iceColor, iceBlend);
                }
                else
                {
                    // --- Land ---

                    // Moisture darkens the base land color.
                    // Wet land is darker — damp rock and soil absorb more light.
                    Vector3 spherePoint = new Vector3(cLat * cosLong[x], sLat, cLat * sinLong[x]);
                    float moisture = UVHelper.GetPixelBilinear(moistureMap.moistureData, 
                        x / (float)(width - 1), 
                        y / (float)(height - 1)).r;

                    Color landColor = baseLandColor * (1f - moisture * moistureDarkenStrength);
                    landColor.a = 1f;

                    float temperature = windTempMap.SampleTemperature(spherePoint);

                    // Snow line elevation drops as temperature drops.
                    // Hot planet = snow only at peaks. Cold planet = snow at sea level.
                    float snowLineElevation = Mathf.Lerp(0f, iceElevationThreshold,
                        Mathf.Clamp01(temperature / iceTemperatureThreshold));

                    float elevationAboveSnowLine = Mathf.Clamp01(
                        (elevationTex - snowLineElevation) / (1f - snowLineElevation + 0.001f));
                    float iceBlend = Mathf.Clamp01(elevationAboveSnowLine * iceBlendSharpness);

                    pixelColor = Color.Lerp(landColor, iceColor, iceBlend);

                    // PLACEHOLDER: Vegetation layer goes here.
                    // Each species will contribute opacity based on its bell curve score
                    // at this pixel's moisture/temperature/elevation values.
                    // Higher priority species are applied last, painting over lower priority ones.

                    // PLACEHOLDER: Volcanic zone coloring goes here.
                    // High blue-channel influence map values near hotspot features
                    // will shift land color toward dark basalt and sulphur yellow.
                }

                pixels[idx] = pixelColor;
            }
        }

        tex.SetPixels(pixels);
        tex.Apply();
        return tex;
    }
}