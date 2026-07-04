using UnityEngine;

// Generates a normal map from the planet heightmap for surface lighting.
// Encodes per-pixel surface orientation so the shader can calculate correct
// shadow direction regardless of sun position, without changing the texture itself.
//
// INPUTS:
//   HeightMap — reads heightData pixel array directly for gradient calculation.
//
// OUTPUTS:
//   normalMap — 512x256 RGB texture encoding surface normals in tangent space.
//               R = X component, G = Y component, B = Z component, each remapped 0-1.
//               Fed directly to URP Lit shader's normal map input slot.
//
// LIFETIME:
//   Generated once at planet creation alongside other maps.
//   Replaced per-chunk by mesh normals once terrain deformation begins at higher LODs.
//   At that point this map is simply no longer sampled — no code changes needed.
public class NormalMap
{
    HeightMap heightMap;

    // Controls how pronounced the normal effect appears.
    // Higher values exaggerate surface relief, useful since the heightmap
    // is compressed into a narrow value range at planet scale.
    float normalStrength;

    public Texture2D normal { get; private set; }

    public NormalMap(HeightMap heightMap, float normalStrength = 4f)
    {
        this.heightMap = heightMap;
        this.normalStrength = normalStrength;
    }

    public void Generate()
    {
        normal = GenerateNormalMap();
    }

    Texture2D GenerateNormalMap()
    {
        int width  = UVHelper.MAP_WIDTH;
        int height = UVHelper.MAP_HEIGHT;

        Texture2D tex = new Texture2D(width, height, TextureFormat.RGB24, false);
        Color[] pixels = new Color[width * height];

        for (int y = 0; y < height; y++)
        {
            // Latitude of this row, used to correct for equirectangular stretching.
            // Near the poles, horizontal pixels represent much less actual surface distance,
            // which would make poles appear artificially steep without this correction.
            float v = y / (float)(height - 1);
            float latitude = (0.5f - v) * Mathf.PI;
            float latitudeScale = 1f / Mathf.Max(Mathf.Cos(latitude), 0.001f);

            for (int x = 0; x < width; x++)
            {
                // Sample height at neighboring pixels, wrapping horizontally at the seam.
                float heightL = SampleHeight((x - 1 + width) % width, y);
                float heightR = SampleHeight((x + 1) % width,         y);
                float heightD = SampleHeight(x, Mathf.Max(y - 1, 0));
                float heightU = SampleHeight(x, Mathf.Min(y + 1, height - 1));

                // Gradient in x and y directions.
                // X gradient scaled by latitude to correct pole distortion.
                float dx = (heightL - heightR) * latitudeScale * normalStrength;
                float dy = (heightD - heightU) * normalStrength;

                // Construct and normalize the surface normal.
                // Z component (forward/up out of surface) keeps normals from
                // flipping to backface on steep slopes.
                Vector3 normal = new Vector3(dx, dy, 1f).normalized;

                // Remap from -1/+1 to 0/1 for texture storage.
                // 0.5 in each channel = flat surface pointing straight up.
                pixels[y * width + x] = new Color(
                    normal.x * 0.5f + 0.5f,
                    normal.y * 0.5f + 0.5f,
                    normal.z * 0.5f + 0.5f
                );
            }
        }

        tex.SetPixels(pixels);
        tex.Apply();
        return tex;
    }

    // Reads the red channel of the heightmap at pixel coordinates.
    // Red channel stores the greyscale height value — all three channels are identical.
    float SampleHeight(int x, int y)
    {
        return heightMap.heightData[y * UVHelper.MAP_WIDTH + x].r;
    }
}