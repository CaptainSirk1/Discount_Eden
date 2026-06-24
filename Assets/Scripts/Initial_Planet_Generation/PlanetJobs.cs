using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace PlanetGeneration
{
    [BurstCompile]
    public struct ConstructMeshJob : IJobParallelFor
    {
        public int resolution;
        public float3 localForward;
        public float3 axisA;
        public float3 axisB;
        public float planetRadius;

        [WriteOnly] public NativeArray<float3> vertices;
        [WriteOnly] public NativeArray<float2> uvs;

        public void Execute(int i)
        {
            int x = i % resolution;
            int y = i / resolution;

            float2 percent = new float2(x, y) / (resolution - 1);
            float3 pointOnUnitCube = localForward + (percent.x - 0.5f) * 2 * axisA + (percent.y - 0.5f) * 2 * axisB;
            float3 normalizedPoint = math.normalize(pointOnUnitCube);

            vertices[i] = normalizedPoint * planetRadius;
            
            // UVHelper.SphereToUV inlined
            float u = 0.5f + math.atan2(normalizedPoint.z, normalizedPoint.x) / (2f * math.PI);
            float v = 0.5f - math.asin(math.clamp(normalizedPoint.y, -1f, 1f)) / math.PI;
            uvs[i] = new float2(u, v);
        }
    }

    [BurstCompile]
    public struct WindTempMapJob : IJobParallelFor
    {
        public int width;
        public int height;
        public float averageTemp;

        [WriteOnly] public NativeArray<Color32> tempPixels;
        [WriteOnly] public NativeArray<Color32> windPixels;

        public void Execute(int i)
        {
            int x = i % width;
            int y = i / width;

            float v = y / (float)(height - 1);
            float latitude = (0.5f - v) * math.PI;
            float cLat = math.cos(latitude);
            // float sLat = math.sin(latitude); // Not used in current logic but available

            float globalShift = averageTemp / 2f;
            float tempValue = math.clamp(math.lerp(cLat * globalShift, globalShift, 0.3f), 0f, 1f);
            
            // tempValue (Red), 0 (Green), 1-tempValue (Blue)
            tempPixels[i] = new Color32((byte)(tempValue * 255), 0, (byte)((1f - tempValue) * 255), 255);

            float zonalStrength = -math.sin(3f * latitude);
            float meridionalStrength = math.sin(6f * latitude) * 0.2f;
            float windR = zonalStrength * 0.5f + 0.5f;
            float windG = meridionalStrength * 0.5f + 0.5f;
            
            windPixels[i] = new Color32((byte)(windR * 255), (byte)(windG * 255), 0, 255);
        }
    }

    public struct NoiseData
    {
        public int noiseType; // 0 = FBm, 1 = Ridged
        public int octaves;
        public float scale;
        public float lacunarity;
        public float persistence;
        public float strength;
        public int seed;

        public static NoiseData FromSettings(NoiseSettings settings, int seed)
        {
            if (settings == null) return new NoiseData { strength = 0 };
            return new NoiseData
            {
                noiseType = (int)settings.noiseType,
                octaves = settings.octaves,
                scale = settings.scale,
                lacunarity = settings.lacunarity,
                persistence = settings.persistence,
                strength = settings.strength,
                seed = seed
            };
        }

        public float Sample(float3 point)
        {
            if (strength <= 0) return 0;
            
            float value;
            if (noiseType == 1) // Ridged
                value = SampleRidged(point);
            else
                value = SampleFBm(point);

            return value * strength;
        }

        private float SampleFBm(float3 point)
        {
            float3 seedOffset = new float3(
                seed * 0.3183f,
                seed * 0.1547f,
                seed * 0.2718f
            );

            float value = 0f;
            float amplitude = 1f;
            float frequency = scale;
            float maxValue = 0f;

            for (int i = 0; i < octaves; i++)
            {
                value += noise.snoise((point + seedOffset) * frequency) * amplitude;
                maxValue += amplitude;
                amplitude *= persistence;
                frequency *= lacunarity;
            }

            return maxValue > 0 ? value / maxValue : 0f;
        }

        private float SampleRidged(float3 point)
        {
            float3 seedOffset = new float3(
                seed * 0.3183f,
                seed * 0.1547f,
                seed * 0.2718f
            );

            float value = 0f;
            float amplitude = 1f;
            float frequency = scale;
            float maxValue = 0f;

            for (int i = 0; i < octaves; i++)
            {
                float n = noise.snoise((point + seedOffset) * frequency);
                n = 1f - math.abs(n);
                n *= n;

                value += n * amplitude;
                maxValue += amplitude;
                amplitude *= persistence;
                frequency *= lacunarity;
            }

            return maxValue > 0 ? value / maxValue : 0f;
        }

        public float3 SampleWarp(float3 point)
        {
            if (strength <= 0) return float3.zero;

            float3 offset;
            offset.x = SampleFBm(point + new float3(1.3f, 2.1f, 0.7f));
            offset.y = SampleFBm(point + new float3(4.7f, 0.9f, 3.2f));
            offset.z = SampleFBm(point + new float3(8.3f, 5.6f, 1.8f));

            return offset * strength;
        }
    }

    [BurstCompile]
    public struct HeightMapJob : IJobParallelFor
    {
        public int width;
        public int height;
        public int worldSeed;
        public NoiseData warpNoise;
        public NoiseData terrainNoise;
        public NoiseData mountainNoise;
        
        [ReadOnly] public NativeArray<TectonicCell> cells;
        // Spatial data for cell lookups
        [ReadOnly] public NativeArray<int> bucketOffsets;
        [ReadOnly] public NativeArray<int> bucketIndices;
        public int bucketSize;

        [ReadOnly] public NativeArray<Color32> influenceData;
        
        // Fault spline data simplified for job
        [ReadOnly] public NativeArray<float3> faultPoints;
        [ReadOnly] public NativeArray<float> faultStrengths;

        [WriteOnly] public NativeArray<Color32> heightData;

        public void Execute(int i)
        {
            int x = i % width;
            int y = i / width;

            float u = x / (float)(width - 1);
            float v = y / (float)(height - 1);

            float longitude = (u - 0.5f) * 2f * math.PI;
            float latitude = (0.5f - v) * math.PI;

            float3 spherePoint = new float3(
                math.cos(latitude) * math.cos(longitude),
                math.sin(latitude),
                math.cos(latitude) * math.sin(longitude)
            );

            float3 warpedPoint = spherePoint;
            if (warpNoise.strength > 0)
            {
                float3 offset = warpNoise.SampleWarp(spherePoint);
                warpedPoint = math.normalize(spherePoint + offset);
            }

            // Fault depression
            float faultDepression = 0;
            float faultRadius = 0.04f;
            float faultDepth = 0.2f;

            // Simplified fault point search
            float minDSq = float.MaxValue;
            float fStrength = 0;
            for(int j = 0; j < faultPoints.Length; j++)
            {
                float dSq = math.distancesq(spherePoint, faultPoints[j]);
                if (dSq < minDSq)
                {
                    minDSq = dSq;
                    fStrength = faultStrengths[j];
                }
            }
            if (minDSq < faultRadius * faultRadius)
            {
                float dist = math.sqrt(minDSq);
                float falloff = math.clamp(1f - dist / faultRadius, 0f, 1f);
                falloff *= falloff;
                faultDepression = falloff * fStrength * faultDepth;
            }

            // Blended elevation
            float elevation = GetBlendedElevation(warpedPoint);
            float baseColor = (elevation + 1f) / 4f;

            float terrainColor = terrainNoise.Sample(spherePoint) * 0.1f;
            
            Color32 infl = influenceData[i];
            float convergent = infl.r / 255f;
            float divergent = infl.b / 255f;

            float noiseOpacity = 0.05f + convergent * 0.95f + divergent * 0.25f;
            float mountainColor = mountainNoise.Sample(spherePoint) * noiseOpacity;

            float finalColor = math.clamp(baseColor + terrainColor + mountainColor - faultDepression, 0f, 1f);
            byte b = (byte)(finalColor * 255);
            heightData[i] = new Color32(b, b, b, 255);
        }

        private float GetBlendedElevation(float3 point)
        {
            // Spatial lookup
            float u = 0.5f + math.atan2(point.z, point.x) / (2f * math.PI);
            float v = 0.5f - math.asin(math.clamp(point.y, -1f, 1f)) / math.PI;
            
            int bx = math.clamp((int)(u * bucketSize), 0, bucketSize - 1);
            int by = math.clamp((int)(v * bucketSize), 0, bucketSize - 1);

            float totalWeight = 0f;
            float weightedElevation = 0f;
            float averageCellSpacing = math.sqrt(4f * math.PI / cells.Length);
            float blendRadiusSq = (averageCellSpacing * 2f) * (averageCellSpacing * 2f);

            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    int nx = (bx + dx + bucketSize) % bucketSize;
                    int ny = by + dy;
                    if (ny < 0 || ny >= bucketSize) continue;

                    int bucketIdx = ny * bucketSize + nx;
                    int start = bucketOffsets[bucketIdx];
                    int end = (bucketIdx + 1 < bucketOffsets.Length) ? bucketOffsets[bucketIdx + 1] : bucketIndices.Length;

                    for (int j = start; j < end; j++)
                    {
                        int cellIdx = bucketIndices[j];
                        float dSq = math.distancesq(point, (float3)cells[cellIdx].position);
                        if (dSq < blendRadiusSq)
                        {
                            float weight = 1f / (dSq + 0.0001f);
                            weightedElevation += cells[cellIdx].baseElevation * weight;
                            totalWeight += weight;
                        }
                    }
                }
            }

            if (totalWeight <= 0f)
            {
                // Fallback to nearest cell if radius search fails
                float nearestDistSq = float.MaxValue;
                float nearestElev = 0;
                for (int j = 0; j < cells.Length; j++)
                {
                    float dSq = math.distancesq(point, (float3)cells[j].position);
                    if (dSq < nearestDistSq)
                    {
                        nearestDistSq = dSq;
                        nearestElev = cells[j].baseElevation;
                    }
                }
                return nearestElev;
            }

            return weightedElevation / totalWeight;
        }
    }
}
