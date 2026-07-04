using UnityEngine;
using System.Collections.Generic;

// Generates the tectonic influence map and transform fault splines from tectonic cell data.
//
// OUTPUTS:
//   tectonicInfluence — 512x256 RGB texture. Red = convergent boundary strength (mountains),
//                       Blue = divergent boundary strength (rifts). Green reserved for volcanic hotspots.
//                       Persists across all LODs as a blueprint for noise application and strength scaling.
//
//   faultSplines      — Fault lines traced along divergent boundaries using a point-cloud nearest-neighbor walk.
//                       Sparse control points are refined and noise-displaced before storage.
//                       Higher LODs read these to generate rift valley geometry and seed perpendicular ridges.
//
//   splineGrid        — Spatial index over faultSplines for cheap proximity queries.
//                       HeightMap and higher LOD systems query this instead of iterating faultSplines directly.

// HeightMap samples tectonicInfluence via GetPixelBilinear for convergent and divergent channel values.
// HeightMap queries splineGrid.GetNearestPoint() for distance/strength to apply fault depression falloff.
public class TectonicInfluenceMap
{
    GenerateTectonics tectonics;
    TerrainSettings settings;

    public Texture2D tectonicInfluence { get; private set; }
    public Color[] influenceData { get; private set; } // Optimization #4
    public List<FaultSpline> faultSplines { get; private set; }
    public FaultSplineGrid splineGrid { get; private set; }

    public TectonicInfluenceMap(GenerateTectonics tectonics, TerrainSettings settings)
    {
        this.tectonics = tectonics;
        this.settings = settings;
    }

    // --- Generation ---

    public void Generate()
    {
        faultSplines = new List<FaultSpline>();
        var seedCandidates = new List<(Vector3 origin, Vector3 toNeighbor, float strength)>();

        tectonicInfluence = GenerateTectonicInfluence(seedCandidates);
        GenerateFaultSplines(seedCandidates);

        foreach (FaultSpline spline in faultSplines)
            spline.Refine(Mathf.Max(8, Mathf.RoundToInt(spline.length * 200f)), settings.worldSeed);

        splineGrid = new FaultSplineGrid(faultSplines);
    }

    // --- Influence Map ---

    // Iterates every pixel on the map, evaluating tectonic boundary relationships
    // for the nearest cell and all its neighbors. Convergent and divergent signals
    // are written to the red and blue channels respectively.
    // Pixels sitting directly on strong divergent boundaries are collected as fault spline seed candidates,
    // storing toNeighbor so splines know which direction runs along the boundary.
    Texture2D GenerateTectonicInfluence(List<(Vector3 origin, Vector3 toNeighbor, float strength)> seedCandidates)
    {
        Texture2D tex = new Texture2D(UVHelper.MAP_WIDTH, UVHelper.MAP_HEIGHT, TextureFormat.RGB24, false);
        influenceData = new Color[UVHelper.MAP_WIDTH * UVHelper.MAP_HEIGHT];

        // 1.5x average spacing catches all adjacent cells without reaching distant ones.
        float neighborRadius = tectonics.averageCellSpacing * 1.5f;

        // Pre-compute trigonometric values
        float[] cosLong = new float[UVHelper.MAP_WIDTH];
        float[] sinLong = new float[UVHelper.MAP_WIDTH];
        for (int x = 0; x < UVHelper.MAP_WIDTH; x++)
        {
            float u = x / (float)(UVHelper.MAP_WIDTH - 1);
            float longitude = (u - 0.5f) * 2f * Mathf.PI;
            cosLong[x] = Mathf.Cos(longitude);
            sinLong[x] = Mathf.Sin(longitude);
        }

        float[] cosLat = new float[UVHelper.MAP_HEIGHT];
        float[] sinLat = new float[UVHelper.MAP_HEIGHT];
        for (int y = 0; y < UVHelper.MAP_HEIGHT; y++)
        {
            float v = y / (float)(UVHelper.MAP_HEIGHT - 1);
            float latitude = (0.5f - v) * Mathf.PI;
            cosLat[y] = Mathf.Cos(latitude);
            sinLat[y] = Mathf.Sin(latitude);
        }

        List<int> nearbyIndices = new List<int>();
        for (int y = 0; y < UVHelper.MAP_HEIGHT; y++)
        {
            float cLat = cosLat[y];
            float sLat = sinLat[y];

            for (int x = 0; x < UVHelper.MAP_WIDTH; x++)
            {
                float convergentStrength = 0f;
                float divergentStrength = 0f;

                // Fast sphere point construction
                Vector3 spherePoint = new Vector3(cLat * cosLong[x], sLat, cLat * sinLong[x]);
                Vector3 warpedPoint = UVHelper.WarpSpherePoint(spherePoint, settings.warpNoise, settings.worldSeed);

                int cellIdx = -1;
                float nearestDistSq = float.MaxValue;
                
                tectonics.GetNearbyCells(warpedPoint, nearbyIndices);

                foreach (int i in nearbyIndices)
                {
                    float distSq = Vector3.SqrMagnitude(warpedPoint - tectonics.cells[i].position);
                    
                    // Track nearest cell
                    if (distSq < nearestDistSq)
                    {
                        nearestDistSq = distSq;
                        cellIdx = i;
                    }
                }

                if (cellIdx == -1) continue;
                TectonicCell cell = tectonics.cells[cellIdx];

                // Now evaluate neighbors in the same bucket neighborhood
                foreach (int neighborIdx in nearbyIndices)
                {
                    if (neighborIdx == cellIdx) continue;
                    TectonicCell neighbor = tectonics.cells[neighborIdx];

                    // Optimization #3: Cache square distance and only sqrt once
                    float distSqBetweenCells = Vector3.SqrMagnitude(cell.position - neighbor.position);

                    // Skip neighbors outside the radius — same filter the old GetCellsInRadius applied
                    if (distSqBetweenCells > neighborRadius * neighborRadius) continue;
                    float cellToNeighbor = Mathf.Sqrt(distSqBetweenCells);
                    

                    // Vector pointing from cell toward neighbor.
                    Vector3 toNeighbor = (neighbor.position - cell.position) / cellToNeighbor;

                    // How much is each cell moving toward the other?
                    float cellApproach     = Vector3.Dot(cell.movementDirection, toNeighbor);
                    float neighborApproach = Vector3.Dot(neighbor.movementDirection, -toNeighbor);

                    // Positive = both approaching = convergent.
                    float boundarySignal = cellApproach + neighborApproach;

                    float boundaryStrength = Mathf.Abs(boundarySignal)
                                          * cell.movementStrength
                                          * neighbor.movementStrength;

                    // Boundary proximity check - still needs linear distance for the specific 1D falloff math
                    float distToCellSq     = Vector3.SqrMagnitude(spherePoint - cell.position);
                    float distToNeighborSq = Vector3.SqrMagnitude(spherePoint - neighbor.position);
                    float distToCell       = Mathf.Sqrt(distToCellSq);
                    float distToNeighbor   = Mathf.Sqrt(distToNeighborSq);
                    
                    float boundaryProximity = Mathf.Clamp01(1f - (Mathf.Abs(distToCell - distToNeighbor) / cellToNeighbor));

                    if (boundarySignal < -0.3f && boundaryStrength > 0.3f && boundaryProximity > 0.7f)
                        seedCandidates.Add((spherePoint, toNeighbor, boundaryStrength));

                    boundaryProximity *= boundaryProximity;
                    float contribution = boundaryStrength * boundaryProximity;

                    if (boundarySignal > 0)
                        convergentStrength = LogarithmicCombine(convergentStrength, contribution);
                    else if (boundarySignal < 0)
                        divergentStrength = LogarithmicCombine(divergentStrength, contribution);
                }

                influenceData[y * UVHelper.MAP_WIDTH + x] = new Color(convergentStrength, 0f, divergentStrength);
            }
        }

        influenceData = BlurTexture(influenceData, UVHelper.MAP_WIDTH, UVHelper.MAP_HEIGHT, 5);
        tex.SetPixels(influenceData);
        tex.Apply();
        return tex;
    }

    // Combines two influence values with logarithmic falloff above 0.5.
    // Prevents any single boundary from dominating — reaching higher values
    // requires compounding influence from multiple boundaries.
    float LogarithmicCombine(float existing, float addition)
    {
        float combined = existing + addition - (existing * addition);
        if (combined > 0.5f)
        {
            float excess = combined - 0.5f;
            combined = 0.5f + Mathf.Log(1f + excess * 10f) / Mathf.Log(11f) * 0.5f;
        }
        return Mathf.Clamp01(combined);
    }

    // Separable Gaussian blur — horizontal pass then vertical pass.
    // Applied after pixel generation so boundary influence bleeds naturally into surrounding terrain.
    Color[] BlurTexture(Color[] pixels, int width, int height, int radius)
    {
        Color[] temp   = new Color[pixels.Length];
        Color[] result = new Color[pixels.Length];

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                Color sum = Color.black;
                int count = 0;
                for (int i = -radius; i <= radius; i++)
                {
                    int sampleX = Mathf.Clamp(x + i, 0, width - 1);
                    sum += pixels[y * width + sampleX];
                    count++;
                }
                temp[y * width + x] = sum / count;
            }
        }

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                Color sum = Color.black;
                int count = 0;
                for (int i = -radius; i <= radius; i++)
                {
                    int sampleY = Mathf.Clamp(y + i, 0, height - 1);
                    sum += temp[sampleY * width + x];
                    count++;
                }
                result[y * width + x] = sum / count;
            }
        }

        return result;
    }

    // --- Job Support ---
    public void GetJobData(out Unity.Collections.NativeArray<Vector3> points, out Unity.Collections.NativeArray<float> strengths)
    {
        List<Vector3> pList = new List<Vector3>();
        List<float> sList = new List<float>();

        if (faultSplines != null)
        {
            foreach (var spline in faultSplines)
            {
                foreach (var cp in spline.controlPoints)
                {
                    pList.Add(cp);
                    sList.Add(spline.strength);
                }
            }
        }

        points = new Unity.Collections.NativeArray<Vector3>(pList.ToArray(), Unity.Collections.Allocator.TempJob);
        strengths = new Unity.Collections.NativeArray<float>(sList.ToArray(), Unity.Collections.Allocator.TempJob);
    }

    // --- Fault Splines ---

    // Builds a point cloud from divergent boundary candidates with minimum spacing enforced,
    // then connects points into splines using a nearest-neighbor walk.
    // A new spline starts whenever no unused point is found within searchRadius,
    // naturally terminating at boundary edges and gaps.
    void GenerateFaultSplines(List<(Vector3 origin, Vector3 toNeighbor, float strength)> candidates)
    {
        float minSpacing   = tectonics.averageCellSpacing * 0.5f;
        float minSpacingSq = minSpacing * minSpacing;
        float searchRadius = tectonics.averageCellSpacing;
        float searchRadiusSq = searchRadius * searchRadius;

        // Build fault point cloud from candidates, enforcing minimum spacing.
        List<Vector3> faultPoints = new List<Vector3>();
        foreach (var c in candidates)
        {
            bool tooClose = false;
            foreach (var existing in faultPoints)
            {
                if (Vector3.SqrMagnitude(c.origin - existing) < minSpacingSq)
                { tooClose = true; break; }
            }
            if (!tooClose)
                faultPoints.Add(c.origin);
        }

        // Nearest-neighbor walk — always connect to the closest unused point within searchRadius.
        // Produces splines that follow boundary curvature rather than scan-order artifacts.
        HashSet<int> used = new HashSet<int>();

        while (true)
        {
            // Find the next unused starting point.
            int startIdx = -1;
            for (int i = 0; i < faultPoints.Count; i++)
                if (!used.Contains(i)) { startIdx = i; break; }
            if (startIdx == -1) break;

            List<Vector3> current = new List<Vector3>();
            current.Add(faultPoints[startIdx]);
            used.Add(startIdx);

            while (true)
            {
                Vector3 last = current[current.Count - 1];
                int bestIdx = -1;
                float bestDistSq = searchRadiusSq;

                for (int i = 0; i < faultPoints.Count; i++)
                {
                    if (used.Contains(i)) continue;
                    float distSq = Vector3.SqrMagnitude(last - faultPoints[i]);
                    if (distSq < bestDistSq) { bestDistSq = distSq; bestIdx = i; }
                }

                if (bestIdx == -1) break;
                used.Add(bestIdx);
                current.Add(faultPoints[bestIdx]);
            }

            if (current.Count >= 2)
                faultSplines.Add(BuildSpline(current));
        }
    }

    // Assembles a point list into a FaultSpline, computing tangents and total length.
    // Applies a smoothing pass before computing tangents to reduce jaggedness
    // from the nearest-neighbor walk on a discrete point cloud.
    FaultSpline BuildSpline(List<Vector3> points)
    {
        // Smooth points by averaging each with its neighbors.
        for (int i = 1; i < points.Count - 1; i++)
            points[i] = ((points[i - 1] + points[i] + points[i + 1]) / 3f).normalized;

        List<Vector3> tangents = new List<Vector3>();
        for (int i = 0; i < points.Count - 1; i++)
            tangents.Add((points[i + 1] - points[i]).normalized);
        tangents.Add(tangents[tangents.Count - 1]);

        float totalLength = 0f;
        for (int i = 1; i < points.Count; i++)
            totalLength += Vector3.Distance(points[i - 1], points[i]);

        Vector2 originUV = UVHelper.SphereToUV(points[0]);
        
        // Optimization #4: Use raw array access instead of GetPixelBilinear
        float strength = UVHelper.GetPixelBilinear(influenceData, originUV.x, originUV.y).b;

        return new FaultSpline
        {
            origin        = points[0],
            strength      = strength,
            length        = totalLength,
            controlPoints = points.ToArray(),
            tangents      = tangents.ToArray()
        };
    }
}