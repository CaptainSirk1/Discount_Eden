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
        Color[] pixels = new Color[UVHelper.MAP_WIDTH * UVHelper.MAP_HEIGHT];

        // 1.5x average spacing catches all adjacent cells without reaching distant ones.
        float neighborRadius = tectonics.averageCellSpacing * 1.5f;

        for (int y = 0; y < UVHelper.MAP_HEIGHT; y++)
        {
            for (int x = 0; x < UVHelper.MAP_WIDTH; x++)
            {
                float u = x / (float)(UVHelper.MAP_WIDTH - 1);
                float v = y / (float)(UVHelper.MAP_HEIGHT - 1);

                float convergentStrength = 0f;
                float divergentStrength = 0f;

                Vector3 spherePoint = UVHelper.UVToSphere(u, v);
                Vector3 warpedPoint = UVHelper.WarpSpherePoint(spherePoint, settings.warpNoise, settings.worldSeed);

                int cellIdx = -1;
                float nearestDist = float.MaxValue;
                
                foreach (int i in tectonics.GetNearbyCells(warpedPoint))
                {
                    float dist = Vector3.SqrMagnitude(warpedPoint - tectonics.cells[i].position);
                    
                    // Track nearest cell
                    if (dist < nearestDist)
                    {
                        nearestDist = dist;
                        cellIdx = i;
                    }
                }

                if (cellIdx == -1) continue;
                TectonicCell cell = tectonics.cells[cellIdx];

                // Now evaluate neighbors in the same bucket neighborhood
                foreach (int neighborIdx in tectonics.GetNearbyCells(warpedPoint))
                {
                    if (neighborIdx == cellIdx) continue;
                    TectonicCell neighbor = tectonics.cells[neighborIdx];

                    float cellToNeighbor = Vector3.Distance(cell.position, neighbor.position);

                    // Vector pointing from cell toward neighbor.
                    Vector3 toNeighbor = (neighbor.position - cell.position).normalized;

                    // How much is each cell moving toward the other?
                    // +1 = directly toward, -1 = directly away.
                    float cellApproach     = Vector3.Dot(cell.movementDirection, toNeighbor);
                    float neighborApproach = Vector3.Dot(neighbor.movementDirection, -toNeighbor);

                    // Positive = both approaching = convergent.
                    // Negative = both retreating = divergent.
                    float boundarySignal = cellApproach + neighborApproach;

                    // Scale by both movement strengths so weak plates produce weaker boundaries.
                    float boundaryStrength = Mathf.Abs(boundarySignal)
                                          * cell.movementStrength
                                          * neighbor.movementStrength;

                    // How close is this pixel to the voronoi boundary between cell and neighbor?
                    // Boundary is where distToCell == distToNeighbor. Falls off to 0 at cell centers.
                    float distToCell     = Vector3.Distance(spherePoint, cell.position);
                    float distToNeighbor = Vector3.Distance(spherePoint, neighbor.position);
                    float boundaryProximity = Mathf.Clamp01(1f - (Mathf.Abs(distToCell - distToNeighbor) / cellToNeighbor));

                    // Collect seeds from pixels sitting directly on strong divergent boundaries.
                    // boundaryProximity > 0.7 filters to the voronoi edge itself, excluding the broader halo.
                    // toNeighbor is stored so GenerateFaultSplines can enforce spatial separation.
                    if (boundarySignal < -0.3f && boundaryStrength > 0.3f && boundaryProximity > 0.7f)
                        seedCandidates.Add((spherePoint, toNeighbor, boundaryStrength));

                    // Sharpen falloff so influence concentrates at boundaries rather than spreading evenly.
                    boundaryProximity *= boundaryProximity;

                    float contribution = boundaryStrength * boundaryProximity;

                    if (boundarySignal > 0)
                        convergentStrength = LogarithmicCombine(convergentStrength, contribution);
                    else if (boundarySignal < 0)
                        divergentStrength = LogarithmicCombine(divergentStrength, contribution);
                }

                // Red = convergent, Green = reserved for volcanic hotspots, Blue = divergent.
                pixels[y * UVHelper.MAP_WIDTH + x] = new Color(convergentStrength, 0f, divergentStrength);
            }
        }

        pixels = BlurTexture(pixels, UVHelper.MAP_WIDTH, UVHelper.MAP_HEIGHT, 5);
        tex.SetPixels(pixels);
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

    // --- Fault Splines ---

    // Builds a point cloud from divergent boundary candidates with minimum spacing enforced,
    // then connects points into splines using a nearest-neighbor walk.
    // A new spline starts whenever no unused point is found within searchRadius,
    // naturally terminating at boundary edges and gaps.
    void GenerateFaultSplines(List<(Vector3 origin, Vector3 toNeighbor, float strength)> candidates)
    {
        float minSpacing   = tectonics.averageCellSpacing * 0.5f;
        float searchRadius = tectonics.averageCellSpacing;

        // Build fault point cloud from candidates, enforcing minimum spacing.
        List<Vector3> faultPoints = new List<Vector3>();
        foreach (var c in candidates)
        {
            bool tooClose = false;
            foreach (var existing in faultPoints)
            {
                if (Vector3.Distance(c.origin, existing) < minSpacing)
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
                float bestDist = searchRadius;

                for (int i = 0; i < faultPoints.Count; i++)
                {
                    if (used.Contains(i)) continue;
                    float dist = Vector3.Distance(last, faultPoints[i]);
                    if (dist < bestDist) { bestDist = dist; bestIdx = i; }
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
        return new FaultSpline
        {
            origin        = points[0],
            strength      = tectonicInfluence.GetPixelBilinear(originUV.x, originUV.y).b,
            length        = totalLength,
            controlPoints = points.ToArray(),
            tangents      = tangents.ToArray()
        };
    }
}