using UnityEngine;
using System.Collections.Generic;

// Generates a fibonacci sphere of TectonicCells and assigns terrain data to each.
// Continent center cells receive randomized data.
// All other cells receive data weighted toward similarity with their nearest cell.
// This naturally creates large continental plates with potentially dramatic elevation changes between them.
public class GenerateTectonics
{
    TerrainSettings settings;
    System.Random rng;

    public TectonicCell[] cells { get; private set; }

    // Average distance between neighboring cells on the unit sphere.
    // Derived from cell count — more cells means smaller spacing.
    // Used by GetCellInDirection to step one cell width in a given direction.
    public readonly float averageCellSpacing;

    // --- Spatial Bucketing ---
    // Divides UV space into a grid so neighbor searches only check nearby cells
    // instead of the entire array. Built once after generation.
    const int BUCKET_SIZE = 8;
    List<int>[,] buckets;

    public GenerateTectonics(TerrainSettings settings)
    {
        this.settings = settings;
        rng = new System.Random(settings.worldSeed);
        averageCellSpacing = Mathf.Sqrt(4f * Mathf.PI / settings.cellCount);
    }

    // --- Generation ---

    public void Generate()
    {
        cells = GenerateFibonacciSphere();
        JitterPositions();
        SelectContinentCenters();
        AssignContinentCenterData();
        BuildSpatialBuckets();
        AssignWeightedData();
    }

    TectonicCell[] GenerateFibonacciSphere()
    {
        int n = settings.cellCount;
        TectonicCell[] result = new TectonicCell[n];

        // Golden angle in radians — irrational step ensures no two points
        // share the same longitude, producing even coverage across the sphere.
        float goldenAngle = Mathf.PI * (3f - Mathf.Sqrt(5f));

        for (int i = 0; i < n; i++)
        {
            float y = 1f - (i / (float)(n - 1)) * 2f;
            float radius = Mathf.Sqrt(1f - y * y);
            float theta = goldenAngle * i;

            result[i] = new TectonicCell
            {
                position = new Vector3(
                    Mathf.Cos(theta) * radius,
                    y,
                    Mathf.Sin(theta) * radius
                )
            };
        }

        return result;
    }

    void JitterPositions()
    {
        // Jitter strength relative to average cell spacing.
        // continentEdgeNoise = 0 produces a clean fibonacci spiral.
        // continentEdgeNoise = 1 produces heavy randomization.
        // Scaling by cellCount keeps jitter proportional regardless of resolution.
        float jitterStrength = settings.continentEdgeNoise * (2f / Mathf.Sqrt(settings.cellCount));

        for (int i = 0; i < cells.Length; i++)
        {
            Vector3 offset = new Vector3(
                (float)(rng.NextDouble() * 2 - 1),
                (float)(rng.NextDouble() * 2 - 1),
                (float)(rng.NextDouble() * 2 - 1)
            ) * jitterStrength;

            cells[i].position = (cells[i].position + offset).normalized;
        }
    }

    void SelectContinentCenters()
    {
        // Shuffle indices then mark the first continentCenters as continent centers.
        // Using a separate index array avoids shuffling actual cell data.
        int[] indices = new int[cells.Length];
        for (int i = 0; i < indices.Length; i++)
            indices[i] = i;

        // Fisher-Yates shuffle — unbiased random permutation
        for (int i = indices.Length - 1; i > 0; i--)
        {
            int j = rng.Next(i + 1);
            int temp = indices[i];
            indices[i] = indices[j];
            indices[j] = temp;
        }

        for (int i = 0; i < settings.continentCenters; i++)
            cells[indices[i]].isContinentCenter = 1;
    }

    void AssignContinentCenterData()
    {
        for (int i = 0; i < cells.Length; i++)
        {
            if (cells[i].isContinentCenter == 0) continue;

            cells[i].baseElevation = (float)(rng.NextDouble() * 2 - 1);
            cells[i].movementDirection = RandomTangent(cells[i].position);
            cells[i].movementStrength = Mathf.Lerp(
                (float)rng.NextDouble(), 1f, settings.strengthMultiplier);
        }
    }

    void AssignWeightedData()
    {
        // Breadth-first expansion from continent centers outward.
        // Each cell weights toward its nearest already-assigned neighbor,
        // producing gradual regional variation rather than sharp jumps.
        Queue<int> frontier = new Queue<int>();
        HashSet<int> assigned = new HashSet<int>();

        for (int i = 0; i < cells.Length; i++)
        {
            if (cells[i].isContinentCenter == 0) continue;
            frontier.Enqueue(i);
            assigned.Add(i);
        }

        List<int> nearby = new List<int>();
        float neighborRadiusSq = neighborRadius * neighborRadius;
        while (frontier.Count > 0)
        {
            int current = frontier.Dequeue();

            GetNearbyCells(cells[current].position, nearby);
            foreach (int i in nearby)
            {
                if (assigned.Contains(i)) continue;
                if (Vector3.SqrMagnitude(cells[current].position - cells[i].position) > neighborRadiusSq) continue;

                AssignWeightedFromNeighbor(i, current);
                assigned.Add(i);
                frontier.Enqueue(i);
            }
        }

        // Fallback for isolated cells not reached by the frontier
        for (int i = 0; i < cells.Length; i++)
        {
            if (assigned.Contains(i)) continue;
            int nearestIdx = GetNearestAssigned(cells[i].position, assigned);
            if (nearestIdx >= 0)
                AssignWeightedFromNeighbor(i, nearestIdx);
        }
    }

    void AssignWeightedFromNeighbor(int cellIndex, int neighborIndex)
    {
        float heightVariation = (float)(rng.NextDouble() * 2 - 1) * 0.15f;
        float strengthVariation = (float)(rng.NextDouble() * 2 - 1) * 0.05f;

        cells[cellIndex].baseElevation = Mathf.Clamp(
            cells[neighborIndex].baseElevation + heightVariation * (1f - settings.elevationCoherence),
            -1f, 1f);

        cells[cellIndex].movementStrength = Mathf.Clamp01(
            cells[neighborIndex].movementStrength + strengthVariation);

        float angleVariation = (float)(rng.NextDouble() * 2 - 1) * 30f * (1f - settings.elevationCoherence);
        cells[cellIndex].movementDirection = Quaternion.AngleAxis(angleVariation, cells[cellIndex].position)
                                        * cells[neighborIndex].movementDirection;
    }

    void BuildSpatialBuckets()
    {
        buckets = new List<int>[BUCKET_SIZE, BUCKET_SIZE];
        for (int y = 0; y < BUCKET_SIZE; y++)
            for (int x = 0; x < BUCKET_SIZE; x++)
                buckets[x, y] = new List<int>();

        for (int i = 0; i < cells.Length; i++)
        {
            Vector2 uv = UVHelper.SphereToUV(cells[i].position);
            int bx = Mathf.Clamp((int)(uv.x * BUCKET_SIZE), 0, BUCKET_SIZE - 1);
            int by = Mathf.Clamp((int)(uv.y * BUCKET_SIZE), 0, BUCKET_SIZE - 1);
            buckets[bx, by].Add(i);
        }
    }

    // --- Spatial Queries ---

    // Populates the provided list with cells in the bucket containing point plus all 8 neighboring buckets.
    // Far cheaper than searching all cells — use this instead of iterating tectonics.cells directly.
    public void GetNearbyCells(Vector3 point, List<int> results)
    {
        results.Clear();
        Vector2 uv = UVHelper.SphereToUV(point);
        int bx = Mathf.Clamp((int)(uv.x * BUCKET_SIZE), 0, BUCKET_SIZE - 1);
        int by = Mathf.Clamp((int)(uv.y * BUCKET_SIZE), 0, BUCKET_SIZE - 1);

        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dx = -1; dx <= 1; dx++)
            {
                // Wrap horizontally — U=0 and U=1 are the same longitude
                int nx = (bx + dx + BUCKET_SIZE) % BUCKET_SIZE;
                int ny = by + dy;

                if (ny < 0 || ny >= BUCKET_SIZE) continue;

                results.AddRange(buckets[nx, ny]);
            }
        }
    }

    // Returns the nearest tectonic cell to a point on the unit sphere.
    public int GetNearestCell(Vector3 point, List<int> scratchpad = null)
    {
        int nearest = -1;
        float nearestDistSq = float.MaxValue;

        List<int> nearby = scratchpad ?? new List<int>(); 
        GetNearbyCells(point, nearby);

        foreach (int i in nearby)
        {
            float distSq = Vector3.SqrMagnitude(point - cells[i].position);
            if (distSq < nearestDistSq)
            {
                nearestDistSq = distSq;
                nearest = i;
            }
        }

        // Fallback to full search if bucket search returns nothing
        // Can happen at poles or seam where buckets may be sparse
        if (nearest == -1)
        {
            for (int i = 0; i < cells.Length; i++)
            {
                float distSq = Vector3.SqrMagnitude(point - cells[i].position);
                if (distSq < nearestDistSq)
                {
                    nearestDistSq = distSq;
                    nearest = i;
                }
            }
        }

        return nearest;
    }

    // Returns the nearest cell excluding a specific cell.
    // Used for smooth voronoi blending — finds the second nearest cell to blend toward.
    public int GetSecondNearestCell(Vector3 point, int exclude, List<int> scratchpad = null)
    {
        int nearest = -1;
        float nearestDistSq = float.MaxValue;

        List<int> nearby = scratchpad ?? new List<int>();
        GetNearbyCells(point, nearby);

        foreach (int i in nearby)
        {
            if (i == exclude) continue;
            float distSq = Vector3.SqrMagnitude(point - cells[i].position);
            if (distSq < nearestDistSq)
            {
                nearestDistSq = distSq;
                nearest = i;
            }
        }

        if (nearest == -1)
        {
            for (int i = 0; i < cells.Length; i++)
            {
                if (i == exclude) continue;
                float distSq = Vector3.SqrMagnitude(point - cells[i].position);
                if (distSq < nearestDistSq)
                {
                    nearestDistSq = distSq;
                    nearest = i;
                }
            }
        }

        return nearest;
    }

    // Steps one average cell spacing from origin along direction,
    // then finds the nearest cell to that point.
    // direction should be normalized.
    public int GetCellInDirection(Vector3 origin, Vector3 direction, List<int> scratchpad = null)
    {
        Vector3 target = (origin + direction * averageCellSpacing).normalized;
        return GetNearestCell(target, scratchpad);
    }

    // Returns a distance-weighted blend of moisture across all nearby cells.
    // Produces smooth gradual transitions rather than hard voronoi edges.
    public float GetBlendedMoisture(Vector3 point, List<int> scratchpad = null)
    {
        float totalWeight = 0f;
        float weightedMoisture = 0f;
        float blendRadius = averageCellSpacing * 2f;

        List<int> nearby = scratchpad ?? new List<int>();
        GetCellsInRadius(point, blendRadius, nearby);

        foreach (int i in nearby)
        {
            float distSq = Vector3.SqrMagnitude(point - cells[i].position);
            float weight = 1f / (distSq + 0.0001f);
            weightedMoisture += cells[i].moistureLevel * weight;
            totalWeight += weight;
        }

        if (totalWeight <= 0f)
        {
            int nearest = GetNearestCell(point, scratchpad);
            return nearest != -1 ? cells[nearest].moistureLevel : 0f;
        }

        return weightedMoisture / totalWeight;
    }
    
    // Returns a distance-weighted blend of elevation across all nearby cells.
    // Produces smooth gradual transitions rather than hard voronoi edges.
    // Called by HeightMap instead of iterating cells directly.
    public float GetBlendedElevation(Vector3 point, List<int> scratchpad = null)
    {
        float totalWeight = 0f;
        float weightedElevation = 0f;
        float blendRadius = averageCellSpacing * 2f;

        List<int> nearby = scratchpad ?? new List<int>();
        GetCellsInRadius(point, blendRadius, nearby);

        foreach (int i in nearby)
        {
            float distSq = Vector3.SqrMagnitude(point - cells[i].position);
            float weight = 1f / (distSq + 0.0001f);
            weightedElevation += cells[i].baseElevation * weight;
            totalWeight += weight;
        }
       
        //Fallback - if no cell found in radius, use nearest cell directly.
        if (totalWeight <= 0f)
        {
            int nearest = GetNearestCell(point, scratchpad);
            return nearest != -1 ? cells[nearest].baseElevation : 0f;
        }
        return totalWeight > 0 ? weightedElevation / totalWeight : 0f;
    }

    // Populates the provided list with all cells within the given radius of a point.
    // Used by TectonicInfluenceMap to evaluate nearby cell relationships.
    public void GetCellsInRadius(Vector3 point, float radius, List<int> results)
    {
        GetNearbyCells(point, results);
        float radiusSq = radius * radius;
        for (int i = results.Count - 1; i >= 0; i--)
        {
            int cellIdx = results[i];
            if (Vector3.SqrMagnitude(point - cells[cellIdx].position) > radiusSq)
            {
                results.RemoveAt(i);
            }
        }
    }

    // --- Private Helpers ---

    // Returns the nearest already-assigned cell to a point.
    // Used as a fallback during weighted data assignment for isolated cells.
    int GetNearestAssigned(Vector3 point, HashSet<int> assigned)
    {
        int nearest = -1;
        float nearestDist = float.MaxValue;

        foreach (int idx in assigned)
        {
            float dist = Vector3.SqrMagnitude(point - cells[idx].position);
            if (dist < nearestDist)
            {
                nearestDist = dist;
                nearest = idx;
            }
        }

        return nearest;
    }

    // Returns a random unit vector tangent to the sphere at the given point.
    // Used for movement directions — movement must be along the surface, not into it.
    Vector3 RandomTangent(Vector3 spherePoint)
    {
        Vector3 reference = Vector3.up;
        if (Vector3.Dot(spherePoint, reference) > 0.99f)
            reference = Vector3.right;

        Vector3 tangent = Vector3.Cross(spherePoint, reference).normalized;
        float angle = (float)(rng.NextDouble() * 360f);
        return Quaternion.AngleAxis(angle, spherePoint) * tangent;
    }

    const float neighborRadius = 0.4f;
}