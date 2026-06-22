using UnityEngine;
using System.Collections.Generic;

// Spatial index over FaultSplines for cheap proximity queries.
// Divides UV space into an 8x8 grid — consistent with GenerateTectonics bucketing.
// Each cell holds references to all splines with control points passing through it.
// Query via GetNearbySplines or GetNearestPoint rather than iterating faultSplines directly.
public class FaultSplineGrid
{
    const int BUCKET_SIZE = 8;
    List<FaultSpline>[,] buckets;

    public FaultSplineGrid(List<FaultSpline> splines)
    {
        buckets = new List<FaultSpline>[BUCKET_SIZE, BUCKET_SIZE];
        for (int y = 0; y < BUCKET_SIZE; y++)
            for (int x = 0; x < BUCKET_SIZE; x++)
                buckets[x, y] = new List<FaultSpline>();

        // Register each spline in every bucket its control points touch.
        // Long splines appear in multiple buckets — references only, no copies.
        foreach (FaultSpline spline in splines)
        {
            HashSet<(int, int)> visited = new HashSet<(int, int)>();
            foreach (Vector3 point in spline.controlPoints)
            {
                Vector2 uv = UVHelper.SphereToUV(point);
                int bx = Mathf.Clamp((int)(uv.x * BUCKET_SIZE), 0, BUCKET_SIZE - 1);
                int by = Mathf.Clamp((int)(uv.y * BUCKET_SIZE), 0, BUCKET_SIZE - 1);

                if (visited.Add((bx, by)))
                    buckets[bx, by].Add(spline);
            }
        }
    }

    // Returns all splines registered in the bucket containing point
    // plus all 8 neighboring buckets. Wraps horizontally at the seam.
    public IEnumerable<FaultSpline> GetNearbySplines(Vector3 point)
    {
        Vector2 uv = UVHelper.SphereToUV(point);
        int bx = Mathf.Clamp((int)(uv.x * BUCKET_SIZE), 0, BUCKET_SIZE - 1);
        int by = Mathf.Clamp((int)(uv.y * BUCKET_SIZE), 0, BUCKET_SIZE - 1);

        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dx = -1; dx <= 1; dx++)
            {
                int nx = (bx + dx + BUCKET_SIZE) % BUCKET_SIZE;
                int ny = by + dy;
                if (ny < 0 || ny >= BUCKET_SIZE) continue;

                foreach (FaultSpline spline in buckets[nx, ny])
                    yield return spline;
            }
        }
    }

    // Returns the closest point on any nearby spline, its squared distance,
    // and the parent spline's strength. HeightMap uses this to apply falloff.
    // Returns false if no splines are found in neighboring buckets.
    public bool GetNearestPoint(Vector3 point, out Vector3 nearestPoint, out float strength, out float distanceSq)
    {
        nearestPoint = Vector3.zero;
        strength = 0f;
        distanceSq = float.MaxValue;
        bool found = false;

        foreach (FaultSpline spline in GetNearbySplines(point))
        {
            foreach (Vector3 cp in spline.controlPoints)
            {
                float dSq = Vector3.SqrMagnitude(point - cp);
                if (dSq < distanceSq)
                {
                    distanceSq = dSq;
                    nearestPoint = cp;
                    strength = spline.strength;
                    found = true;
                }
            }
        }

        return found;
    }
}