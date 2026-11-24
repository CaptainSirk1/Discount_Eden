using UnityEngine;
using System.Collections.Generic;

// Container for splines that approximate transform faults seeded from divergent plate boundaries.
// Sparse control points are traced during influence map generation, then refined once
// before storage — replacing the sparse points with a noise-displaced dense version.
// Higher LODs read controlPoints and tangents to seed perpendicular child splines.
public class FaultSpline
{
    public Vector3 origin;
    public float length;
    public float strength;
    public Vector3[] controlPoints;
    public Vector3[] tangents;

    public Vector3 PointAt(float t)
    {
        if (controlPoints == null || controlPoints.Length == 0) return origin;
        if (controlPoints.Length == 1) return controlPoints[0];

        float scaled = t * (controlPoints.Length - 1);
        int i = Mathf.Clamp((int)scaled, 0, controlPoints.Length - 2);
        float frac = scaled - i;
        return Vector3.Lerp(controlPoints[i], controlPoints[i + 1], frac);
    }

    // Returns the perpendicular direction on the sphere surface at point index i.
    // Child splines are seeded by stepping from controlPoints[i] along this vector.
    public Vector3 PerpAt(int i)
    {
        return Vector3.Cross(tangents[i], controlPoints[i]).normalized;
    }

    // Subdivides control points to target resolution and applies perpendicular
    // noise for organic variation. Call once after tracing — replaces sparse
    // control points and tangents with dense final versions.
    // Same noise curve for every spline on the same planet — seed drives the
    // curve shape, t drives position along it.
    public void Refine(int resolution, int seed)
    {
        if (controlPoints == null || controlPoints.Length < 2) return;

        List<Vector3> dense = new List<Vector3>();
        List<Vector3> denseTangents = new List<Vector3>();

        int segmentCount = controlPoints.Length - 1;
        int pointsPerSegment = Mathf.Max(1, resolution / segmentCount);

        for (int s = 0; s < segmentCount; s++)
        {
            for (int p = 0; p < pointsPerSegment; p++)
            {
                float t = p / (float)pointsPerSegment;
                dense.Add(Vector3.Slerp(controlPoints[s], controlPoints[s + 1], t));
                denseTangents.Add(Vector3.Slerp(tangents[s], tangents[s + 1], t).normalized);
            }
        }
        dense.Add(controlPoints[controlPoints.Length - 1]);
        denseTangents.Add(tangents[tangents.Length - 1]);

        float noiseScale = 2f;
        float noiseStrength = length * 0.05f;

        for (int i = 0; i < dense.Count; i++)
        {
            Vector3 point = dense[i];
            float t = i / (float)(dense.Count - 1);

            // Displace perpendicular to the local tangent on the sphere surface
            Vector3 perp = Vector3.Cross(denseTangents[i], point).normalized;

            // seed * 0.001f shifts the curve per planet while keeping it
            // identical across all splines on the same planet
            float noise = Mathf.PerlinNoise(t * noiseScale + seed * 0.001f, 0f) * 2f - 1f;
            dense[i] = (point + perp * noise * noiseStrength).normalized;
        }

        length = 0f;
        for (int i = 1; i < dense.Count; i++)
            length += Vector3.Distance(dense[i - 1], dense[i]);

        controlPoints = dense.ToArray();
        tangents = denseTangents.ToArray();
    }
}