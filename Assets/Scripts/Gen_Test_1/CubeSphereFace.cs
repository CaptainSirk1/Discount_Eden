using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using PlanetGeneration;

// The initial planet mesh will essentially be a cube consisting of 6 faces
// The vertices making up each face will be 'normalized,' or shifted to an equal distance from the center of the cube
// It's basically pulling in the corners of your cube to turn it into a sphere.

// The purpose of this class is to create the faces of our cube sphere planet.

// The contents of this class will have little bearing on the texture displayed on any given face.
// If you're looking to change something about how the planet looks, this script is largely irrelevant.
public class CubeSphereFace
{
    public int resolution { get; private set; }
    TerrainShapeGenerator shapeGen;
    Mesh mesh;
    
    Vector3 localForward;
    Vector3 axisA;
    Vector3 axisB;

    Vector3[] vertices;
    Vector2[] uvs;
    int[] triangles;
    Dictionary<int, int> duplicateCache = new Dictionary<int, int>();

    public CubeSphereFace(TerrainShapeGenerator shapeGen, Mesh mesh, int resolution, Vector3 localForward)
    {
        this.shapeGen = shapeGen;
        this.mesh = mesh;
        this.resolution = resolution;
        this.localForward = localForward;

        axisA = new Vector3(localForward.y, localForward.z, localForward.x); 
        axisB = Vector3.Cross(localForward, axisA);

        int baseVertexCount = resolution * resolution;
        vertices = new Vector3[baseVertexCount + resolution * 2 + 64]; 
        uvs = new Vector2[vertices.Length];
        triangles = new int[(resolution - 1) * (resolution - 1) * 6];
        duplicateCache = new Dictionary<int, int>();
    }

    public JobHandle ScheduleMeshJob(NativeArray<Vector3> jobVertices, NativeArray<Vector2> jobUVs)
    {
        var job = new ConstructMeshJob
        {
            resolution = resolution,
            localForward = localForward,
            axisA = axisA,
            axisB = axisB,
            planetRadius = shapeGen.settings.planetRadius,
            vertices = jobVertices.Reinterpret<Unity.Mathematics.float3>(),
            uvs = jobUVs.Reinterpret<Unity.Mathematics.float2>()
        };

        return job.Schedule(resolution * resolution, 64);
    }

    public void ConstructMesh(Vector3[] computedVertices = null, Vector2[] computedUVs = null)
    {
        int vertexCount = resolution * resolution;
        duplicateCache.Clear();

        if (computedVertices != null && computedUVs != null)
        {
            System.Array.Copy(computedVertices, vertices, vertexCount);
            System.Array.Copy(computedUVs, uvs, vertexCount);
        }
        else
        {
            // Fallback for non-jobified calls
            for (int y = 0; y < resolution; y++)
            {
                for (int x = 0; x < resolution; x++)
                {
                    int i = x + y * resolution;
                    Vector2 percent = new Vector2(x, y) / (resolution - 1);
                    Vector3 pointOnUnitCube = localForward + (percent.x - 0.5f) * 2 * axisA + (percent.y - 0.5f) * 2 * axisB;
                    Vector3 normalizedPoint = pointOnUnitCube.normalized;
                    vertices[i] = normalizedPoint * shapeGen.settings.planetRadius;
                    uvs[i] = UVHelper.SphereToUV(normalizedPoint);
                }
            }
        }

        int triIndex = 0;
        int currentVertexCount = vertexCount;

        for (int y = 0; y < resolution - 1; y++)
        {
            for (int x = 0; x < resolution - 1; x++)
            {
                int i = x + y * resolution;

                int iA = i;
                int iB = i + resolution + 1;
                int iC = i + resolution;
                int iD = i + 1;

                ProcessTriangle(iA, iB, iC, ref currentVertexCount, ref triIndex);
                ProcessTriangle(iA, iD, iB, ref currentVertexCount, ref triIndex);
            }
        }

        mesh.Clear();
        mesh.SetVertices(vertices, 0, currentVertexCount);
        mesh.SetUVs(0, uvs, 0, currentVertexCount);
        mesh.SetTriangles(triangles, 0, triIndex, 0);
        mesh.RecalculateNormals();
    }


    void ProcessTriangle(int a, int b, int c, ref int currentCount, ref int triIdx)
    {
        float uA = uvs[a].x;
        float uB = uvs[b].x;
        float uC = uvs[c].x;

        bool crossesSeam = Mathf.Abs(uA - uB) > 0.5f ||
                          Mathf.Abs(uB - uC) > 0.5f ||
                          Mathf.Abs(uA - uC) > 0.5f;

        if (crossesSeam)
        {
            if (uA < 0.5f) a = GetOrCreateDuplicate(a, ref currentCount);
            if (uB < 0.5f) b = GetOrCreateDuplicate(b, ref currentCount);
            if (uC < 0.5f) c = GetOrCreateDuplicate(c, ref currentCount);
        }

        triangles[triIdx++] = a;
        triangles[triIdx++] = b;
        triangles[triIdx++] = c;
    }

    int GetOrCreateDuplicate(int originalIndex, ref int currentCount)
    {
        if (duplicateCache.TryGetValue(originalIndex, out int existingIndex))
        {
            return existingIndex;
        }

        int newIndex = currentCount++;
        vertices[newIndex] = vertices[originalIndex];
        uvs[newIndex] = new Vector2(uvs[originalIndex].x + 1f, uvs[originalIndex].y);
        duplicateCache.Add(originalIndex, newIndex);
        return newIndex;
    }
}