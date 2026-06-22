using UnityEngine;
using System.Collections.Generic;

// The initial planet mesh will essentially be a cube consisting of 6 faces
// The vertices making up each face will be 'normalized,' or shifted to an equal distance from the center of the cube
// It's basically pulling in the corners of your cube to turn it into a sphere.

// The purpose of this class is to create the faces of our cube sphere planet.

// The contents of this class will have little bearing on the texture displayed on any given face.
// If you're looking to change something about how the planet looks, this script is largely irrelevant.
public class CubeSphereFace
{
    TerrainShapeGenerator shapeGen;
    Mesh mesh;
    int resolution;
    
    // localForward will be SomeVector[x, y, z]: X covers the global left/right axis, Y covers up/down, and Z covers forward/back.
    // If your standing in "front" of the final cube sphere, the localForward of right side face of the sphere would be [1, 0, 0]
    // The face on the left side of the sphere would be [-1, 0, 0], indicating that the global left is the local forward for that face.
    Vector3 localForward;
    Vector3 axisA;
    Vector3 axisB;

    public CubeSphereFace(TerrainShapeGenerator shapeGen, Mesh mesh, int resolution, Vector3 localForward)
    {
        this.shapeGen = shapeGen;
        this.mesh = mesh;
        this.resolution = resolution;
        this.localForward = localForward;

        // Create a new Vector based on localForward, ordered y, z, x instead of x, y, z.
        // If LocalForward was [1, 0, 0], AxisA would be [0, 0, 1].
        // The cross product automatically produces the correct B axis using our existing axis'. 
        // Just using the same shuffle method again would invert half your normals. For some reason.
        // This essenitally gives each of your faces a complete set of local axis, all perpendicular to eachother. 
        // Note: This shuffle method only works when localForward is a cardinal direction, which it always will be for the sake of our cube sphere.
        axisA = new Vector3(localForward.y, localForward.z, localForward.x); 
        axisB = Vector3.Cross(localForward, axisA);
    }

    public void ConstructMesh()
    {
        List<Vector3> vertexList = new List<Vector3>();
        List<Vector2> uvList = new List<Vector2>();
        List<int> triangleList = new List<int>();

        // First pass — build all vertices and UVs exactly as before
        Vector3[] baseVertices = new Vector3[resolution * resolution];
        Vector2[] baseUVs = new Vector2[resolution * resolution];

        for (int y = 0; y < resolution; y++)
        {
            for (int x = 0; x < resolution; x++)
            {
                int i = x + y * resolution;
                Vector2 percent = new Vector2(x, y) / (resolution - 1);
                Vector3 pointOnUnitCube = localForward + (percent.x - 0.5f) * 2 * axisA + (percent.y - 0.5f) * 2 * axisB;

                baseVertices[i] = shapeGen.CalculatePointOnPlanet(pointOnUnitCube.normalized);
                baseUVs[i] = UVHelper.SphereToUV(pointOnUnitCube.normalized);
            }
        }

        // Add all base vertices to lists
        vertexList.AddRange(baseVertices);
        uvList.AddRange(baseUVs);

        // Second pass — build triangles, duplicating vertices at the seam
        for (int y = 0; y < resolution - 1; y++)
        {
            for (int x = 0; x < resolution - 1; x++)
            {
                int i = x + y * resolution;

                // The two triangles that make up this quad
                int[] quadTriangles = new int[]
                {
                    i, i + resolution + 1, i + resolution,
                    i, i + 1, i + resolution + 1
                };

                // Process each triangle
                for (int t = 0; t < 2; t++)
                {
                    int idxA = quadTriangles[t * 3];
                    int idxB = quadTriangles[t * 3 + 1];
                    int idxC = quadTriangles[t * 3 + 2];

                    float uA = uvList[idxA].x;
                    float uB = uvList[idxB].x;
                    float uC = uvList[idxC].x;

                    bool crossesSeam = Mathf.Abs(uA - uB) > 0.5f ||
                                    Mathf.Abs(uB - uC) > 0.5f ||
                                    Mathf.Abs(uA - uC) > 0.5f;

                    if (crossesSeam)
                    {
                        // Duplicate any vertex that's on the low side of the seam
                        // giving it a UV shifted up by 1, leaving the original untouched
                        if (uA < 0.5f) idxA = DuplicateVertex(idxA, 1f, vertexList, uvList);
                        if (uB < 0.5f) idxB = DuplicateVertex(idxB, 1f, vertexList, uvList);
                        if (uC < 0.5f) idxC = DuplicateVertex(idxC, 1f, vertexList, uvList);
                    }

                    triangleList.Add(idxA);
                    triangleList.Add(idxB);
                    triangleList.Add(idxC);
                }
            }
        }
        mesh.Clear();
        mesh.vertices = vertexList.ToArray();
        mesh.triangles = triangleList.ToArray();
        mesh.uv = uvList.ToArray();
        mesh.RecalculateNormals();
    }

    // Creates a copy of a vertex with its U coordinate shifted by uOffset.
    // Returns the index of the new duplicate in the vertex list.
    int DuplicateVertex(int originalIndex, float uOffset, List<Vector3> vertices, List<Vector2> uvs)
    {
        int newIndex = vertices.Count;
        vertices.Add(vertices[originalIndex]);
        uvs.Add(new Vector2(uvs[originalIndex].x + uOffset, uvs[originalIndex].y));
        return newIndex;
    }
}