using UnityEngine;


// Data container to store planetary Terrain/Climate settings within individual voronoi points.
public struct TectonicCell
{
    // A given point's position on the sphere
    public Vector3 position;
    
    // Determines whether a given cell is a "continent center"
    // When assigning values, a selection fo continent center cells will be assigned random data. 
    // All other cells will be assigned weighted data, pushed towards similarity with the nearest cell.
    // 0 = false, 1 = true.
    public byte isContinentCenter;

    // This value will inform how this cell appears on the heightmap and comprehensive view.
    [Range(-1, 1)]public float baseElevation;

    // Determines which direction the cell is "moving." 
    // Movement is purely there to inform geography at cell boundaries, not to simulate continental drift.
    // Stored as a Vector3 tangent to the sphere at this point.
    public Vector3 movementDirection;

    // Determines the strength whith which a cell is trying to move.
    // Informs the width and height range of geographical noise at cell boundaries.
    [Range(0f, 1f)] public float movementStrength;
    
    // Informs how this cell appears on the mositure map.
    // Caculated using global moisture, wind direction, and nearby cell boundaries.
    [Range(0f, 1f)] public float moistureLevel;
}