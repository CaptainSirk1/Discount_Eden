using UnityEngine;

// Contains universal data and helpful functions that remain consistent across multiple texture maps.
public static class UVHelper
{
    public const int MAP_WIDTH = 512;  // Covers Longitude, 360 degrees
    public const int MAP_HEIGHT = 256; // Covers Latitude, 180 degrees

    // --- Coordinate Conversion ---

    // SphereToUV and UVToSphere are helper Functions used for reading and generating spherical texture data.
    // Both methods being public static means any script can call them directly.
    // UVHelper.SphereToUV() or UVHelper.UVToSphere()

    // SphereToUV:
    
    //  A note about equirectangular projection:
    //  u wraps longitude, v covers latitude.
    //  * Think of longitude and latitude lines like ticks on a ruler.
    //  * Longitude lines run vertically from the North pole of the sphere to the South pole, but measure horizontal distance.
    //  * Lattiude lines run horizontal around the sphere, but measure vertical distance.
    //  This function runs the same math used to stretch our spherical planet into rectangular maps.
    //  The same number of horizontal u values will map to fewer and fewer points as you approach the poles, causing stretching on a 2d texture.
    //  * u = 0 and u = 1 represent the same longitude on a sphere.
    //  * If you bent a ruler into a circle, this would be the point where the end meets the beginning.
    //  * This can cause a vertical seam if UVs aren't handled carefully. It shouldn't be as big of an issue with procedural textures.

    // Given a point on a 3d unit sphere, returns a UV coordinate in a 2 dimensional projection.
    // This function is used for reading a texture on a sphere.
    // IMPORTANT: Any script that needs to read the texture map to extract climate data will call this function.
    public static Vector2 SphereToUV(Vector3 pointOnUnitSphere)
    {
        // u = 0.5 + arctan(z, x) / 2π
        // v = 0.5 - arcsin(y) / π
        float u = 0.5f + Mathf.Atan2(pointOnUnitSphere.z, pointOnUnitSphere.x) / (2f * Mathf.PI);
        float v = 0.5f - Mathf.Asin(Mathf.Clamp(pointOnUnitSphere.y, -1f, 1f)) / Mathf.PI;
        return new Vector2(u, v);
    }
    
    // UVToSphere:

    // Given a 2d uv coordinate, returns the associated point on a 3d unit sphere.
    // This function is used for generating a texture on a sphere
    public static Vector3 UVToSphere(float u, float v)
    {   // longitude = (u - 0.5) * 2π
        // latitude = (0.5 - v) * π
        // x = cos(latitude) * cos(longitude)
        // y = sin(latitude)
        // z = cos(latitude) * sin(longitude)
        float longitude = (u - 0.5f) * 2f * Mathf.PI;
        float latitude = (0.5f - v) * Mathf.PI;
        return new Vector3(
            Mathf.Cos(latitude) * Mathf.Cos(longitude),
            Mathf.Sin(latitude),
            Mathf.Cos(latitude) * Mathf.Sin(longitude)
        );
    }

    // Applies domain warping to a sphere point before cell lookup.
    // Offsets the sample position by a noise value, making cell boundaries
    // appear organic rather than mathematically clean.
    // Called before GetNearestCell() in any map that wants warped voronoi boundaries.
    public static Vector3 WarpSpherePoint(Vector3 point, NoiseSettings warpNoise, int seed)
    {
        if (warpNoise == null) return point;
        Vector3 offset = warpNoise.SampleWarp(point, seed);
        return (point + offset).normalized;
    }
}