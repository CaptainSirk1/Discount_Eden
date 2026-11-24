using System;
using Unity.Mathematics;
using UnityEngine;

namespace PlanetGen.Tectonics
{
    public enum FaultType : byte
    {
        None = 0,
        Divergent = 1,   // ridge / rift
        Subduction = 2,  // trench + volcanic arc
        Collision = 3,   // crumple zone / orogen
        Transform = 4    // strike-slip
    }

    /// <summary>A sparse convection point on the unit sphere. temp > 0 = upwelling (surface flow away),
    /// temp &lt; 0 = downwelling (surface flow toward).</summary>
    public struct Convection
    {
        public float3 pos;
        public float temp;      // effective temperature used by the drive job
        public float baseTemp;  // slow random walk
        public float feedback;  // contribution from continental insulation / subduction cooling
    }

    /// <summary>Blittable bag of every tunable the jobs need. Built by PlanetTectonics.</summary>
    public struct TectonicParams
    {
        // time / grid
        public float dt, spacing, interactR, minDist, collDist, gapDist, holeClear, sepStrength;
        public float invCell; public int gridDim; public int capacity; public uint seed;
        public float maxSpeed;

        // convection
        public float convRadius, convStrength, plumeThreshold, plumeRadius, convDrift, convMinSep;
        public float tempNoise, insulationHeat, subductionCool, feedbackRelax;

        // resistance / coupling
        public float ageResist, ageRef, velTau, couple, slabPull;

        // subduction / divergence
        public float subductThickness, subductDensity, subductRate, closeThreshold, openThreshold;
        public float riftStiffness, riftBlend, thinRate;

        // crumple / terrain-facing accumulators
        public float crumpleThick, overThick, orogenyGain, orogenyDecay, erosion, maxThickness, arcGain, arcDecay;

        // fault classification
        public float faultThreshold, faultMemory, velRef, transformDominance;

        // crust constants (km)
        public float oceanThick, contThick;

        // gravity cap: convergence stalls once crust can no longer thicken
        public float lockStart, lockMargin, lockStrength;
        // trenches: subducting points this close to the overriding plate are consumed
        public float consumeDist;

        // void filling
        public float expectNeighbors;   // neighbours within interactR on an even lattice
        public float voidFill;          // crowding below this proposes a spawn (0 = off)
        public float voidBias;          // |mean neighbour direction| above this = hole edge
        public float invSpawnCell;      // 1 / spawn-hash cell size
    }

    public static class TMath
    {
        /// <summary>0 = oceanic crust thickness, 1 = normal continental thickness.</summary>
        public static float CrustFrac(float thickness, in TectonicParams p)
        {
            return math.saturate((thickness - p.oceanThick) / math.max(1e-3f, p.contThick - p.oceanThick));
        }

        /// <summary>Rough isostatic elevation in km. Shared by the extractor and the live preview.</summary>
        public static float ElevationKm(float thickness, float age, in TectonicParams p)
        {
            float cf = CrustFrac(thickness, p);
            return -2.5f + 0.118f * (thickness - p.oceanThick)
                   - 0.25f * math.sqrt(math.min(age, 150f)) * (1f - cf);
        }

        /// <summary>g/cm3-ish. Oceanic crust densifies as it cools (age); continental is light.</summary>
        public static float Density(float thickness, float age, in TectonicParams p)
        {
            float cf = CrustFrac(thickness, p);
            float ocean = 2.9f + 0.35f * math.saturate(age / 200f);
            return math.lerp(ocean, 2.75f, cf);
        }

        /// <summary>0..1. Continental and old = stiff. Recently active faults are weakened.</summary>
        public static float Stiffness(float thickness, float age, float faultActivity, in TectonicParams p)
        {
            float cf = CrustFrac(thickness, p);
            float ageF = math.saturate(age / p.ageRef);
            float s = math.lerp(0.2f, 0.6f, cf) + 0.3f * ageF;
            return math.saturate(s * (1f - 0.5f * math.saturate(faultActivity)));
        }

        /// <summary>Resistance to convective drive: denser and older = harder to move.</summary>
        public static float Resistance(float density, float age, in TectonicParams p)
        {
            return (density / 2.9f) * (1f + p.ageResist * math.saturate(age / p.ageRef));
        }

        public static uint Hash(uint x)
        {
            x ^= x >> 16; x *= 0x7feb352du;
            x ^= x >> 15; x *= 0x846ca68bu;
            x ^= x >> 16;
            return x;
        }

        public static float Rand01(uint x)
        {
            return (Hash(x) & 0xFFFFFFu) / 16777216f;
        }

        public static uint HashCell(int3 c)
        {
            return Hash((uint)c.x * 73856093u ^ (uint)c.y * 19349663u ^ (uint)c.z * 83492791u);
        }

        /// <summary>Continuous tangent frame used by the preview quads and the Velocity view.</summary>
        public static void TangentFrame(float3 n, out float3 t, out float3 bt)
        {
            float3 up = math.abs(n.y) < 0.99f ? new float3(0f, 1f, 0f) : new float3(1f, 0f, 0f);
            t = math.normalize(math.cross(up, n));
            bt = math.cross(n, t);
        }

        public static int Cell(float3 p, in TectonicParams P)
        {
            int3 c = (int3)math.floor((p + 1f) * P.invCell);
            c = math.clamp(c, new int3(0), new int3(P.gridDim - 1));
            return c.x + P.gridDim * (c.y + P.gridDim * c.z);
        }
    }

    /// <summary>Per-point result handed to terrain generation.</summary>
    [Serializable]
    public struct PlanetPoint
    {
        public Vector3 position;      // unit sphere
        public Vector3 velocity;      // tangent, radians / Myr
        public float thickness;       // crust km
        public float density;
        public float age;             // Myr since this crust formed
        public float stiffness;
        public FaultType faultType;   // dominant (possibly fossil) fault behaviour
        public float faultActivity;   // 0..~1+, decays over faultMemory
        public Vector3 faultDir;      // unit tangent ALONG the boundary
        public float orogeny;         // decaying accumulated compression
        public float extension;       // accumulated continental stretching
        public float arc;             // decaying subduction-arc volcanism
        public float shear;           // decaying accumulated transform shear
        public float hotspot;         // accumulated plume exposure
        public float crowding;        // local point density, 1 = even lattice
        public float elevationKm;     // rough isostatic starting point, replace freely
    }

    [Serializable]
    public struct FaultSample
    {
        public Vector3 position;
        public Vector3 tangent;
        public FaultType type;
        public float activity;
        public float thickness;
    }
}
