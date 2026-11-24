using Unity.Mathematics;

namespace PlanetGen.Tectonics.MeshSim
{
    /// <summary>
    /// Cumulative crust material ledger (SphereMeshState.ledger, doubles, unit-sphere km; multiply by R^2 for km^3).
    ///
    ///   crust(t) = Initial + RiftCrust + FloorFill + Magmatic
    ///              - SubductedSqueeze - SubductedCollapse - ErosionRecycled - Delaminated + residual
    ///
    /// Every term is written where the change happens, so the residual measures only floating-point error and
    /// any code path that changes volume without booking it. Volume moved between vertices (motion, flips,
    /// splits, merges, erosion transport, gravitational spreading) is conservative by construction and is not a
    /// term. The magma reservoir is mantle-side: Reservoir = MagmaCharged - Magmatic. Relaminated felsic material
    /// is a transfer between crust vertices; while it waits in a cell's accretion buffer it is "in transit" and is
    /// included on the crust side of the balance (SphereMeshState.InTransit).
    /// </summary>
    public static class MeshLedger
    {
        public const int
            Initial = 0,
            RiftCrust = 1,            // + new crust inserted by rift / ridge splits (MeshTopologyJob)
            FloorFill = 2,            // + crust thinner than minCrust topped up from the mantle (MeshIntegrateJob)
            Magmatic = 3,             // + arc magmatism deposited from the reservoirs (MeshBudgetJob)
            SubductedSqueeze = 4,     // - slab volume consumed while converging under an overriding plate (MeshIntegrateJob)
            SubductedCollapse = 5,    // - slab volume consumed when a slab vertex is removed (MeshTopologyJob)
            ErosionRecycled = 6,      // - sediment removed on reaching oceanic crust (MeshErosionJob)
            ErosionTransported = 7,   //   gross volume moved by erosion (information only)
            MagmaCharged = 8,         //   subducted volume x return fraction, added to the reservoirs (information)
            BuoyantSubducted = 9,     //   part of the consumed volume that was buoyant crust (cf >= 0.5) (information)
            // differentiated (felsic) material, the part of the volume that is continental in composition:
            //   felsic(t) = FelsicInitial + FelsicMagmatic - FelsicSubducted - FelsicRecycled + residual
            FelsicInitial = 10,
            FelsicMagmatic = 11,      // + every arc-magmatic deposit is felsic
            FelsicSubducted = 12,     // - felsic part of consumed slab volume (squeeze + collapse)
            FelsicRecycled = 13,      // - felsic part of recycled sediment
            Delaminated = 14,         // - over-thick crustal root foundered into the mantle (MeshIntegrateJob); also in Sinks
            FelsicDelaminated = 15,   // - felsic part of it
            Relaminated = 16,         //   felsic slab material moved to overriding margins (a transfer, information only)
            Count = 20;

        public static double Sources(in Unity.Collections.NativeArray<double> l) => l[RiftCrust] + l[FloorFill] + l[Magmatic];
        public static double Sinks(in Unity.Collections.NativeArray<double> l) => l[SubductedSqueeze] + l[SubductedCollapse] + l[ErosionRecycled] + l[Delaminated];
        public static double Expected(in Unity.Collections.NativeArray<double> l) => l[Initial] + Sources(l) - Sinks(l);
        public static double ExpectedFelsic(in Unity.Collections.NativeArray<double> l)
            => l[FelsicInitial] + l[FelsicMagmatic] - l[FelsicSubducted] - l[FelsicRecycled] - l[FelsicDelaminated];
    }

    /// <summary>
    /// Area-weighted planet-wide measurements, for telling mechanical behaviour apart from numerical material loss.
    /// O(V); run periodically, not every step. Volumes in km^3, areas as fractions of the surface.
    /// "Continental" = thickness above 0.75 x continental thickness (the convection feedback's threshold);
    /// "buoyant" = crust fraction >= 0.5 (what can no longer subduct).
    /// </summary>
    public struct MeshDiagnostics
    {
        public float timeMyr, globalThermalAge;
        public float crustVolume, expectedVolume, residual;  // km^3; residual = crust - ledger expectation
        public float continentalAreaFrac, buoyantAreaFrac;
        public float continentalVolume, buoyantVolume;       // km^3
        public float meanContinentalThickness, maxThickness;
        public float riftCrust, floorFill, magmatic, subducted, erosionRecycled, erosionMoved, reservoir, buoyantSubducted, delaminated, relaminated; // cumulative km^3
        public float meanDamage, maxDamage, continentalMeanDamage, continentalFracturedFrac;
        public float meanStress, continentalStress, continentalAchieved; // velRef / rad
        public float meanSpeed, continentalSpeed;            // rad / Myr
        public float minConvTemp, maxConvTemp, meanAbsConvTemp;
        // differentiated (felsic) material
        public float felsicVolume, felsicResidual, felsicMagmatic, felsicSubducted; // km^3
        public float felsicAreaFrac;          // surface fraction whose crust is mostly felsic (fraction >= 0.5)
        public float felsicMeanThickness;     // km, mean thickness of that crust
        public float felsicInThin;            // km^3 of felsic material sitting in crust thinner than 21 km (cf < 0.5)

        public static MeshDiagnostics Compute(SphereMeshState s, in MeshParams M, float radiusKm, float timeMyr)
        {
            var d = new MeshDiagnostics { timeMyr = timeMyr, globalThermalAge = M.globalThermal };
            double transit = s.InTransit();
            float contLimit = 0.75f * M.P.contThick;
            double A = 0, Ac = 0, Ab = 0, vol = 0, volC = 0, volB = 0, dmg = 0, dmgC = 0, fracC = 0, st = 0, stC = 0, achC = 0, sp = 0, spC = 0;
            float maxD = 0f, maxT = 0f;
            double fel = 0, felA = 0, felAV = 0, felThin = 0;
            for (int i = 0; i < s.VCap; i++)
            {
                if (s.alive[i] == 0) continue;
                double a = s.area[i];
                float t = s.thickness[i], D = s.damage[i];
                float speed = math.length(s.vel[i]);
                A += a; vol += s.volume[i]; dmg += D * a; st += s.tension[i] * a; sp += speed * a;
                float fv = s.felsic[i];
                fel += fv;
                if (fv >= 0.5f * s.volume[i]) { felA += a; felAV += s.volume[i]; }
                if (s.cfrac[i] < 0.5f) felThin += fv;
                maxD = math.max(maxD, D); maxT = math.max(maxT, t);
                if (s.cfrac[i] >= 0.5f) { Ab += a; volB += s.volume[i]; }
                if (t > contLimit)
                {
                    Ac += a; volC += s.volume[i]; dmgC += D * a; stC += s.tension[i] * a; achC += s.achieved[i] * a; spC += speed * a;
                    if (D >= M.riftDamage) fracC += a;
                }
            }
            double r2 = (double)radiusKm * radiusKm;
            if (A > 0)
            {
                d.continentalAreaFrac = (float)(Ac / A); d.buoyantAreaFrac = (float)(Ab / A);
                d.meanDamage = (float)(dmg / A); d.meanStress = (float)(st / A); d.meanSpeed = (float)(sp / A);
            }
            if (Ac > 0)
            {
                d.continentalMeanDamage = (float)(dmgC / Ac); d.continentalFracturedFrac = (float)(fracC / Ac);
                d.continentalStress = (float)(stC / Ac); d.continentalAchieved = (float)(achC / Ac);
                d.continentalSpeed = (float)(spC / Ac); d.meanContinentalThickness = (float)(volC / Ac);
            }
            d.maxDamage = maxD; d.maxThickness = maxT;
            var l = s.ledger;
            d.crustVolume = (float)(vol * r2);
            d.expectedVolume = (float)(MeshLedger.Expected(l) * r2);
            d.residual = (float)((vol + transit - MeshLedger.Expected(l)) * r2);
            d.relaminated = (float)(l[MeshLedger.Relaminated] * r2);
            d.continentalVolume = (float)(volC * r2); d.buoyantVolume = (float)(volB * r2);
            d.riftCrust = (float)(l[MeshLedger.RiftCrust] * r2); d.floorFill = (float)(l[MeshLedger.FloorFill] * r2);
            d.magmatic = (float)(l[MeshLedger.Magmatic] * r2);
            d.subducted = (float)((l[MeshLedger.SubductedSqueeze] + l[MeshLedger.SubductedCollapse]) * r2);
            d.erosionRecycled = (float)(l[MeshLedger.ErosionRecycled] * r2); d.erosionMoved = (float)(l[MeshLedger.ErosionTransported] * r2);
            d.buoyantSubducted = (float)(l[MeshLedger.BuoyantSubducted] * r2);
            d.delaminated = (float)(l[MeshLedger.Delaminated] * r2);
            double pool = 0; for (int k = 0; k < s.ConvCount; k++) pool += s.convPool[k];
            d.reservoir = (float)(pool * r2);
            float mn = float.MaxValue, mx = float.MinValue, ma = 0f;
            for (int k = 0; k < s.ConvCount; k++) { float T = s.conv[k].temp; mn = math.min(mn, T); mx = math.max(mx, T); ma += math.abs(T); }
            d.felsicVolume = (float)(fel * r2);
            d.felsicResidual = (float)((fel + transit - MeshLedger.ExpectedFelsic(l)) * r2);
            d.felsicMagmatic = (float)(l[MeshLedger.FelsicMagmatic] * r2);
            d.felsicSubducted = (float)((l[MeshLedger.FelsicSubducted] + l[MeshLedger.FelsicDelaminated] + l[MeshLedger.FelsicRecycled]) * r2);
            d.felsicAreaFrac = A > 0 ? (float)(felA / A) : 0f;
            d.felsicMeanThickness = felA > 0 ? (float)(felAV / felA) : 0f;
            d.felsicInThin = (float)(felThin * r2);
            d.minConvTemp = mn; d.maxConvTemp = mx; d.meanAbsConvTemp = s.ConvCount > 0 ? ma / s.ConvCount : 0f;
            return d;
        }
    }
}
