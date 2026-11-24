// Headless test harness for the revised mesh tectonics. Runs MeshRun.ScheduleSteps (the exact chain the Unity
// component schedules) serially on stubbed Unity.Jobs / Collections. Harness-only measurements (tracers,
// clustering, localisation) never touch the simulation state.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Unity.Mathematics;
using Unity.Jobs;
using PlanetGen.Tectonics;
using PlanetGen.Tectonics.MeshSim;

static class Harness
{
    static MeshRun run;
    static SphereMeshState S => run.State;
    static float R2 => run.Settings.planetRadiusKm * run.Settings.planetRadiusKm;

    // ---------------------------------------------------------------- tracers (continental shape retention)
    struct Tracer { public float3 p; public int v; }
    static Tracer[] tracers = Array.Empty<Tracer>();
    static (int a, int b, float d0)[] pairs = Array.Empty<(int, int, float)>();

    static int Walk(float3 p, int v)
    {
        if (v < 0 || S.alive[v] == 0) { for (int i = 0; i < S.VCap; i++) if (S.alive[i] == 1) { v = i; break; } }
        for (int it = 0; it < 400; it++)
        {
            float best = math.dot(S.pos[v], p); int bv = v;
            int c0 = S.vc[v], c = c0, n = 0;
            do { int j = S.V[MeshTopo.Next(c)]; float d = math.dot(S.pos[j], p); if (d > best) { best = d; bv = j; } c = MeshTopo.Next(S.O[MeshTopo.Next(c)]); } while (c != c0 && ++n < 64);
            if (bv == v) return v;
            v = bv;
        }
        return v;
    }

    static void InitTracers(int count, uint seed)
    {
        var cand = new List<int>();
        float lim = 0.75f * run.Settings.continentThicknessKm;
        for (int i = 0; i < S.VCap; i++)
        {
            if (S.alive[i] == 0 || S.thickness[i] <= lim) continue;
            bool interior = true; int c0 = S.vc[i], c = c0, n = 0;
            do { if (S.thickness[S.V[MeshTopo.Next(c)]] <= lim) interior = false; c = MeshTopo.Next(S.O[MeshTopo.Next(c)]); } while (c != c0 && ++n < 64);
            if (interior) cand.Add(i);
        }
        var rng = new Unity.Mathematics.Random(seed | 1);
        var pick = cand.OrderBy(_ => rng.NextUInt()).Take(count).ToArray();
        tracers = pick.Select(v => new Tracer { p = S.pos[v], v = v }).ToArray();
        var pl = new List<(int, int, float)>();
        for (int a = 0; a < tracers.Length; a++)
        {
            int found = 0;
            for (int b = 0; b < tracers.Length && found < 4; b++)
            {
                if (a == b) continue;
                float d = math.acos(math.clamp(math.dot(tracers[a].p, tracers[b].p), -1f, 1f));
                if (d > 0.06f && d < 0.2f) { pl.Add((a, b, d)); found++; }
            }
        }
        pairs = pl.ToArray();
    }

    static void AdvectTracers(float dt)
    {
        for (int k = 0; k < tracers.Length; k++)
        {
            var t = tracers[k];
            t.v = Walk(t.p, t.v);
            float3 v = S.vel[t.v];
            v -= t.p * math.dot(v, t.p);
            t.p = math.normalize(t.p + v * dt);
            tracers[k] = t;
        }
    }

    /// <summary>(mean |ln(d/d0)| over tracer pairs, fraction of tracers still on continental crust)</summary>
    static (float distortion, float onCont) TracerStats()
    {
        if (pairs.Length == 0) return (0, 0);
        double s = 0;
        foreach (var (a, b, d0) in pairs)
        {
            float d = math.acos(math.clamp(math.dot(tracers[a].p, tracers[b].p), -1f, 1f));
            s += Math.Abs(Math.Log(Math.Max(d, 1e-5) / d0));
        }
        int on = 0; float lim = 0.75f * run.Settings.continentThicknessKm;
        foreach (var t in tracers) if (S.thickness[Walk(t.p, t.v)] > lim) on++;
        return ((float)(s / pairs.Length), tracers.Length > 0 ? (float)on / tracers.Length : 0);
    }

    // ---------------------------------------------------------------- clustering + localisation
    /// <summary>|sum A p| / sum A over continental crust: ~1 one cluster, ~0 spread around the planet.</summary>
    static float Clustering()
    {
        float3 m = 0; double a = 0; float lim = 0.75f * run.Settings.continentThicknessKm;
        for (int i = 0; i < S.VCap; i++) if (S.alive[i] == 1 && S.thickness[i] > lim) { m += S.pos[i] * S.area[i]; a += S.area[i]; }
        return a > 0 ? (float)(math.length(m) / a) : 0f;
    }

    /// <summary>Share of the continental extension (achieved opening x area) carried by the top 10% of continental area.</summary>
    static float Localisation()
    {
        var l = new List<(float e, float a)>(); float lim = 0.75f * run.Settings.continentThicknessKm;
        for (int i = 0; i < S.VCap; i++) if (S.alive[i] == 1 && S.thickness[i] > lim) l.Add((S.achieved[i], S.area[i]));
        if (l.Count < 10) return 0;
        double tot = l.Sum(x => (double)x.e * x.a), at = l.Sum(x => (double)x.a);
        if (tot <= 0) return 0;
        double acc = 0, aa = 0;
        foreach (var x in l.OrderByDescending(x => x.e)) { if (aa > 0.1 * at) break; acc += x.e * x.a; aa += x.a; }
        return (float)(acc / tot);
    }

    /// <summary>Continental-classified connected components with >= 1% of the surface.</summary>
    static int Landmasses()
    {
        float lim = 0.75f * run.Settings.continentThicknessKm;
        var seen = new bool[S.VCap]; int count = 0; double total = 0;
        for (int i = 0; i < S.VCap; i++) if (S.alive[i] == 1) total += S.area[i];
        var stack = new Stack<int>();
        for (int i = 0; i < S.VCap; i++)
        {
            if (S.alive[i] == 0 || seen[i] || S.thickness[i] <= lim) continue;
            double a = 0; stack.Push(i); seen[i] = true;
            while (stack.Count > 0)
            {
                int v = stack.Pop(); a += S.area[v];
                int c0 = S.vc[v], c = c0, n = 0;
                do { int j = S.V[MeshTopo.Next(c)]; if (!seen[j] && S.thickness[j] > lim) { seen[j] = true; stack.Push(j); } c = MeshTopo.Next(S.O[MeshTopo.Next(c)]); } while (c != c0 && ++n < 64);
            }
            if (a > 0.01 * total) count++;
        }
        return count;
    }

    // independent material check: thickness x barycentric area
    static double BaryVolume()
    {
        var A = new double[S.VCap];
        for (int t = 0; t < S.TCap; t++)
        {
            if (S.triAlive[t] == 0) continue;
            int a = S.V[3 * t], b = S.V[3 * t + 1], c = S.V[3 * t + 2];
            double ar = 0.5 * math.length(math.cross(S.pos[b] - S.pos[a], S.pos[c] - S.pos[a])) / 3.0;
            A[a] += ar; A[b] += ar; A[c] += ar;
        }
        double v = 0; for (int i = 0; i < S.VCap; i++) if (S.alive[i] == 1) v += S.thickness[i] * A[i];
        return v;
    }

    // ---------------------------------------------------------------- scenario setup
    static void SetupFracture(float radius)
    {
        // one circular continent over one hot cell; all other cells cold and fixed; feedback frozen by settings
        var s = S; var M = run.CurrentParams;
        var c0 = s.conv[0]; float3 centre = c0.pos;
        for (int k = 0; k < s.ConvCount; k++)
        {
            var c = s.conv[k];
            if (k == 0) { c.temp = c.baseTemp = 1f; c.feedback = 0f; }
            else { c.temp = c.baseTemp = -0.3f; c.feedback = 0f; }
            s.conv[k] = c;
        }
        var rng = new Unity.Mathematics.Random(7);
        for (int i = 0; i < s.VCap; i++)
        {
            if (s.alive[i] == 0) continue;
            float th = math.acos(math.clamp(math.dot(s.pos[i], centre), -1f, 1f));
            float sm = 1f - math.smoothstep(radius - 0.03f, radius + 0.03f, th);
            float t = M.P.oceanThick + (M.P.contThick - M.P.oceanThick) * sm;
            float age = sm > 0.5f ? 2000f + rng.NextFloat(-200f, 200f) : rng.NextFloat(0f, 100f);
            s.thickness[i] = t; s.volume[i] = t * s.area[i]; s.felsic[i] = s.volume[i] * sm; s.age[i] = age; s.damage[i] = 0f;
            float cf = MeshLimits.CrustType(t, s.volume[i], s.felsic[i], M); s.cfrac[i] = cf;
            s.density[i] = MeshLimits.Density(cf, age); s.stiffness[i] = MeshLimits.Stiffness(cf, age, 0f, 0f, M);
        }
        s.ledger[MeshLedger.Initial] = s.TotalVolume();
        s.ledger[MeshLedger.FelsicInitial] = s.TotalFelsic();
    }


    // ---------------------------------------------------------------- stage decomposition (diagnosis only)
    static readonly Dictionary<string, double[]> stageD = new Dictionary<string, double[]>();
    static double[] dbg = new double[4];
    static double[] Classes()
    {
        var P = run.CurrentParams.P; float lim = 0.75f * P.contThick;
        double all = 0, buoy = 0, cont = 0, felThin = 0, fel = 0;
        for (int i = 0; i < S.VCap; i++)
        {
            if (S.alive[i] == 0) continue;
            double v = S.volume[i]; all += v;
            float t = S.volume[i] / Math.Max(S.area[i], 1e-12f);
            if (S.cfrac[i] >= 0.5f) buoy += v;
            if (t > lim) cont += v;
            fel += S.felsic[i];
            if (S.cfrac[i] < 0.5f) felThin += S.felsic[i];
        }
        return new[] { all, buoy, cont, felThin, fel };
    }
    static void Acc(string k, double[] a, double[] b)
    {
        if (!stageD.TryGetValue(k, out var x)) stageD[k] = x = new double[5];
        for (int j = 0; j < 5; j++) x[j] += b[j] - a[j];
    }
    static void StagedStep()
    {
        var s = S; var M = run.CurrentParams; float dt = run.Dt; uint step = (uint)run.StepIndex; int cap = s.VCap;
        int passes = MeshStep.CouplePasses(run.Settings.constraintPasses, run.Settings.plateRigidityPasses);
        var c0 = Classes();
        MeshStep.Drive(s, M, dt).Run(cap); MeshStep.Velocity(s, M, dt).Run(cap);
        if (M.springOn) { MeshSpringSchedule.Schedule(s, M, run.Settings.strengthSolveIterations, run.Settings.strengthSolveTolerance, default); MeshStep.Blocks(s, M).Run(); }
        for (int p = 0; p < passes; p++) MeshStep.Couple(s, M, p, run.Settings.plateRigidityPasses).Run(cap);
        MeshStep.Move(s, M, dt).Run(cap); MeshStep.Area(s).Run(cap);
        // the area change alone, at constant volume, is "deformation"; Integrate's own sources/sinks come next
        for (int i = 0; i < cap; i++) if (s.alive[i] == 1) s.volumeTmp[i] = s.area[i];
        for (int i = 0; i < cap; i++) if (s.alive[i] == 1) s.area[i] = s.areaNew[i];
        var c1 = Classes(); Acc("1 deformation (reclassification by area change)", c0, c1);
        for (int i = 0; i < cap; i++) if (s.alive[i] == 1) s.area[i] = s.volumeTmp[i];
        var pre = new float[cap]; for (int i = 0; i < cap; i++) pre[i] = s.alive[i] == 1 ? s.volume[i] / Math.Max(s.area[i], 1e-12f) : 0f;
        MeshStep.Integrate(s, M, dt).Run(cap);
        for (int i = 0; i < cap; i++)
        {
            if (s.alive[i] == 0) continue;
            float cfPre = TMath.CrustFrac(pre[i], M.P), cfPost = TMath.CrustFrac(s.thickness[i], M.P);
            if (s.sinkSub[i] > 0f && cfPre >= 0.5f) dbg[0] += s.sinkSub[i];
            if (cfPre >= 0.5f && cfPost < 0.5f) dbg[1] += s.volume[i];
            if (cfPre < 0.5f && cfPost >= 0.5f) dbg[2] += s.volume[i];
            if (s.sinkSub[i] > 0f && s.subFrac[i] >= 0.999f) dbg[3] += 1;
        }
        var c2 = Classes(); Acc("2 integrate: squeeze sink + floor fill", c1, c2);
        if (M.erosionK > 0f) { MeshStep.Erosion(s, M, dt).Run(cap); MeshStep.ErosionApply(s, M).Run(cap); }
        var c3 = Classes(); Acc("3 erosion transport", c2, c3);
        var topo = MeshStep.Topology(s, M, false);
        topo.Begin();
        var t0 = Classes(); topo.PassRepair(); var t1 = Classes(); Acc("4a topo repair", t0, t1);
        topo.FlipPass(); var t2 = Classes(); Acc("4b topo flip", t1, t2);
        topo.PassCollisions(); var t3 = Classes(); Acc("4c topo collisions", t2, t3);
        topo.PassCollapse(); var t4 = Classes(); Acc("4d topo collapse", t3, t4);
        topo.PassSplit(); var t5 = Classes(); Acc("4e topo split+flip", t4, t5);
        topo.End();
        MeshStep.Sync(s, M).Run(cap);
        var c5 = Classes(); Acc("5 sync", t5, c5);
        MeshStep.Budget(s, M, dt, step).Run();
        var c6 = Classes(); Acc("6 budget (magmatism)", c5, c6);
        run.StepIndex++;
    }


    // ---------------------------------------------------------------- solver unit test
    static double MeanStrain(Func<int, float3> v)
    {
        double sum = 0; int n = 0;
        for (int i = 0; i < S.VCap; i++)
        {
            if (S.alive[i] == 0) continue;
            int c0 = S.vc[i], c = c0, g = 0;
            do
            {
                int j = S.V[MeshTopo.Next(c)];
                float3 d = S.pos[j] - S.pos[i]; float len = math.length(d);
                sum += Math.Abs(math.dot(v(j) - v(i), d / len) / len); n++;
                c = MeshTopo.Next(S.O[MeshTopo.Next(c)]);
            } while (c != c0 && ++g < 64);
        }
        return sum / n;
    }
    static void SolverTest(int iters, float tol)
    {
        var s = S; var M = run.CurrentParams; int cap = s.VCap;
        for (int i = 0; i < cap; i++)
        {
            if (s.alive[i] == 0) continue;
            float t = 35f; s.volume[i] = t * s.area[i]; s.felsic[i] = s.volume[i]; s.thickness[i] = t; s.age[i] = 2000f; s.damage[i] = 0f;
            s.cfrac[i] = 1f; s.density[i] = MeshLimits.Density(1f, 2000f); s.vel[i] = 0f;
        }
        float3 axis = math.normalize(new float3(0.3f, 1f, 0.2f)), c = math.normalize(new float3(1f, 0.2f, -0.4f));
        foreach (var name in new[] { "rigid rotation", "divergent (hot cell)", "rotation + divergent" })
        {
            for (int i = 0; i < cap; i++)
            {
                if (s.alive[i] == 0) continue;
                float3 p = s.pos[i];
                float3 rot = math.cross(axis, p) * 0.003f;
                float3 div = MeshDriveJob.CellDrive(p, c, 1f, M.P.convRadius, out _) * 0.003f;
                s.velNext[i] = name.StartsWith("rigid") ? rot : name.StartsWith("div") ? div : rot + div;
                s.vel[i] = 0f;
            }
            var d = s.velNext.ToArray();
            MeshSpringSchedule.Schedule(s, M, iters, tol, default);
            double sd = MeanStrain(j => d[j]), sv = MeanStrain(j => s.velNext[j]);
            double rotKeep = 0, rotN = 0;
            for (int i = 0; i < cap; i++) if (s.alive[i] == 1) { rotKeep += math.dot(s.velNext[i], math.cross(axis, s.pos[i])); rotN += math.lengthsq(math.cross(axis, s.pos[i])); }
            Console.WriteLine($"# {name,-22}: drive strain {sd:E3}  solved strain {sv:E3}  ratio {sv / Math.Max(sd, 1e-30):F4}   rotation component kept {rotKeep / rotN / 0.003:F3}   rz/rz0 {s.springScalars[MeshSpring.Rz] / Math.Max(s.springScalars[MeshSpring.Rz0], 1e-30):E2}");
        }
    }

    static void Snapshot(string path)
    {
        using var w = new StreamWriter(path);
        w.WriteLine("lon,lat,thick,felsic,damage,stress,ach,block,age,speed,cf");
        for (int i = 0; i < S.VCap; i++)
        {
            if (S.alive[i] == 0) continue;
            float3 p = S.pos[i];
            float lon = math.degrees(math.atan2(p.z, p.x)), lat = math.degrees(math.asin(math.clamp(p.y, -1f, 1f)));
            w.WriteLine(string.Join(",", new[] { lon, lat, S.thickness[i], S.felsic[i] / Math.Max(S.volume[i], 1e-20f), S.damage[i], S.tension[i], S.achieved[i], (float)S.blockId[i], S.age[i], math.length(S.vel[i]) / run.Settings.velocityReference, S.cfrac[i] }.Select(F)));
        }
    }
    // convection tracking: cumulative cell travel (rad) and temperature sign reversals
    static float3[] convPrev; static float[] convTPrev; static double convTravel; static int convFlips;
    static void TrackConvection()
    {
        int n = S.ConvCount;
        if (convPrev == null) { convPrev = new float3[n]; convTPrev = new float[n]; for (int k = 0; k < n; k++) { convPrev[k] = S.conv[k].pos; convTPrev[k] = S.conv[k].temp; } return; }
        for (int k = 0; k < n; k++)
        {
            var c = S.conv[k];
            convTravel += math.acos(math.clamp(math.dot(c.pos, convPrev[k]), -1f, 1f));
            if (math.sign(c.temp) != math.sign(convTPrev[k]) && math.abs(c.temp - convTPrev[k]) > 0.2f) convFlips++;
            convPrev[k] = c.pos; convTPrev[k] = c.temp;
        }
    }
    static string F(float x) => x.ToString("G5", CultureInfo.InvariantCulture);

    public static int Main(string[] args)
    {
        var st = MeshSettings.D;
        st.targetVertexCount = 5000;
        float T = 4000f, every = 100f; string outPath = null; string scenario = "normal"; int tracerCount = 300;
        bool validate = false, staged = false; string snapPrefix = null; float snapEvery = 0f;
        foreach (var a in args)
        {
            var kv = a.Split('='); string k = kv[0], v = kv.Length > 1 ? kv[1] : "1";
            float fv() => float.Parse(v, CultureInfo.InvariantCulture);
            switch (k)
            {
                case "n": st.targetVertexCount = int.Parse(v); break;
                case "T": T = fv(); break;
                case "every": every = fv(); break;
                case "seed": st.seed = int.Parse(v); break;
                case "dt": st.stepMyr = fv(); break;
                case "seeded": st.uniformInitialCrust = false; break;
                case "uniform": st.uniformInitialCrust = true; break;
                case "out": outPath = v; break;
                case "scenario": scenario = v; break;
                case "validate": validate = true; break;
                case "staged": staged = true; break;
                case "snap": snapPrefix = v; break;
                case "snapEvery": snapEvery = fv(); break;
                // process toggles
                case "nosinks": st.enableSubductionSink = false; st.erosionRecycleFraction = 0f; break;
                case "nosources": st.enableRifting = false; st.minimumCrustKm = 0f; st.enableMagmatism = false; break;
                case "noerosion": st.erosionDiffusivityKm2PerMyr = 0f; break;
                case "nomagma": st.enableMagmatism = false; break;
                case "nospread": st.gravitySpreading = 0f; break;
                // any MeshSettings field by name, e.g. extensionYield=0.3
                default:
                {
                    var f = typeof(MeshSettings).GetField(k);
                    if (f == null) { Console.Error.WriteLine($"unknown arg {k}"); return 2; }
                    object boxed = st;
                    if (f.FieldType == typeof(float)) f.SetValue(boxed, fv());
                    else if (f.FieldType == typeof(int)) f.SetValue(boxed, int.Parse(v));
                    else if (f.FieldType == typeof(bool)) f.SetValue(boxed, v == "1" || v == "true");
                    st = (MeshSettings)boxed;
                    break;
                }
            }
        }
        if (scenario == "fracture")
        {
            st.uniformInitialCrust = false; st.insulationHeat = 0f; st.surfaceCooling = 0f; st.subductionCool = 0f;
            st.temperatureNoise = 0f; st.convectionDrift = 0f; st.feedbackRelaxPerMyr = 0f;
        }

        run = new MeshRun();
        var sw = System.Diagnostics.Stopwatch.StartNew();
        run.Begin(st);
        if (scenario == "fracture") SetupFracture(0.75f);
        if (scenario == "solvertest") { SolverTest(run.Settings.strengthSolveIterations, run.Settings.strengthSolveTolerance); return 0; }
        InitTracers(tracerCount, (uint)st.seed);
        double barry0 = BaryVolume();
        var w = outPath != null ? new StreamWriter(outPath) : null;
        string header = "t,crust,expected,residual_rel,bary_rel,contArea,buoyArea,contVol,buoyVol,meanContT,maxT,rift,floor,magma,subducted,eroRec,eroMoved,reservoir,buoySub,delam," +
                        "dmg,contDmg,contFrac,stress,contStress,contAch,speed,contSpeed,Tmin,Tmax,Tabs,distortion,onCont,cluster,local,landmasses,convTravel,convFlips,felsic,felsicRes,felsicMag,felsicSub,felsicArea,felsicMeanT,felsicThin,verts,splits,rifts,collapses,subductedN,inverted,ms";
        w?.WriteLine(header);
        Console.WriteLine($"# {scenario} {(st.uniformInitialCrust ? "uniform" : "seeded")} n={st.targetVertexCount} seed={st.seed} dt={run.Dt:F3} L={run.L:F4} T={T}");
        Console.WriteLine("#   t    crust  resid%   contA%  contV  buoyA%  meanCT  maxT   rift  magma   subd  floor  cDmg  cFrac cStress cAch  cSpeed  Tmin  Tmax  dist  onC  clus  loc  LM | felsic vol, area, meanT, in thin crust, +magmatic, -subducted");
        int steps = (int)Math.Ceiling(T / run.Dt);
        int reportEvery = Math.Max(1, (int)Math.Round(every / run.Dt));
        string valErr = null;
        void Report()
        {
            var d = run.Diagnostics();
            var (dist, onC) = TracerStats();
            float clus = Clustering(), loc = Localisation(); int lm = Landmasses();
            double bary = BaryVolume();
            float rel = d.crustVolume > 0 ? d.residual / d.crustVolume : 0f;
            float baryRel = (float)((bary - S.TotalVolume()) / S.TotalVolume());
            var c = S.counters;
            w?.WriteLine(string.Join(",", new[] { d.timeMyr, d.crustVolume / 1e9f, d.expectedVolume / 1e9f, rel, baryRel, d.continentalAreaFrac, d.buoyantAreaFrac,
                d.continentalVolume / 1e9f, d.buoyantVolume / 1e9f, d.meanContinentalThickness, d.maxThickness, d.riftCrust / 1e9f, d.floorFill / 1e9f, d.magmatic / 1e9f,
                d.subducted / 1e9f, d.erosionRecycled / 1e9f, d.erosionMoved / 1e9f, d.reservoir / 1e9f, d.buoyantSubducted / 1e9f, d.delaminated / 1e9f,
                d.meanDamage, d.continentalMeanDamage, d.continentalFracturedFrac, d.meanStress, d.continentalStress, d.continentalAchieved,
                d.meanSpeed, d.continentalSpeed, d.minConvTemp, d.maxConvTemp, d.meanAbsConvTemp, dist, onC, clus, loc, lm, (float)convTravel, convFlips,
                d.felsicVolume / 1e9f, d.felsicResidual / 1e9f, d.felsicMagmatic / 1e9f, d.felsicSubducted / 1e9f, d.felsicAreaFrac, d.felsicMeanThickness, d.felsicInThin / 1e9f,
                c[MeshTopo.AliveV], c[MeshTopo.Splits], c[MeshTopo.Rifts], c[MeshTopo.Collapses], c[MeshTopo.Subducted], c[MeshTopo.Inverted], (float)sw.ElapsedMilliseconds }
                .Select(x => F((float)x))));
            w?.Flush();
            Console.WriteLine($"{d.timeMyr,6:F0} {d.crustVolume / 1e9f,7:F3} {100 * rel,7:F4} {100 * d.continentalAreaFrac,6:F1} {d.continentalVolume / 1e9f,6:F3} {100 * d.buoyantAreaFrac,6:F1} {d.meanContinentalThickness,6:F1} {d.maxThickness,5:F0} " +
                              $"{d.riftCrust / 1e9f,6:F2} {d.magmatic / 1e9f,6:F2} {d.subducted / 1e9f,6:F2} {d.floorFill / 1e9f,6:F2} {d.continentalMeanDamage,5:F3} {d.continentalFracturedFrac,5:F3} {d.continentalStress,6:F3} {d.continentalAchieved,5:F3} " +
                              $"{d.continentalSpeed * 1e3f,6:F3} {d.minConvTemp,5:F2} {d.maxConvTemp,5:F2} {dist,5:F3} {onC,4:F2} {clus,5:F2} {loc,4:F2} {lm,3} | fel {d.felsicVolume / 1e9f,6:F3} {100 * d.felsicAreaFrac,5:F1}% {d.felsicMeanThickness,5:F1}km thin {d.felsicInThin / 1e9f,5:F2} +{d.felsicMagmatic / 1e9f,5:F2} -{d.felsicSubducted / 1e9f,5:F2}");
        }
        Report();
        TrackConvection();
        if (snapPrefix != null) Snapshot($"{snapPrefix}_00000.csv");
        for (int k = 0; k < steps; k++)
        {
            if (staged) StagedStep(); else run.ScheduleSteps(1, default);
            AdvectTracers(run.Dt);
            if ((k + 1) % Math.Max(1, (int)(25f / run.Dt)) == 0) TrackConvection();
            if (snapPrefix != null && snapEvery > 0f && (int)(run.TimeMyr / snapEvery) != (int)((run.TimeMyr - run.Dt) / snapEvery))
                Snapshot($"{snapPrefix}_{(int)(Math.Round(run.TimeMyr / snapEvery) * snapEvery):D5}.csv");
            if ((k + 1) % reportEvery == 0 || k == steps - 1)
            {
                Report();
                if (validate) { valErr = S.Validate(0f, out _); if (valErr != null) { Console.WriteLine("VALIDATION FAILED\n" + valErr); break; } }
            }
        }
        if (scenario == "normal" || true)
        {
            var conv = string.Join(" ", Enumerable.Range(0, S.ConvCount).Select(k => $"{S.conv[k].temp:F2}"));
            Console.WriteLine($"# final cell temps: {conv}");
        }
        var l = S.ledger; double r2 = R2 / 1e9;
        Console.WriteLine($"# ledger (1e9 km^3): initial {l[MeshLedger.Initial] * r2:F4}  +rift {l[MeshLedger.RiftCrust] * r2:F4}  +floor {l[MeshLedger.FloorFill] * r2:F4}  +magma {l[MeshLedger.Magmatic] * r2:F4}" +
                          $"  -squeeze {l[MeshLedger.SubductedSqueeze] * r2:F4}  -slabCollapse {l[MeshLedger.SubductedCollapse] * r2:F4}  -erosionRec {l[MeshLedger.ErosionRecycled] * r2:F4}  -delam {l[MeshLedger.Delaminated] * r2:F4}" +
                          $"  (charged {l[MeshLedger.MagmaCharged] * r2:F4}, buoyantSubducted {l[MeshLedger.BuoyantSubducted] * r2:F4}, eroded moved {l[MeshLedger.ErosionTransported] * r2:F4})");
        Console.WriteLine($"# spring solve: residual rz/rz0 = {(S.springScalars[MeshSpring.Rz0] > 0 ? S.springScalars[MeshSpring.Rz] / S.springScalars[MeshSpring.Rz0] : 0):E2}");
        Console.WriteLine($"# crust {S.TotalVolume() * r2:F6}  expected {MeshLedger.Expected(l) * r2:F6}  residual {(S.TotalVolume() + S.InTransit() - MeshLedger.Expected(l)) * r2:E3}  felsic residual {(S.TotalFelsic() + S.InTransit() - MeshLedger.ExpectedFelsic(l)) * r2:E3}  relaminated {l[MeshLedger.Relaminated] * r2:F3}  " +
                          $"thickness x barycentric-area total differs by {100 * (BaryVolume() / S.TotalVolume() - 1):F2}%   wall {sw.Elapsed.TotalSeconds:F0}s");
        if (staged)
        {
            Console.WriteLine("# stage decomposition (1e9 km^3):                      all       buoyant(cf>=0.5)   continental(t>26)  felsicInThin   felsic");
            Console.WriteLine($"  dbg: sink on buoyant(pre) {dbg[0] * r2:F3}  buoy->non {dbg[1] * r2:F3}  non->buoy {dbg[2] * r2:F3}  full-subFrac sink events {dbg[3]}");
            foreach (var kv in stageD.OrderBy(x => x.Key)) Console.WriteLine($"  {kv.Key,-52} {kv.Value[0] * r2,9:F3} {kv.Value[1] * r2,12:F3} {kv.Value[2] * r2,12:F3} {kv.Value[3] * r2,12:F3} {kv.Value[4] * r2,9:F3}");
        }
        if (validate) Console.WriteLine(valErr == null ? "# validation OK" : "# validation FAILED");
        w?.Close();
        run.Dispose();
        return valErr == null ? 0 : 1;
    }
}
