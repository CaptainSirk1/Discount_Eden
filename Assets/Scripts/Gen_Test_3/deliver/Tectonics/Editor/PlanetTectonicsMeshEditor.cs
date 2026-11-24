using UnityEditor;
using UnityEngine;

namespace PlanetGen.Tectonics.MeshSim.EditorTools
{
    [CustomEditor(typeof(PlanetTectonicsMesh))]
    public class PlanetTectonicsMeshEditor : UnityEditor.Editor
    {
        static bool _showLedger = true, _showMechanics = true, _showTrends = true;

        public override bool RequiresConstantRepaint() => ((PlanetTectonicsMesh)target).IsAnimating;

        public override void OnInspectorGUI()
        {
            var t = (PlanetTectonicsMesh)target;

            EditorGUILayout.Space(2);
            using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
            {
                EditorGUILayout.LabelField("Simulation", EditorStyles.boldLabel);

                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button(t.HasSimulation ? "Restart" : "Animate", GUILayout.Height(24))) t.StartAnimation();
                    using (new EditorGUI.DisabledScope(!t.HasSimulation || t.IsFinished))
                    {
                        if (t.IsAnimating) { if (GUILayout.Button("Pause", GUILayout.Height(24))) t.Pause(); }
                        else if (GUILayout.Button("Resume", GUILayout.Height(24))) t.Resume();
                        if (GUILayout.Button("Step", GUILayout.Height(24), GUILayout.Width(50))) t.StepOnce();
                        if (GUILayout.Button("Finish", GUILayout.Height(24), GUILayout.Width(56))) t.FinishNow();
                    }
                }
                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button("Generate (instant)")) t.Generate();
                    using (new EditorGUI.DisabledScope(!t.HasSimulation))
                        if (GUILayout.Button("Validate", GUILayout.Width(70))) t.ValidateMesh();
                    if (GUILayout.Button("Clear", GUILayout.Width(60))) t.Clear();
                }

                Rect r = GUILayoutUtility.GetRect(18, 18, GUILayout.ExpandWidth(true));
                string label = t.HasSimulation
                    ? $"{t.CurrentTimeMyr:F0} / {t.totalTimeMyr:F0} Myr   ·   step {t.StepIndex}/{t.TotalSteps}"
                    : "No simulation";
                EditorGUI.ProgressBar(r, t.Progress, label);

                if (t.HasSimulation)
                {
                    var mini = EditorStyles.miniLabel;
                    EditorGUILayout.LabelField(
                        $"{t.VertexCount} verts / {t.TriangleCount} tris   ·   capacity {t.VertexCapacity}   ·   dt {t.LastStepMyr:F2} Myr", mini);
                    EditorGUILayout.LabelField(
                        $"+{t.TotalSplits} splits ({t.TotalRifts} rift)   -{t.TotalCollapses} collapses ({t.TotalSubducted} subducted)   ·   {t.TotalFlips} flips", mini);
                    EditorGUILayout.LabelField(
                        $"per Myr: {t.SplitsPerMyr:F1} split  {t.CollapsesPerMyr:F1} collapse  {t.FlipsPerMyr:F1} flip", mini);

                    bool capOk = t.MaxEdgeRatio <= t.maxEdgeLength + 1e-3f;
                    var old = GUI.color;
                    if (!capOk || t.InvertedTriangles > 0 || t.CapacityHits > 0) GUI.color = new Color(1f, 0.75f, 0.4f);
                    EditorGUILayout.LabelField(
                        $"edges {t.MinEdgeRatio:F2} – {t.MaxEdgeRatio:F2} x target (cap {t.maxEdgeLength:F2})   ·   " +
                        $"{t.InvertedTriangles} inverted   ·   {t.CapacityHits} capacity hits", mini);
                    GUI.color = old;

                    var dg = t.Diagnostics;
                    EditorGUILayout.LabelField(
                        $"global thermal age {dg.globalThermalAge:F0} Myr   ·   continents {dg.continentalAreaFrac * 100f:F1}% of surface, " +
                        $"buoyant {dg.buoyantAreaFrac * 100f:F1}%   ·   mean continental thickness {dg.meanContinentalThickness:F1} km", mini);

                    _showLedger = EditorGUILayout.Foldout(_showLedger, "Material ledger (10⁹ km³, cumulative)", true);
                    if (_showLedger)
                    {
                        float rel = dg.crustVolume > 0f ? dg.residual / dg.crustVolume : 0f;
                        GUI.color = Mathf.Abs(rel) > 1e-4f ? new Color(1f, 0.6f, 0.4f) : old;
                        EditorGUILayout.LabelField($"crust {dg.crustVolume / 1e9f:F3}   expected {dg.expectedVolume / 1e9f:F3}   " +
                                                   $"numerical residual {rel * 100f:F5}%", mini);
                        GUI.color = old;
                        EditorGUILayout.LabelField($"+ rift {dg.riftCrust / 1e9f:F2}   + floor {dg.floorFill / 1e9f:F2}   + magmatic {dg.magmatic / 1e9f:F2}   " +
                                                   $"(reservoir {dg.reservoir / 1e9f:F2})", mini);
                        EditorGUILayout.LabelField($"- subducted {dg.subducted / 1e9f:F2}   - delaminated {dg.delaminated / 1e9f:F2}   " +
                                                   $"- erosion recycled {dg.erosionRecycled / 1e9f:F2}   (eroded & moved {dg.erosionMoved / 1e9f:F2})", mini);
                        EditorGUILayout.LabelField($"continental volume {dg.continentalVolume / 1e9f:F3}   buoyant {dg.buoyantVolume / 1e9f:F3}", mini);
                        EditorGUILayout.LabelField($"felsic {dg.felsicVolume / 1e9f:F3} (+{dg.felsicMagmatic / 1e9f:F2} arcs, -{dg.felsicSubducted / 1e9f:F2} recycled, " +
                                                   $"relaminated {dg.relaminated / 1e9f:F2})   felsic crust {dg.felsicAreaFrac * 100f:F1}% at {dg.felsicMeanThickness:F1} km", mini);
                    }

                    _showMechanics = EditorGUILayout.Foldout(_showMechanics, "Mechanics", true);
                    if (_showMechanics)
                    {
                        EditorGUILayout.LabelField(
                            $"damage mean {dg.meanDamage:F3} max {dg.maxDamage:F2}   ·   continents: mean {dg.continentalMeanDamage:F3}, " +
                            $"{dg.continentalFracturedFrac * 100f:F1}% fractured", mini);
                        EditorGUILayout.LabelField(
                            $"stress mean {dg.meanStress:F3}, continental {dg.continentalStress:F3}   ·   continental achieved opening {dg.continentalAchieved:F2}", mini);
                        EditorGUILayout.LabelField(
                            $"speed mean {dg.meanSpeed / t.velocityReference:F2}, continental {dg.continentalSpeed / t.velocityReference:F2} x velocityReference   ·   " +
                            $"cells {dg.minConvTemp:F2} .. {dg.maxConvTemp:F2} (mean |T| {dg.meanAbsConvTemp:F2})   ·   continental vertices subducted {t.ContinentalSubducted}", mini);
                    }

                    _showTrends = EditorGUILayout.Foldout(_showTrends, "Trends over geological time", true);
                    if (_showTrends && t.History.Count > 1)
                    {
                        Graph("continental volume (white) · felsic (sand) · crust (grey), 10⁹ km³", t,
                            d => d.continentalVolume / 1e9f, Color.white, d => d.felsicVolume / 1e9f, new Color(0.9f, 0.75f, 0.45f),
                            d => d.crustVolume / 1e9f, new Color(0.6f, 0.6f, 0.6f));
                        Graph("continental area (white) · buoyant area (blue), fraction", t,
                            d => d.continentalAreaFrac, Color.white, d => d.buoyantAreaFrac, new Color(0.4f, 0.7f, 1f), null, Color.clear);
                        Graph("mean speed (green) · continental damage (red), relative", t,
                            d => d.meanSpeed / Mathf.Max(t.velocityReference, 1e-6f), new Color(0.4f, 1f, 0.5f),
                            d => d.continentalMeanDamage, new Color(1f, 0.4f, 0.3f), null, Color.clear);
                    }

                    if (t.IsAnimating)
                    {
                        bool behind = t.EffectiveMyrPerSecond < t.animMyrPerSecond * 0.9f;
                        EditorGUILayout.LabelField($"Running at {t.EffectiveMyrPerSecond:F0} Myr/s" + (behind ? "  (sim-limited)" : ""), mini);
                    }
                    if (!string.IsNullOrEmpty(t.LastValidation))
                        EditorGUILayout.HelpBox(t.LastValidation, t.LastValidation.StartsWith("OK") ? MessageType.Info : MessageType.Error);
                }
            }

            EditorGUILayout.Space(4);
            DrawDefaultInspector();
        }

        /// <summary>Up to three series over the run's diagnostics history, each scaled to the shared max.</summary>
        static void Graph(string title, PlanetTectonicsMesh t,
            System.Func<MeshDiagnostics, float> a, Color ca, System.Func<MeshDiagnostics, float> b, Color cb,
            System.Func<MeshDiagnostics, float> c, Color cc)
        {
            EditorGUILayout.LabelField(title, EditorStyles.miniLabel);
            Rect r = GUILayoutUtility.GetRect(100, 54, GUILayout.ExpandWidth(true));
            EditorGUI.DrawRect(r, new Color(0.13f, 0.13f, 0.13f));
            var h = t.History;
            float max = 1e-6f, tMax = Mathf.Max(h[h.Count - 1].timeMyr, 1f);
            foreach (var d in h)
            {
                max = Mathf.Max(max, a(d));
                if (b != null) max = Mathf.Max(max, b(d));
                if (c != null) max = Mathf.Max(max, c(d));
            }
            Line(r, h, a, ca, max, tMax);
            if (b != null) Line(r, h, b, cb, max, tMax);
            if (c != null) Line(r, h, c, cc, max, tMax);
            GUI.Label(new Rect(r.x + 2, r.y, 80, 14), max.ToString("G3"), EditorStyles.miniLabel);
        }

        static void Line(Rect r, System.Collections.Generic.IReadOnlyList<MeshDiagnostics> h, System.Func<MeshDiagnostics, float> f, Color col, float max, float tMax)
        {
            var pts = new Vector3[h.Count];
            for (int i = 0; i < h.Count; i++)
                pts[i] = new Vector3(r.x + r.width * h[i].timeMyr / tMax, r.yMax - r.height * Mathf.Clamp01(f(h[i]) / max), 0f);
            Handles.color = col;
            Handles.DrawAAPolyLine(2f, pts);
        }
    }
}
