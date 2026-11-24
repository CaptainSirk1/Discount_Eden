using UnityEditor;
using UnityEngine;

namespace PlanetGen.Tectonics.MeshSim.EditorTools
{
    [CustomEditor(typeof(PlanetTectonicsMesh))]
    public class PlanetTectonicsMeshEditor : UnityEditor.Editor
    {
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

                    if (t.IsAnimating)
                    {
                        bool behind = t.EffectiveMyrPerSecond < t.animMyrPerSecond * 0.9f;
                        EditorGUILayout.LabelField($"Running at {t.EffectiveMyrPerSecond:F0} Myr/s" +
                                                   (behind ? "  (sim-limited)" : ""), mini);
                    }
                    if (!string.IsNullOrEmpty(t.LastValidation))
                        EditorGUILayout.HelpBox(t.LastValidation, t.LastValidation.StartsWith("OK") ? MessageType.Info : MessageType.Error);
                }
            }

            EditorGUILayout.Space(4);
            DrawDefaultInspector();
        }
    }
}
