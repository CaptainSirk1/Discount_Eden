using UnityEditor;
using UnityEngine;

namespace PlanetGen.Tectonics.EditorTools
{
    [CustomEditor(typeof(PlanetTectonics))]
    public class PlanetTectonicsEditor : UnityEditor.Editor
    {
        public override bool RequiresConstantRepaint() => ((PlanetTectonics)target).IsAnimating;

        public override void OnInspectorGUI()
        {
            var t = (PlanetTectonics)target;

            EditorGUILayout.Space(2);
            using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
            {
                EditorGUILayout.LabelField("Simulation", EditorStyles.boldLabel);

                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button(t.HasSimulation ? "Restart" : "Animate", GUILayout.Height(24)))
                        t.StartAnimation();

                    using (new EditorGUI.DisabledScope(!t.HasSimulation || t.IsFinished))
                    {
                        if (t.IsAnimating)
                        {
                            if (GUILayout.Button("Pause", GUILayout.Height(24))) t.Pause();
                        }
                        else if (GUILayout.Button("Resume", GUILayout.Height(24))) t.Resume();

                        if (GUILayout.Button("Step", GUILayout.Height(24), GUILayout.Width(50))) t.StepOnce();
                        if (GUILayout.Button("Finish", GUILayout.Height(24), GUILayout.Width(56))) t.FinishNow();
                    }
                }

                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button("Generate (instant)")) t.Generate();
                    if (GUILayout.Button("Clear", GUILayout.Width(60))) t.Clear();
                }

                Rect r = GUILayoutUtility.GetRect(18, 18, GUILayout.ExpandWidth(true));
                string label = t.HasSimulation
                    ? $"{t.CurrentTimeMyr:F0} / {t.totalTimeMyr:F0} Myr   ·   step {t.StepIndex}/{t.TotalSteps}   ·   {t.AliveCount} pts"
                    : "No simulation";
                EditorGUI.ProgressBar(r, t.Progress, label);

                if (t.HasSimulation)
                    EditorGUILayout.LabelField($"+{t.SpawnedCount} spawned   -{t.SubductedCount} subducted   ·   dt {t.LastStepMyr:F2} Myr",
                        EditorStyles.miniLabel);
            }

            EditorGUILayout.Space(4);
            DrawDefaultInspector();
        }
    }
}
