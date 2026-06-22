using UnityEngine;
using UnityEditor;

// This script is for letting you adjust the scriptable object settings your planet object pulls from directly in the inspector.
// Anything you change will immediately show on your planet object.
[CustomEditor(typeof(PlanetGenerator))]
public class PlanetEditor : Editor
{
    PlanetGenerator planet;
    Editor terrainEditor;
    Editor climateEditor;

    public override void OnInspectorGUI()
    {
        // Toolbar gets its own change check — switching maps only applies the texture,
        // it doesn't need to regenerate anything
        EditorGUILayout.LabelField("Map Visualization", EditorStyles.boldLabel);
        EditorGUI.BeginChangeCheck();
        planet.displayMap = (PlanetGenerator.MapType)GUILayout.Toolbar(
            (int)planet.displayMap, 
            System.Enum.GetNames(typeof(PlanetGenerator.MapType))
        );
        if (EditorGUI.EndChangeCheck())
        {
            planet.ApplyDisplayMap();
            return;
        }

        EditorGUILayout.Space();

        // Everything else triggers full regeneration if changed
        using (var check = new EditorGUI.ChangeCheckScope())
        {
            base.OnInspectorGUI();
            if (check.changed)
                planet.GeneratePlanet();
        }

        if (GUILayout.Button("GeneratePlanet"))
            planet.GeneratePlanet();
        
        DrawSettingsEditor(planet.terrainSettings, planet.OnShapeSettingsUpdated, ref planet.terrainSettingsFoldout, ref terrainEditor);
        DrawSettingsEditor(planet.climateSettings, planet.OnColorSettingsUpdated, ref planet.climateSettingsFoldout, ref climateEditor);
    }

    void DrawSettingsEditor(Object settings, System.Action onSettingsUpdated, ref bool foldout, ref Editor editor)
    {
        foldout = EditorGUILayout.InspectorTitlebar(foldout, settings);
        using (var check = new EditorGUI.ChangeCheckScope())
        {
            if (settings != null)
            {
                if (foldout)
                {
                    CreateCachedEditor(settings, null, ref editor);
                    editor.OnInspectorGUI();

                    if (check.changed)
                    {
                        if (onSettingsUpdated != null)
                        {
                            onSettingsUpdated();
                        }
                    }
                }   
            } 
        }
    }

    private void OnEnable()
    {
        planet = (PlanetGenerator)target;
    }
}