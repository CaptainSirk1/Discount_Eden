using UnityEngine;

[CreateAssetMenu]
public class ClimateSettings : ScriptableObject
{
    [Header("Atmospheric Composition")]
    [Tooltip("Dry gas ratios. These will be normalized automatically — they don't need to sum to 1.")]
    public AtmosphericData atmosphere;

    [Header("Climate Inputs")]
    [Tooltip("The maximum possible depth of the oceans if all water is liquid.")]
    [Range(0.1f, 1f)] public float globalMoisture = 0.5f;

    [Tooltip("0.0 = Absolute Zero, 0.5 = Freezing, 1.0 = Boiling, 2.0 = Scorching")]
    [Range(0f, 2f)] public float averageTemp = 1.5f;

    [Header("Live Preview")]
    public ClimateState preview;

    // Editor-only preview so the designer can see derived values while authoring.
    // ClimateState handles all of this at runtime — this never runs in a build.
    #if UNITY_EDITOR
    private void OnValidate()
    {
        this.preview = new ClimateState(this);
        UnityEditor.EditorUtility.SetDirty(this);
    }
    #endif
}