using UnityEngine;

[CreateAssetMenu]
public class ClimateSettings : ScriptableObject
{
    [Header("Atmospheric Composition")]
    [Tooltip("Dry gas ratios. These will be normalized automatically — they don't need to sum to 1.")]
    public AtmosphericData atmosphere;

    [Tooltip("0.0 = Absolute Zero, 0.5 = Freezing, 1.0 = Boiling, 2.0 = Scorching")]
    [Range(0f, 2f)] public float averageTemp = 1.5f;

    [Header("Live Preview")]
    public ClimateState preview;
}