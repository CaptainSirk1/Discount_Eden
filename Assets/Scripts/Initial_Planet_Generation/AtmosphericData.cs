using UnityEngine;

[System.Serializable]
public class AtmosphericData
{
    [Range(0f, 1f)] public float nitrogen = 0.65f;
    [Range(0f, 1f)] public float carbonDioxide = 0.25f;
    [Range(0f, 1f)] public float methane = 0.02f;
    [Range(0f, 1f)] public float oxygen = 0.01f;
    [Range(0f, 1f)] public float traceElements = 0.01f;

    // These are to be written by ClimateState — never set directly.
    [HideInInspector] public float finalWaterVapor;
    [HideInInspector] public float finalNitrogen;
    [HideInInspector] public float finalCarbonDioxide;
    [HideInInspector] public float finalMethane;
    [HideInInspector] public float finalOxygen;
    [HideInInspector] public float finalTraceElements;

    // Normalizes dry sliders into final* fields without touching the sliders.
    public void BalanceDryRecipe()
    {
        float sum = nitrogen + carbonDioxide + methane + oxygen + traceElements;
        if (sum <= 0) return;
        finalNitrogen = nitrogen / sum;
        finalCarbonDioxide = carbonDioxide / sum;
        finalMethane = methane / sum;
        finalOxygen = oxygen / sum;
        finalTraceElements = traceElements / sum;
    }

    // Scales normalized dry values to make room for water vapor.
    // Always call BalanceDryRecipe() before this.
    public void CalculateFinalState(float vaporPercent)
    {
        finalWaterVapor = vaporPercent;
        float scale = 1f - vaporPercent;
        finalNitrogen *= scale;
        finalCarbonDioxide *= scale;
        finalMethane *= scale;
        finalOxygen *= scale;
        finalTraceElements *= scale;
    }
}