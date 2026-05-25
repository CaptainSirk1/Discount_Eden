using UnityEngine;

//Determines global settings. Maximum sea level, average temperature, ect...


[System.Serializable] //Atmospheric Percentages. Increasing one percentage must decrease one or all of the others.
public class AtmosphericData
{
    [Range(0f, 1f)] public float nitrogen = 0.65f;
    [Range(0f, 1f)] public float carbonDioxide = 0.25f;
    [Range(0f, 1f)] public float methane = 0.02f;
    [Range(0f, 1f)] public float oxygen = 0.01f;
    [Range(0f, 1f)] public float traceElements = 0.01f;

    [Header("Final Calculated Composition")]
    public float finalWaterVapor;
    public float finalNitrogen;
    public float finalCarbonDioxide;
    public float finalMethane;
    public float finalOxygen;
    public float finalTraceElements;

    // This method balances the DRY sliders ONLY.
    // It doesn't care about water vapor yet.
    public void BalanceDryRecipe()
    {
        float sum = nitrogen + carbonDioxide + methane + oxygen + traceElements;
        if (sum > 0)
        {
            nitrogen /= sum;
            carbonDioxide /= sum;
            methane /= sum;
            oxygen /= sum;
            traceElements /= sum;
        }
    }

    // This method calculates the "Wet" atmosphere for the actual game logic.
    // It DOES NOT change the slider values.
    public void CalculateFinalState(float vaporPercent)
    {
        finalWaterVapor = vaporPercent;
        float displacementMultiplier = 1f - finalWaterVapor;

        // The "Dry" sliders remain untouched, we just scale their output
        finalNitrogen = nitrogen * displacementMultiplier;
        finalCarbonDioxide = carbonDioxide * displacementMultiplier;
        finalMethane = methane * displacementMultiplier;
        finalOxygen = oxygen * displacementMultiplier;
        finalTraceElements = traceElements * displacementMultiplier;
        // ... scale the others for the generator ...
    }
}

//Establishes planetary presets that will inform procedural generation.
[CreateAssetMenu(fileName = "PlanetSettings", menuName = "Scriptable Objects/PlanetSettings")]
public class PlanetSettings : ScriptableObject
{
    //Adjustable Terrain Presets that will change how the planet looks given the same seed
    [Header("Global Terrain Presets")]
    
    [Tooltip("The maximum possible depth of the oceans if all water is liquid.")]
    [Range(0.1f, 1f)] public float maxSeaLevel = 0.5f;

    [Tooltip("0.0 = Absolute Zero, 0.5 = Freezing, 1.0 = Boiling, 2.0 = Scorching")]
    [Range(0f, 2f)] public float averageTemp = 1.5f;

    [Tooltip("Number of continent sections the world is divided into. 'Tectonic resolution,' as it were. Constructed via Voronoi patterns")]
    [Range(10, 100)] public int continentCellCount = 32;

    [Tooltip("Determines the strength of noise used to break up inorganic voronoi divisions")]
    [Range(0f, 1f)] public float continentEdgeNoise;

    [Tooltip("Actual Sea Level calculated from maximum sea level and temperature.")]
    [Header("Calculated Results (Read Only)")]
    public float actualSeaLevel;

    //Derived from Atmospheric Percentages.
    [Header("Atmospheric Data")]
    public AtmosphericData atmosphere;

    private void OnValidate()
    {
        atmosphere.BalanceDryRecipe();
        
        // --- 1. Calculate Actual Sea Level (The "Liquid" State) ---
        // We want sea level to peak at boiling (1.0) and fall off in both directions.
        if (averageTemp <= 1.0f)
        {
            // RISING PHASE (Freezing -> Boiling)
            // Using a square root (Pow 0.5) makes the sea level rise quickly at first 
            // then slow down as it approaches the boiling point.
            actualSeaLevel = maxSeaLevel * Mathf.Pow(Mathf.Clamp01(averageTemp), 0.5f);
        }
        else
        {
            // BOILING PHASE (Boiling -> Scorching)
            // Use a reciprocal function so it drops but NEVER hits zero.
            // As temp increases, sea level approaches 0 but always keeps "puddles."
            float excessTemp = averageTemp - 1.0f;
            actualSeaLevel = maxSeaLevel / (1.0f + (excessTemp * excessTemp * 10.0f));
        }

        // --- 2. Calculate Water Vapor Potential ---
        // Vapor comes from two sources: ambient evaporation and boiling.
        float ambientEvap = actualSeaLevel * Mathf.Min(averageTemp, 1.0f) * 0.2f;
        float boiledOffSteam = Mathf.Max(0, maxSeaLevel - actualSeaLevel);
        
        // This is our "Raw" vapor value which could technically exceed 1.0
        float rawVapor = ambientEvap + boiledOffSteam;

        // --- 3. Apply the Asymptotic Soft-Clamp ---
        // This formula: y = x / (k + x) ensures the result approaches 1.0 
        // but needs an "infinitely" high rawVapor to actually get there.
        // The 0.1f constant controls how "steep" the curve is.
        float vaporPercentage = rawVapor / (0.1f + rawVapor);

        // --- 4. Finalize Atmosphere ---
        // We cap it at 0.99f just to ensure the player can always see 
        // at least a sliver of the other gases in the UI.
        atmosphere.CalculateFinalState(Mathf.Min(vaporPercentage, 0.99f));
    }
}


