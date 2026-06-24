using UnityEngine;

[System.Serializable]
public class ClimateState
{
    
    public AtmosphericData atmosphere;
    [System.NonSerialized] public float globalMoisture;
    public float averageTemp;
    [System.NonSerialized] public float actualSeaLevel;

    public ClimateState(ClimateSettings cSettings, TerrainSettings tSettings)
    {
        
        // Copy starting values from the asset — we never touch the asset again.
        atmosphere = new AtmosphericData
        {
            nitrogen = cSettings.atmosphere.nitrogen,
            carbonDioxide = cSettings.atmosphere.carbonDioxide,
            methane = cSettings.atmosphere.methane,
            oxygen = cSettings.atmosphere.oxygen,
            traceElements = cSettings.atmosphere.traceElements
        };
        globalMoisture = tSettings.globalMoisture;
        averageTemp = cSettings.averageTemp;
        RecalculateDerivedValues();
    }

    // Call this whenever a game event changes globalMoisture or averageTemp.
    public void RecalculateDerivedValues()
    {
        if (averageTemp <= 1.0f)
        {
            actualSeaLevel = globalMoisture * UnityEngine.Mathf.Pow(
                UnityEngine.Mathf.Clamp01(averageTemp), 0.5f);
        }
        else
        {
            float excess = averageTemp - 1.0f;
            actualSeaLevel = globalMoisture / (1.0f + (excess * excess * 10.0f));
        }

        float ambientEvap = actualSeaLevel * UnityEngine.Mathf.Min(averageTemp, 1.0f) * 0.2f;
        float boiledOffSteam = UnityEngine.Mathf.Max(0, globalMoisture - actualSeaLevel);
        float rawVapor = ambientEvap + boiledOffSteam;
        float vaporPercentage = rawVapor / (0.1f + rawVapor);

        atmosphere.BalanceDryRecipe();
        atmosphere.CalculateFinalState(UnityEngine.Mathf.Min(vaporPercentage, 0.99f));
    }
}